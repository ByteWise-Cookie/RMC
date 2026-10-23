// rmc_cif_w_burst_spltr.sv
// WRITE burst splitter: pure sequencer AND ROB master, sole consumer of the AW request buffer.
// It is the ONLY writer of the ROB and writes it exactly once per request - entry_valid at the
// last packet (seg_last) - so there is no separate claim/reserve: the ROB holds rob_free high
// while not full and offers rob_idx = alloc_ptr, which the splitter tags onto every packet and
// which advances on that one entry write. Walks the 64 B lines in address order and emits one
// line descriptor per line to the packet_formation_unit. Controls the accumulator (shape +
// skip). A front-end error (error != 0) issues no packets: drain the accumulator, then write
// the entry with npkts = 0 at drain_done.
//
// Internals: on capture it latches the request and computes its in-page line span from
// address + len + size - cap_l0 (first line), cap_llast (last line), cap_npkts (count). It then
// walks line_q from cap_l0, maps each line to ch/daddr through the shared address map, and
// counts lines per channel in cnt_q while n_left counts down to the last line. A line is
// consumed when pkt_gen takes it (fire = ld_valid & ld_ready). One request at a time:
// burst_splitter_ready is low during a walk, so the W stream and the ROB stay in order.

import rmc_cfg_pkg::*;

module rmc_cif_w_burst_spltr #(
  parameter int ROB_DEPTH = 32,         // ROB entries             [root knob]
  parameter int MAX_PKTS  = 64,         // max packets per request [root knob]
  parameter int AXI_AW    = SYS_ADDR_W, // AXI address width       [root knob]
  parameter int AXI_IDW   = 8,          // AXI id width            [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int CNT_W     = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int LINE_W    = PAGE_LSB - PKT_OFF_W       // in-page line index width (= 6)
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // a_request_buf (post-validator): id + burst params + validator verdict
  input  logic                   aw_request_valid,
  output logic                   burst_splitter_ready,
  input  logic [AXI_IDW-1:0]     aw_id,
  input  logic [7:0]             aw_len,
  input  logic [2:0]             aw_size,
  input  logic [AXI_AW-1:0]      phy_adr,
  input  logic [1:0]             error,            // 00 ok / 10 SLVERR / 11 DECERR

  // live address map (CSR)
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // w_rob: slot offer in, single entry write-back out (the only signal to the ROB)
  input  logic [ROB_IDX_W-1:0]   rob_idx_in,       // offered slot (= alloc_ptr), held for the req
  input  logic                   rob_free,         // slot free (ROB not full)
  output logic                   entry_valid,      // one write at seg_last (drain_done on error)
  output logic [AXI_IDW-1:0]     entry_id,
  output logic [1:0]             entry_resp,
  output logic [CNT_W-1:0]       entry_npkts,      // 0 on a front error
  output logic [N_CH-1:0][CNT_W-1:0]  entry_npkts_ch,

  // accumulator control (shape held for the W stream + drop; drain_done back)
  output logic                   stage_en,
  output logic                   skip,
  output logic [PKT_OFF_W-1:0]   stage_len,        // in-line start byte offset
  output logic [2:0]             stage_size,
  input  logic                   drain_done,

  // packet_formation_unit: line descriptor (consumed on ld_valid & ld_ready)
  output logic                   ld_valid,
  input  logic                   ld_ready,
  output logic [CH_W-1:0]        ld_ch,
  output logic [DADDR_W-1:0]     ld_daddr,
  output logic [ROB_IDX_W-1:0]   ld_rob_idx
);

  // walk state for the one request being split
  logic                 run;
  logic [AXI_AW-1:0]    s_addr;
  logic [2:0]           s_size;
  logic [AXI_IDW-1:0]   s_id;
  logic [1:0]           s_err;
  logic                 drop_q;        // the current request is drain-only (front error)
  logic [ROB_IDX_W-1:0] s_rob;         // the ROB slot this request holds (rob_idx_in latched)
  logic [LINE_W-1:0]    line_q;        // current in-page line index
  logic [CNT_W-1:0]     n_left;        // lines still to issue

  // per-channel line count (recorded during the walk, shipped at the entry write for the free)
  logic [N_CH-1:0][CNT_W-1:0]  cnt_q;

  // bounds of an incoming request (combinational, used at capture)
  logic [PAGE_LSB-1:0] size_mask, a_al;
  logic [15:0]         n_bytes, last_byte;
  logic [LINE_W-1:0]   cap_l0, cap_llast;
  logic [CNT_W-1:0]    cap_npkts;
  assign size_mask = PAGE_LSB'((32'd1 << aw_size) - 32'd1);
  assign a_al      = phy_adr[PAGE_LSB-1:0] & ~size_mask;
  assign n_bytes   = (16'(aw_len) + 16'd1) << aw_size;
  assign last_byte = 16'(a_al) + n_bytes - 16'd1;
  assign cap_l0    = phy_adr[PAGE_LSB-1:PKT_OFF_W];
  assign cap_llast = last_byte[PAGE_LSB-1:PKT_OFF_W];
  assign cap_npkts = CNT_W'(cap_llast) - CNT_W'(cap_l0) + CNT_W'(1);

  // current line address -> channel + DRAM coords via the shared address map
  wire [AXI_AW-1:0]  line_addr = {s_addr[AXI_AW-1:PAGE_LSB], line_q, {PKT_OFF_W{1'b0}}};
  wire [CH_W-1:0]    w_ch;
  wire [DADDR_W-1:0] w_daddr;
  rmc_cif_addrmap #(.AXI_AW (AXI_AW), .DADDR_W (DADDR_W)) u_amap (
    .sys_addr (line_addr),
    .addr_map (addr_map),
    .daddr    (w_daddr),
    .ch       (w_ch),
    .offset   (/* unused: byte-in-line, mask is in WD-SRAM */)
  );

  // capture a request when idle and a ROB slot is free; accept = pop the request buffer
  assign burst_splitter_ready = ~run & rob_free;
  wire   capture      = burst_splitter_ready & aw_request_valid;
  wire   is_last      = (n_left == CNT_W'(1));
  wire   fire         = ld_valid & ld_ready;        // pkt_gen consumed this line

  // line descriptor (combinational off the walk state). A dropped request emits no packets.
  assign ld_valid     = run & ~drop_q;
  assign ld_ch        = w_ch;
  assign ld_daddr     = w_daddr;
  assign ld_rob_idx   = s_rob;

  // accumulator control: hold the shape for the whole W stream
  assign stage_en     = run;
  assign skip         = drop_q;
  assign stage_len    = s_addr[PKT_OFF_W-1:0];
  assign stage_size   = s_size;

  // per-channel counts and totals (bumped by the line being consumed this cycle)
  logic [N_CH-1:0][CNT_W-1:0] cnt_final;
  logic [CNT_W-1:0]           npkts_sum;
  always_comb begin
    npkts_sum = '0;
    for (int c = 0; c < N_CH; c++) begin
      cnt_final[c] = cnt_q[c] + ((fire && w_ch == CH_W'(c)) ? CNT_W'(1) : '0);
      npkts_sum    = npkts_sum + cnt_final[c];
    end
  end

  // ENTRY to w_rob: good request at its last consumed line; dropped request at drain_done
  assign entry_valid    = (fire & is_last) | drain_done;
  assign entry_id       = s_id;
  assign entry_resp     = drop_q ? s_err : 2'b00;
  assign entry_npkts    = npkts_sum;                // 0 on a drop (no line ever fired)
  assign entry_npkts_ch = cnt_final;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      run    <= 1'b0;
      drop_q <= 1'b0;
      cnt_q  <= '0;
    end else if (capture) begin
      run    <= 1'b1;
      drop_q <= (error != 2'b00);
      s_addr <= phy_adr;
      s_size <= aw_size;
      s_id   <= aw_id;
      s_err  <= error;
      s_rob  <= rob_idx_in;
      line_q <= cap_l0;
      n_left <= cap_npkts;
      cnt_q  <= '0;
    end else if (drop_q) begin
      if (drain_done) run <= 1'b0;                  // dropped burst drained, release
    end else if (fire) begin
      cnt_q[w_ch] <= cnt_q[w_ch] + CNT_W'(1);       // one more line on this channel
      if (is_last) run <= 1'b0;                     // request fully issued
      else begin
        line_q <= line_q + 1'b1;
        n_left <= n_left - CNT_W'(1);
      end
    end
  end

endmodule : rmc_cif_w_burst_spltr
