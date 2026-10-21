// rmc_cif_w_burst_spltr.sv
// WRITE burst splitter (twin of rmc_cif_r_burst_spltr, but in order and issuing writes).
// Pure control: it moves no data and owns no slot pointers. For one admitted write request
// it walks the 64B lines in address order and emits one DRAM write command per line, then
// hands w_rob the per-channel run descriptor.
//
// Per line, paced by the write accumulator's `filled` pulse (the line's data is staged):
//   ch    = addrmap(line address).ch           which DRAM channel (asserted via ch/ch_valid)
//   daddr = addrmap(line address).daddr         DRAM coords
//   slot  = write_in_ptr[ch]  (from the frptr block, the same value the accumulator writes at)
//   emit  packet_*[ch] = { rob_idx, daddr, slot } -> MC request FIFO (op=write; mask in SRAM)
//   pulse alloc[ch]        -> frptr advances write_in_ptr[ch]
//   bookkeep: first hit of ch records write_in_ptr[ch] as that channel's run base;
//             cnt[ch]++ ; one line fewer to go.
// At the last line: drive SEG_LAST + SEG_NPKTS (total) + SEG_HEAD_PTR[N_CH] + SEG_NPKTS_CH[N_CH]
// to w_rob, then capture the next admitted request. Narrow/unaligned masking rides as the
// per-slot wstrb the accumulator stored in WD-SRAM, not in the packet.
//
// The accumulator and the splitter step in lockstep on line order (W beats arrive in AW
// order; the splitter walks the same order), so `filled` marks exactly the line at line_q.

import rmc_cfg_pkg::*;

module rmc_cif_w_burst_spltr #(
  parameter int ROB_DEPTH = 32,         // ROB entries             [root knob]
  parameter int MAX_PKTS  = 64,         // max packets per request [root knob]
  parameter int AXI_AW    = SYS_ADDR_W, // AXI address width       [root knob]
  parameter int N_WDB     = 64,         // WD-SRAM slots per channel [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int CNT_W     = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int SLOT_W    = (N_WDB     > 1) ? $clog2(N_WDB)      : 1,
  localparam int LINE_W    = PAGE_LSB - PKT_OFF_W        // in-page line index width (= 6)
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // request in (from the AW request buffer): address + burst params + the ROB index it got
  input  logic                   req_in_valid,
  input  logic [AXI_AW-1:0]      req_in_addr,
  input  logic [7:0]             req_in_len,
  input  logic [2:0]             req_in_size,
  input  logic [ROB_IDX_W-1:0]   req_in_rob_idx,
  output logic                   req_in_ready,     // pop the buffer when we capture a request

  // live address map (CSR)
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // frptr: per-channel write pointer + space in; alloc out (advance the issued channel)
  input  logic [N_CH-1:0][SLOT_W-1:0] write_in_ptr,
  input  logic [N_CH-1:0]             sram_ready,
  output logic [N_CH-1:0]             alloc,

  // accumulator: filled (a 64B line is staged) in; assert the line's channel (ch + ch_valid)
  input  logic                   filled,
  output logic [CH_W-1:0]        ch,
  output logic                   ch_valid,

  // back to w_rob when the split finishes: total count + per-channel run descriptor
  output logic                   SEG_LAST,
  output logic [CNT_W-1:0]       SEG_NPKTS,
  output logic [N_CH-1:0][SLOT_W-1:0] SEG_HEAD_PTR,  // each channel's first slot (run base)
  output logic [N_CH-1:0][CNT_W-1:0]  SEG_NPKTS_CH,  // each channel's slot count

  // MC write packet stream, ONE PER CHANNEL (mask rides in WD-SRAM, not the packet)
  output logic [N_CH-1:0]                packet_valid,
  input  logic [N_CH-1:0]                packet_ready,
  output logic [N_CH-1:0][ROB_IDX_W-1:0] packet_rob_index,
  output logic [N_CH-1:0][DADDR_W-1:0]   packet_daddr,
  output logic [N_CH-1:0][SLOT_W-1:0]    packet_slot
);

  // walk state for the one request being split
  logic                 run;
  logic [AXI_AW-1:0]    s_addr;
  logic [2:0]           s_size;
  logic [ROB_IDX_W-1:0] s_rob;
  logic [LINE_W-1:0]    line_q;        // current in-page line index
  logic [LINE_W-1:0]    l_last;        // last line of the request
  logic [CNT_W-1:0]     n_left;        // lines still to issue

  // per-channel run bookkeeping (recorded during the walk, shipped at SEG_LAST)
  logic [N_CH-1:0][SLOT_W-1:0] head_q;
  logic [N_CH-1:0][CNT_W-1:0]  cnt_q;
  logic [N_CH-1:0]             seen_q;

  // bounds of an incoming request (combinational, used at capture)
  logic [PAGE_LSB-1:0] size_mask, a_al;
  logic [15:0]         n_bytes, last_byte;
  logic [LINE_W-1:0]   cap_l0, cap_llast;
  logic [CNT_W-1:0]    cap_npkts;
  assign size_mask = PAGE_LSB'((32'd1 << req_in_size) - 32'd1);
  assign a_al      = req_in_addr[PAGE_LSB-1:0] & ~size_mask;
  assign n_bytes   = (16'(req_in_len) + 16'd1) << req_in_size;
  assign last_byte = 16'(a_al) + n_bytes - 16'd1;
  assign cap_l0    = req_in_addr[PAGE_LSB-1:PKT_OFF_W];
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

  // capture a request when idle; otherwise issue the current line
  assign req_in_ready = ~run & req_in_valid;        // one-cycle capture/pop
  wire   capture      = req_in_ready;

  // issue the current line: it is staged (filled), its channel has room, and the MC FIFO
  // accepts it. packet_valid asserts (does not depend on packet_ready); fire advances.
  wire issue_ok = run & filled & sram_ready[w_ch];
  wire fire     = issue_ok & packet_ready[w_ch];
  wire is_last  = (n_left == CNT_W'(1));

  always_comb begin
    packet_valid     = '0;
    packet_rob_index = '0;
    packet_daddr     = '0;
    packet_slot      = '0;
    alloc            = '0;
    if (issue_ok) begin
      packet_valid[w_ch]     = 1'b1;
      packet_rob_index[w_ch] = s_rob;
      packet_daddr[w_ch]     = w_daddr;
      packet_slot[w_ch]      = write_in_ptr[w_ch];
    end
    if (fire) alloc[w_ch] = 1'b1;              // advance that channel's write pointer
  end

  // tell the accumulator which channel to commit the staged line to, as it is accepted
  assign ch       = w_ch;
  assign ch_valid = fire;

  // SEG_* to w_rob at the last accepted line, with the final per-channel counts/bases
  logic [N_CH-1:0][SLOT_W-1:0] head_final;
  logic [N_CH-1:0][CNT_W-1:0]  cnt_final;
  always_comb
    for (int c = 0; c < N_CH; c++) begin
      cnt_final[c]  = cnt_q[c]  + ((fire && w_ch == CH_W'(c)) ? CNT_W'(1) : '0);
      head_final[c] = (seen_q[c] || !(fire && w_ch == CH_W'(c))) ? head_q[c]
                                                                 : write_in_ptr[c];
    end
  logic [CNT_W-1:0] npkts_sum;
  always_comb begin
    npkts_sum = '0;
    for (int c = 0; c < N_CH; c++) npkts_sum = npkts_sum + cnt_final[c];
  end
  assign SEG_LAST     = fire & is_last;
  assign SEG_NPKTS    = SEG_LAST ? npkts_sum : '0;
  assign SEG_HEAD_PTR = head_final;
  assign SEG_NPKTS_CH = cnt_final;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      run    <= 1'b0;
      seen_q <= '0;
      cnt_q  <= '0;
    end else if (capture) begin
      run    <= 1'b1;
      s_addr <= req_in_addr;
      s_size <= req_in_size;
      s_rob  <= req_in_rob_idx;
      line_q <= cap_l0;
      l_last <= cap_llast;
      n_left <= cap_npkts;
      seen_q <= '0;
      cnt_q  <= '0;
    end else if (fire) begin
      // record this channel's run base on its first hit, bump its count
      if (!seen_q[w_ch]) head_q[w_ch] <= write_in_ptr[w_ch];
      seen_q[w_ch] <= 1'b1;
      cnt_q[w_ch]  <= cnt_q[w_ch] + CNT_W'(1);
      if (is_last) run <= 1'b0;                 // request fully issued
      else begin
        line_q <= line_q + 1'b1;
        n_left <= n_left - CNT_W'(1);
      end
    end
  end

`ifndef SYNTHESIS
  // the request must stay inside its 4KB page (no line index carry out)
  always @(posedge aclk) if (aresetn && capture)
    assert (last_byte[15:PAGE_LSB] == '0)
      else $error("w_burst_spltr: AW crosses 4KB (addr=%h len=%0d size=%0d)",
                  req_in_addr, req_in_len, req_in_size);
`endif

endmodule : rmc_cif_w_burst_spltr
