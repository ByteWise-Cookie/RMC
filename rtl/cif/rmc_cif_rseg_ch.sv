// rmc_cif_rseg_ch.sv
// READ burst segmenter - one channel. Instanced once per DRAM channel by rmc_cif_rseg.
//
// Given a read request {base addr, len, size, dbuf_base, rob_idx} and the live address
// map, this unit issues only the 64B-line packets that land in ITS channel (my_ch),
// one CAS each, to its channel's req FIFO. No ROB, ring, or queue here - just the
// per-channel issue engine.
//
// The unaligned start offset addr[5:0] is NOT consumed or zeroed: it is read for the
// n_pkts count and also passed straight out on seg_offset, riding unchanged to the ROB
// for response-path trimming.
//
// STRIDE (contract map: ch = low CH_W bits of the line number):
//   consecutive lines round-robin channels with period N_CH, so this channel's lines
//   are an arithmetic sequence. With base_ch = line[CH_W-1:0] (= addr[6 +: CH_W]):
//     k0 = (my_ch - base_ch) mod N_CH          first packet index in the request
//     k  = k0, k0+N_CH, k0+2*N_CH, ...          each subsequent channel-my_ch packet
//   line addr = (aligned_base + k) << 6 ; addrmap(line) -> daddr (DRAM coords).
//   dbuf slot = dbuf_base + k (global contiguous).
//
// Assumes the ch field is the low contiguous line field (the contract/default map).
// A scattered CSR map would need a scan instead - not built here.

import rmc_cfg_pkg::*;

module rmc_cif_rseg_ch #(
  parameter int ROB_DEPTH = 32,         // ROB entries (rob_idx width) [root knob]
  parameter int MAX_PKTS  = 64,         // max packets per request     [root knob]
  parameter int AXI_AW    = SYS_ADDR_W, // AXI address width           [root knob]
  parameter int N_SLOTS   = 32,         // RD-SRAM slots               [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W  = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int CNT_W      = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int SLOT_W     = (N_SLOTS   > 1) ? $clog2(N_SLOTS)    : 1,
  localparam int LINE_W     = AXI_AW - PKT_OFF_W,                    // 64B line number
  localparam int DATA_BYTES = AXI_DW / 8,                           // = N_CH*8 (beat width)
  localparam int SIZE_MAX   = (DATA_BYTES > 1) ? $clog2(DATA_BYTES) : 0,
  localparam int MAX_BEATS  = MAX_REQ_BYTES / DATA_BYTES,
  localparam int LEN_W      = (MAX_BEATS  > 1) ? $clog2(MAX_BEATS)  : 1, // awlen (beats-1)
  localparam int SIZE_W     = (SIZE_MAX   > 0) ? $clog2(SIZE_MAX+1) : 1, // awsize
  localparam int SPAN_W     = $clog2(MAX_REQ_BYTES + 1)             // bytes spanned (<=4KB)
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // which DRAM channel this instance owns
  input  logic [CH_W-1:0]        my_ch,

  // live address map (CSR)
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // request in. Captured on start & ready.
  input  logic                   start,
  input  logic [AXI_AW-1:0]      req_addr,
  input  logic [LEN_W-1:0]       req_len,
  input  logic [SIZE_W-1:0]      req_size,
  input  logic [SLOT_W-1:0]      req_dbuf_base,
  input  logic [ROB_IDX_W-1:0]   req_rob_idx,    // pre-allocated ROB index for this request
  output logic                   ready,          // ~busy: can take a new request

  // packet stream out, this channel only
  output logic                   pkt_valid,
  input  logic                   pkt_ready,
  output logic [ROB_IDX_W-1:0]   pkt_rob_idx,    // ROB index tag (MC completion routes by it)
  output logic [DADDR_W-1:0]     pkt_daddr,      // mapped DRAM coords
  output logic [SLOT_W-1:0]      pkt_dbuf_adr,   // RD-SRAM slot = dbuf_base + k

  // request-level sideband that rides to the ROB (unchanged)
  output logic [PKT_OFF_W-1:0]   seg_offset,     // unaligned start offset addr[5:0]

  // done pulse: this channel's share of the request is fully issued
  output logic                   done
);

  // captured request (held through the issue walk)
  logic                 busy;
  logic [LINE_W-1:0]    aligned_base;   // base 64B line = req_addr >> 6
  logic [PKT_OFF_W-1:0] offset;         // unaligned start offset addr[5:0]; rides to ROB
  logic [SLOT_W-1:0]    dbuf_base;      // RD-SRAM base slot
  logic [ROB_IDX_W-1:0] rob_idx;        // ROB index of the in-flight request
  logic [CNT_W-1:0]     n_pkts;         // total 64B packets in the request
  logic [CNT_W-1:0]     k;              // current line index within the request

  // --- combinational geometry of the incoming request (valid at start) ---
  // span from the 64B line start = unaligned head + burst bytes; n_pkts = ceil(span/64).
  logic [SPAN_W-1:0]    in_span;
  logic [CNT_W-1:0]     in_npkts;
  logic [CH_W-1:0]      in_base_ch;
  logic [CH_W-1:0]      in_k0;
  assign in_span    = SPAN_W'(req_addr[PKT_OFF_W-1:0]) + ((SPAN_W'(req_len) + 1'b1) << req_size);
  assign in_npkts   = CNT_W'((in_span + SPAN_W'((1 << PKT_OFF_W) - 1)) >> PKT_OFF_W);
  // contract map: channel = low CH_W bits of the line number (N_CH is a power of 2).
  assign in_base_ch = req_addr[PKT_OFF_W +: CH_W];
  assign in_k0      = (my_ch - in_base_ch) & CH_W'(N_CH - 1);

  assign ready = ~busy;

  // per-packet address -> DRAM coords. line k sits at byte (aligned_base + k) << 6.
  logic [AXI_AW-1:0]    line_sys_addr;
  logic [CH_W-1:0]      line_ch_unused;
  logic [PKT_OFF_W-1:0] line_off_unused;
  assign line_sys_addr = AXI_AW'(aligned_base + k) << PKT_OFF_W;

  rmc_cif_addrmap #(
    .AXI_AW  (AXI_AW),
    .DADDR_W (DADDR_W)
  ) u_map (
    .sys_addr (line_sys_addr),
    .addr_map (addr_map),
    .daddr    (pkt_daddr),
    .ch       (line_ch_unused),
    .offset   (line_off_unused)
  );

  assign pkt_rob_idx  = rob_idx;
  assign pkt_dbuf_adr = dbuf_base + SLOT_W'(k);
  assign pkt_valid    = busy & (k < n_pkts);
  assign seg_offset   = offset;                 // rides to ROB, untouched
  assign done         = busy & (k >= n_pkts);   // walk finished (incl. zero-packet case)

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      busy         <= 1'b0;
      aligned_base <= '0;
      offset       <= '0;
      dbuf_base    <= '0;
      rob_idx      <= '0;
      n_pkts       <= '0;
      k            <= '0;
    end else if (start & ready) begin
      busy         <= 1'b1;
      aligned_base <= req_addr[AXI_AW-1:PKT_OFF_W];
      offset       <= req_addr[PKT_OFF_W-1:0];   // capture, pass through unchanged
      dbuf_base    <= req_dbuf_base;
      rob_idx      <= req_rob_idx;
      n_pkts       <= in_npkts;
      k            <= CNT_W'(in_k0);             // first packet for this channel
    end else if (busy) begin
      if (k >= n_pkts)
        busy <= 1'b0;                            // done; drop busy
      else if (pkt_valid & pkt_ready)
        k <= k + CNT_W'(N_CH);                   // stride to next channel-my_ch packet
    end
  end

endmodule : rmc_cif_rseg_ch
