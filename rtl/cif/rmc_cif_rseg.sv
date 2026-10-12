// rmc_cif_rseg.sv
// READ burst segmenter - full. N_CH per-channel engines (rmc_cif_rseg_ch) run in
// LOCKSTEP on one request: the request is broadcast to all, each engine issues only
// its channel's 64B packets, and when every engine has finished its share the wrapper
// pulses seg_last to the ROB.
//
// LOCKSTEP = one request in flight across all channels at a time. A channel that owns
// many packets holds up the next request even while other channels are idle. Pipelining
// (engines on different requests) needs the queue + per-engine ring ptr - a later step.
//
// n_pkts / offset / rob_idx are request-global (same for every channel): the wrapper
// computes/latches them once and drives the ROB sideband. Each engine also derives its
// own n_pkts internally for its loop bound (identical value).

import rmc_cfg_pkg::*;

module rmc_cif_rseg #(
  parameter int ROB_DEPTH = 32,         // ROB entries (rob_idx width) [root knob]
  parameter int MAX_PKTS  = 64,         // max packets per request     [root knob]
  parameter int AXI_AW    = SYS_ADDR_W, // AXI address width           [root knob]
  parameter int N_SLOTS   = 32,         // RD-SRAM slots               [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W  = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int CNT_W      = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int SLOT_W     = (N_SLOTS   > 1) ? $clog2(N_SLOTS)    : 1,
  localparam int DATA_BYTES = AXI_DW / 8,                           // = N_CH*8 (beat width)
  localparam int SIZE_MAX   = (DATA_BYTES > 1) ? $clog2(DATA_BYTES) : 0,
  localparam int MAX_BEATS  = MAX_REQ_BYTES / DATA_BYTES,
  localparam int LEN_W      = (MAX_BEATS  > 1) ? $clog2(MAX_BEATS)  : 1, // awlen (beats-1)
  localparam int SIZE_W     = (SIZE_MAX   > 0) ? $clog2(SIZE_MAX+1) : 1, // awsize
  localparam int SPAN_W     = $clog2(MAX_REQ_BYTES + 1)             // bytes spanned (<=4KB)
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // live address map (CSR), broadcast to every engine
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // request in (broadcast to all engines on launch)
  input  logic                   start_req,
  input  logic [AXI_AW-1:0]      req_addr,
  input  logic [LEN_W-1:0]       req_len,
  input  logic [SIZE_W-1:0]      req_size,
  input  logic [SLOT_W-1:0]      req_dbuf_base,
  input  logic [ROB_IDX_W-1:0]   req_rob_idx,
  output logic                   ready,           // wrapper can accept a request

  // per-channel packet streams out (one req FIFO per channel -> MC)
  output logic [N_CH-1:0]                pkt_valid,
  input  logic [N_CH-1:0]                pkt_ready,
  output logic [N_CH-1:0][ROB_IDX_W-1:0] pkt_rob_idx,
  output logic [N_CH-1:0][DADDR_W-1:0]   pkt_daddr,
  output logic [N_CH-1:0][SLOT_W-1:0]    pkt_dbuf_adr,

  // to the ROB: one pulse when the whole request is segmented
  output logic                   seg_last,
  output logic [ROB_IDX_W-1:0]   seg_rob_idx,
  output logic [CNT_W-1:0]       seg_npkts,       // total packets across all channels
  output logic [PKT_OFF_W-1:0]   seg_offset       // unaligned start offset, for resp trim
);

  // request-global packet count (same formula the engines use internally)
  logic [SPAN_W-1:0] span;
  logic [CNT_W-1:0]  npkts_calc;
  assign span       = SPAN_W'(req_addr[PKT_OFF_W-1:0]) + ((SPAN_W'(req_len) + 1'b1) << req_size);
  assign npkts_calc = CNT_W'((span + SPAN_W'((1 << PKT_OFF_W) - 1)) >> PKT_OFF_W);

  // per-engine handshakes
  logic [N_CH-1:0] eng_ready;
  logic [N_CH-1:0] eng_done;

  // wrapper control
  logic wrap_busy;
  assign ready = ~wrap_busy;

  wire launch       = ready & start_req & (&eng_ready);   // all engines free -> start all
  wire all_eng_done = wrap_busy & (&eng_ready);           // every engine finished its share

  // latched request-global meta, driven to the ROB at seg_last
  logic [ROB_IDX_W-1:0] rob_idx_q;
  logic [CNT_W-1:0]     npkts_q;
  logic [PKT_OFF_W-1:0] offset_q;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      wrap_busy <= 1'b0;
      rob_idx_q <= '0;
      npkts_q   <= '0;
      offset_q  <= '0;
    end else if (launch) begin
      wrap_busy <= 1'b1;
      rob_idx_q <= req_rob_idx;
      npkts_q   <= npkts_calc;
      offset_q  <= req_addr[PKT_OFF_W-1:0];
    end else if (all_eng_done) begin
      wrap_busy <= 1'b0;
    end
  end

  assign seg_last    = all_eng_done;
  assign seg_rob_idx = rob_idx_q;
  assign seg_npkts   = npkts_q;
  assign seg_offset  = offset_q;

  // one engine per channel, all launched together on `launch`
  genvar c;
  generate
    for (c = 0; c < N_CH; c++) begin : g_ch
      rmc_cif_rseg_ch #(
        .ROB_DEPTH (ROB_DEPTH),
        .MAX_PKTS  (MAX_PKTS),
        .AXI_AW    (AXI_AW),
        .N_SLOTS   (N_SLOTS)
      ) u_ch (
        .aclk         (aclk),
        .aresetn      (aresetn),
        .my_ch        (CH_W'(c)),
        .addr_map     (addr_map),
        .start        (launch),
        .req_addr     (req_addr),
        .req_len      (req_len),
        .req_size     (req_size),
        .req_dbuf_base(req_dbuf_base),
        .req_rob_idx  (req_rob_idx),
        .ready        (eng_ready[c]),
        .pkt_valid    (pkt_valid[c]),
        .pkt_ready    (pkt_ready[c]),
        .pkt_rob_idx  (pkt_rob_idx[c]),
        .pkt_daddr    (pkt_daddr[c]),
        .pkt_dbuf_adr (pkt_dbuf_adr[c]),
        .seg_offset   (),                  // request-global; wrapper drives the ROB copy
        .done         (eng_done[c])
      );
    end
  endgenerate

endmodule : rmc_cif_rseg
