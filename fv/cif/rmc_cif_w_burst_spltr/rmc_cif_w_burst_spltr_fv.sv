// rmc_cif_w_burst_spltr_fv.sv
// Formal checker for rmc_cif_w_burst_spltr. Bound by rmc_cif_w_burst_spltr_bind.sv. Scaffold:
// capture gating (one request at a time), the descriptor + accumulator drives, and the single
// ROB entry-write condition. A full per-line walk reference model (line count = address span,
// per-channel counts) is left as an extension.

import rmc_cfg_pkg::*;

module rmc_cif_w_burst_spltr_fv #(
  parameter int ROB_DEPTH = 32,
  parameter int MAX_PKTS  = 64,
  parameter int AXI_AW    = 48,
  parameter int AXI_IDW   = 8,
  parameter int ROB_IDX_W = 5,
  parameter int CNT_W     = 7,
  parameter int LINE_W    = 6
)(
  input logic                   aclk,
  input logic                   aresetn,
  input logic                   aw_request_valid,
  input logic                   burst_splitter_ready,
  input logic [AXI_IDW-1:0]     aw_id,
  input logic [1:0]             error,
  input logic [ROB_IDX_W-1:0]   rob_idx_in,
  input logic                   rob_free,
  input logic                   entry_valid,
  input logic [AXI_IDW-1:0]     entry_id,
  input logic [1:0]             entry_resp,
  input logic                   stage_en,
  input logic                   skip,
  input logic [PKT_OFF_W-1:0]   stage_len,
  input logic [2:0]             stage_size,
  input logic                   drain_done,
  input logic                   ld_valid,
  input logic                   ld_ready,
  input logic [ROB_IDX_W-1:0]   ld_rob_idx,
  // internals
  input logic                   run,
  input logic                   drop_q,
  input logic [AXI_AW-1:0]      s_addr,
  input logic [2:0]             s_size,
  input logic [AXI_IDW-1:0]     s_id,
  input logic [1:0]             s_err,
  input logic [ROB_IDX_W-1:0]   s_rob,
  input logic [CNT_W-1:0]       n_left
);
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  wire capture = burst_splitter_ready & aw_request_valid;
  wire is_last = (n_left == CNT_W'(1));
  wire fire    = ld_valid & ld_ready;

  // capture gating: idle + ROB slot free; one request at a time (no ready while running)
  as_ready: assert property (@(posedge aclk) disable iff (!aresetn)
    burst_splitter_ready == (~run & rob_free));

  // line descriptor + accumulator control
  as_ld_valid:  assert property (@(posedge aclk) disable iff (!aresetn) ld_valid == (run & ~drop_q));
  as_ld_rob:    assert property (@(posedge aclk) disable iff (!aresetn) ld_rob_idx == s_rob);
  as_stage_en:  assert property (@(posedge aclk) disable iff (!aresetn) stage_en == run);
  as_skip:      assert property (@(posedge aclk) disable iff (!aresetn) skip == drop_q);
  as_stage_len: assert property (@(posedge aclk) disable iff (!aresetn) stage_len == s_addr[PKT_OFF_W-1:0]);
  as_stage_sz:  assert property (@(posedge aclk) disable iff (!aresetn) stage_size == s_size);

  // single ROB entry write: last consumed line (good) or drain_done (dropped)
  as_entry:      assert property (@(posedge aclk) disable iff (!aresetn)
    entry_valid == ((fire & is_last) | drain_done));
  as_entry_id:   assert property (@(posedge aclk) disable iff (!aresetn) entry_id == s_id);
  as_entry_resp: assert property (@(posedge aclk) disable iff (!aresetn)
    entry_resp == (drop_q ? s_err : 2'b00));

  // capture latches the request and starts the run
  as_capture_run: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(capture) |->
      run & (drop_q == $past(error != 2'b00)) & (s_id == $past(aw_id))
      & (s_rob == $past(rob_idx_in)) & (s_err == $past(error)));

endmodule : rmc_cif_w_burst_spltr_fv
