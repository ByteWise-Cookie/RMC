// rob_test.sv
// Module-level test harness for rmc_cif_rob: one read ROB (DIR=0) and one write
// ROB (DIR=1) side by side, wired the way the CIF top uses them.
//
//   u_r_rob : retires to the AXI R path, read extras (head ptr / offset / len /
//             size) brought out
//   u_w_rob : retires to the AXI B path, id + resp only. NEED_META=0: the B
//             response uses no per-slot metadata, so no mem_meta is built
//
// The two ROBs share clock and reset only. Every other port is brought to the
// top with an r_ / w_ prefix so a testbench or a formal run can drive each side
// on its own. The formal checker (fv/cif/rmc_cif_rob) binds into both instances.

import rmc_cfg_pkg::*;

module rob_test #(
  parameter int ROB_DEPTH = 32,
  parameter int MAX_PKTS  = 16,
  parameter int AXI_IDW   = 8,
  parameter int AXI_AW    = 40,

  // derived (same expressions as rmc_cif_rob)
  localparam int ROB_IDX_W  = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int CNT_W      = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int REQ_META_W = AXI_IDW + 8 + 3 + 1,
  localparam int SLOT_PTR_W = 16,
  localparam int AMETA_W    = PKT_OFF_W + SLOT_PTR_W
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // ---- read ROB (AR admit -> R response) ----
  input  logic                   r_cmpl_valid,
  input  logic [ROB_IDX_W-1:0]   r_cmpl_rob_index,
  input  logic                   r_cmpl_err,
  input  logic [REQ_META_W-1:0]  r_request,        // {ARID, ARLEN, ARSIZE, validated}
  input  logic [AMETA_W-1:0]     r_meta,           // {start_offset, head_mem_ptr}
  input  logic                   r_id_valid,
  output logic                   r_id_ready,
  output logic [ROB_IDX_W-1:0]   r_rob_idx,
  output logic                   r_rob_idx_valid,
  input  logic                   r_seg_last,
  input  logic [CNT_W-1:0]       r_seg_npkts,
  output logic                   r_resp_valid,
  input  logic                   r_resp_ready,
  output logic [AXI_IDW-1:0]     r_resp_id,        // RID
  output logic [1:0]             r_resp_resp,      // RRESP
  output logic [SLOT_PTR_W-1:0]  r_resp_head_ptr,
  output logic [PKT_OFF_W-1:0]   r_resp_offset,
  output logic [7:0]             r_resp_len,
  output logic [2:0]             r_resp_size,

  // ---- write ROB (AW admit -> B response) ----
  input  logic                   w_cmpl_valid,
  input  logic [ROB_IDX_W-1:0]   w_cmpl_rob_index,
  input  logic                   w_cmpl_err,
  input  logic [REQ_META_W-1:0]  w_request,        // {AWID, AWLEN, AWSIZE, validated}
  input  logic [AMETA_W-1:0]     w_meta,
  input  logic                   w_id_valid,
  output logic                   w_id_ready,
  output logic [ROB_IDX_W-1:0]   w_rob_idx,
  output logic                   w_rob_idx_valid,
  input  logic                   w_seg_last,
  input  logic [CNT_W-1:0]       w_seg_npkts,
  output logic                   w_resp_valid,
  input  logic                   w_resp_ready,
  output logic [AXI_IDW-1:0]     w_resp_id,        // BID
  output logic [1:0]             w_resp_resp       // BRESP
);

  rmc_cif_rob #(
    .DIR       (0),
    .ROB_DEPTH (ROB_DEPTH),
    .MAX_PKTS  (MAX_PKTS),
    .AXI_IDW   (AXI_IDW),
    .AXI_AW    (AXI_AW)
  ) u_r_rob (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .cmpl_valid     (r_cmpl_valid),
    .cmpl_rob_index (r_cmpl_rob_index),
    .cmpl_err       (r_cmpl_err),
    .AW_REQUEST     (r_request),
    .A_META         (r_meta),
    .A_ID_VALID     (r_id_valid),
    .A_ID_READY     (r_id_ready),
    .ROB_IDX        (r_rob_idx),
    .ROB_IDX_VALID  (r_rob_idx_valid),
    .SEG_LAST       (r_seg_last),
    .SEG_NPKTS      (r_seg_npkts),
    .RESP_VALID     (r_resp_valid),
    .RESP_READY     (r_resp_ready),
    .RESP_ID        (r_resp_id),
    .RESP_RESP      (r_resp_resp),
    .RESP_HEAD_PTR  (r_resp_head_ptr),
    .RESP_OFFSET    (r_resp_offset),
    .RESP_LEN       (r_resp_len),
    .RESP_SIZE      (r_resp_size)
  );

  rmc_cif_rob #(
    .DIR       (1),
    .ROB_DEPTH (ROB_DEPTH),
    .MAX_PKTS  (MAX_PKTS),
    .AXI_IDW   (AXI_IDW),
    .AXI_AW    (AXI_AW),
    .NEED_META (1'b0)
  ) u_w_rob (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .cmpl_valid     (w_cmpl_valid),
    .cmpl_rob_index (w_cmpl_rob_index),
    .cmpl_err       (w_cmpl_err),
    .AW_REQUEST     (w_request),
    .A_META         (w_meta),
    .A_ID_VALID     (w_id_valid),
    .A_ID_READY     (w_id_ready),
    .ROB_IDX        (w_rob_idx),
    .ROB_IDX_VALID  (w_rob_idx_valid),
    .SEG_LAST       (w_seg_last),
    .SEG_NPKTS      (w_seg_npkts),
    .RESP_VALID     (w_resp_valid),
    .RESP_READY     (w_resp_ready),
    .RESP_ID        (w_resp_id),
    .RESP_RESP      (w_resp_resp),
    // read extras: tied to 0 by NEED_META=0, unused on the B path
    .RESP_HEAD_PTR  (),
    .RESP_OFFSET    (),
    .RESP_LEN       (),
    .RESP_SIZE      ()
  );

`ifdef FORMAL
  // both sides can be busy at once: a read and a write retire in the same cycle,
  // and one side is full while the other is empty
  c_both_retire: cover property (@(posedge aclk) disable iff (!aresetn)
    r_resp_valid && r_resp_ready && w_resp_valid && w_resp_ready);
  c_r_full_w_idle: cover property (@(posedge aclk) disable iff (!aresetn)
    !r_id_ready && !r_rob_idx_valid && w_id_ready && !w_resp_valid);
`endif

endmodule : rob_test
