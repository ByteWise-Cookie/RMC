// rmc_cif_rob_bind.sv
// Attaches the checker to every rmc_cif_rob instance. Names on the right resolve
// inside the DUT, so internals are reachable without touching the RTL.

bind rmc_cif_rob rmc_cif_rob_fv #(
  .ROB_DEPTH  (ROB_DEPTH),
  .MAX_PKTS   (MAX_PKTS),
  .AXI_IDW    (AXI_IDW),
  .NEED_META  (NEED_META),
  .PTR_W      (PTR_W),
  .OCC_W      (OCC_W),
  .CNT_W      (CNT_W),
  .REQ_META_W (REQ_META_W),
  .SLOT_PTR_W (SLOT_PTR_W),
  .AMETA_W    (AMETA_W),
  .META_W     (META_W)
) u_fv (
  .aclk           (aclk),
  .aresetn        (aresetn),
  .cmpl_valid     (cmpl_valid),
  .cmpl_rob_index (cmpl_rob_index),
  .cmpl_err       (cmpl_err),
  .AW_REQUEST     (AW_REQUEST),
  .A_META         (A_META),
  .A_ID_VALID     (A_ID_VALID),
  .A_ID_READY     (A_ID_READY),
  .ROB_IDX        (ROB_IDX),
  .ROB_IDX_VALID  (ROB_IDX_VALID),
  .SEG_LAST       (SEG_LAST),
  .SEG_NPKTS      (SEG_NPKTS),
  .RESP_VALID     (RESP_VALID),
  .RESP_READY     (RESP_READY),
  .RESP_ID        (RESP_ID),
  .RESP_RESP      (RESP_RESP),
  .RESP_HEAD_PTR  (RESP_HEAD_PTR),
  .RESP_OFFSET    (RESP_OFFSET),
  .RESP_LEN       (RESP_LEN),
  .RESP_SIZE      (RESP_SIZE),
  .alloc_ptr      (alloc_ptr),
  .retire_ptr     (retire_ptr),
  .occ            (occ),
  .pending_split  (pending_split),
  .ent_id         (ent_id),
  .ent_err        (ent_err),
  .ent_derr       (ent_derr),
  .inflow_complt  (inflow_complt),
  .n_pkts         (n_pkts),
  .outflow_cnt    (outflow_cnt)
);
