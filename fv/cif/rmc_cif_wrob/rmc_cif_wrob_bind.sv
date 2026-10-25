// rmc_cif_wrob_bind.sv
// Attaches the checker to every rmc_cif_wrob instance.

bind rmc_cif_wrob rmc_cif_wrob_fv #(
  .ROB_DEPTH (ROB_DEPTH),
  .MAX_PKTS  (MAX_PKTS),
  .AXI_IDW   (AXI_IDW),
  .ROB_IDX_W (ROB_IDX_W),
  .PTR_W     (PTR_W),
  .CNT_W     (CNT_W),
  .OCC_W     (OCC_W)
) u_fv (
  .aclk           (aclk),
  .aresetn        (aresetn),
  .cmpl_valid     (cmpl_valid),
  .cmpl_rob_index (cmpl_rob_index),
  .cmpl_err       (cmpl_err),
  .ROB_IDX        (ROB_IDX),
  .rob_free       (rob_free),
  .admit_valid    (admit_valid),
  .admit_id       (admit_id),
  .admit_resp     (admit_resp),
  .SEG_LAST       (SEG_LAST),
  .SEG_ROB_IDX    (SEG_ROB_IDX),
  .SEG_NPKTS_CH   (SEG_NPKTS_CH),
  .RESP_VALID     (RESP_VALID),
  .RESP_READY     (RESP_READY),
  .RESP_ID        (RESP_ID),
  .RESP_RESP      (RESP_RESP),
  .RESP_NPKTS_CH  (RESP_NPKTS_CH),
  // internals
  .alloc_ptr      (alloc_ptr),
  .retire_ptr     (retire_ptr),
  .occ            (occ),
  .ent_resp       (ent_resp),
  .ent_derr       (ent_derr),
  .inflow_complt  (inflow_complt),
  .run_cnt        (run_cnt),
  .outflow_cnt    (outflow_cnt)
);
