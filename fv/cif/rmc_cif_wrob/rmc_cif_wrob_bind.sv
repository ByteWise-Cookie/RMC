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
  .rob_idx        (rob_idx),
  .rob_free       (rob_free),
  .entry_valid    (entry_valid),
  .entry_id       (entry_id),
  .entry_resp     (entry_resp),
  .entry_npkts    (entry_npkts),
  .RESP_VALID     (RESP_VALID),
  .RESP_READY     (RESP_READY),
  .RESP_ID        (RESP_ID),
  .RESP_RESP      (RESP_RESP),
  // internals
  .alloc_ptr      (alloc_ptr),
  .retire_ptr     (retire_ptr),
  .occ            (occ),
  .ent_resp       (ent_resp),
  .ent_derr       (ent_derr),
  .inflow_complt  (inflow_complt),
  .n_pkts         (n_pkts),
  .outflow_cnt    (outflow_cnt)
);
