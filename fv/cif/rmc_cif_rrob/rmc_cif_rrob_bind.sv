// rmc_cif_rrob_bind.sv
// Attaches the checker to every rmc_cif_rrob instance. Names on the right resolve
// inside the DUT, so the per-slot arrays and pointers are reachable without touching
// the RTL.

bind rmc_cif_rrob rmc_cif_rrob_fv #(
  .ROB_DEPTH  (ROB_DEPTH),
  .MAX_PKTS   (MAX_PKTS),
  .AXI_IDW    (AXI_IDW),
  .ROB_IDX_W  (ROB_IDX_W),
  .PTR_W      (PTR_W),
  .OCC_W      (OCC_W),
  .CNT_W      (CNT_W),
  .REQ_META_W (REQ_META_W),
  .SLOT_PTR_W (SLOT_PTR_W),
  .ADM_W      (8 + 3 + rmc_cfg_pkg::PAGE_LSB + rmc_cfg_pkg::CH_W)
) u_fv (
  .aclk           (aclk),
  .aresetn        (aresetn),
  .cmpl_valid     (cmpl_valid),
  .cmpl_rob_index (cmpl_rob_index),
  .cmpl_err       (cmpl_err),
  .AW_REQUEST     (AW_REQUEST),
  .ADMIT_NPKTS    (ADMIT_NPKTS),
  .ADMIT_ADDR_LO  (ADMIT_ADDR_LO),
  .ADMIT_LEN      (ADMIT_LEN),
  .ADMIT_SIZE     (ADMIT_SIZE),
  .ADMIT_BASE_CH  (ADMIT_BASE_CH),
  .A_ID_VALID     (A_ID_VALID),
  .A_ID_READY     (A_ID_READY),
  .ROB_IDX        (ROB_IDX),
  .SEG_LAST       (SEG_LAST),
  .SEG_ROB_IDX    (SEG_ROB_IDX),
  .SEG_HEAD_PTR   (SEG_HEAD_PTR),
  .RESP_VALID     (RESP_VALID),
  .RESP_READY     (RESP_READY),
  .RESP_ID        (RESP_ID),
  .RESP_RESP      (RESP_RESP),
  .RESP_HEAD_PTR  (RESP_HEAD_PTR),
  .RESP_BASE_CH   (RESP_BASE_CH),
  .RESP_ADDR_LO   (RESP_ADDR_LO),
  .RESP_LEN       (RESP_LEN),
  .RESP_SIZE      (RESP_SIZE),
  // internals
  .alloc_ptr      (alloc_ptr),
  .retire_ptr     (retire_ptr),
  .occ            (occ),
  .ent_id         (ent_id),
  .ent_err        (ent_err),
  .ent_derr       (ent_derr),
  .inflow_complt  (inflow_complt),
  .n_pkts         (n_pkts),
  .outflow_cnt    (outflow_cnt),
  .madm_len       (madm_len),
  .madm_size      (madm_size),
  .madm_alo       (madm_alo),
  .madm_bch       (madm_bch),
  .mem_head       (mem_head)
);
