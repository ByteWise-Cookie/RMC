// rmc_cif_mc_req_arb_bind.sv
// Attaches the checker to every rmc_cif_mc_req_arb instance.

bind rmc_cif_mc_req_arb rmc_cif_mc_req_arb_fv #(
  .ROB_IDX_W (ROB_IDX_W),
  .DADDR_W   (DADDR_W),
  .SLOT_W    (SLOT_W),
  .PKT_NUM_W (PKT_NUM_W),
  .SEL_W     (SEL_W)
) u_fv (
  .aclk                   (aclk),
  .aresetn                (aresetn),
  .rd_valid               (rd_valid),
  .rd_ready               (rd_ready),
  .rd_rob_idx             (rd_rob_idx),
  .rd_daddr               (rd_daddr),
  .rd_slot                (rd_slot),
  .wr_valid               (wr_valid),
  .wr_ready               (wr_ready),
  .wr_rob_idx             (wr_rob_idx),
  .wr_daddr               (wr_daddr),
  .wr_slot                (wr_slot),
  .async_mc_req_valid     (async_mc_req_valid),
  .async_mc_req_ready     (async_mc_req_ready),
  .async_mc_req_rob_index (async_mc_req_rob_index),
  .async_mc_req_op        (async_mc_req_op),
  .async_mc_req_daddr     (async_mc_req_daddr),
  .async_mc_req_sram_slot (async_mc_req_sram_slot),
  // internals
  .rr_ptr                 (rr_ptr)
);
