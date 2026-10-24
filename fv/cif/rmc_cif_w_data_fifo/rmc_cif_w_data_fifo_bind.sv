// rmc_cif_w_data_fifo_bind.sv
// Attaches the checker to every rmc_cif_w_data_fifo instance.

bind rmc_cif_w_data_fifo rmc_cif_w_data_fifo_fv #(
  .DEPTH (DEPTH),
  .PTR_W (PTR_W),
  .OCC_W (OCC_W)
) u_fv (
  .aclk    (aclk),
  .aresetn (aresetn),
  .s_valid (s_valid),
  .s_ready (s_ready),
  .s_wlast (s_wlast),
  .m_valid (m_valid),
  .m_ready (m_ready),
  .m_wlast (m_wlast),
  .wptr    (wptr),
  .rptr    (rptr),
  .occ     (occ)
);
