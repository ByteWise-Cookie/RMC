// rmc_cif_w_accum_bind.sv
// Attaches the checker to every rmc_cif_w_accum instance.

bind rmc_cif_w_accum rmc_cif_w_accum_fv #(
  .OFF_W  (OFF_W),
  .DW     (DW),
  .STRB_W (STRB_W)
) u_fv (
  .aclk              (aclk),
  .aresetn           (aresetn),
  .wvalid            (wvalid),
  .wready            (wready),
  .wlast             (wlast),
  .stage_en          (stage_en),
  .skip              (skip),
  .take              (take),
  .drain_done        (drain_done),
  .stage_valid       (stage_valid),
  .write_data        (write_data),
  .write_data_strobe (write_data_strobe),
  // internals
  .line_data         (line_data),
  .line_strobe       (line_strobe),
  .in_req            (in_req),
  .drop_q            (drop_q),
  .staged            (staged)
);
