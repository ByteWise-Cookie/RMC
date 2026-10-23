// rmc_cif_w_resp_bind.sv
// Attaches the checker to every rmc_cif_w_resp instance.

bind rmc_cif_w_resp rmc_cif_w_resp_fv #(
  .AXI_IDW (AXI_IDW)
) u_fv (
  .aclk       (aclk),
  .aresetn    (aresetn),
  .RESP_VALID (RESP_VALID),
  .RESP_READY (RESP_READY),
  .RESP_ID    (RESP_ID),
  .RESP_RESP  (RESP_RESP),
  .bvalid     (bvalid),
  .bready     (bready),
  .bid        (bid),
  .bresp      (bresp),
  .b_val      (b_val),
  .b_id       (b_id),
  .b_resp     (b_resp)
);
