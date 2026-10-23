// rmc_cif_aw_reqbuf_bind.sv
// Attaches the checker to every rmc_cif_aw_reqbuf instance.

bind rmc_cif_aw_reqbuf rmc_cif_aw_reqbuf_fv #(
  .DEPTH (DEPTH),
  .PTR_W (PTR_W),
  .OCC_W (OCC_W)
) u_fv (
  .aclk    (aclk),
  .aresetn (aresetn),
  .push    (push),
  .full    (full),
  .valid   (valid),
  .pop     (pop),
  .wptr    (wptr),
  .rptr    (rptr),
  .occ     (occ)
);
