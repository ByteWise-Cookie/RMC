// rmc_cif_frptr_bind.sv
// Attaches the checker to every rmc_cif_frptr instance. Right-hand names resolve inside the
// DUT, so the ring pointer and occupancy are reachable without touching the RTL.

bind rmc_cif_frptr rmc_cif_frptr_fv #(
  .N_SLOTS (N_SLOTS),
  .CNT_W   (CNT_W),
  .SLOT_W  (SLOT_W),
  .OCC_W   (OCC_W)
) u_fv (
  .aclk         (aclk),
  .aresetn      (aresetn),
  .write_in_ptr (write_in_ptr),
  .sram_ready   (sram_ready),
  .alloc        (alloc),
  .free_valid   (free_valid),
  .free_burst   (free_burst),
  // internals
  .wptr         (wptr),
  .occ          (occ)
);
