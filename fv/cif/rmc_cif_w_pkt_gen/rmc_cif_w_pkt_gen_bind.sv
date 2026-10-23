// rmc_cif_w_pkt_gen_bind.sv
// Attaches the checker to every rmc_cif_w_pkt_gen instance.

bind rmc_cif_w_pkt_gen rmc_cif_w_pkt_gen_fv #(
  .ROB_IDX_W (ROB_IDX_W),
  .SLOT_W    (SLOT_W)
) u_fv (
  .aclk             (aclk),
  .aresetn          (aresetn),
  .ld_valid         (ld_valid),
  .ld_ready         (ld_ready),
  .ld_ch            (ld_ch),
  .ld_daddr         (ld_daddr),
  .ld_rob_idx       (ld_rob_idx),
  .stage_valid      (stage_valid),
  .take             (take),
  .write_in_ptr     (write_in_ptr),
  .sram_ready       (sram_ready),
  .alloc            (alloc),
  .wr_en            (wr_en),
  .req_fifo_ready   (req_fifo_ready),
  .packet_valid     (packet_valid),
  .packet_rob_index (packet_rob_index),
  .packet_daddr     (packet_daddr),
  .packet_slot      (packet_slot),
  // internals
  .cap_valid        (cap_valid),
  .cap_ch           (cap_ch),
  .cap_rob          (cap_rob),
  .cap_daddr        (cap_daddr),
  .cap_slot         (cap_slot)
);
