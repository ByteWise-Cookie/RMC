// rmc_cif_w_burst_spltr_bind.sv
// Attaches the checker to every rmc_cif_w_burst_spltr instance.

bind rmc_cif_w_burst_spltr rmc_cif_w_burst_spltr_fv #(
  .ROB_DEPTH (ROB_DEPTH),
  .MAX_PKTS  (MAX_PKTS),
  .AXI_AW    (AXI_AW),
  .AXI_IDW   (AXI_IDW),
  .ROB_IDX_W (ROB_IDX_W),
  .CNT_W     (CNT_W),
  .LINE_W    (LINE_W)
) u_fv (
  .aclk                 (aclk),
  .aresetn              (aresetn),
  .aw_request_valid     (aw_request_valid),
  .burst_splitter_ready (burst_splitter_ready),
  .aw_id                (aw_id),
  .error                (error),
  .rob_idx_in           (rob_idx_in),
  .rob_free             (rob_free),
  .entry_valid          (entry_valid),
  .entry_id             (entry_id),
  .entry_resp           (entry_resp),
  .stage_en             (stage_en),
  .skip                 (skip),
  .stage_len            (stage_len),
  .stage_size           (stage_size),
  .drain_done           (drain_done),
  .ld_valid             (ld_valid),
  .ld_ready             (ld_ready),
  .ld_rob_idx           (ld_rob_idx),
  // internals
  .run                  (run),
  .drop_q               (drop_q),
  .s_addr               (s_addr),
  .s_size               (s_size),
  .s_id                 (s_id),
  .s_err                (s_err),
  .s_rob                (s_rob),
  .n_left               (n_left)
);
