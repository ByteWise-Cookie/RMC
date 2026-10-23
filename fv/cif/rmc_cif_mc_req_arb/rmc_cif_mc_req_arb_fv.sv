// rmc_cif_mc_req_arb_fv.sv
// Formal checker for rmc_cif_mc_req_arb. Bound by rmc_cif_mc_req_arb_bind.sv. Scaffold: safe
// serialization - request valid = any source wants to go, at most one source granted, a grant
// only on an accepted cycle, and the driven op/payload matches the granted source. Fairness
// (no source starves) is a liveness property and is left as an extension (needs fair req_ready).

import rmc_cfg_pkg::*;

module rmc_cif_mc_req_arb_fv #(
  parameter int ROB_IDX_W = 5,
  parameter int DADDR_W   = 32,
  parameter int SLOT_W    = 6,
  parameter int PKT_NUM_W = 6,
  parameter int SEL_W     = 2
)(
  input logic                            aclk,
  input logic                            aresetn,
  input logic [N_CH-1:0]                 rd_valid,
  input logic [N_CH-1:0]                 rd_ready,
  input logic [N_CH-1:0][ROB_IDX_W-1:0]  rd_rob_idx,
  input logic [N_CH-1:0][DADDR_W-1:0]    rd_daddr,
  input logic [N_CH-1:0][SLOT_W-1:0]     rd_slot,
  input logic [N_CH-1:0]                 wr_valid,
  input logic [N_CH-1:0]                 wr_ready,
  input logic [N_CH-1:0][ROB_IDX_W-1:0]  wr_rob_idx,
  input logic [N_CH-1:0][DADDR_W-1:0]    wr_daddr,
  input logic [N_CH-1:0][SLOT_W-1:0]     wr_slot,
  input logic                            async_mc_req_valid,
  input logic                            async_mc_req_ready,
  input logic [ROB_IDX_W-1:0]            async_mc_req_rob_index,
  input logic                            async_mc_req_op,
  input logic [DADDR_W-1:0]              async_mc_req_daddr,
  input logic [SLOT_W-1:0]               async_mc_req_sram_slot,
  // internals
  input logic [SEL_W-1:0]                rr_ptr
);
  // request valid iff some source wants to go
  as_req_valid: assert property (@(posedge aclk) disable iff (!aresetn)
    async_mc_req_valid == (|rd_valid | |wr_valid));

  // at most one source granted, and only on an accepted cycle
  as_ready_onehot0: assert property (@(posedge aclk) disable iff (!aresetn)
    $onehot0({wr_ready, rd_ready}));
  as_ready_accept: assert property (@(posedge aclk) disable iff (!aresetn)
    (|{wr_ready, rd_ready}) == (async_mc_req_valid & async_mc_req_ready));

  // per-source: a granted source was valid, and the port carries its direction + payload
  for (genvar c = 0; c < N_CH; c++) begin : g_chk
    as_rd_valid: assert property (@(posedge aclk) disable iff (!aresetn)
      rd_ready[c] |-> rd_valid[c]);
    as_wr_valid: assert property (@(posedge aclk) disable iff (!aresetn)
      wr_ready[c] |-> wr_valid[c]);
    as_rd_payload: assert property (@(posedge aclk) disable iff (!aresetn)
      rd_ready[c] |-> (~async_mc_req_op
                       & (async_mc_req_rob_index == rd_rob_idx[c])
                       & (async_mc_req_daddr     == rd_daddr[c])
                       & (async_mc_req_sram_slot == rd_slot[c])));
    as_wr_payload: assert property (@(posedge aclk) disable iff (!aresetn)
      wr_ready[c] |-> (async_mc_req_op
                       & (async_mc_req_rob_index == wr_rob_idx[c])
                       & (async_mc_req_daddr     == wr_daddr[c])
                       & (async_mc_req_sram_slot == wr_slot[c])));
  end

endmodule : rmc_cif_mc_req_arb_fv
