// rmc_cif_wrob_fv.sv
// Formal checker for rmc_cif_wrob. Bound by rmc_cif_wrob_bind.sv. Scaffold: ring safety, the
// retire/response formulas, the B-handshake hold, and occupancy accounting. A completion-count
// reference model (exactly n_pkts completions retires in order with the right BRESP) is left
// as an extension and would add the neighbour assumption "<= n_pkts completions per slot".

import rmc_cfg_pkg::*;

module rmc_cif_wrob_fv #(
  parameter int ROB_DEPTH = 32,
  parameter int MAX_PKTS  = 64,
  parameter int AXI_IDW   = 8,
  parameter int ROB_IDX_W = 5,
  parameter int PTR_W     = 5,
  parameter int CNT_W     = 7,
  parameter int OCC_W     = 6
)(
  input logic                  aclk,
  input logic                  aresetn,
  input logic                  cmpl_valid,
  input logic [ROB_IDX_W-1:0]  cmpl_rob_index,
  input logic                  cmpl_err,
  input logic [ROB_IDX_W-1:0]  rob_idx,
  input logic                  rob_free,
  input logic                  entry_valid,
  input logic [AXI_IDW-1:0]    entry_id,
  input logic [1:0]            entry_resp,
  input logic [CNT_W-1:0]      entry_npkts,
  input logic                  RESP_VALID,
  input logic                  RESP_READY,
  input logic [AXI_IDW-1:0]    RESP_ID,
  input logic [1:0]            RESP_RESP,
  // internals
  input logic [PTR_W-1:0]      alloc_ptr,
  input logic [PTR_W-1:0]      retire_ptr,
  input logic [OCC_W-1:0]      occ,
  input logic [1:0]            ent_resp      [ROB_DEPTH],
  input logic                  ent_derr      [ROB_DEPTH],
  input logic                  inflow_complt [ROB_DEPTH],
  input logic [CNT_W-1:0]      n_pkts        [ROB_DEPTH],
  input logic [CNT_W-1:0]      outflow_cnt   [ROB_DEPTH]
);
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  wire retire_fire = RESP_VALID & RESP_READY;

  // ring safety + offer
  as_occ_range: assert property (@(posedge aclk) disable iff (!aresetn) occ <= OCC_W'(ROB_DEPTH));
  as_rob_free:  assert property (@(posedge aclk) disable iff (!aresetn)
    rob_free == (occ != OCC_W'(ROB_DEPTH)));
  as_rob_idx:   assert property (@(posedge aclk) disable iff (!aresetn) rob_idx == alloc_ptr);

  // retire / response formulas
  as_resp_valid: assert property (@(posedge aclk) disable iff (!aresetn)
    RESP_VALID == ((occ != '0) & inflow_complt[retire_ptr]
                   & (outflow_cnt[retire_ptr] == n_pkts[retire_ptr])));
  as_resp_resp:  assert property (@(posedge aclk) disable iff (!aresetn)
    RESP_RESP == (ent_derr[retire_ptr] ? 2'b10 : ent_resp[retire_ptr]));

  // B handshake: hold valid + payload until accepted
  as_resp_hold: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(RESP_VALID) & ~$past(RESP_READY) |->
      RESP_VALID & (RESP_ID == $past(RESP_ID)) & (RESP_RESP == $past(RESP_RESP)));

  // occupancy accounting: +entry_valid -retire_fire
  as_occ_acct: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) |-> occ == ($past(occ) + OCC_W'($past(entry_valid)) - OCC_W'($past(retire_fire))));

  // pointers advance by one (mod depth) on their event
  as_alloc_adv: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(entry_valid) |->
      alloc_ptr == (($past(alloc_ptr) == PTR_W'(ROB_DEPTH-1)) ? '0 : $past(alloc_ptr) + 1'b1));
  as_retire_adv: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(retire_fire) |->
      retire_ptr == (($past(retire_ptr) == PTR_W'(ROB_DEPTH-1)) ? '0 : $past(retire_ptr) + 1'b1));

endmodule : rmc_cif_wrob_fv
