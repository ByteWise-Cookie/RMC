// rmc_cif_frptr_fv.sv
// Formal checker for rmc_cif_frptr (per-channel SRAM slot ring). Bound by
// rmc_cif_frptr_bind.sv. Scaffold: core ring-safety contract, to be extended.
//
// Neighbour contract (assumptions): the issuer only allocs when there is room, and a retire
// never frees more slots than are currently in use (both hold in the real design - pkt_gen
// gates alloc on sram_ready, and w_rob frees exactly the run it earlier allocated).
// Properties (assertions): occupancy stays in range, sram_ready tracks "not full", and the
// write pointer advances by one (mod N_SLOTS) exactly on an accepted alloc.

import rmc_cfg_pkg::*;

module rmc_cif_frptr_fv #(
  parameter int N_SLOTS = 64,
  // DUT-derived widths (passed by the bind)
  parameter int CNT_W   = 7,
  parameter int SLOT_W  = 6,
  parameter int OCC_W   = 7
)(
  input logic               aclk,
  input logic               aresetn,
  // DUT ports
  input logic [SLOT_W-1:0]  write_in_ptr,
  input logic               sram_ready,
  input logic               alloc,
  input logic               free_valid,
  input logic [CNT_W-1:0]   free_burst,
  // DUT internals
  input logic [SLOT_W-1:0]  wptr,
  input logic [OCC_W-1:0]   occ
);

  // only check steady state (past reset)
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  // ---- assumptions: legal neighbour behaviour ----
  // no alloc when the ring is full (issuer gates on sram_ready)
  am_no_alloc_when_full: assume property (@(posedge aclk) disable iff (!aresetn)
    !(alloc & (occ == OCC_W'(N_SLOTS))));
  // a free never releases more than is in use (net of a same-cycle alloc)
  am_free_le_occ: assume property (@(posedge aclk) disable iff (!aresetn)
    !free_valid | (free_burst <= (occ + OCC_W'(alloc))));

  // ---- assertions: ring safety ----
  as_occ_range:     assert property (@(posedge aclk) disable iff (!aresetn)
    occ <= OCC_W'(N_SLOTS));
  as_ready_is_room: assert property (@(posedge aclk) disable iff (!aresetn)
    sram_ready == (occ != OCC_W'(N_SLOTS)));
  as_ptr_is_occptr: assert property (@(posedge aclk) disable iff (!aresetn)
    write_in_ptr == wptr);
  // wptr advances by one (mod N_SLOTS) exactly on an alloc
  as_wptr_adv: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(alloc) |->
      wptr == (($past(wptr) == SLOT_W'(N_SLOTS-1)) ? '0 : $past(wptr) + 1'b1));
  as_wptr_hold: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & ~$past(alloc) |-> wptr == $past(wptr));

endmodule : rmc_cif_frptr_fv
