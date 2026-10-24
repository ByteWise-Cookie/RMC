// rmc_cif_w_pkt_gen_fv.sv
// Formal checker for rmc_cif_w_pkt_gen. Bound by rmc_cif_w_pkt_gen_bind.sv. Scaffold: the
// two-stage commit/issue contract - commit gating, one-hot alloc/wr_en, the captured packet's
// payload + channel-gated valid, slot sampled at commit (one cycle before the pointer moves),
// and no orphan (a full MC FIFO holds the packet rather than dropping it).

import rmc_cfg_pkg::*;

module rmc_cif_w_pkt_gen_fv #(
  parameter int ROB_IDX_W = 5,
  parameter int SLOT_W    = 6
)(
  input logic                         aclk,
  input logic                         aresetn,
  input logic                         ld_valid,
  input logic                         ld_ready,
  input logic [CH_W-1:0]              ld_ch,
  input logic [DADDR_W-1:0]           ld_daddr,
  input logic [ROB_IDX_W-1:0]         ld_rob_idx,
  input logic                         stage_valid,
  input logic                         take,
  input logic [N_CH-1:0][SLOT_W-1:0]  write_in_ptr,
  input logic [N_CH-1:0]              sram_ready,
  input logic [N_CH-1:0]              alloc,
  input logic [N_CH-1:0]              wr_en,
  input logic [N_CH-1:0]              req_fifo_ready,
  input logic [N_CH-1:0]              packet_valid,
  input logic [ROB_IDX_W-1:0]         packet_rob_index,
  input logic [DADDR_W-1:0]           packet_daddr,
  input logic [SLOT_W-1:0]            packet_slot,
  // internals
  input logic                         cap_valid,
  input logic [CH_W-1:0]              cap_ch,
  input logic [ROB_IDX_W-1:0]         cap_rob,
  input logic [DADDR_W-1:0]           cap_daddr,
  input logic [SLOT_W-1:0]            cap_slot
);
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  wire issue_fire = cap_valid & req_fifo_ready[cap_ch];
  wire cap_free   = ~cap_valid | issue_fire;
  wire commit     = ld_valid & stage_valid & sram_ready[ld_ch] & cap_free;

  // stage-1 strobes
  as_ld_ready: assert property (@(posedge aclk) disable iff (!aresetn) ld_ready == commit);
  as_take:     assert property (@(posedge aclk) disable iff (!aresetn) take == (commit));
  as_alloc:    assert property (@(posedge aclk) disable iff (!aresetn)
    alloc == (commit ? (N_CH'(1) << ld_ch) : '0));
  as_wren_eq_alloc: assert property (@(posedge aclk) disable iff (!aresetn) wr_en == alloc);

  // stage-2 outputs: channel-gated valid + the captured payload
  as_pkt_valid: assert property (@(posedge aclk) disable iff (!aresetn)
    packet_valid == (cap_valid ? (N_CH'(1) << cap_ch) : '0));
  as_pkt_payload: assert property (@(posedge aclk) disable iff (!aresetn)
    (packet_rob_index == cap_rob) & (packet_daddr == cap_daddr) & (packet_slot == cap_slot));

  // capture loads on commit with the line descriptor, slot sampled this cycle (pre-advance)
  as_cap_load: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(commit) |->
      cap_valid & (cap_ch == $past(ld_ch)) & (cap_rob == $past(ld_rob_idx))
      & (cap_daddr == $past(ld_daddr)) & (cap_slot == $past(write_in_ptr[ld_ch])));

  // drain-only cycle (no new commit) clears the capture; a full FIFO just holds it (no orphan)
  as_cap_drain: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(~commit & issue_fire) |-> ~cap_valid);
  as_cap_hold:  assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(cap_valid & ~issue_fire & ~commit) |-> cap_valid);

endmodule : rmc_cif_w_pkt_gen_fv
