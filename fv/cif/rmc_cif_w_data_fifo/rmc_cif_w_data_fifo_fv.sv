// rmc_cif_w_data_fifo_fv.sv
// Formal checker for rmc_cif_w_data_fifo. Bound by rmc_cif_w_data_fifo_bind.sv. Scaffold:
// occupancy safety + flag correctness + pointer advance. A full in-order data-integrity
// scoreboard (pushed beat emerges unchanged, in order) is left as an extension.

import rmc_cfg_pkg::*;

module rmc_cif_w_data_fifo_fv #(
  parameter int DEPTH = 8,
  // DUT-derived widths (passed by the bind)
  parameter int PTR_W = 3,
  parameter int OCC_W = 4
)(
  input logic               aclk,
  input logic               aresetn,
  input logic               s_valid,
  input logic               s_ready,
  input logic               s_wlast,
  input logic               m_valid,
  input logic               m_ready,
  input logic               m_wlast,
  // internals
  input logic [PTR_W-1:0]   wptr,
  input logic [PTR_W-1:0]   rptr,
  input logic [OCC_W-1:0]   occ
);
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  wire do_push = s_valid & s_ready;
  wire do_pop  = m_valid & m_ready;

  as_occ_range: assert property (@(posedge aclk) disable iff (!aresetn)
    occ <= OCC_W'(DEPTH));
  as_sready:    assert property (@(posedge aclk) disable iff (!aresetn)
    s_ready == (occ != OCC_W'(DEPTH)));
  as_mvalid:    assert property (@(posedge aclk) disable iff (!aresetn)
    m_valid == (occ != '0));
  as_wptr: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(do_push) |->
      wptr == (($past(wptr) == PTR_W'(DEPTH-1)) ? '0 : $past(wptr) + 1'b1));
  as_rptr: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(do_pop) |->
      rptr == (($past(rptr) == PTR_W'(DEPTH-1)) ? '0 : $past(rptr) + 1'b1));

endmodule : rmc_cif_w_data_fifo_fv
