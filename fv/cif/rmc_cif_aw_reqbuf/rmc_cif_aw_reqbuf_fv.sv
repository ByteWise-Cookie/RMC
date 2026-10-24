// rmc_cif_aw_reqbuf_fv.sv
// Formal checker for rmc_cif_aw_reqbuf. Bound by rmc_cif_aw_reqbuf_bind.sv. Scaffold:
// occupancy safety + flag correctness + pointer advance. In-order descriptor integrity is
// left as an extension.

import rmc_cfg_pkg::*;

module rmc_cif_aw_reqbuf_fv #(
  parameter int DEPTH = 32,
  // DUT-derived widths (passed by the bind)
  parameter int PTR_W = 5,
  parameter int OCC_W = 6
)(
  input logic             aclk,
  input logic             aresetn,
  input logic             push,
  input logic             full,
  input logic             valid,
  input logic             pop,
  // internals
  input logic [PTR_W-1:0] wptr,
  input logic [PTR_W-1:0] rptr,
  input logic [OCC_W-1:0] occ
);
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  wire do_push = push & ~full;
  wire do_pop  = pop  & valid;

  as_occ_range: assert property (@(posedge aclk) disable iff (!aresetn)
    occ <= OCC_W'(DEPTH));
  as_full:  assert property (@(posedge aclk) disable iff (!aresetn)
    full == (occ == OCC_W'(DEPTH)));
  as_valid: assert property (@(posedge aclk) disable iff (!aresetn)
    valid == (occ != '0));
  as_wptr: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(do_push) |->
      wptr == (($past(wptr) == PTR_W'(DEPTH-1)) ? '0 : $past(wptr) + 1'b1));
  as_rptr: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(do_pop) |->
      rptr == (($past(rptr) == PTR_W'(DEPTH-1)) ? '0 : $past(rptr) + 1'b1));

endmodule : rmc_cif_aw_reqbuf_fv
