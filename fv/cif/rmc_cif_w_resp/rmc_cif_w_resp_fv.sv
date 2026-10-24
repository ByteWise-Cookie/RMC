// rmc_cif_w_resp_fv.sv
// Formal checker for rmc_cif_w_resp (AXI B register slice). Bound by rmc_cif_w_resp_bind.sv.
// Scaffold: the slice's ready formula, AXI-B stability (valid held with stable payload until
// bready), and capture correctness.

import rmc_cfg_pkg::*;

module rmc_cif_w_resp_fv #(
  parameter int AXI_IDW = 8
)(
  input logic               aclk,
  input logic               aresetn,
  input logic               RESP_VALID,
  input logic               RESP_READY,
  input logic [AXI_IDW-1:0] RESP_ID,
  input logic [1:0]         RESP_RESP,
  input logic               bvalid,
  input logic               bready,
  input logic [AXI_IDW-1:0] bid,
  input logic [1:0]         bresp,
  // internals
  input logic               b_val,
  input logic [AXI_IDW-1:0] b_id,
  input logic [1:0]         b_resp
);
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  // slice outputs track the register
  as_ready:  assert property (@(posedge aclk) disable iff (!aresetn)
    RESP_READY == (~b_val | bready));
  as_bvalid: assert property (@(posedge aclk) disable iff (!aresetn) bvalid == b_val);
  as_bid:    assert property (@(posedge aclk) disable iff (!aresetn) bid  == b_id);
  as_bresp:  assert property (@(posedge aclk) disable iff (!aresetn) bresp == b_resp);

  // AXI B: once valid and not accepted, stay valid with stable payload
  as_b_stable: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(bvalid) & ~$past(bready) |->
      bvalid & (bid == $past(bid)) & (bresp == $past(bresp)));

  // a captured beat carries exactly what RESP presented on the accepting cycle
  as_capture: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(RESP_VALID & RESP_READY) |->
      bvalid & (bid == $past(RESP_ID)) & (bresp == $past(RESP_RESP)));

endmodule : rmc_cif_w_resp_fv
