// rmc_cif_w_accum_fv.sv
// Formal checker for rmc_cif_w_accum. Bound by rmc_cif_w_accum_bind.sv. Scaffold: staging
// handshake safety (one line held until taken), the wready gate, output wiring, and that
// drain_done only fires while dropping. A full mask-accumulation reference model (bytes land
// by wstrb, mask = OR of wstrb over the line) is left as an extension.

import rmc_cfg_pkg::*;

module rmc_cif_w_accum_fv #(
  parameter int OFF_W  = 6,
  parameter int DW     = 512,
  parameter int STRB_W = 64
)(
  input logic               aclk,
  input logic               aresetn,
  input logic               wvalid,
  input logic               wready,
  input logic               wlast,
  input logic               stage_en,
  input logic               skip,
  input logic               take,
  input logic               drain_done,
  input logic               stage_valid,
  input logic [DW-1:0]      write_data,
  input logic [STRB_W-1:0]  write_data_strobe,
  // internals
  input logic [DW-1:0]      line_data,
  input logic [STRB_W-1:0]  line_strobe,
  input logic               in_req,
  input logic               drop_q,
  input logic               staged
);
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  wire drop_cur = in_req ? drop_q : skip;

  // output wiring
  as_stage_valid: assert property (@(posedge aclk) disable iff (!aresetn) stage_valid == staged);
  as_wdata:  assert property (@(posedge aclk) disable iff (!aresetn) write_data == line_data);
  as_wstrb:  assert property (@(posedge aclk) disable iff (!aresetn) write_data_strobe == line_strobe);

  // wready gate: no accept outside an active request; a staged (non-drop) line blocks accept
  as_wready: assert property (@(posedge aclk) disable iff (!aresetn)
    wready == (stage_en & (drop_cur | ~staged)));

  // a staged line is held until pkt_gen takes it (never silently dropped)
  as_staged_hold: assert property (@(posedge aclk) disable iff (!aresetn)
    en & $past(en) & $past(staged) & ~$past(take) |-> staged);

  // drain_done only while draining a dropped burst, on its last beat
  as_drain_in_drop: assert property (@(posedge aclk) disable iff (!aresetn)
    drain_done |-> (drop_cur & wvalid & wready & wlast));

  // never stage a line for a dropped request
  as_no_stage_on_drop: assert property (@(posedge aclk) disable iff (!aresetn)
    (in_req & drop_q) |-> ~staged);

endmodule : rmc_cif_w_accum_fv
