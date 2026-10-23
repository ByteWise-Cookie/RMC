// rmc_cif_w_resp.sv
// WRITE response assembler (AXI B). Drains w_rob's retire port into the AXI B channel, one beat
// per retired request. That is ALL it does: it RESPONDS only. It does NOT free the WD-SRAM -
// the w_rob frees each channel's slot run at retire itself (RESP_NPKTS_CH -> frptr). This is the
// contrast with the read path, where r_resp relieves the RD-SRAM slots as it reads the data out;
// a write response carries no data, so there is nothing to read and nothing to free here.
//
// One-deep output register slice: decouples the AXI B timing from the ROB retire path. w_rob
// holds RESP_VALID until RESP_READY, so retire_fire = RESP_VALID & RESP_READY lands exactly when
// this slice captures the beat.

import rmc_cfg_pkg::*;

module rmc_cif_w_resp #(
  parameter int AXI_IDW = 8                     // AXI id width            [root knob]
)(
  input  logic                 aclk,
  input  logic                 aresetn,

  // from w_rob retire
  input  logic                 RESP_VALID,
  output logic                 RESP_READY,
  input  logic [AXI_IDW-1:0]   RESP_ID,
  input  logic [1:0]           RESP_RESP,        // 00 OKAY / 10 SLVERR / 11 DECERR

  // AXI B channel (to master)
  output logic                 bvalid,
  input  logic                 bready,
  output logic [AXI_IDW-1:0]   bid,
  output logic [1:0]           bresp
);

  logic                 b_val;
  logic [AXI_IDW-1:0]   b_id;
  logic [1:0]           b_resp;

  // accept a retired response when the slice is empty or draining this cycle
  assign RESP_READY = ~b_val | bready;

  assign bvalid = b_val;
  assign bid    = b_id;
  assign bresp  = b_resp;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      b_val <= 1'b0;
    end else if (RESP_VALID & RESP_READY) begin   // capture a retired beat
      b_val  <= 1'b1;
      b_id   <= RESP_ID;
      b_resp <= RESP_RESP;
    end else if (bready) begin                     // drained with nothing new
      b_val  <= 1'b0;
    end
  end

endmodule : rmc_cif_w_resp
