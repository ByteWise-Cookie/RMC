// rmc_cif_aw_reqbuf.sv
// AW request buffer. A small in-order FIFO of write request descriptors that sits between the
// request validator and the write burst splitter, which is its SOLE consumer. There is no
// pre-admit: the ROB index is NOT carried here - the splitter grabs a ROB slot itself when it
// captures a request. The buffer holds each validated request's {id, addr, len, size, error}
// and hands them out in order, one at a time, decoupling the AW channel from the split so up
// to DEPTH requests can wait.
//
// Push on a validated AW request (id/addr/len/size from the AW channel, error[1:0] = the
// validator's BRESP verdict: 00 ok / 10 SLVERR / 11 DECERR). Pop when the splitter captures
// the request to walk it. full gates the AW accept so the buffer never overflows (size it
// >= the ROB depth so it never fills before the ROB does).

import rmc_cfg_pkg::*;

module rmc_cif_aw_reqbuf #(
  parameter int DEPTH   = 32,                            // outstanding write descriptors
  parameter int AXI_AW  = SYS_ADDR_W,
  parameter int AXI_IDW = 8,

  localparam int PTR_W = (DEPTH > 1) ? $clog2(DEPTH)     : 1,
  localparam int OCC_W = $clog2(DEPTH + 1)
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // push: a validated AW request (id/addr/len/size from AW, error from the validator)
  input  logic                   push,
  input  logic [AXI_IDW-1:0]     in_id,
  input  logic [AXI_AW-1:0]      in_addr,
  input  logic [7:0]             in_len,
  input  logic [2:0]             in_size,
  input  logic [1:0]             in_err,            // validator BRESP (00 ok / 10 SLVERR / 11 DECERR)
  output logic                   full,              // gates the AW accept

  // pop: the splitter takes the oldest request to walk it
  output logic                   valid,             // a request is waiting
  input  logic                   pop,
  output logic [AXI_IDW-1:0]     out_id,
  output logic [AXI_AW-1:0]      out_addr,
  output logic [7:0]             out_len,
  output logic [2:0]             out_size,
  output logic [1:0]             out_err
);

  // packed descriptor store
  localparam int DESC_W = AXI_IDW + AXI_AW + 8 + 3 + 2;
  logic [DESC_W-1:0] mem [DEPTH];

  logic [PTR_W-1:0] wptr, rptr;
  logic [OCC_W-1:0] occ;

  assign full  = (occ == OCC_W'(DEPTH));
  assign valid = (occ != '0);
  assign {out_id, out_addr, out_len, out_size, out_err} = mem[rptr];

  wire do_push = push & ~full;
  wire do_pop  = pop  & valid;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      wptr <= '0;
      rptr <= '0;
      occ  <= '0;
    end else begin
      if (do_push) wptr <= (wptr == PTR_W'(DEPTH-1)) ? '0 : wptr + 1'b1;
      if (do_pop)  rptr <= (rptr == PTR_W'(DEPTH-1)) ? '0 : rptr + 1'b1;
      occ <= occ + OCC_W'(do_push) - OCC_W'(do_pop);
    end
  end

  always_ff @(posedge aclk)
    if (do_push) mem[wptr] <= {in_id, in_addr, in_len, in_size, in_err};

endmodule : rmc_cif_aw_reqbuf
