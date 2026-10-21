// rmc_cif_aw_reqbuf.sv
// AW request buffer. A small in-order FIFO of write request descriptors that sits between
// admit and the write burst splitter. The write ROB keeps only completion/B-response state
// and does NOT store the address, so this buffer holds each admitted request's
// {addr, len, size, rob_idx} and hands them to the splitter in admit order, one at a time.
// Decouples admit from the split so up to DEPTH requests can be admitted and waiting.
//
// Push on admit (a write request accepted into the ROB, carrying the ROB index it got).
// Pop when the splitter captures the request to walk it. full gates admit so the buffer
// never overflows (size it >= the ROB depth and it never fills before the ROB does).

import rmc_cfg_pkg::*;

module rmc_cif_aw_reqbuf #(
  parameter int DEPTH     = 32,                          // outstanding write descriptors
  parameter int AXI_AW    = SYS_ADDR_W,
  parameter int ROB_IDX_W = 5,

  localparam int PTR_W = (DEPTH > 1) ? $clog2(DEPTH)     : 1,
  localparam int OCC_W = $clog2(DEPTH + 1)
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // push: a write request was admitted (addr/len/size from AW+validator, rob_idx from w_rob)
  input  logic                   push,
  input  logic [AXI_AW-1:0]      in_addr,
  input  logic [7:0]             in_len,
  input  logic [2:0]             in_size,
  input  logic [ROB_IDX_W-1:0]   in_rob_idx,
  output logic                   full,              // gates admit

  // pop: the splitter takes the oldest request to walk it
  output logic                   valid,             // a request is waiting
  input  logic                   pop,
  output logic [AXI_AW-1:0]      out_addr,
  output logic [7:0]             out_len,
  output logic [2:0]             out_size,
  output logic [ROB_IDX_W-1:0]   out_rob_idx
);

  // packed descriptor store
  localparam int DESC_W = AXI_AW + 8 + 3 + ROB_IDX_W;
  logic [DESC_W-1:0] mem [DEPTH];

  logic [PTR_W-1:0] wptr, rptr;
  logic [OCC_W-1:0] occ;

  assign full  = (occ == OCC_W'(DEPTH));
  assign valid = (occ != '0);
  assign {out_addr, out_len, out_size, out_rob_idx} = mem[rptr];

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
    if (do_push) mem[wptr] <= {in_addr, in_len, in_size, in_rob_idx};

endmodule : rmc_cif_aw_reqbuf
