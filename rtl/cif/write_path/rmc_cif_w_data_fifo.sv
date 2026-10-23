// rmc_cif_w_data_fifo.sv
// Write-data FIFO. A small in-order buffer of AXI W beats {wdata, wstrb, wlast} between the
// AXI W channel and the accumulator. It decouples the W stream from the AW/split timing: W
// beats can arrive and bank up while the splitter is still working out which request's data
// this is, and the accumulator drains them at its own pace. Pure data path - no control, no
// reordering (AXI4 W has no WID; beats follow AW order).
//
// Standard valid/ready FIFO: s_* is the AXI W side (s_ready gates wready to the master), m_*
// is the accumulator side. full/empty derive from the occupancy counter.

import rmc_cfg_pkg::*;

module rmc_cif_w_data_fifo #(
  parameter int DEPTH = 8,                              // buffered W beats

  // derived (localparam: never passed in)
  localparam int DW     = rmc_cfg_pkg::AXI_DW,
  localparam int STRB_W = DW / 8,
  localparam int PTR_W  = (DEPTH > 1) ? $clog2(DEPTH) : 1,
  localparam int OCC_W  = $clog2(DEPTH + 1)
)(
  input  logic                aclk,
  input  logic                aresetn,

  // slave side: AXI W channel in
  input  logic                s_valid,
  output logic                s_ready,
  input  logic [DW-1:0]       s_wdata,
  input  logic [STRB_W-1:0]   s_wstrb,
  input  logic                s_wlast,

  // master side: to the accumulator
  output logic                m_valid,
  input  logic                m_ready,
  output logic [DW-1:0]       m_wdata,
  output logic [STRB_W-1:0]   m_wstrb,
  output logic                m_wlast
);

  localparam int ENT_W = DW + STRB_W + 1;              // {wdata, wstrb, wlast}
  logic [ENT_W-1:0] mem [DEPTH];

  logic [PTR_W-1:0] wptr, rptr;
  logic [OCC_W-1:0] occ;

  assign s_ready = (occ != OCC_W'(DEPTH));             // room for a beat
  assign m_valid = (occ != '0);                        // a beat is waiting
  assign {m_wdata, m_wstrb, m_wlast} = mem[rptr];

  wire do_push = s_valid & s_ready;
  wire do_pop  = m_valid & m_ready;

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
    if (do_push) mem[wptr] <= {s_wdata, s_wstrb, s_wlast};

endmodule : rmc_cif_w_data_fifo
