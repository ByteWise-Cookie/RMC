// rmc_cif_w_accum.sv
// Write accumulator. Consumes the AXI W data channel, stages beats into 64 B lines (full
// size = one beat per line; narrow = several beats packed by byte offset, with the strobe
// accumulated), and drives the per-channel WD-SRAM WRITE-ENABLE + DATA + MASK. It does NOT
// drive the write ADDRESS: the frptr owns write_in_ptr and that is wired to wdb_addr at the
// top level, so the SRAM port has one address source (frptr) and one data source (here).
// The burst splitter asserts the line's channel (ch_in + ch_in_valid) so it commits to that
// channel; wready to the master is gated by w_ready[ch] (room) from the frptr blocks.
// Handshake: filled (a line is staged) out <-> ch_in_valid (splitter consumed it) in.
//
// STUB: ports fixed; the W-beat staging / narrow packing and the SRAM write are TODO.

import rmc_cfg_pkg::*;

module rmc_cif_w_accum #(
  parameter int N_WDB = 64,                              // WD-SRAM slots per channel

  // derived (localparam: pkg-sourced so strict front-ends resolve them in the header)
  localparam int N_CH   = rmc_cfg_pkg::N_CH,
  localparam int CH_W   = rmc_cfg_pkg::CH_W,
  localparam int DW     = rmc_cfg_pkg::AXI_DW,           // 512 b (one 64 B line)
  localparam int STRB_W = DW / 8,                        // 64
  localparam int SLOT_W = (N_WDB > 1) ? $clog2(N_WDB) : 1
)(
  input  logic                         aclk,
  input  logic                         aresetn,

  // AXI W data channel (from master)
  input  logic                         wvalid,
  output logic                         wready,
  input  logic [DW-1:0]                wdata,
  input  logic [STRB_W-1:0]            wstrb,
  input  logic                         wlast,

  // burst shape of the request whose W data is arriving (for narrow packing)
  input  logic [7:0]                   axi_len,
  input  logic [2:0]                   axi_size,
  input  logic                         req_valid,        // a request's W stream is active

  // splitter handshake: filled (a 64 B line is staged) out; the splitter asserts the line's
  // channel (ch_in + ch_in_valid) so the accumulator commits it to that channel's WD-SRAM.
  output logic                         filled,
  input  logic                         ch_in_valid,
  input  logic [CH_W-1:0]              ch_in,

  // WD-SRAM write port, per channel (owned here): write-enable + data + byte mask. The
  // address comes from the frptr (write_in_ptr), wired at the top level; w_sram_ready is the
  // per-channel room from the frptr, gating this write and wready to the master.
  input  logic [N_CH-1:0]              w_sram_ready,
  output logic [N_CH-1:0]              wdb_we,
  output logic [N_CH-1:0][DW-1:0]      write_data,
  output logic [N_CH-1:0][STRB_W-1:0]  write_data_strobe
);

  // STUB body: safe defaults so it elaborates. TODO: W-beat staging, narrow packing, and the
  // per-channel SRAM write-enable + data at the frptr's write_in_ptr[ch_in].
  assign wready            = 1'b0;
  assign filled            = 1'b0;
  assign wdb_we            = '0;
  assign write_data        = '0;
  assign write_data_strobe = '0;

endmodule : rmc_cif_w_accum
