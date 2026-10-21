// rmc_cif_w_accum.sv
// Write accumulator. Consumes the AXI W data channel, stages beats into 64 B lines (full
// size = one beat per line; narrow = several beats packed by byte offset, with wstrb
// accumulated), and OWNS the per-channel WD-SRAM write port (we/addr/din/wstrb). The slot
// address is write_in_ptr[ch] from the per-channel frptr block (the accumulator does not
// own the pointer); the burst splitter asserts ch for the staged line and uses the same
// write_in_ptr[ch] as the packet slot. wready to the master is gated by sram_ready[ch]
// (space) from the frptr blocks. Handshake with the splitter: filled (a line is staged)
// <-> take (the splitter consumed it).
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

  // per-channel pointer + space from the frptr blocks (fanned in)
  input  logic [N_CH-1:0][SLOT_W-1:0]  write_in_ptr,
  input  logic [N_CH-1:0]              sram_ready,

  // splitter handshake: filled (a 64 B line is staged) <-> take; ch_in = the channel the
  // splitter asserts for that line (selects which WD-SRAM to commit to)
  output logic                         filled,
  input  logic                         take,
  input  logic [CH_W-1:0]              ch_in,

  // WD-SRAM write port, per channel (owned here): data + the accumulated byte mask
  output logic [N_CH-1:0]              wdb_we,
  output logic [N_CH-1:0][SLOT_W-1:0]  wdb_addr,
  output logic [N_CH-1:0][DW-1:0]      wdb_din,
  output logic [N_CH-1:0][STRB_W-1:0]  wdb_wstrb
);

  // STUB body: safe defaults so it elaborates. TODO: W-beat staging, narrow packing,
  // and the per-channel SRAM write at write_in_ptr[ch_in].
  assign wready    = 1'b0;
  assign filled    = 1'b0;
  assign wdb_we    = '0;
  assign wdb_addr  = '0;
  assign wdb_din   = '0;
  assign wdb_wstrb = '0;

endmodule : rmc_cif_w_accum
