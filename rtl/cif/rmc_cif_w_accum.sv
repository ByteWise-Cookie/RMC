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
// HOW IT WORKS
// One 64 B line is assembled at a time in a line buffer {data[64B], strobe[64b]}. Each W beat
// is merged in at its in-line byte offset, and the MASK is generated purely from the
// accumulated wstrb over the 64 B packet - NOT from aw_size. aw_size only paces the address
// (how many beats and where each byte lands); wstrb says which bytes are valid, so the write
// is a byte-masked DRAM write (no read-modify-write).
//
//   off  = (aw_addr + beat*2^size)[5:0]            byte position in the 64 B line
//   for each byte b with wstrb[b]: data[off..][b] <= wdata[b]
//   strobe |= wstrb (at those byte positions)
// AXI_DW is 64 B, so each beat's wstrb is already line-wide and this is a straight merge/OR.
// When the line is done (the byte position wraps past 64 B, or wlast) the buffer is written to
// WD-SRAM at the frptr's write_in_ptr[ch] (ch asserted by the splitter) and `filled` pulses.
//
// EXAMPLES
//  a) full ALIGNED 64 B write (aw_size = 6 AND start offset == 0): one beat per line,
//        wstrb = {64{1}} -> strobe = {64{1}}, whole line written, commit on wlast. Every
//        line of such a burst is fully written.
//  b) narrow burst, 4 x 1 B (aw_size = 0), starting at in-line offset 4 (several beats pack
//        into ONE line):
//        beat0 off=4 -> data[4]=B0 ; beat1 off=5 -> data[5]=B1 ;
//        beat2 off=6 -> data[6]=B2 ; beat3 off=7 -> data[7]=B3 (+ wlast)
//        => one packet: strobe = ...0000_1111_0000 (bytes 4..7 set) ; data = ..B0 B1 B2 B3..
//           only those 4 DRAM bytes are written; the rest masked off.
//  c) full-size UNALIGNED burst (aw_size = 6, start offset != 0, one beat per line, N lines):
//        FIRST line  : masked at the FRONT  - wstrb sets [off..63], bytes 0..off-1 masked.
//        MIDDLE lines : FULL - wstrb = {64{1}} (must be all 1s; anything less is a bug).
//        LAST line   : masked at the BACK   - wstrb sets [0..end], bytes end+1..63 masked.
//        The accumulator just stores each beat's own wstrb, so this pattern falls out of the
//        master's strobes; the accumulator never pulls a middle line's mask down.
//  d) a long NARROW burst spanning several lines: the byte position wraps every 64 B, so each
//     line commits on its own wrap; the final (partial) line commits on wlast with its mask.
//
// STUB: ports fixed; the beat merge / line commit / mask generation are TODO (this comment).

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
