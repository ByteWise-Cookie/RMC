// rmc_cif_addrmap.sv
// System byte address -> DRAM coords {rank, bg, bank, row, col}, plus ch (channel
// select, routes to the per-channel core) and offset (byte-in-64B-packet, for the
// write mask / narrow-access byte lanes).
//
// Runtime-programmable (CSR-driven), bit level. The map is a per-DESTINATION-bit
// source select: each decoded output bit is a mux that picks one sys_addr bit, so
// the decode is a pure bit permutation of the input - no popcount, no priority net.
// (Chosen over a per-source FIELD_ID scheme: placing a bit inside its field there
// needs a running population count per field. Per-dest muxing avoids that.)
//
// The live map arrives on addr_map (DEC_W selects, each MAP_SEL_W wide) from the
// CSR; its reset value = rmc_cfg_pkg::default_addr_map(), which reproduces the
// fixed STAGE-24 interleave:
//   [ row | col_hi | bank | bg_hi | col_lo(P) | bg_lo | rank | ch | offset ]
//    MSB                                                              LSB
// Destination bus layout (LSB->MSB): offset | ch | daddr, with
// daddr = {rank,bg,bank,row,col} (DRAM coords only); ch is routing, offset a byte
// lane - neither is a DRAM coordinate.

module rmc_cif_addrmap import rmc_cfg_pkg::*; #(
  parameter int AXI_AW  = SYS_ADDR_W,          // sys_addr width (must match SYS_ADDR_W)
  parameter int DADDR_W = rmc_cfg_pkg::DADDR_W  // DRAM-coord width (top may re-pass)
)(
  input  logic [AXI_AW-1:0]                sys_addr,
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0]  addr_map,  // per-dest source select (CSR live map)
  output logic [DADDR_W-1:0]               daddr,
  output logic [CH_W-1:0]                  ch,        // channel select (-> per-channel core)
  output logic [PKT_OFF_W-1:0]             offset     // byte-in-64B-packet (write mask)
);

  // every decoded bit = sys_addr[ its programmed source select ]
  // selects in default_addr_map() reach at most ROW_POS+ROW_W-1 (< AXI_AW); a CSR
  // map must keep each select < AXI_AW or it reads an out-of-range sys_addr bit.
  logic [DEC_W-1:0] dec;
  always_comb
    for (int b = 0; b < DEC_W; b++)
      dec[b] = sys_addr[ addr_map[b] ];

  // slice the decoded bus: offset | ch | daddr   (LSB->MSB)
  assign offset = dec[PKT_OFF_W-1:0];
  assign ch     = dec[PKT_OFF_W            +: CH_W];
  assign daddr  = dec[PKT_OFF_W + CH_W     +: DADDR_W];

endmodule : rmc_cif_addrmap
