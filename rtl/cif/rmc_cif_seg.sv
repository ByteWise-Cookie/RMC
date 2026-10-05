// rmc_cif_seg.sv
// Burst segmentation: an AXI burst -> a sequence of <=16-beat packets, one 64B
// line (one SRAM word, one CAS) each. Emits pkt_num 0..N-1 per request.
// Narrow / unaligned / sub-64B handling (WSTRB, DM mask) flagged here for the
// write accumulator (stage 20): narrow = (size<full) | (wstrb!=all-1s) | unaligned.
//
// STUB: ports only. Beat-count -> packet-count and offset gen TODO.

import rmc_cfg_pkg::*;

module rmc_cif_seg #(
  parameter int AXI_AW    = 40,
  parameter int MAX_PKTS  = 16,
  parameter int PKT_NUM_W = 4
)(
  input  logic aclk,
  input  logic aresetn
  // TODO: AW/AR {addr,len,size} in -> {pkt_num, pkt_addr_offset, last_in_txn} out
  // TODO: narrow flag out (per-byte WSTRB placement, split at 64B boundary)
);

  // intentionally empty stub

endmodule : rmc_cif_seg
