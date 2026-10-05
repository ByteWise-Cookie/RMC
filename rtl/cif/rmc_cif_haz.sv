// rmc_cif_haz.sv
// Same-line R/W hazard router. CIF owns RAW + WAR ordering (stage 14): MC is a
// pure reorderable engine and does NO overlap detection.
//   RAW: a new read overlaps an OLDER write  -> hold the read until the write done.
//   WAR: a new write overlaps an OLDER read  -> hold the write until the read done.
//   WAW: cannot occur - AXI requires write data in order, so same-line writes keep
//        program order through the single write path. Not handled.
//
// There are TWO ROBs (r_rob, w_rob). Each owns an address TCAM (store + overlap
// compare, exact via the AXI 4KB rule). The cross-search:
//   new read  probes w_rob's TCAM -> raw_hit
//   new write probes r_rob's TCAM -> war_hit
// haz is thin: it only turns a hit into a stall bit on the REQUESTER's own entry.
//
// NOTE on the ptr: the stall bit is indexed by the NEW requester's slot
// (raw_rd_ptr = the new read's r_rob slot; war_wr_ptr = the new write's w_rob
// slot), NOT the matched older entry's ptr - the requester is the one that holds.
// The matched ptr (if tracked) is only for release bookkeeping.
//
// STUB: sets the stall bit on hit. Release (clear when the older entry frees) is
// TODO - either re-probe continuously, or latch + clear on that ptr's completion.

import rmc_cfg_pkg::*;

module rmc_cif_haz #(
  parameter int ROB_DEPTH = 32,
  localparam int PTR_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH) : 1
)(
  // RAW: new read vs w_rob TCAM
  input  logic                 raw_hit,
  input  logic [PTR_W-1:0]     raw_rd_ptr,    // new read's r_rob slot to hold
  // WAR: new write vs r_rob TCAM
  input  logic                 war_hit,
  input  logic [PTR_W-1:0]     war_wr_ptr,    // new write's w_rob slot to hold

  output logic [ROB_DEPTH-1:0] r_stall_vector, // -> r_rob: hold these reads
  output logic [ROB_DEPTH-1:0] w_stall_vector  // -> w_rob: hold these writes
);

  always_comb begin
    r_stall_vector = '0;
    w_stall_vector = '0;
    if (raw_hit) r_stall_vector[raw_rd_ptr] = 1'b1;  // hold the new read
    if (war_hit) w_stall_vector[war_wr_ptr] = 1'b1;  // hold the new write
  end

endmodule : rmc_cif_haz
