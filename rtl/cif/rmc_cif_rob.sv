// rmc_cif_rob.sv
// Reorder buffer, ONE DIRECTION per instance (r_rob and w_rob are two instances;
// DIR picks the data-side behaviour). Allocates rob_index per admitted request,
// allocates the data SRAM slot (dbuf_addr for reads, wd_slot for writes), tracks
// per-packet completion, and retires in program order to drive B / R.
// MC completions are a pulse {rob_index}; the ROB just counts them to retire.
//
// Order is held by a shift register of POINTERS, not of the entries themselves.
// Every admitted request is index-mapped: one slot index addresses all of its
// backing stores, so only the narrow pointer moves down the shift reg while the
// wide metadata stays put. Retire pops the head pointer. Split the metadata by
// access pattern instead of one fat flop array:
//
//   inflow/      -> counter memory: a completion pulse ++ for the entry; compare
//   outflow         to N (=from aw_len/ar_len). done when the count hits N.
//   cold metadata-> SRAM: aw_id/ar_id, size, len, resp - read only at retire.
//
// All stores share depth = ROB_DEPTH so the slot index lines up across them.
// Address hazard search + request ok/err tagging live in a pre-ROB
// request_validator block, not here; hazards themselves are handled in MC.
//
// STUB: params + completion sink + stores sketched. Logic TODO.

import rmc_cfg_pkg::*;

module rmc_cif_rob #(
  parameter int DIR       = 0,    // 0 = read ROB, 1 = write ROB
  parameter int ROB_DEPTH = 32,   // physical entries (16-32)   [root knob]
  parameter int MAX_PKTS  = 16,   // max packets per request    [root knob]
  parameter int AXI_AW    = 40,

  // derived widths (localparam: never passed in - computed from the knobs above)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1, // rob_index tag
  localparam int PTR_W     = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int CNT_W     = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // completion in (from MC compl async FIFO). No ready: completion never
  // back-pressures (tied high at the CIF top); a completion only bumps a counter
  // on an already-allocated entry, always absorbable in one cycle.
  // completion = one PULSE (cmpl_valid) paired with rob_index. The ROB tracks NO
  // packet number or status: the pulse just bumps this entry's done-counter
  // (retire at N). Per-packet num/status dropped.
  input  logic                   cmpl_valid,
  input  logic [ROB_IDX_W-1:0]   cmpl_rob_index

  // Hazard search + request ok/err tagging moved to a pre-ROB request_validator
  // block (hazards handled in MC); no srch_*/stall ports here.
  // TODO ports (contract to fill in):
  //   alloc  : admit a (validated) request -> {rob_index, sram_slot}; id, len, size
  //   retire : head pointer -> B/R response + slot free
);

  // request-level program-order pointer shift reg: order_q[0] = head (oldest).
  // Ordering + retire are per REQUEST, not per packet (packets just bump the
  // completion counter; the request retires when the count hits N).
  logic [PTR_W-1:0] order_q [ROB_DEPTH];

  // write-side inflow / outflow counters, slot-indexed (DIR==1). TODO.
  logic [CNT_W-1:0] inflow_cnt  [ROB_DEPTH];
  logic [CNT_W-1:0] outflow_cnt [ROB_DEPTH];

  // TODO: completion pulse -> outflow_cnt[rob_index]++ ; retire head at count==N.

endmodule : rmc_cif_rob
