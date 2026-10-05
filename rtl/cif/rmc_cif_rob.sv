// rmc_cif_rob.sv
// Reorder buffer, ONE DIRECTION per instance (r_rob and w_rob are two instances;
// DIR picks the data-side behaviour). Allocates rob_index per admitted request,
// allocates the data SRAM slot (dbuf_addr for reads, wd_slot for writes), tracks
// per-packet completion, and retires in program order to drive B / R.
// MC completions are tag-only {rob_index, pkt_num, status}; the ROB reassembles.
//
// Order is held by a shift register of POINTERS, not of the entries themselves.
// Every admitted request is index-mapped: one slot index addresses all of its
// backing stores, so only the narrow pointer moves down the shift reg while the
// wide metadata stays put. Retire pops the head pointer. Split the metadata by
// access pattern instead of one fat flop array:
//
//   addr TCAM    -> owned HERE. Per entry {page, start12, last12, valid} from the
//                   AXI 4KB rule. The opposite-direction ROB probes it (srch_*);
//                   a hit feeds rmc_cif_haz, which returns this ROB's stall_vector.
//   inflow/      -> counter memory (write side): ++ per packet, compare to N
//   outflow         (=from aw_len). done when the count hits N.
//   cold metadata-> SRAM: aw_id/ar_id, size, len, resp - read only at retire.
//
// All stores share depth = ROB_DEPTH so the slot index lines up across them.
//
// STUB: params + completion sink + addr TCAM search + stores sketched. Logic TODO.

import rmc_cfg_pkg::*;

module rmc_cif_rob #(
  parameter int DIR       = 0,    // 0 = read ROB, 1 = write ROB
  parameter int ROB_DEPTH = 32,   // physical entries (16-32)
  parameter int ROB_IDX_W = 8,    // tag width = CIF ROB_INDEX[7:0] contract
  parameter int MAX_PKTS  = 16,
  parameter int PKT_NUM_W = 4,
  parameter int AXI_AW    = 40,

  localparam int PTR_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int CNT_W = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int PG_W  = AXI_AW - 12
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // completion in (from MC compl async FIFO). No ready: completion never
  // back-pressures (tied high at the CIF top); a completion only bumps a counter
  // on an already-allocated entry, always absorbable in one cycle.
  input  logic                   cmpl_valid,
  input  logic [ROB_IDX_W-1:0]   cmpl_rob_index,
  input  logic [PKT_NUM_W-1:0]   cmpl_pkt_num,
  input  logic                   cmpl_status,

  // address-match search port: probed by the OPPOSITE-direction ROB's new req.
  // (r_rob is probed by a new write = WAR; w_rob is probed by a new read = RAW.)
  input  logic                   srch_valid,
  input  logic [PG_W-1:0]        srch_page,
  input  logic [11:0]            srch_start,
  input  logic [11:0]            srch_last,
  output logic                   srch_hit,
  output logic [PTR_W-1:0]       srch_ptr,

  // stall flags from haz: gate these entries' packet issue to the req async FIFO
  input  logic [ROB_DEPTH-1:0]   stall_vector

  // TODO ports (contract to fill in):
  //   alloc  : admit a request -> {rob_index, sram_slot}; carries id, len, size,
  //            addr (for the addr store range calc + search probe)
  //   retire : head pointer -> B/R response + slot free
);

  // program-order pointer shift reg: order_q[0] = head (oldest)
  logic [PTR_W-1:0] order_q [ROB_DEPTH];

  // write-side inflow / outflow counters, slot-indexed (DIR==1). TODO.
  logic [CNT_W-1:0] inflow_cnt  [ROB_DEPTH];
  logic [CNT_W-1:0] outflow_cnt [ROB_DEPTH];

  // addr TCAM (owned here). {page,start,last} computed at alloc:
  //   last = addr[11:0] + (((len+1) << size) - 1)   (12b, no overflow: 4KB rule)
  //   page = addr[AXI_AW-1:12]
  logic [ROB_DEPTH-1:0]           ent_valid;
  logic [ROB_DEPTH-1:0][PG_W-1:0] ent_page;
  logic [ROB_DEPTH-1:0][11:0]     ent_start;
  logic [ROB_DEPTH-1:0][11:0]     ent_last;

  // exact overlap within a 4KB page (AXI INCR never crosses the boundary)
  function automatic logic overlap(
      input logic [PG_W-1:0] pa, input logic [11:0] sa, input logic [11:0] la,
      input logic [PG_W-1:0] pb, input logic [11:0] sb, input logic [11:0] lb);
    overlap = (pa == pb) && (sa <= lb) && (sb <= la);
  endfunction

  // TCAM search: does the probe overlap any valid entry? return a matched slot.
  // TODO: return the OLDEST match (by order_q age), not just the lowest index.
  always_comb begin
    srch_hit = 1'b0;
    srch_ptr = '0;
    for (int i = 0; i < ROB_DEPTH; i++) begin
      if (srch_valid && ent_valid[i]
          && overlap(srch_page, srch_start, srch_last,
                     ent_page[i], ent_start[i], ent_last[i])) begin
        srch_hit = 1'b1;
        srch_ptr = PTR_W'(i);
      end
    end
  end

  // TODO: gate each entry's packet issue to the req async FIFO with ~stall_vector

endmodule : rmc_cif_rob
