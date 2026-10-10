// rmc_cif_rob.sv
// CIF reorder buffer. One instance per direction (DIR=0 read, DIR=1 write).
//
// Role:
//   - admit a validated request and allocate its rob_index (a ring slot)
//   - hand the slot to the segmenter (burst splitter), track when inflow is done //PREHANDED.. the REQUIER VALIDOARED ogive a meta data bvlaid the phy address
//   - count MC completions (outflow) and retire in program order to drive B / R
//
// Slot ring (no shift register, no per-slot valid bit):
//   - alloc_ptr  : current fill/split slot; advances on SEG_LAST
//   - retire_ptr : oldest slot; advances on retire
//   - occ        : entries in flight; gives full / empty
//
// One request, start to finish:
//   admit    : write {id, err} at alloc_ptr, clear its counters, set pending_split
//   handoff  : present ROB_IDX = alloc_ptr, ROB_IDX_VALID = pending_split (both reg)
//   SEG_LAST : inflow_complt[slot]=1, n_pkts[slot]=SEG_NPKTS, advance alloc_ptr
//   cmpl     : outflow_cnt[rob_index]++ ; cmpl_err tags a downstream error
//   retire   : when inflow_complt && outflow_cnt==n_pkts, drive RESP_*, pop slot
//
// Not handled here: the phys addr (lives in the pre-ROB AW_req buffer -> segmenter),
// address hazards and ok/err tagging (pre-ROB request_validator / MC). Completion
// is a pulse {rob_index, err}; the ROB counts pulses, it tracks no packet number.

import rmc_cfg_pkg::*;

module rmc_cif_rob #(
  parameter int DIR       = 0,    // 0 = read ROB, 1 = write ROB    [root knob]
  parameter int ROB_DEPTH = 32,   // physical entries (16-32)        [root knob]
  parameter int MAX_PKTS  = 16,   // max packets per request         [root knob]
  parameter int AXI_IDW   = 8,    // AXI id width                    [root knob]
  parameter int AXI_AW    = 40,   // AXI address width               [root knob]
  parameter bit NEED_META = 1,    // 1 = build the per-slot metadata RAM (mem_meta)

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W  = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1, // rob_index tag width
  localparam int PTR_W      = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1, // ring pointer width
  localparam int CNT_W      = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1, // packet counter width
  localparam int REQ_META_W = AXI_IDW + 8 + 3 + 1, // admit payload {id,len8,size3,validated1}
  localparam int SLOT_PTR_W = 16,                  // TODO: = $clog2(data-SRAM depth)
  localparam int META_W     = 8 + 3 + SLOT_PTR_W   // mem_meta: {len8, size3, head_mem_ptr}
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // completion in (MC compl async FIFO). Pulse, never back-pressures (ready tied
  // high at the CIF top): it only bumps an already-allocated entry's counter.
  input  logic                   cmpl_valid,       // completion pulse
  input  logic [ROB_IDX_W-1:0]   cmpl_rob_index,   // slot it completes
  input  logic                   cmpl_err,         // downstream error for this slot -> SLVERR

  // admit (from the pre-ROB AW_req buffer, post request_validator)
  input  logic [REQ_META_W-1:0]  AW_REQUEST,       // {A_ID, A_LEN, A_SIZE, A_VALIDATED}
  input  logic                   A_ID_VALID,       // a request is offered
  output logic                   A_ID_READY,       // ROB can admit (slot free, none mid-split)

  // ROB <-> segmenter (burst splitter) handoff
  output logic [ROB_IDX_W-1:0]   ROB_IDX,          // current fill/split slot (registered)
  output logic                   ROB_IDX_VALID,    // slot handed to the segmenter
  input  logic                   SEG_LAST,         // last packet of the request issued
  input  logic [CNT_W-1:0]       SEG_NPKTS,        // packet count of the request (valid at SEG_LAST)

  // retire / response handshake. Semantics by DIR:
  //   DIR=0 read : R path  - id, resp, and {head_ptr,len,size} so the resp-path
  //                read splitter can fetch RD_SRAM and drive the R beats
  //   DIR=1 write: B path  - id + resp (BRESP); the read extras tie to 0
  output logic                   RESP_VALID,       // oldest entry fully done
  input  logic                   RESP_READY,       // resp path accepts the retire
  output logic [AXI_IDW-1:0]     RESP_ID,          // AXI id for the response
  output logic [1:0]             RESP_RESP,        // AXI resp: 00 OKAY / 10 SLVERR
  output logic [SLOT_PTR_W-1:0]  RESP_HEAD_PTR,    // read only: RD_SRAM head slot
  output logic [7:0]             RESP_LEN,         // read only: ar_len
  output logic [2:0]             RESP_SIZE         // read only: ar_size
);

  // slot ring
  localparam int OCC_W = $clog2(ROB_DEPTH + 1);
  logic [PTR_W-1:0] alloc_ptr;    // current fill/split slot; advances on SEG_LAST
  logic [PTR_W-1:0] retire_ptr;   // oldest slot; advances on retire
  logic [OCC_W-1:0] occ;          // entries in flight
  logic             rob_full;
  logic             rob_empty;
  assign rob_full  = (occ == OCC_W'(ROB_DEPTH));
  assign rob_empty = (occ == '0);

  // ROB entry (flops, uniform r/w, slot-indexed)
  logic [AXI_IDW-1:0] ent_id        [ROB_DEPTH]; // AXI id, for the B/R response
  logic               ent_err       [ROB_DEPTH]; // validation fail (A_VALIDATED=0) -> SLVERR
  logic               ent_derr      [ROB_DEPTH]; // downstream error (cmpl_err)      -> SLVERR
  logic               inflow_complt [ROB_DEPTH]; // all packets issued (set on SEG_LAST)
  logic [CNT_W-1:0]   n_pkts        [ROB_DEPTH]; // packet count, latched on SEG_LAST
  logic [CNT_W-1:0]   outflow_cnt   [ROB_DEPTH]; // completions seen, ++ per pulse

  // optional per-slot metadata RAM ("mem_")
  // read side stashes {len, size, head_mem_ptr} to rebuild RD_SRAM offsets on the
  // resp path; write side stashes the slot chain. NEED_META=0 drops it entirely.
  // layout (MSB->LSB): {len[8], size[3], head_mem_ptr[SLOT_PTR_W]}.
  generate
    if (NEED_META) begin : g_meta
      logic [META_W-1:0] mem_meta [ROB_DEPTH];    // write@admit/seg, read@resp/retire
      // TODO: mem_meta[alloc_ptr] <= {len, size, head_mem_ptr}; read on resp path
    end
  endgenerate

  // admit payload field-slice
  logic [AXI_IDW-1:0] admit_id;
  logic               admit_ok;   // A_VALIDATED: 1 = address validated OK
  assign admit_id = AW_REQUEST[REQ_META_W-1 -: AXI_IDW]; // top bits = A_ID
  assign admit_ok = AW_REQUEST[0];                       // LSB      = A_VALIDATED

  // handoff (registered, no comb path off admit_fire)
  // ROB_IDX is just alloc_ptr (a flop). pending_split marks the slot as handed to
  // the segmenter and holds until SEG_LAST, which is also when alloc_ptr advances.
  logic pending_split;
  wire  admit_fire = A_ID_VALID & A_ID_READY;
  assign A_ID_READY    = ~rob_full & ~pending_split; // free slot, none mid-split
  assign ROB_IDX       = alloc_ptr;                  // registered current slot
  assign ROB_IDX_VALID = pending_split;              // registered handoff valid

  // retire / response (comb)
  // done = inflow latched AND every packet completed (outflow == n_pkts). We ALWAYS
  // wait for the full outflow, even on an error: MC completes every packet (it may
  // still have packets in its pipeline), so retiring early would free the slot
  // before they drain and a late completion would corrupt the reused slot. The
  // error does not change WHEN we retire - only the resp code (SLVERR) via ent_*.
  assign RESP_VALID = ~rob_empty
                    & inflow_complt[retire_ptr]
                    & (outflow_cnt[retire_ptr] == n_pkts[retire_ptr]);
  assign RESP_ID    = ent_id[retire_ptr];
  assign RESP_RESP  = (ent_err[retire_ptr] | ent_derr[retire_ptr]) ? 2'b10 : 2'b00; // SLVERR / OKAY
  wire retire_fire  = RESP_VALID & RESP_READY;

  // read-side extras (DIR=0): rebuild the R beats from the metadata RAM. For a
  // write ROB these tie to 0 (the B path uses only id + resp).
  generate
    if (NEED_META) begin : g_resp
      assign RESP_LEN      = g_meta.mem_meta[retire_ptr][META_W-1 -: 8];
      assign RESP_SIZE     = g_meta.mem_meta[retire_ptr][SLOT_PTR_W +: 3];
      assign RESP_HEAD_PTR = g_meta.mem_meta[retire_ptr][SLOT_PTR_W-1:0];
    end else begin : g_resp
      assign RESP_LEN      = '0;
      assign RESP_SIZE     = '0;
      assign RESP_HEAD_PTR = '0;
    end
  endgenerate

  // all flop updates (one driver each; admit/seg/cmpl/retire are parallel)
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      alloc_ptr     <= '0;
      retire_ptr    <= '0;
      occ           <= '0;
      pending_split <= 1'b0;
    end else begin
      // admit: fill slot alloc_ptr, clear its counters, mark pending (no ptr move)
      if (admit_fire) begin
        ent_id[alloc_ptr]        <= admit_id;
        ent_err[alloc_ptr]       <= ~admit_ok;
        ent_derr[alloc_ptr]      <= 1'b0;
        inflow_complt[alloc_ptr] <= 1'b0;
        outflow_cnt[alloc_ptr]   <= '0;
        pending_split            <= 1'b1;
      end
      // SEG_LAST: inflow done + packet count, clear pending, advance alloc_ptr
      if (SEG_LAST) begin
        inflow_complt[alloc_ptr] <= 1'b1;
        n_pkts[alloc_ptr]        <= SEG_NPKTS;
        pending_split            <= 1'b0;
        alloc_ptr <= (alloc_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : alloc_ptr + 1'b1;
      end
      // cmpl: one packet done for cmpl_rob_index (+ downstream error tag)
      if (cmpl_valid) begin
        outflow_cnt[cmpl_rob_index] <= outflow_cnt[cmpl_rob_index] + 1'b1;
        if (cmpl_err) ent_derr[cmpl_rob_index] <= 1'b1;
      end
      // retire: pop the oldest slot
      if (retire_fire)
        retire_ptr <= (retire_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : retire_ptr + 1'b1;
      // occupancy: admit adds, retire removes
      // on retire fire give out the meta and mem head prt and axi_len seize form mem prt// we forgot that
      occ <= occ + OCC_W'(admit_fire) - OCC_W'(retire_fire);
    end
  end

endmodule : rmc_cif_rob
