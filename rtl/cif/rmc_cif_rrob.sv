// rmc_cif_rrob.sv
// READ reorder buffer. Admits read requests, tracks completion, retires in program
// order to drive the AXI R response path.
//
// Slot ring (no shift register, no per-slot valid bit):
//   - alloc_ptr  : next free slot; advances on admit (many requests may be mid-split)
//   - retire_ptr : oldest slot; advances on retire
//   - occ        : entries in flight; gives full / empty
//
// Multi-outstanding: the pipelined rseg splits several requests at once, so admit is
// decoupled from SEG_LAST. alloc_ptr advances at admit (handing ROB_IDX out); SEG_LAST
// arrives later, carrying SEG_ROB_IDX to close that specific entry. rseg's seg_last is
// in program order, but indexing by SEG_ROB_IDX keeps the ROB robust to it either way.
//
// n_pkts ships EARLY: it is known the moment the request is admitted (the segmenter
// computes it at enqueue), so the ROB writes it at admit. MC completions can beat the
// split's SEG_LAST, so the count target must already be valid. SEG_LAST then only marks
// the split done and delivers the resp meta.
//
// One request:
//   admit    : write {id, err, n_pkts, len, size, addr_lo, base_ch} at alloc_ptr, clear
//              outflow, advance alloc_ptr. addr_lo + base_ch are the resp channel-walk seed.
//   handoff  : rseg captures ROB_IDX (= alloc_ptr) on admit; it rides the request
//   SEG_LAST : inflow_complt[SEG_ROB_IDX]=1, mem_head[SEG_ROB_IDX] <= per-channel head slots
//   cmpl     : outflow_cnt[rob_index]++ ; cmpl_err tags a downstream error
//   retire   : when inflow_complt && outflow_cnt==n_pkts, drive RESP_* (id, resp, and the read
//              meta head_ptr/addr_lo/base_ch/len/size for the resp-path rebuild+trim), pop slot
//
// Completion is a pulse {rob_index, err}; the ROB counts pulses, no packet number.

import rmc_cfg_pkg::*;

module rmc_cif_rrob #(
  parameter int ROB_DEPTH = 32,   // physical entries (16-32)        [root knob]
  parameter int MAX_PKTS  = 64,   // max packets per request         [root knob]
  parameter int AXI_IDW   = 8,    // AXI id width                    [root knob]
  parameter int N_SLOTS   = 32,   // RD-SRAM slots (head_mem_ptr width) [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W  = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1, // rob_idx tag width
  localparam int PTR_W      = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1, // ring pointer width
  localparam int CNT_W      = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1, // packet-count width
  localparam int REQ_META_W = AXI_IDW + 1,                    // admit payload {id, validated}
  localparam int SLOT_PTR_W = (N_SLOTS > 1) ? $clog2(N_SLOTS) : 1, // RD-SRAM head-slot width
  localparam int META_W     = 8 + 3 + PAGE_LSB + CH_W + N_CH*SLOT_PTR_W
                              // resp meta {len8, size3, addr_lo, base_ch, head[N_CH]}
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // completion in (MC compl async FIFO). Pulse, never back-pressures.
  input  logic                   cmpl_valid,
  input  logic [ROB_IDX_W-1:0]   cmpl_rob_index,
  input  logic                   cmpl_err,         // downstream error -> SLVERR

  // admit (from the burst splitter). The splitter now works the whole request descriptor
  // out at admit (per-line channel popcount), so the resp meta ships EARLY with admit, not
  // at SEG_LAST: n_pkts, the trim fields (offset/len/size) and base_ch are all known now.
  // A_ID_VALID is the splitter's admit_valid pulse; A_ID_READY = free slot. Only the
  // per-channel head pointers are still unknown here - they arrive at SEG_LAST.
  input  logic [REQ_META_W-1:0]  AW_REQUEST,       // {A_ID, A_VALIDATED}
  input  logic [CNT_W-1:0]       ADMIT_NPKTS,      // total packets for this request (at admit)
  input  logic [PAGE_LSB-1:0]    ADMIT_ADDR_LO,    // ar_addr[PAGE_LSB-1:0]: [PKT_OFF_W-1:0] = trim
                                                   //   offset, [PAGE_LSB-1:PKT_OFF_W] = start line l0.
                                                   //   The resp rebuild walks this per line to re-derive
                                                   //   the channel of each line (see base_ch below).
  input  logic [7:0]             ADMIT_LEN,        // ar_len  (beats-1)       (resp trim)
  input  logic [2:0]             ADMIT_SIZE,       // ar_size (log2 beat bytes)(resp trim)
  input  logic [CH_W-1:0]        ADMIT_BASE_CH,    // channel of line 0. Freezes the CH source bits that
                                                   //   sit >= PAGE_LSB (constant across the request, so
                                                   //   absent from ADDR_LO); the rebuild takes those
                                                   //   bits from here and the in-page ones from ADDR_LO.
  input  logic                   A_ID_VALID,
  output logic                   A_ID_READY,       // free slot

  // rrob -> splitter handoff. ROB_IDX (= alloc_ptr) is handed out at admit and rides the
  // request; the entry is "filled" (n_pkts + trim meta) right then. Completions for it may
  // arrive before the split even finishes, so those must already be valid - hence the early
  // ship. SEG_LAST later marks the split done for that index and brings only the per-channel
  // head pointers. Sample ROB_IDX only on admit (A_ID_VALID & A_ID_READY); it tracks
  // alloc_ptr and is meaningless between admits.
  output logic [ROB_IDX_W-1:0]   ROB_IDX,          // offered slot index (= alloc_ptr)
  input  logic                   SEG_LAST,         // a request finished splitting
  input  logic [ROB_IDX_W-1:0]   SEG_ROB_IDX,      // which entry it was (closes that slot)
  input  logic [N_CH-1:0][SLOT_PTR_W-1:0] SEG_HEAD_PTR, // per-channel RD-SRAM head slots (at SEG_LAST)

  // retire / R response
  output logic                   RESP_VALID,
  input  logic                   RESP_READY,
  output logic [AXI_IDW-1:0]     RESP_ID,
  output logic [1:0]             RESP_RESP,        // 00 OKAY / 10 SLVERR
  output logic [N_CH-1:0][SLOT_PTR_W-1:0] RESP_HEAD_PTR, // per-channel RD-SRAM head slots
  output logic [CH_W-1:0]        RESP_BASE_CH,     // channel of line 0 (frozen out-of-page CH bits)
  output logic [PAGE_LSB-1:0]    RESP_ADDR_LO,     // ar_addr[PAGE_LSB-1:0]: [PKT_OFF_W-1:0] trim offset,
                                                   //   [PAGE_LSB-1:PKT_OFF_W] start line (channel walk)
  output logic [7:0]             RESP_LEN,         // ar_len
  output logic [2:0]             RESP_SIZE         // ar_size
);

  // slot ring
  localparam int OCC_W = $clog2(ROB_DEPTH + 1);    // occupancy counter width
  logic [PTR_W-1:0] alloc_ptr;                      // next free slot (advances on admit)
  logic [PTR_W-1:0] retire_ptr;                     // oldest slot (advances on retire)
  logic [OCC_W-1:0] occ;                            // entries in flight
  logic             rob_full;                       // no free slot
  logic             rob_empty;                      // nothing in flight
  assign rob_full  = (occ == OCC_W'(ROB_DEPTH));
  assign rob_empty = (occ == '0);

  // per-slot entry (flops)
  logic [AXI_IDW-1:0] ent_id        [ROB_DEPTH];    // AXI id (-> RESP_ID)
  logic               ent_err       [ROB_DEPTH]; // validation fail -> SLVERR
  logic               ent_derr      [ROB_DEPTH]; // downstream error -> SLVERR
  logic               inflow_complt [ROB_DEPTH]; // all packets issued (SEG_LAST)
  logic [CNT_W-1:0]   n_pkts        [ROB_DEPTH];    // packets owed (set at admit)
  logic [CNT_W-1:0]   outflow_cnt   [ROB_DEPTH];    // completions seen so far

  // admit payload field-slice (len/size reach mem_meta from rseg at SEG_LAST)
  logic [AXI_IDW-1:0] admit_id;                     // id sliced from AW_REQUEST
  logic               admit_ok;                     // validated bit (0 -> SLVERR)
  assign admit_id = AW_REQUEST[REQ_META_W-1 -: AXI_IDW];
  assign admit_ok = AW_REQUEST[0];

  // handoff: A_ID_READY is the ROB's ready. Multi-outstanding - admit only needs a free
  // slot, so the segmenter can keep several requests in flight at once. ROB_IDX (the slot
  // handed out) rides with the request and tags every packet for completion routing.
  wire  admit_fire = A_ID_VALID & A_ID_READY;  // a request is admitted this cycle
  assign A_ID_READY = ~rob_full;
  assign ROB_IDX    = alloc_ptr;

  // retire / response (comb)
  assign RESP_VALID = ~rob_empty
                    & inflow_complt[retire_ptr]
                    & (outflow_cnt[retire_ptr] == n_pkts[retire_ptr]);
  assign RESP_ID    = ent_id[retire_ptr];
  assign RESP_RESP  = (ent_err[retire_ptr] | ent_derr[retire_ptr]) ? 2'b10 : 2'b00;
  wire retire_fire  = RESP_VALID & RESP_READY;  // oldest entry pops this cycle

  // resp meta, read out at retire for the resp-path trim. Split by when it is known:
  //   mem_adm  {len, size, addr_lo, base_ch} - written at ADMIT (all known then)
  //   mem_head per-channel RD-SRAM head slots - written at SEG_LAST (filled by the engines)
  logic [7:0]                      madm_len  [ROB_DEPTH];
  logic [2:0]                      madm_size [ROB_DEPTH];
  logic [PAGE_LSB-1:0]             madm_alo  [ROB_DEPTH];
  logic [CH_W-1:0]                 madm_bch  [ROB_DEPTH];
  logic [N_CH-1:0][SLOT_PTR_W-1:0] mem_head  [ROB_DEPTH];
  always_ff @(posedge aclk)
    if (SEG_LAST) mem_head[SEG_ROB_IDX] <= SEG_HEAD_PTR;
  assign RESP_LEN      = madm_len [retire_ptr];
  assign RESP_SIZE     = madm_size[retire_ptr];
  assign RESP_ADDR_LO  = madm_alo [retire_ptr];
  assign RESP_BASE_CH  = madm_bch [retire_ptr];
  assign RESP_HEAD_PTR = mem_head [retire_ptr];

  // all flop updates (admit / SEG_LAST / cmpl / retire are parallel)
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      alloc_ptr     <= '0;
      retire_ptr    <= '0;
      occ           <= '0;
    end else begin
      // admit: claim the slot, write n_pkts + the trim meta (all known now), advance alloc_ptr.
      if (admit_fire) begin
        ent_id[alloc_ptr]        <= admit_id;
        ent_err[alloc_ptr]       <= ~admit_ok;
        ent_derr[alloc_ptr]      <= 1'b0;
        inflow_complt[alloc_ptr] <= 1'b0;
        outflow_cnt[alloc_ptr]   <= '0;
        n_pkts[alloc_ptr]        <= ADMIT_NPKTS;
        madm_len[alloc_ptr]      <= ADMIT_LEN;
        madm_size[alloc_ptr]     <= ADMIT_SIZE;
        madm_alo[alloc_ptr]      <= ADMIT_ADDR_LO;
        madm_bch[alloc_ptr]      <= ADMIT_BASE_CH;
        alloc_ptr <= (alloc_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : alloc_ptr + 1'b1;
      end
      // SEG_LAST closes the entry named by SEG_ROB_IDX (not alloc_ptr): its split done.
      // n_pkts + trim meta were written at admit; here only inflow + the head pointers land.
      if (SEG_LAST)
        inflow_complt[SEG_ROB_IDX] <= 1'b1;
      if (cmpl_valid) begin
        outflow_cnt[cmpl_rob_index] <= outflow_cnt[cmpl_rob_index] + 1'b1;
        if (cmpl_err) ent_derr[cmpl_rob_index] <= 1'b1;
      end
      if (retire_fire)
        retire_ptr <= (retire_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : retire_ptr + 1'b1;
      occ <= occ + OCC_W'(admit_fire) - OCC_W'(retire_fire);
    end
  end

endmodule : rmc_cif_rrob
