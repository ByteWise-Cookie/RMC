// rmc_cif_rob_fv.sv
// Formal checker for rmc_cif_rob. Bound into the DUT by rmc_cif_rob_bind.sv, so
// the RTL carries no verification code. Spec: rmc_cif_rob_fv.md (rmc_theory).
//
// Three layers, top to bottom:
//   1. reference model : a tiny independent ring + per-slot scoreboard, driven
//                        only by the DUT ports (what the neighbours can see)
//   2. assumptions     : legal input behaviour, written against the model only
//   3. assertions      : (a) outputs match the model         -> the real checks
//                        (b) DUT state matches the model     -> induction helpers
//                        plus covers that show the interesting cases are reachable

import rmc_cfg_pkg::*;

module rmc_cif_rob_fv #(
  parameter int ROB_DEPTH  = 32,
  parameter int MAX_PKTS   = 16,
  parameter int AXI_IDW    = 8,
  parameter bit NEED_META  = 1,
  // DUT-derived widths, passed straight from the DUT localparams by the bind
  parameter int PTR_W      = 5,
  parameter int OCC_W      = 6,
  parameter int CNT_W      = 5,
  parameter int REQ_META_W = 20,
  parameter int SLOT_PTR_W = 16,
  parameter int AMETA_W    = 22,
  parameter int META_W     = 33
)(
  input logic                  aclk,
  input logic                  aresetn,

  // DUT ports
  input logic                  cmpl_valid,
  input logic [PTR_W-1:0]      cmpl_rob_index,
  input logic                  cmpl_err,
  input logic [REQ_META_W-1:0] AW_REQUEST,
  input logic [AMETA_W-1:0]    A_META,
  input logic                  A_ID_VALID,
  input logic                  A_ID_READY,
  input logic [PTR_W-1:0]      ROB_IDX,
  input logic                  ROB_IDX_VALID,
  input logic                  SEG_LAST,
  input logic [CNT_W-1:0]      SEG_NPKTS,
  input logic                  RESP_VALID,
  input logic                  RESP_READY,
  input logic [AXI_IDW-1:0]    RESP_ID,
  input logic [1:0]            RESP_RESP,
  input logic [SLOT_PTR_W-1:0] RESP_HEAD_PTR,
  input logic [PKT_OFF_W-1:0]  RESP_OFFSET,
  input logic [7:0]            RESP_LEN,
  input logic [2:0]            RESP_SIZE,

  // DUT internals (only used by the induction helpers)
  input logic [PTR_W-1:0]      alloc_ptr,
  input logic [PTR_W-1:0]      retire_ptr,
  input logic [OCC_W-1:0]      occ,
  input logic                  pending_split,
  input logic [AXI_IDW-1:0]    ent_id        [ROB_DEPTH],
  input logic                  ent_err       [ROB_DEPTH],
  input logic                  ent_derr      [ROB_DEPTH],
  input logic                  inflow_complt [ROB_DEPTH],
  input logic [CNT_W-1:0]      n_pkts        [ROB_DEPTH],
  input logic [CNT_W-1:0]      outflow_cnt   [ROB_DEPTH]
  // mem_meta only exists when NEED_META=1, so it is not a port: g_slot reaches
  // it by name (rmc_cif_rob.g_meta.mem_meta) inside the NEED_META branch
);

  // Property style: the open-source flow (yosys-slang) takes only boolean
  // properties, no |-> / |=> and no sequences. So:
  //   a |-> b   is written   !a || b
  //   a |=> b   is written   !$past(a) || b      (via `NEXT, which also skips the
  //                                               first cycle out of reset)
  // Every property is sampled on aclk; asserts and covers are off during reset.
`define ASSERT(name, expr) name: assert property (@(posedge aclk) disable iff (!aresetn) (expr))
`define COVER(name, expr)  name: cover  property (@(posedge aclk) disable iff (!aresetn) (expr))
`define ASSUME(name, expr) name: assume property (@(posedge aclk) (expr))
`define NEXT(a, b)         (!$past(aresetn) || !$past(a) || (b))

  // ------------------------------------------------------------------------
  // 1. reference model
  // ------------------------------------------------------------------------
  wire admit  = A_ID_VALID & A_ID_READY;
  wire retire = RESP_VALID & RESP_READY;

  // admit payload, sliced per the spec (not copied from the DUT's own slices)
  wire [AXI_IDW-1:0] admit_id   = AW_REQUEST[REQ_META_W-1 -: AXI_IDW];
  wire [7:0]         admit_len  = AW_REQUEST[11:4];
  wire [2:0]         admit_size = AW_REQUEST[3:1];
  wire               admit_ok   = AW_REQUEST[0];

  function automatic logic [PTR_W-1:0] ring_inc(input logic [PTR_W-1:0] p);
    return (p == PTR_W'(ROB_DEPTH-1)) ? '0 : p + 1'b1;
  endfunction

  logic [PTR_W-1:0]     fv_tail;    // slot of the current / next request
  logic [PTR_W-1:0]     fv_head;    // oldest slot
  logic                 fv_split;   // a request is with the segmenter
  logic [ROB_DEPTH-1:0] fv_live;    // slot admitted and not yet retired
  logic [ROB_DEPTH-1:0] fv_sealed;  // slot has seen its SEG_LAST

  logic [AXI_IDW-1:0]   fv_id     [ROB_DEPTH];
  logic                 fv_slverr [ROB_DEPTH]; // validation fail or any cmpl_err
  logic [CNT_W-1:0]     fv_npkts  [ROB_DEPTH];
  logic [CNT_W-1:0]     fv_cmpl   [ROB_DEPTH]; // completions seen
  logic [META_W-1:0]    fv_meta   [ROB_DEPTH];

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      fv_tail   <= '0;
      fv_head   <= '0;
      fv_split  <= 1'b0;
      fv_live   <= '0;
      fv_sealed <= '0;
    end else begin
      if (admit) begin
        fv_live[fv_tail]   <= 1'b1;
        fv_sealed[fv_tail] <= 1'b0;
        fv_id[fv_tail]     <= admit_id;
        fv_slverr[fv_tail] <= ~admit_ok;
        fv_cmpl[fv_tail]   <= '0;
        fv_meta[fv_tail]   <= {admit_len, admit_size, A_META};
        fv_split           <= 1'b1;
      end
      if (SEG_LAST) begin
        fv_sealed[fv_tail] <= 1'b1;
        fv_npkts[fv_tail]  <= SEG_NPKTS;
        fv_split           <= 1'b0;
        fv_tail            <= ring_inc(fv_tail);
      end
      if (cmpl_valid) begin
        fv_cmpl[cmpl_rob_index] <= fv_cmpl[cmpl_rob_index] + 1'b1;
        if (cmpl_err) fv_slverr[cmpl_rob_index] <= 1'b1;
      end
      if (retire) begin
        fv_live[fv_head] <= 1'b0;
        fv_head          <= ring_inc(fv_head);
      end
    end
  end

  wire fv_head_done = fv_live[fv_head] & fv_sealed[fv_head]
                    & (fv_cmpl[fv_head] == fv_npkts[fv_head]);

  // ------------------------------------------------------------------------
  // 2. assumptions (spec section 8). Model state only, never DUT internals.
  // ------------------------------------------------------------------------
  // the proof starts in reset; after that reset is free to come back any time
  logic fv_started = 1'b0;
  always_ff @(posedge aclk) fv_started <= 1'b1;
  `ASSUME(m_reset_first, fv_started || !aresetn);

  // 8.3 one SEG_LAST per request, only while it is with the segmenter, with a
  //     legal packet count that covers every completion already returned
  wire cmpl_tail = cmpl_valid & (cmpl_rob_index == fv_tail);
  `ASSUME(m_seg_last, !SEG_LAST || (
       fv_split
    && SEG_NPKTS >= CNT_W'(1) && SEG_NPKTS <= CNT_W'(MAX_PKTS)
    && SEG_NPKTS >= fv_cmpl[fv_tail] + CNT_W'(cmpl_tail)));

  // 8.1 / 8.2 / 8.6 a completion targets a live slot and never exceeds the
  //     packets issued for it (n_pkts once sealed, MAX_PKTS while splitting)
  `ASSUME(m_cmpl, !cmpl_valid || (
       cmpl_rob_index < ROB_DEPTH
    && fv_live[cmpl_rob_index]
    && fv_cmpl[cmpl_rob_index] <
         (fv_sealed[cmpl_rob_index] ? fv_npkts[cmpl_rob_index] : CNT_W'(MAX_PKTS))));

  // ------------------------------------------------------------------------
  // 3a. interface checks: what the neighbours rely on
  // ------------------------------------------------------------------------
  // admit side
  `ASSERT(a_ready,       A_ID_READY == (~&fv_live & ~fv_split));
  `ASSERT(a_no_reuse,    !admit || !fv_live[ROB_IDX]);                     // spec 11

  // segmenter handoff
  `ASSERT(a_idx_valid,   ROB_IDX_VALID == fv_split);                       // spec 4
  `ASSERT(a_idx,         ROB_IDX == fv_tail);                              // spec 12
  `ASSERT(a_idx_hold,    `NEXT(ROB_IDX_VALID && !SEG_LAST,
                               ROB_IDX_VALID && $stable(ROB_IDX)));
  `ASSERT(a_admit_split, `NEXT(admit, ROB_IDX_VALID));

  // retire side: valid exactly when the oldest request is done, in order,
  // carrying that request's own id / resp / metadata           spec 8, 9, 10, 13
  `ASSERT(a_resp_valid,  RESP_VALID == fv_head_done);
  `ASSERT(a_resp_id,     !RESP_VALID || RESP_ID == fv_id[fv_head]);
  `ASSERT(a_resp_resp,   !RESP_VALID ||
                         RESP_RESP == (fv_slverr[fv_head] ? 2'b10 : 2'b00));
  `ASSERT(a_resp_hold,   `NEXT(RESP_VALID && !RESP_READY,
                               RESP_VALID && $stable(RESP_ID) && $stable(RESP_RESP)));
  if (NEED_META) begin : g_meta_chk
    `ASSERT(a_resp_meta, !RESP_VALID ||
            {RESP_LEN, RESP_SIZE, RESP_OFFSET, RESP_HEAD_PTR} == fv_meta[fv_head]);
  end else begin : g_meta_chk
    `ASSERT(a_resp_meta, {RESP_LEN, RESP_SIZE, RESP_OFFSET, RESP_HEAD_PTR} == '0);
  end

  // reset: first cycle out of reset the ROB is empty, idle and ready   spec 10
  `ASSERT(a_reset,       $past(aresetn) || (
       occ == '0 && alloc_ptr == '0 && retire_ptr == '0 && !pending_split
    && A_ID_READY && !ROB_IDX_VALID && !RESP_VALID));

  // back-pressure: a full ring never accepts, and an offered request that is
  // refused leaves the ring untouched
  `ASSERT(a_full_bp,     occ != ROB_DEPTH || !A_ID_READY);
  `ASSERT(a_refuse,      `NEXT(A_ID_VALID && !A_ID_READY && !retire && !SEG_LAST,
                               $stable(occ) && $stable(alloc_ptr)));

`ifdef FV_LIVE
  // spec 15 as bounded liveness: if RESP_READY is never low for more than
  // FV_RDY_MAX cycles in a row, a response never waits longer than that.
  // The fairness assumption narrows RESP_READY, so it is only compiled into
  // the *_live task and never into the safety proofs.
  localparam int FV_RDY_MAX = 4;
  logic [3:0] fv_rdy_low;   // cycles in a row with RESP_READY low
  logic [3:0] fv_wait;      // cycles in a row a response has been waiting
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      fv_rdy_low <= '0;
      fv_wait    <= '0;
    end else begin
      fv_rdy_low <= RESP_READY ? '0 : fv_rdy_low + 1'b1;
      fv_wait    <= (RESP_VALID && !RESP_READY) ? fv_wait + 1'b1 : '0;
    end
  end
  `ASSUME(m_ready_fair,  !aresetn || fv_rdy_low < FV_RDY_MAX || RESP_READY);
  `ASSERT(a_wait_helper, fv_wait <= fv_rdy_low && fv_rdy_low <= FV_RDY_MAX);
  `ASSERT(a_resp_live,   fv_wait <= FV_RDY_MAX);
  `COVER(c_wait_max,     fv_wait == FV_RDY_MAX);

  // spec 14 as bounded liveness, for the oldest request: given a SEG_LAST within
  // FV_SEG_MAX cycles of the split and a completion for the oldest request at
  // least every FV_CMPL_MAX cycles, RESP_VALID rises within FV_HEAD_MAX cycles.
  // With a_resp_live the oldest then retires and the next request becomes the
  // oldest, so every admitted request reaches RESP_VALID.
  localparam int FV_SEG_MAX  = 3;
  localparam int FV_CMPL_MAX = 3;
  localparam int FV_HEAD_MAX = (FV_SEG_MAX + 1) + (FV_CMPL_MAX + 1) * MAX_PKTS;

  wire fv_head_live   = fv_live[fv_head];
  wire fv_head_sealed = fv_sealed[fv_head];
  wire fv_head_short  = fv_head_live & fv_head_sealed          // still owed completions
                      & (fv_cmpl[fv_head] != fv_npkts[fv_head]);
  wire fv_head_hit    = cmpl_valid & (cmpl_rob_index == fv_head);

  logic [15:0] fv_seg_wait;   // cycles in a row a split has waited for SEG_LAST
  logic [15:0] fv_cmpl_wait;  // cycles in a row the oldest waited for a completion
  logic [15:0] fv_head_age;   // cycles in a row the oldest was live without RESP_VALID
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      fv_seg_wait  <= '0;
      fv_cmpl_wait <= '0;
      fv_head_age  <= '0;
    end else begin
      fv_seg_wait  <= (fv_split && !SEG_LAST)         ? fv_seg_wait  + 1'b1 : '0;
      fv_cmpl_wait <= (fv_head_short && !fv_head_hit) ? fv_cmpl_wait + 1'b1 : '0;
      fv_head_age  <= (fv_head_live && !RESP_VALID)   ? fv_head_age  + 1'b1 : '0;
    end
  end

  `ASSUME(m_seg_fair,  !aresetn || !fv_split      || fv_seg_wait  < FV_SEG_MAX  || SEG_LAST);
  `ASSUME(m_cmpl_fair, !aresetn || !fv_head_short || fv_cmpl_wait < FV_CMPL_MAX || fv_head_hit);

  `ASSERT(a_head_live, fv_head_age <= FV_HEAD_MAX);

  // helpers: the age is bounded by the fairness budget already spent
  `ASSERT(a_fair_range, fv_seg_wait <= FV_SEG_MAX && fv_cmpl_wait <= FV_CMPL_MAX);
  `ASSERT(a_age_split,  !(fv_head_live && !fv_head_sealed) || fv_head_age <= fv_seg_wait);
  `ASSERT(a_age_cmpl,   !(fv_head_live &&  fv_head_sealed) || fv_head_age <=
                        16'(FV_SEG_MAX + 1) + 16'(FV_CMPL_MAX + 1) * 16'(fv_cmpl[fv_head])
                        + fv_cmpl_wait);

  `COVER(c_head_age_max, fv_head_age == FV_HEAD_MAX);
`endif

  // ------------------------------------------------------------------------
  // 3b. induction helpers: DUT state == model state
  // ------------------------------------------------------------------------
  // ring                                                            spec 1-5
  `ASSERT(a_ptr_range,  alloc_ptr < ROB_DEPTH && retire_ptr < ROB_DEPTH);
  `ASSERT(a_occ_max,    occ <= ROB_DEPTH);
  `ASSERT(a_tail,       alloc_ptr     == fv_tail);
  `ASSERT(a_head,       retire_ptr    == fv_head);
  `ASSERT(a_split,      pending_split == fv_split);
  `ASSERT(a_occ,        occ == $countones(fv_live));
  // spec 3, corrected: alloc_ptr sits (occ - pending_split) slots past
  // retire_ptr, modulo ROB_DEPTH. The spec's "occ == (alloc-retire) mod D +
  // pending" form breaks when the ring is full (alloc == retire, occ == D).
  wire [OCC_W-1:0] fill      = occ - OCC_W'(pending_split);
  wire [OCC_W:0]   tail_lin  = (OCC_W+1)'(retire_ptr) + (OCC_W+1)'(fill);
  wire [OCC_W:0]   tail_wrap = (tail_lin >= ROB_DEPTH) ? tail_lin - (OCC_W+1)'(ROB_DEPTH) : tail_lin;
  `ASSERT(a_ring,       occ >= OCC_W'(pending_split) && (OCC_W+1)'(alloc_ptr) == tail_wrap);
  `ASSERT(a_alloc_move, `NEXT(!SEG_LAST, $stable(alloc_ptr)));
  `ASSERT(a_ret_move,   `NEXT(!retire,   $stable(retire_ptr)));

  // the live slots are exactly the occ slots starting at retire_ptr
  for (genvar i = 0; i < ROB_DEPTH; i++) begin : g_slot
    wire [OCC_W-1:0] age = (i >= retire_ptr) ? OCC_W'(i) - OCC_W'(retire_ptr)
                                             : OCC_W'(i) + OCC_W'(ROB_DEPTH) - OCC_W'(retire_ptr);
    wire live   = fv_live[i];
    wire sealed = fv_sealed[i];

    `ASSERT(a_live,     live == (age < occ));
    // the only live-but-unsealed slot is the one being split
    `ASSERT(a_unsealed, (live && !sealed) == (fv_split && fv_tail == i));

    `ASSERT(a_id,       !live || ent_id[i] == fv_id[i]);
    `ASSERT(a_err,      !live || (ent_err[i] | ent_derr[i]) == fv_slverr[i]);
    `ASSERT(a_inflow,   !live || inflow_complt[i] == sealed);
    `ASSERT(a_cnt,      !live || outflow_cnt[i] == fv_cmpl[i]);
    `ASSERT(a_npkts,    !(live && sealed) || (n_pkts[i] == fv_npkts[i]
                        && fv_npkts[i] >= 1 && fv_npkts[i] <= MAX_PKTS));
    `ASSERT(a_bound,    !live || fv_cmpl[i] <=                             // spec 7
                        (sealed ? fv_npkts[i] : CNT_W'(MAX_PKTS)));
    if (NEED_META) begin : g_meta
      `ASSERT(a_meta,   !live || rmc_cif_rob.g_meta.mem_meta[i] == fv_meta[i]);
    end
  end

  // ------------------------------------------------------------------------
  // covers: the assumptions leave these reachable (guards against a vacuous proof)
  // ------------------------------------------------------------------------
  `COVER(c_retire,      retire);
  `COVER(c_full,        occ == ROB_DEPTH);
  `COVER(c_slverr,      retire && RESP_RESP == 2'b10);
  `COVER(c_okay_max,    retire && RESP_RESP == 2'b00 && fv_npkts[fv_head] == MAX_PKTS);
  `COVER(c_wrap,        retire && retire_ptr == ROB_DEPTH-1);
  `COVER(c_early_cmpl,  SEG_LAST && fv_cmpl[fv_tail] != 0);
  `COVER(c_full_retire, occ == ROB_DEPTH && retire);

  // scenario trackers for the covers below (no asserts depend on them)
  logic               fv_rst_busy = 1'b0; // reset hit while requests were in flight
  logic [AXI_IDW-1:0] fv_adm_n;           // admits since reset
  logic               fv_ids_seq;         // every admit so far used id == its number
  logic               fv_was_full;        // ring has been full
  logic               fv_bp_seen;         // a request was refused while full
  always_ff @(posedge aclk)
    if (fv_started && !aresetn && $past(aresetn && |fv_live)) fv_rst_busy <= 1'b1;
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      fv_adm_n    <= '0;
      fv_ids_seq  <= 1'b1;
      fv_was_full <= 1'b0;
      fv_bp_seen  <= 1'b0;
    end else begin
      if (admit) begin
        fv_adm_n <= fv_adm_n + 1'b1;
        if (admit_id != fv_adm_n) fv_ids_seq <= 1'b0;
      end
      if (&fv_live)                              fv_was_full <= 1'b1;
      if (&fv_live && A_ID_VALID && !A_ID_READY) fv_bp_seen  <= 1'b1;
    end
  end

  // reset in the middle of traffic, then a clean request all the way to retire
  `COVER(c_reset_midrun, fv_rst_busy && retire);
  // fill the ring with ids 0,1,2.., get refused on full, then drain it empty;
  // a_resp_id holds on the way, so the ids come back out in the same order
  `COVER(c_fill_bp_drain, fv_ids_seq && fv_was_full && fv_bp_seen && fv_live == '0);

`undef ASSERT
`undef COVER
`undef ASSUME
`undef NEXT

endmodule : rmc_cif_rob_fv
