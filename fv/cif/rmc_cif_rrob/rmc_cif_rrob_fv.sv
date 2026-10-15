// rmc_cif_rrob_fv.sv
// Formal checker for the MULTI-OUTSTANDING rmc_cif_rrob. Bound by rmc_cif_rrob_bind.sv.
//
// Differs from the old one-at-a-time rob checker:
//   - admit advances the tail (alloc_ptr) immediately; many requests may be mid-split.
//   - there is no pending_split; A_ID_READY = ~rob_full.
//   - SEG_LAST is indexed by SEG_ROB_IDX (seals THAT slot), not the tail; it does not
//     move the tail.
//
// Three layers:
//   1. reference model : an independent ring + per-slot scoreboard from the ports only
//   2. assumptions     : legal neighbour behaviour (completions, one SEG_LAST per slot)
//   3. assertions      : outputs match the model + DUT state == model (induction)

import rmc_cfg_pkg::*;

module rmc_cif_rrob_fv #(
  parameter int ROB_DEPTH  = 32,
  parameter int MAX_PKTS   = 64,
  parameter int AXI_IDW    = 8,
  // DUT-derived widths, passed from the DUT localparams by the bind
  parameter int ROB_IDX_W  = 5,
  parameter int PTR_W      = 5,
  parameter int OCC_W      = 6,
  parameter int CNT_W      = 7,
  parameter int REQ_META_W = 20,
  parameter int SLOT_PTR_W = 5,
  parameter int ADM_W      = 24       // admit-time resp meta {len8,size3,addr_lo[PAGE_LSB],base_ch[CH_W]}
)(
  input logic                  aclk,
  input logic                  aresetn,

  // DUT ports
  input logic                  cmpl_valid,
  input logic [ROB_IDX_W-1:0]  cmpl_rob_index,
  input logic                  cmpl_err,
  input logic [REQ_META_W-1:0] AW_REQUEST,
  input logic [CNT_W-1:0]      ADMIT_NPKTS,
  input logic [PAGE_LSB-1:0]   ADMIT_ADDR_LO,
  input logic [7:0]            ADMIT_LEN,
  input logic [2:0]            ADMIT_SIZE,
  input logic [CH_W-1:0]       ADMIT_BASE_CH,
  input logic                  A_ID_VALID,
  input logic                  A_ID_READY,
  input logic [ROB_IDX_W-1:0]  ROB_IDX,
  input logic                  SEG_LAST,
  input logic [ROB_IDX_W-1:0]  SEG_ROB_IDX,
  input logic [N_CH-1:0][SLOT_PTR_W-1:0] SEG_HEAD_PTR,
  input logic                  RESP_VALID,
  input logic                  RESP_READY,
  input logic [AXI_IDW-1:0]    RESP_ID,
  input logic [1:0]            RESP_RESP,
  input logic [N_CH-1:0][SLOT_PTR_W-1:0] RESP_HEAD_PTR,
  input logic [CH_W-1:0]       RESP_BASE_CH,
  input logic [PAGE_LSB-1:0]   RESP_ADDR_LO,
  input logic [7:0]            RESP_LEN,
  input logic [2:0]            RESP_SIZE,

  // DUT internals (induction helpers)
  input logic [PTR_W-1:0]      alloc_ptr,
  input logic [PTR_W-1:0]      retire_ptr,
  input logic [OCC_W-1:0]      occ,
  input logic [AXI_IDW-1:0]    ent_id        [ROB_DEPTH],
  input logic                  ent_err       [ROB_DEPTH],
  input logic                  ent_derr      [ROB_DEPTH],
  input logic                  inflow_complt [ROB_DEPTH],
  input logic [CNT_W-1:0]      n_pkts        [ROB_DEPTH],
  input logic [CNT_W-1:0]      outflow_cnt   [ROB_DEPTH],
  input logic [7:0]            madm_len      [ROB_DEPTH],
  input logic [2:0]            madm_size     [ROB_DEPTH],
  input logic [PAGE_LSB-1:0]   madm_alo      [ROB_DEPTH],
  input logic [CH_W-1:0]       madm_bch      [ROB_DEPTH],
  input logic [N_CH-1:0][SLOT_PTR_W-1:0] mem_head [ROB_DEPTH]
);

`define ASSERT(name, expr) name: assert property (@(posedge aclk) disable iff (!aresetn) (expr))
`define COVER(name, expr)  name: cover  property (@(posedge aclk) disable iff (!aresetn) (expr))
`define ASSUME(name, expr) name: assume property (@(posedge aclk) (expr))
`define NEXT(a, b)         (!$past(aresetn) || !$past(a) || (b))
// what the neighbours (rseg, MC) must guarantee: assumed when the ROB is proven
// alone, PROVEN when rseg is really there (FV_JOIN, the rseg_rrob run)
`ifdef FV_JOIN
`define ENV(name, expr)    name: assert property (@(posedge aclk) disable iff (!aresetn) (expr))
`else
`define ENV(name, expr)    name: assume property (@(posedge aclk) (expr))
`endif

  // ------------------------------------------------------------------------
  // 1. reference model
  // ------------------------------------------------------------------------
  wire admit  = A_ID_VALID & A_ID_READY;
  wire retire = RESP_VALID & RESP_READY;

  wire [AXI_IDW-1:0] admit_id = AW_REQUEST[REQ_META_W-1 -: AXI_IDW];
  wire               admit_ok = AW_REQUEST[0];

  function automatic logic [PTR_W-1:0] ring_inc(input logic [PTR_W-1:0] p);
    return (p == PTR_W'(ROB_DEPTH-1)) ? '0 : p + 1'b1;
  endfunction

  logic [PTR_W-1:0]     fv_tail;    // next free slot (advances at admit)
  logic [PTR_W-1:0]     fv_head;    // oldest slot
  logic [ROB_DEPTH-1:0] fv_live;    // admitted, not yet retired
  logic [ROB_DEPTH-1:0] fv_sealed;  // seen its SEG_LAST

  logic [AXI_IDW-1:0]   fv_id     [ROB_DEPTH];
  logic                 fv_slverr [ROB_DEPTH]; // validation fail or any cmpl_err
  logic [CNT_W-1:0]     fv_npkts  [ROB_DEPTH];
  logic [CNT_W-1:0]     fv_cmpl   [ROB_DEPTH]; // completions seen
  logic [ADM_W-1:0]     fv_adm    [ROB_DEPTH]; // trim meta, known at admit
  logic [N_CH-1:0][SLOT_PTR_W-1:0] fv_hd [ROB_DEPTH]; // head pointers, at SEG_LAST

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      fv_tail   <= '0;
      fv_head   <= '0;
      fv_live   <= '0;
      fv_sealed <= '0;
    end else begin
      // admit: claim the tail slot, record n_pkts (known now), and advance
      if (admit) begin
        fv_live[fv_tail]   <= 1'b1;
        fv_sealed[fv_tail] <= 1'b0;
        fv_id[fv_tail]     <= admit_id;
        fv_slverr[fv_tail] <= ~admit_ok;
        fv_cmpl[fv_tail]   <= '0;
        fv_npkts[fv_tail]  <= ADMIT_NPKTS;
        fv_adm[fv_tail]    <= {ADMIT_LEN, ADMIT_SIZE, ADMIT_ADDR_LO, ADMIT_BASE_CH};
        fv_tail            <= ring_inc(fv_tail);
      end
      // SEG_LAST: seal the slot it names (not the tail); brings only the head pointers
      if (SEG_LAST) begin
        fv_sealed[SEG_ROB_IDX] <= 1'b1;
        fv_hd[SEG_ROB_IDX]     <= SEG_HEAD_PTR;
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
  // 2. assumptions (model state only)
  // ------------------------------------------------------------------------
  logic fv_started = 1'b0;
  always_ff @(posedge aclk) fv_started <= 1'b1;
  `ASSUME(m_reset_first, fv_started || !aresetn);

  // n_pkts ships at admit: a legal count for the request being admitted
  // splitter: a validated request has 1..MAX_PKTS packets, a rejected one none
  `ENV(m_admit_npkts, !admit || (ADMIT_NPKTS <= CNT_W'(MAX_PKTS) && admit_ok == (ADMIT_NPKTS != '0)));

  // one SEG_LAST per slot: it names a live, not-yet-sealed slot (no n_pkts here anymore)
  `ENV(m_seg_last, !SEG_LAST || (
       32'(SEG_ROB_IDX) < ROB_DEPTH
    && fv_live[SEG_ROB_IDX] && !fv_sealed[SEG_ROB_IDX]));

  // a completion targets a live slot and never exceeds its packet count (known at admit)
  `ENV(m_cmpl, !cmpl_valid || (
       32'(cmpl_rob_index) < ROB_DEPTH
    && fv_live[cmpl_rob_index]
    && fv_cmpl[cmpl_rob_index] < fv_npkts[cmpl_rob_index]));

  // ------------------------------------------------------------------------
  // 3a. interface checks
  // ------------------------------------------------------------------------
  // admit side: ready iff not full; a fresh admit lands in a free slot at the tail
  `ASSERT(a_ready,    A_ID_READY == ~(&fv_live));
  `ASSERT(a_no_reuse, !admit || !fv_live[ROB_IDX]);
  `ASSERT(a_idx,      ROB_IDX == fv_tail);

  // retire side: valid exactly when the oldest request is done, in order, carrying its
  // own id / resp / metadata
  `ASSERT(a_resp_valid, RESP_VALID == fv_head_done);
  `ASSERT(a_resp_id,    !RESP_VALID || RESP_ID == fv_id[fv_head]);
  `ASSERT(a_resp_resp,  !RESP_VALID || RESP_RESP == (fv_slverr[fv_head] ? 2'b10 : 2'b00));
  `ASSERT(a_resp_hold,  `NEXT(RESP_VALID && !RESP_READY,
                              RESP_VALID && $stable(RESP_ID) && $stable(RESP_RESP)));
  `ASSERT(a_resp_meta,  !RESP_VALID ||
          ({RESP_LEN, RESP_SIZE, RESP_ADDR_LO, RESP_BASE_CH} == fv_adm[fv_head]
           && RESP_HEAD_PTR == fv_hd[fv_head]));

  // reset: first cycle out of reset the ROB is empty, idle and ready
  `ASSERT(a_reset, $past(aresetn) || (
       alloc_ptr == '0 && retire_ptr == '0 && occ == '0 && fv_live == '0
    && A_ID_READY && !RESP_VALID));

  // back-pressure: a full ring never accepts; a refused admit leaves the ring untouched
  `ASSERT(a_full_bp, !(&fv_live) || !A_ID_READY);
  `ASSERT(a_refuse,  `NEXT(A_ID_VALID && !A_ID_READY && !retire,
                           $stable(alloc_ptr)));

  // ------------------------------------------------------------------------
  // 3b. induction helpers: DUT state == model state
  // ------------------------------------------------------------------------
  // DUT occupancy == number of live slots in the model
  `ASSERT(a_occ,       occ == OCC_W'($countones(fv_live)) && occ <= OCC_W'(ROB_DEPTH));
  `ASSERT(a_ptr_range, 32'(alloc_ptr) < ROB_DEPTH && 32'(retire_ptr) < ROB_DEPTH);
  `ASSERT(a_tail,      alloc_ptr  == fv_tail);
  `ASSERT(a_head,      retire_ptr == fv_head);

  // alloc sits occ slots ahead of retire (mod ROB_DEPTH)
  wire [OCC_W:0] tail_lin  = (OCC_W+1)'(retire_ptr) + (OCC_W+1)'(occ);
  wire [OCC_W:0] tail_wrap = (tail_lin >= (OCC_W+1)'(ROB_DEPTH)) ? tail_lin - (OCC_W+1)'(ROB_DEPTH) : tail_lin;
  `ASSERT(a_ring,      (OCC_W+1)'(alloc_ptr) == tail_wrap);
  `ASSERT(a_alloc_move, `NEXT(!admit,  $stable(alloc_ptr)));
  `ASSERT(a_ret_move,   `NEXT(!retire, $stable(retire_ptr)));

  // the live slots are exactly the occ slots starting at retire_ptr
  for (genvar i = 0; i < ROB_DEPTH; i++) begin : g_slot
    wire [OCC_W-1:0] age = (i >= retire_ptr) ? OCC_W'(i) - OCC_W'(retire_ptr)
                                             : OCC_W'(i) + OCC_W'(ROB_DEPTH) - OCC_W'(retire_ptr);
    wire live   = fv_live[i];
    wire sealed = fv_sealed[i];

    `ASSERT(a_live,   live == (age < occ));
    `ASSERT(a_id,     !live || ent_id[i] == fv_id[i]);
    `ASSERT(a_err,    !live || (ent_err[i] | ent_derr[i]) == fv_slverr[i]);
    `ASSERT(a_inflow, !live || inflow_complt[i] == sealed);
    `ASSERT(a_cnt,    !live || outflow_cnt[i] == fv_cmpl[i]);
    `ASSERT(a_npkts,  !live || (n_pkts[i] == fv_npkts[i]
                      && fv_npkts[i] <= CNT_W'(MAX_PKTS)));
    `ASSERT(a_bound,  !live || fv_cmpl[i] <= fv_npkts[i]);
    `ASSERT(a_adm,    !live || {madm_len[i], madm_size[i], madm_alo[i], madm_bch[i]} == fv_adm[i]);
    `ASSERT(a_head,   !(live && sealed) || mem_head[i] == fv_hd[i]);
  end

  // ------------------------------------------------------------------------
  // covers
  // ------------------------------------------------------------------------
  `COVER(c_retire,     retire);
  `COVER(c_full,       &fv_live);
  `COVER(c_slverr,     retire && RESP_RESP == 2'b10);
  `COVER(c_reject,     retire && fv_npkts[fv_head] == '0);   // rejected request retires, no packets
  `COVER(c_okay_max,   retire && RESP_RESP == 2'b00 && fv_npkts[fv_head] == CNT_W'(MAX_PKTS));
  `COVER(c_wrap,       retire && retire_ptr == PTR_W'(ROB_DEPTH-1));
  `COVER(c_early_cmpl, SEG_LAST && fv_cmpl[SEG_ROB_IDX] != 0);
  // multi-outstanding: two slots live-but-unsealed at once (several mid-split)
  `COVER(c_two_split,  $countones(fv_live & ~fv_sealed) >= 2);
  // out-of-tail seal: SEG_LAST names a slot that is not the tail (pipelined splitter)
  `COVER(c_seal_old,   SEG_LAST && SEG_ROB_IDX != fv_tail);

`ifdef FV_LIVE
  // ------------------------------------------------------------------------
  // bounded liveness (d4_live task only: the fairness assumptions narrow the
  // inputs, so they are never compiled into the safety proofs)
  // ------------------------------------------------------------------------
  // a response never waits longer than RESP_READY may stay low
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

  // the oldest request reaches RESP_VALID within FV_HEAD_MAX cycles, given: rseg
  // closes the oldest request within FV_SEG_MAX cycles of it becoming the oldest,
  // and MC returns a completion for it at least every FV_CMPL_MAX cycles.
  localparam int FV_SEG_MAX  = 3;
  localparam int FV_CMPL_MAX = 3;
  localparam int FV_HEAD_MAX = (FV_SEG_MAX + 1) + (FV_CMPL_MAX + 1) * MAX_PKTS;

  wire fv_head_live   = fv_live[fv_head];
  wire fv_head_sealed = fv_sealed[fv_head];
  wire fv_head_open   = fv_head_live & ~fv_head_sealed;         // still with rseg
  wire fv_head_short  = fv_head_live & fv_head_sealed           // still owed completions
                      & (fv_cmpl[fv_head] != fv_npkts[fv_head]);
  wire fv_head_seal   = SEG_LAST   & (SEG_ROB_IDX    == fv_head);
  wire fv_head_hit    = cmpl_valid & (cmpl_rob_index == fv_head);

  logic [15:0] fv_seg_wait;   // cycles in a row the oldest waited for its SEG_LAST
  logic [15:0] fv_cmpl_wait;  // cycles in a row the oldest waited for a completion
  logic [15:0] fv_head_age;   // cycles in a row the oldest was live without RESP_VALID
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      fv_seg_wait  <= '0;
      fv_cmpl_wait <= '0;
      fv_head_age  <= '0;
    end else begin
      fv_seg_wait  <= (fv_head_open  && !fv_head_seal) ? fv_seg_wait  + 1'b1 : '0;
      fv_cmpl_wait <= (fv_head_short && !fv_head_hit)  ? fv_cmpl_wait + 1'b1 : '0;
      fv_head_age  <= (fv_head_live  && !RESP_VALID)   ? fv_head_age  + 1'b1 : '0;
    end
  end

  `ASSUME(m_seg_fair,  !aresetn || !fv_head_open  || fv_seg_wait  < FV_SEG_MAX  || fv_head_seal);
  `ASSUME(m_cmpl_fair, !aresetn || !fv_head_short || fv_cmpl_wait < FV_CMPL_MAX || fv_head_hit);

  `ASSERT(a_head_live, fv_head_age <= FV_HEAD_MAX);

  // helpers: the age is bounded by the fairness budget already spent
  `ASSERT(a_fair_range, fv_seg_wait <= FV_SEG_MAX && fv_cmpl_wait <= FV_CMPL_MAX);
  `ASSERT(a_age_split,  !fv_head_open || fv_head_age <= fv_seg_wait);
  `ASSERT(a_age_cmpl,   !(fv_head_live && fv_head_sealed) || fv_head_age <=
                        16'(FV_SEG_MAX + 1) + 16'(FV_CMPL_MAX + 1) * 16'(fv_cmpl[fv_head])
                        + fv_cmpl_wait);

  `COVER(c_head_age_max, fv_head_age == FV_HEAD_MAX);
`endif

`undef ASSERT
`undef COVER
`undef ASSUME
`undef NEXT
`undef ENV

endmodule : rmc_cif_rrob_fv
