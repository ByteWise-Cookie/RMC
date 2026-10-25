// rmc_cif_rrob_fv.sv
// Formal checker for the MULTI-OUTSTANDING rmc_cif_rrob. Bound by rmc_cif_rrob_bind.sv.
//
// Differs from the old one-at-a-time rob checker:
//   - admit advances the tail (alloc_ptr) immediately; many requests may be mid-split.
//   - there is no pending_split; A_ID_READY = ~rob_full.
//   - SEG_LAST is indexed by SEG_ROB_IDX (seals THAT slot), not the tail; it does not
//     move the tail.
//   - completions are PER CHANNEL (one resp FIFO per channel): two channels can complete the
//     same entry the same cycle, so outflow + the target (npkts_ch) are per channel, and
//     retire needs every channel matched. npkts_ch doubles as RESP_NPKTS_CH (the RD-SRAM free).
//
// Three layers:
//   1. reference model : an independent ring + per-slot, per-channel scoreboard from the ports
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
  input logic [N_CH-1:0]                cmpl_valid,
  input logic [N_CH-1:0][ROB_IDX_W-1:0] cmpl_rob_index,
  input logic [N_CH-1:0]                cmpl_err,
  input logic [REQ_META_W-1:0] AW_REQUEST,
  input logic [CNT_W-1:0]      ADMIT_NPKTS,
  input logic [N_CH-1:0][CNT_W-1:0] ADMIT_NPKTS_CH,
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
  input logic [N_CH-1:0][CNT_W-1:0] RESP_NPKTS_CH,
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
  input logic [N_CH-1:0][CNT_W-1:0] npkts_ch [ROB_DEPTH],
  input logic [N_CH-1:0][CNT_W-1:0] outflow_cnt [ROB_DEPTH],
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
`ifdef FV_JOIN
`define ENV(name, expr)    name: assert property (@(posedge aclk) disable iff (!aresetn) (expr))
`else
`define ENV(name, expr)    name: assume property (@(posedge aclk) (expr))
`endif

  localparam int NCH = rmc_cfg_pkg::N_CH;

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

  logic [AXI_IDW-1:0]        fv_id     [ROB_DEPTH];
  logic                      fv_slverr [ROB_DEPTH]; // validation fail or any cmpl_err
  logic [CNT_W-1:0]          fv_npkts  [ROB_DEPTH]; // total (informational)
  logic [NCH-1:0][CNT_W-1:0] fv_run    [ROB_DEPTH]; // per-channel target
  logic [NCH-1:0][CNT_W-1:0] fv_cmpl   [ROB_DEPTH]; // completions seen, per channel
  logic [ADM_W-1:0]          fv_adm    [ROB_DEPTH]; // trim meta, known at admit
  logic [NCH-1:0][SLOT_PTR_W-1:0] fv_hd [ROB_DEPTH]; // head pointers, at SEG_LAST

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      fv_tail   <= '0;
      fv_head   <= '0;
      fv_live   <= '0;
      fv_sealed <= '0;
    end else begin
      if (admit) begin
        fv_live[fv_tail]   <= 1'b1;
        fv_sealed[fv_tail] <= 1'b0;
        fv_id[fv_tail]     <= admit_id;
        fv_slverr[fv_tail] <= ~admit_ok;
        fv_cmpl[fv_tail]   <= '0;
        fv_npkts[fv_tail]  <= ADMIT_NPKTS;
        fv_run[fv_tail]    <= ADMIT_NPKTS_CH;
        fv_adm[fv_tail]    <= {ADMIT_LEN, ADMIT_SIZE, ADMIT_ADDR_LO, ADMIT_BASE_CH};
        fv_tail            <= ring_inc(fv_tail);
      end
      if (SEG_LAST) begin
        fv_sealed[SEG_ROB_IDX] <= 1'b1;
        fv_hd[SEG_ROB_IDX]     <= SEG_HEAD_PTR;
      end
      for (int c = 0; c < NCH; c++)
        if (cmpl_valid[c]) begin
          fv_cmpl[cmpl_rob_index[c]][c] <= fv_cmpl[cmpl_rob_index[c]][c] + 1'b1;
          if (cmpl_err[c]) fv_slverr[cmpl_rob_index[c]] <= 1'b1;
        end
      if (retire) begin
        fv_live[fv_head] <= 1'b0;
        fv_head          <= ring_inc(fv_head);
      end
    end
  end

  // head done: sealed and every channel matched its per-channel target
  logic head_match;
  always_comb begin
    head_match = 1'b1;
    for (int c = 0; c < NCH; c++)
      if (fv_cmpl[fv_head][c] != fv_run[fv_head][c]) head_match = 1'b0;
  end
  wire fv_head_done = fv_live[fv_head] & fv_sealed[fv_head] & head_match;

  // per-channel sum of the admitted counts (must equal the total)
  logic [CNT_W+NCH:0] adm_ch_sum;
  always_comb begin
    adm_ch_sum = '0;
    for (int c = 0; c < NCH; c++) adm_ch_sum = adm_ch_sum + (CNT_W+NCH+1)'(ADMIT_NPKTS_CH[c]);
  end

  // ------------------------------------------------------------------------
  // 2. assumptions (model state only)
  // ------------------------------------------------------------------------
  logic fv_started = 1'b0;
  always_ff @(posedge aclk) fv_started <= 1'b1;
  `ASSUME(m_reset_first, fv_started || !aresetn);

  // splitter: a validated request has 1..MAX_PKTS packets (rejected none); per-channel adds up
  `ENV(m_admit_npkts, !admit || (ADMIT_NPKTS <= CNT_W'(MAX_PKTS) && admit_ok == (ADMIT_NPKTS != '0)
                                 && adm_ch_sum == (CNT_W+NCH+1)'(ADMIT_NPKTS)));

  // one SEG_LAST per slot: it names a live, not-yet-sealed slot
  `ENV(m_seg_last, !SEG_LAST || (
       32'(SEG_ROB_IDX) < ROB_DEPTH
    && fv_live[SEG_ROB_IDX] && !fv_sealed[SEG_ROB_IDX]));

  // each channel's completion targets a live slot and never exceeds its per-channel count
  logic cmpl_ok;
  always_comb begin
    cmpl_ok = 1'b1;
    for (int c = 0; c < NCH; c++)
      if (cmpl_valid[c] &&
          !(32'(cmpl_rob_index[c]) < ROB_DEPTH
            && fv_live[cmpl_rob_index[c]]
            && fv_cmpl[cmpl_rob_index[c]][c] < fv_run[cmpl_rob_index[c]][c]))
        cmpl_ok = 1'b0;
  end
  `ENV(m_cmpl, cmpl_ok);

  // ------------------------------------------------------------------------
  // 3a. interface checks
  // ------------------------------------------------------------------------
  `ASSERT(a_ready,    A_ID_READY == ~(&fv_live));
  `ASSERT(a_no_reuse, !admit || !fv_live[ROB_IDX]);
  `ASSERT(a_idx,      ROB_IDX == fv_tail);

  `ASSERT(a_resp_valid, RESP_VALID == fv_head_done);
  `ASSERT(a_resp_id,    !RESP_VALID || RESP_ID == fv_id[fv_head]);
  `ASSERT(a_resp_resp,  !RESP_VALID || RESP_RESP == (fv_slverr[fv_head] ? 2'b10 : 2'b00));
  `ASSERT(a_resp_hold,  `NEXT(RESP_VALID && !RESP_READY,
                              RESP_VALID && $stable(RESP_ID) && $stable(RESP_RESP)));
  `ASSERT(a_resp_meta,  !RESP_VALID ||
          ({RESP_LEN, RESP_SIZE, RESP_ADDR_LO, RESP_BASE_CH} == fv_adm[fv_head]
           && RESP_HEAD_PTR == fv_hd[fv_head] && RESP_NPKTS_CH == fv_run[fv_head]));

  `ASSERT(a_reset, $past(aresetn) || (
       alloc_ptr == '0 && retire_ptr == '0 && occ == '0 && fv_live == '0
    && A_ID_READY && !RESP_VALID));

  `ASSERT(a_full_bp, !(&fv_live) || !A_ID_READY);
  `ASSERT(a_refuse,  `NEXT(A_ID_VALID && !A_ID_READY && !retire,
                           $stable(alloc_ptr)));

  // ------------------------------------------------------------------------
  // 3b. induction helpers: DUT state == model state
  // ------------------------------------------------------------------------
  `ASSERT(a_occ,       occ == OCC_W'($countones(fv_live)) && occ <= OCC_W'(ROB_DEPTH));
  `ASSERT(a_ptr_range, 32'(alloc_ptr) < ROB_DEPTH && 32'(retire_ptr) < ROB_DEPTH);
  `ASSERT(a_tail,      alloc_ptr  == fv_tail);
  `ASSERT(a_headp,     retire_ptr == fv_head);

  wire [OCC_W:0] tail_lin  = (OCC_W+1)'(retire_ptr) + (OCC_W+1)'(occ);
  wire [OCC_W:0] tail_wrap = (tail_lin >= (OCC_W+1)'(ROB_DEPTH)) ? tail_lin - (OCC_W+1)'(ROB_DEPTH) : tail_lin;
  `ASSERT(a_ring,      (OCC_W+1)'(alloc_ptr) == tail_wrap);
  `ASSERT(a_alloc_move, `NEXT(!admit,  $stable(alloc_ptr)));
  `ASSERT(a_ret_move,   `NEXT(!retire, $stable(retire_ptr)));

  for (genvar i = 0; i < ROB_DEPTH; i++) begin : g_slot
    wire [OCC_W-1:0] age = (i >= retire_ptr) ? OCC_W'(i) - OCC_W'(retire_ptr)
                                             : OCC_W'(i) + OCC_W'(ROB_DEPTH) - OCC_W'(retire_ptr);
    wire live   = fv_live[i];
    wire sealed = fv_sealed[i];
    logic cnt_eq, cnt_le;
    logic [CNT_W+NCH:0] run_sum;
    always_comb begin
      cnt_eq = 1'b1; cnt_le = 1'b1; run_sum = '0;
      for (int c = 0; c < NCH; c++) begin
        if (outflow_cnt[i][c] != fv_cmpl[i][c]) cnt_eq = 1'b0;
        if (fv_cmpl[i][c]     >  fv_run[i][c])   cnt_le = 1'b0;
        run_sum = run_sum + (CNT_W+NCH+1)'(fv_run[i][c]);
      end
    end

    `ASSERT(a_live,   live == (age < occ));
    `ASSERT(a_id,     !live || ent_id[i] == fv_id[i]);
    `ASSERT(a_err,    !live || (ent_err[i] | ent_derr[i]) == fv_slverr[i]);
    `ASSERT(a_inflow, !live || inflow_complt[i] == sealed);
    `ASSERT(a_cnt,    !live || cnt_eq);
    `ASSERT(a_npkts,  !live || (n_pkts[i] == fv_npkts[i] && fv_npkts[i] <= CNT_W'(MAX_PKTS)
                      && npkts_ch[i] == fv_run[i]));
    `ASSERT(a_bound,  !live || cnt_le);
    `ASSERT(a_run_sum, !live || run_sum == (CNT_W+NCH+1)'(fv_npkts[i]));
    `ASSERT(a_adm,    !live || {madm_len[i], madm_size[i], madm_alo[i], madm_bch[i]} == fv_adm[i]);
    `ASSERT(a_head,   !(live && sealed) || mem_head[i] == fv_hd[i]);
  end

  // ------------------------------------------------------------------------
  // covers
  // ------------------------------------------------------------------------
  `COVER(c_retire,     retire);
  `COVER(c_full,       &fv_live);
  `COVER(c_slverr,     retire && RESP_RESP == 2'b10);
  `COVER(c_reject,     retire && fv_npkts[fv_head] == '0);
  `COVER(c_okay_max,   retire && RESP_RESP == 2'b00 && fv_npkts[fv_head] == CNT_W'(MAX_PKTS));
  `COVER(c_wrap,       retire && retire_ptr == PTR_W'(ROB_DEPTH-1));
  `COVER(c_bothch,     cmpl_valid == '1);
  `COVER(c_two_split,  $countones(fv_live & ~fv_sealed) >= 2);
  `COVER(c_seal_old,   SEG_LAST && SEG_ROB_IDX != fv_tail);

`ifdef FV_LIVE
  // ------------------------------------------------------------------------
  // bounded liveness (d4_live task only)
  // ------------------------------------------------------------------------
  localparam int FV_RDY_MAX = 4;
  logic [3:0] fv_rdy_low;
  logic [3:0] fv_wait;
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

  localparam int FV_SEG_MAX  = 3;
  localparam int FV_CMPL_MAX = 3;
  localparam int FV_HEAD_MAX = (FV_SEG_MAX + 1) + (FV_CMPL_MAX + 1) * MAX_PKTS;

  // total completions seen by the head, summed over channels
  logic [CNT_W+NCH:0] head_cmpl_tot;
  always_comb begin
    head_cmpl_tot = '0;
    for (int c = 0; c < NCH; c++) head_cmpl_tot = head_cmpl_tot + (CNT_W+NCH+1)'(fv_cmpl[fv_head][c]);
  end

  wire fv_head_live   = fv_live[fv_head];
  wire fv_head_sealed = fv_sealed[fv_head];
  wire fv_head_open   = fv_head_live & ~fv_head_sealed;
  wire fv_head_short  = fv_head_live & fv_head_sealed & ~head_match;  // still owed on some channel
  wire fv_head_seal   = SEG_LAST & (SEG_ROB_IDX == fv_head);
  logic fv_head_hit;                                                  // some channel completes head
  always_comb begin
    fv_head_hit = 1'b0;
    for (int c = 0; c < NCH; c++) if (cmpl_valid[c] && cmpl_rob_index[c] == fv_head) fv_head_hit = 1'b1;
  end

  logic [15:0] fv_seg_wait, fv_cmpl_wait, fv_head_age;
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
  `ASSERT(a_fair_range, fv_seg_wait <= FV_SEG_MAX && fv_cmpl_wait <= FV_CMPL_MAX);
  `ASSERT(a_age_split,  !fv_head_open || fv_head_age <= fv_seg_wait);
  `ASSERT(a_age_cmpl,   !(fv_head_live && fv_head_sealed) || fv_head_age <=
                        16'(FV_SEG_MAX + 1) + 16'(FV_CMPL_MAX + 1) * 16'(head_cmpl_tot)
                        + fv_cmpl_wait);

  `COVER(c_head_age_max, fv_head_age == FV_HEAD_MAX);
`endif

`undef ASSERT
`undef COVER
`undef ASSUME
`undef NEXT
`undef ENV

endmodule : rmc_cif_rrob_fv
