// rmc_cif_wrob_fv.sv
// Formal checker for rmc_cif_wrob (write ROB). Bound by rmc_cif_wrob_bind.sv.
//
// The splitter admits in two steps: ADMIT at request arrival (reserve the slot, write {id,
// resp}, clear its completion state, advance alloc_ptr) and SEAL at the last packet (write the
// per-channel target, set inflow). Completions are PER CHANNEL and may beat the seal, so they
// count on the already-entered slot. Retire in order when sealed and every channel reached its
// per-channel target; a back-end error forces SLVERR, else the stored front BRESP.
//
// Layers: 1. reference model (ring + per-slot, per-channel scoreboard, from the ports)
//         2. assumptions (splitter + MC guarantees)   3. assertions (outputs + state match)
import rmc_cfg_pkg::*;
module rmc_cif_wrob_fv #(
  parameter int ROB_DEPTH = 32,
  parameter int MAX_PKTS  = 64,
  parameter int AXI_IDW   = 8,
  parameter int ROB_IDX_W = 5,
  parameter int PTR_W     = 5,
  parameter int CNT_W     = 7,
  parameter int OCC_W     = 6
)(
  input logic                           aclk,
  input logic                           aresetn,
  input logic [N_CH-1:0]                cmpl_valid,
  input logic [N_CH-1:0][ROB_IDX_W-1:0] cmpl_rob_index,
  input logic [N_CH-1:0]                cmpl_err,
  input logic [ROB_IDX_W-1:0]           ROB_IDX,
  input logic                           rob_free,
  input logic                           admit_valid,
  input logic [AXI_IDW-1:0]             admit_id,
  input logic [1:0]                     admit_resp,
  input logic                           SEG_LAST,
  input logic [ROB_IDX_W-1:0]           SEG_ROB_IDX,
  input logic [N_CH-1:0][CNT_W-1:0]     SEG_NPKTS_CH,
  input logic                           RESP_VALID,
  input logic                           RESP_READY,
  input logic [AXI_IDW-1:0]             RESP_ID,
  input logic [1:0]                     RESP_RESP,
  input logic [N_CH-1:0][CNT_W-1:0]     RESP_NPKTS_CH,
  // internals
  input logic [PTR_W-1:0]               alloc_ptr,
  input logic [PTR_W-1:0]               retire_ptr,
  input logic [OCC_W-1:0]               occ,
  input logic [1:0]                     ent_resp      [ROB_DEPTH],
  input logic                           ent_derr      [ROB_DEPTH],
  input logic                           inflow_complt [ROB_DEPTH],
  input logic [N_CH-1:0][CNT_W-1:0]     run_cnt       [ROB_DEPTH],
  input logic [N_CH-1:0][CNT_W-1:0]     outflow_cnt   [ROB_DEPTH]
);
`define ASSERT(name, expr) name: assert property (@(posedge aclk) disable iff (!aresetn) (expr))
`define COVER(name, expr)  name: cover  property (@(posedge aclk) disable iff (!aresetn) (expr))
`define ASSUME(name, expr) name: assume property (@(posedge aclk) (expr))
`define NEXT(a, b)         (!$past(aresetn) || !$past(a) || (b))

  localparam int NCH = rmc_cfg_pkg::N_CH;

  // 1. reference model
  wire admit  = admit_valid & rob_free;
  wire retire = RESP_VALID & RESP_READY;

  function automatic logic [PTR_W-1:0] ring_inc(input logic [PTR_W-1:0] p);
    return (p == PTR_W'(ROB_DEPTH-1)) ? '0 : p + 1'b1;
  endfunction

  logic [PTR_W-1:0]          fv_tail, fv_head;
  logic [ROB_DEPTH-1:0]      fv_live;
  logic [ROB_DEPTH-1:0]      fv_sealed;
  logic [AXI_IDW-1:0]        fv_id   [ROB_DEPTH];
  logic [1:0]                fv_resp [ROB_DEPTH];
  logic [NCH-1:0][CNT_W-1:0] fv_run  [ROB_DEPTH];
  logic [NCH-1:0][CNT_W-1:0] fv_cmpl [ROB_DEPTH];
  logic [ROB_DEPTH-1:0]      fv_derr;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      fv_tail <= '0; fv_head <= '0; fv_live <= '0; fv_sealed <= '0; fv_derr <= '0;
      for (int i = 0; i < ROB_DEPTH; i++) fv_cmpl[i] <= '0;
    end else begin
      if (admit) begin
        fv_live[fv_tail]   <= 1'b1;
        fv_sealed[fv_tail] <= 1'b0;
        fv_id[fv_tail]     <= admit_id;
        fv_resp[fv_tail]   <= admit_resp;
        fv_cmpl[fv_tail]   <= '0;
        fv_derr[fv_tail]   <= 1'b0;
        fv_tail            <= ring_inc(fv_tail);
      end
      if (SEG_LAST) begin
        fv_sealed[SEG_ROB_IDX] <= 1'b1;
        fv_run[SEG_ROB_IDX]    <= SEG_NPKTS_CH;
      end
      for (int c = 0; c < NCH; c++)
        if (cmpl_valid[c]) begin
          fv_cmpl[cmpl_rob_index[c]][c] <= fv_cmpl[cmpl_rob_index[c]][c] + 1'b1;
          if (cmpl_err[c]) fv_derr[cmpl_rob_index[c]] <= 1'b1;
        end
      if (retire) begin
        fv_live[fv_head] <= 1'b0;
        fv_head          <= ring_inc(fv_head);
      end
    end
  end

  logic head_match;
  always_comb begin
    head_match = 1'b1;
    for (int c = 0; c < NCH; c++)
      if (fv_cmpl[fv_head][c] != fv_run[fv_head][c]) head_match = 1'b0;
  end
  wire fv_head_done = fv_live[fv_head] & fv_sealed[fv_head] & head_match;

  // 2. assumptions
  logic fv_started = 1'b0;
  always_ff @(posedge aclk) fv_started <= 1'b1;
  `ASSUME(m_reset_first, fv_started || !aresetn);

  // one SEAL per slot: names a live, not-yet-sealed slot; its counts cover completions so far
  logic seal_cov;
  always_comb begin
    seal_cov = 1'b1;
    for (int c = 0; c < NCH; c++)
      if (32'(SEG_NPKTS_CH[c]) <
          32'(fv_cmpl[SEG_ROB_IDX][c]) + ((cmpl_valid[c] && cmpl_rob_index[c] == SEG_ROB_IDX) ? 1 : 0))
        seal_cov = 1'b0;
  end
  `ASSUME(m_seal, !SEG_LAST || (32'(SEG_ROB_IDX) < ROB_DEPTH
    && fv_live[SEG_ROB_IDX] && !fv_sealed[SEG_ROB_IDX] && seal_cov));

  // each channel's completion targets a live slot; below its target once sealed, below MAX before
  logic cmpl_ok;
  always_comb begin
    cmpl_ok = 1'b1;
    for (int c = 0; c < NCH; c++)
      if (cmpl_valid[c] &&
          !(32'(cmpl_rob_index[c]) < ROB_DEPTH && fv_live[cmpl_rob_index[c]]
            && (fv_sealed[cmpl_rob_index[c]]
                  ? fv_cmpl[cmpl_rob_index[c]][c] <  fv_run[cmpl_rob_index[c]][c]
                  : 32'(fv_cmpl[cmpl_rob_index[c]][c]) < MAX_PKTS)))
        cmpl_ok = 1'b0;
  end
  `ASSUME(m_cmpl, cmpl_ok);

  // 3a. interface
  `ASSERT(a_rob_idx,   ROB_IDX == fv_tail);
  `ASSERT(a_rob_free,  rob_free == ~(&fv_live));
  `ASSERT(a_resp_valid, RESP_VALID == fv_head_done);
  `ASSERT(a_resp_id,    !RESP_VALID || RESP_ID == fv_id[fv_head]);
  `ASSERT(a_resp_resp,  !RESP_VALID || RESP_RESP == (fv_derr[fv_head] ? 2'b10 : fv_resp[fv_head]));
  `ASSERT(a_resp_run,   !RESP_VALID || RESP_NPKTS_CH == fv_run[fv_head]);
  `ASSERT(a_resp_hold,  `NEXT(RESP_VALID && !RESP_READY, RESP_VALID
                              && $stable(RESP_ID) && $stable(RESP_RESP) && $stable(RESP_NPKTS_CH)));
  `ASSERT(a_reset,      $past(aresetn) || (occ == '0 && !RESP_VALID && rob_free && fv_live == '0));

  // 3b. induction: DUT state == model
  `ASSERT(a_occ,   occ == OCC_W'($countones(fv_live)) && occ <= OCC_W'(ROB_DEPTH));
  `ASSERT(a_tail,  alloc_ptr  == fv_tail);
  `ASSERT(a_head,  retire_ptr == fv_head);
  wire [OCC_W:0] tail_lin  = (OCC_W+1)'(retire_ptr) + (OCC_W+1)'(occ);
  `ASSERT(a_ring,  (OCC_W+1)'(alloc_ptr) ==
                   ((tail_lin >= (OCC_W+1)'(ROB_DEPTH)) ? tail_lin - (OCC_W+1)'(ROB_DEPTH) : tail_lin));
  `ASSERT(a_ptr_range, 32'(alloc_ptr) < ROB_DEPTH && 32'(retire_ptr) < ROB_DEPTH);
  for (genvar i = 0; i < ROB_DEPTH; i++) begin : g_slot
    wire [OCC_W-1:0] age = (i >= retire_ptr) ? OCC_W'(i) - OCC_W'(retire_ptr)
                                             : OCC_W'(i) + OCC_W'(ROB_DEPTH) - OCC_W'(retire_ptr);
    wire live = fv_live[i];
    logic cnt_eq, cnt_le;
    always_comb begin
      cnt_eq = 1'b1; cnt_le = 1'b1;
      for (int c = 0; c < NCH; c++) begin
        if (outflow_cnt[i][c] != fv_cmpl[i][c]) cnt_eq = 1'b0;
        if (fv_sealed[i] && fv_cmpl[i][c] > fv_run[i][c]) cnt_le = 1'b0;
      end
    end
    `ASSERT(a_live,   live == (age < occ));
    `ASSERT(a_inflow, !live || inflow_complt[i] == fv_sealed[i]);
    `ASSERT(a_cnt,    !live || cnt_eq);
    `ASSERT(a_derr,   !live || ent_derr[i] == fv_derr[i]);
    `ASSERT(a_bound,  !live || cnt_le);
    `ASSERT(a_ent,    !live || (rmc_cif_wrob.ent_id[i] == fv_id[i] && ent_resp[i] == fv_resp[i]
                                && (!fv_sealed[i] || run_cnt[i] == fv_run[i])));
  end

  // covers
  `COVER(c_retire,   retire);
  `COVER(c_full,     &fv_live);
  `COVER(c_slverr,   retire && RESP_RESP == 2'b10);
  `COVER(c_decerr,   retire && RESP_RESP == 2'b11);
  `COVER(c_early,    SEG_LAST && fv_cmpl[SEG_ROB_IDX] != '0); // completions before the seal
  `COVER(c_bothch,   cmpl_valid == '1);
  `COVER(c_wrap,     retire && retire_ptr == PTR_W'(ROB_DEPTH-1));

`undef ASSERT
`undef COVER
`undef ASSUME
`undef NEXT
endmodule : rmc_cif_wrob_fv
