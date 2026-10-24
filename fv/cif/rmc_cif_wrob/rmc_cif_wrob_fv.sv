// rmc_cif_wrob_fv.sv
// Formal checker for rmc_cif_wrob (write ROB). Bound by rmc_cif_wrob_bind.sv.
//
// How the write ROB is used (w_burst_spltr is its only writer):
//   - the splitter tags every packet of a request with rob_idx = alloc_ptr (the TAIL slot)
//     and writes the entry ONCE, at the end of the request (entry_valid), with its per-channel
//     packet counts. So completions for the tail slot can arrive BEFORE its entry is written.
//   - completions are PER CHANNEL (one resp FIFO per channel): two channels can complete the
//     same entry the same cycle, so outflow is counted per channel.
//   - retire in order when every channel of the oldest entry has seen its per-channel count of
//     completions; B carries the front BRESP, or SLVERR if any completion reported an error.
//
// Layers:
//   1. reference model : ring + per-slot, per-channel scoreboard, from the ports only
//   2. assumptions     : what the splitter and the MC guarantee
//   3. assertions      : B response / ring outputs match the model; DUT state == model
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
  input logic [ROB_IDX_W-1:0]           rob_idx,
  input logic                           rob_free,
  input logic                           entry_valid,
  input logic [AXI_IDW-1:0]             entry_id,
  input logic [1:0]                     entry_resp,
  input logic [CNT_W-1:0]               entry_npkts,
  input logic                           RESP_VALID,
  input logic                           RESP_READY,
  input logic [AXI_IDW-1:0]             RESP_ID,
  input logic [1:0]                     RESP_RESP,
  // internals
  input logic [PTR_W-1:0]               alloc_ptr,
  input logic [PTR_W-1:0]               retire_ptr,
  input logic [OCC_W-1:0]               occ,
  input logic [1:0]                     ent_resp      [ROB_DEPTH],
  input logic                           ent_derr      [ROB_DEPTH],
  input logic                           inflow_complt [ROB_DEPTH],
  input logic [CNT_W-1:0]               n_pkts        [ROB_DEPTH],
  input logic [N_CH-1:0][CNT_W-1:0]     outflow_cnt   [ROB_DEPTH]
);

`define ASSERT(name, expr) name: assert property (@(posedge aclk) disable iff (!aresetn) (expr))
`define COVER(name, expr)  name: cover  property (@(posedge aclk) disable iff (!aresetn) (expr))
`define ASSUME(name, expr) name: assume property (@(posedge aclk) (expr))
`define NEXT(a, b)         (!$past(aresetn) || !$past(a) || (b))

  localparam int NCH = rmc_cfg_pkg::N_CH;
  wire [NCH-1:0][CNT_W-1:0] entry_npkts_ch = rmc_cif_wrob.entry_npkts_ch;
  wire [NCH-1:0][CNT_W-1:0] RESP_NPKTS_CH  = rmc_cif_wrob.RESP_NPKTS_CH;

  // ------------------------------------------------------------------------
  // 1. reference model
  // ------------------------------------------------------------------------
  wire retire = RESP_VALID & RESP_READY;

  function automatic logic [PTR_W-1:0] ring_inc(input logic [PTR_W-1:0] p);
    return (p == PTR_W'(ROB_DEPTH-1)) ? '0 : p + 1'b1;
  endfunction

  logic [PTR_W-1:0]          fv_tail, fv_head;
  logic [ROB_DEPTH-1:0]      fv_live;             // entry written, not yet retired
  logic [AXI_IDW-1:0]        fv_id    [ROB_DEPTH];
  logic [1:0]                fv_resp  [ROB_DEPTH];
  logic [CNT_W-1:0]          fv_npkts [ROB_DEPTH];
  logic [NCH-1:0][CNT_W-1:0] fv_run   [ROB_DEPTH];
  logic [NCH-1:0][CNT_W-1:0] fv_cmpl  [ROB_DEPTH];   // completions seen per channel
  logic [ROB_DEPTH-1:0]      fv_derr;                // a completion reported an error

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      fv_tail <= '0;
      fv_head <= '0;
      fv_live <= '0;
      fv_derr <= '0;
      for (int i = 0; i < ROB_DEPTH; i++) fv_cmpl[i] <= '0;
    end else begin
      for (int c = 0; c < NCH; c++)
        if (cmpl_valid[c]) begin
          fv_cmpl[cmpl_rob_index[c]][c] <= fv_cmpl[cmpl_rob_index[c]][c] + 1'b1;
          if (cmpl_err[c]) fv_derr[cmpl_rob_index[c]] <= 1'b1;
        end
      if (entry_valid) begin
        fv_live[fv_tail]  <= 1'b1;
        fv_id[fv_tail]    <= entry_id;
        fv_resp[fv_tail]  <= entry_resp;
        fv_npkts[fv_tail] <= entry_npkts;
        fv_run[fv_tail]   <= entry_npkts_ch;
        fv_tail           <= ring_inc(fv_tail);
      end
      if (retire) begin
        fv_live[fv_head] <= 1'b0;
        fv_cmpl[fv_head] <= '0;              // slot empty again for its next request
        fv_derr[fv_head] <= 1'b0;
        fv_head          <= ring_inc(fv_head);
      end
    end
  end

  wire [OCC_W-1:0] fv_occ  = OCC_W'($countones(fv_live));
  wire             fv_full = (fv_occ == OCC_W'(ROB_DEPTH));
  // every channel of the head entry has reached its per-channel target
  logic head_match;
  always_comb begin
    head_match = 1'b1;
    for (int c = 0; c < NCH; c++)
      if (fv_cmpl[fv_head][c] != fv_run[fv_head][c]) head_match = 1'b0;
  end
  wire head_done = fv_live[fv_head] & head_match;

  // ------------------------------------------------------------------------
  // 2. assumptions
  // ------------------------------------------------------------------------
  logic fv_started = 1'b0;
  always_ff @(posedge aclk) fv_started <= 1'b1;
  `ASSUME(m_reset_first, fv_started || !aresetn);

  // splitter: it writes an entry only into a free slot, with legal per-channel counts that add
  // up to the total and already cover the completions that came back for that slot
  logic [CNT_W+NCH:0] ch_sum;
  always_comb begin
    ch_sum = '0;
    for (int c = 0; c < NCH; c++) ch_sum = ch_sum + (CNT_W+NCH+1)'(entry_npkts_ch[c]);
  end
  logic entry_cov;
  always_comb begin
    entry_cov = 1'b1;
    for (int c = 0; c < NCH; c++)
      if (32'(entry_npkts_ch[c]) <
          32'(fv_cmpl[fv_tail][c]) + ((cmpl_valid[c] && cmpl_rob_index[c] == fv_tail) ? 1 : 0))
        entry_cov = 1'b0;
  end
  `ASSUME(m_entry, !entry_valid || (!fv_full
    && 32'(entry_npkts) <= MAX_PKTS
    && ch_sum == (CNT_W+NCH+1)'(entry_npkts)
    && entry_cov));
  // a front-error request (non-OKAY resp) carries no packets
  `ASSUME(m_entry_err, !entry_valid || entry_resp == 2'b00 || entry_npkts == '0);

  // MC: each channel's completion targets a live slot (count below its per-channel target) or
  // the tail slot still being split (count below MAX while the per-channel target is unknown)
  logic cmpl_ok;
  always_comb begin
    cmpl_ok = 1'b1;
    for (int c = 0; c < NCH; c++)
      if (cmpl_valid[c] &&
          !(32'(cmpl_rob_index[c]) < ROB_DEPTH && (
               ( fv_live[cmpl_rob_index[c]] && fv_cmpl[cmpl_rob_index[c]][c] < fv_run[cmpl_rob_index[c]][c])
            || (!fv_live[cmpl_rob_index[c]] && cmpl_rob_index[c] == fv_tail && !fv_full
                && 32'(fv_cmpl[cmpl_rob_index[c]][c]) < MAX_PKTS))))
        cmpl_ok = 1'b0;
  end
  `ASSUME(m_cmpl, cmpl_ok);

  // ------------------------------------------------------------------------
  // 3a. interface checks
  // ------------------------------------------------------------------------
  `ASSERT(a_rob_idx,   rob_idx == fv_tail);
  `ASSERT(a_rob_free,  rob_free == !fv_full);
  `ASSERT(a_resp_valid, RESP_VALID == head_done);
  `ASSERT(a_resp_id,    !RESP_VALID || RESP_ID == fv_id[fv_head]);
  `ASSERT(a_resp_resp,  !RESP_VALID || RESP_RESP == (fv_derr[fv_head] ? 2'b10 : fv_resp[fv_head]));
  `ASSERT(a_resp_run,   !RESP_VALID || RESP_NPKTS_CH == fv_run[fv_head]);
  `ASSERT(a_resp_hold,  `NEXT(RESP_VALID && !RESP_READY, RESP_VALID
                              && $stable(RESP_ID) && $stable(RESP_RESP) && $stable(RESP_NPKTS_CH)));
  `ASSERT(a_reset,      $past(aresetn) || (occ == '0 && !RESP_VALID && rob_free));

  // ------------------------------------------------------------------------
  // 3b. induction helpers: DUT state == model
  // ------------------------------------------------------------------------
  `ASSERT(a_tail, alloc_ptr == fv_tail);
  `ASSERT(a_head, retire_ptr == fv_head);
  `ASSERT(a_occ,  occ == fv_occ && fv_occ <= OCC_W'(ROB_DEPTH));
  // the tail sits occ slots after the head (mod ROB_DEPTH)
  wire [OCC_W:0] tail_lin = (OCC_W+1)'(fv_head) + (OCC_W+1)'(fv_occ);
  `ASSERT(a_ring, (OCC_W+1)'(fv_tail) ==
                  ((tail_lin >= (OCC_W+1)'(ROB_DEPTH)) ? tail_lin - (OCC_W+1)'(ROB_DEPTH) : tail_lin));
  `ASSERT(a_ptr_range, 32'(alloc_ptr) < ROB_DEPTH && 32'(retire_ptr) < ROB_DEPTH);
  for (genvar i = 0; i < ROB_DEPTH; i++) begin : g_slot
    wire [OCC_W-1:0] age = (i >= fv_head) ? OCC_W'(i) - OCC_W'(fv_head)
                                          : OCC_W'(i) + OCC_W'(ROB_DEPTH) - OCC_W'(fv_head);
    wire live = fv_live[i];
    // per-channel outflow matches the model, and stays within the per-channel target when live
    logic cnt_eq, cnt_le, cnt_zero;
    always_comb begin
      cnt_eq = 1'b1; cnt_le = 1'b1; cnt_zero = 1'b1;
      for (int c = 0; c < NCH; c++) begin
        if (outflow_cnt[i][c] != fv_cmpl[i][c])    cnt_eq   = 1'b0;
        if (fv_cmpl[i][c]     >  fv_run[i][c])      cnt_le   = 1'b0;
        if (fv_cmpl[i][c]     != '0)                cnt_zero = 1'b0;
      end
    end
    `ASSERT(a_live,   live == (age < fv_occ));
    `ASSERT(a_inflow, inflow_complt[i] == live);
    `ASSERT(a_cnt,    cnt_eq);
    `ASSERT(a_derr,   ent_derr[i] == fv_derr[i]);
    `ASSERT(a_bound,  live ? (cnt_le && 32'(fv_npkts[i]) <= MAX_PKTS)
                           : (i == fv_tail || (cnt_zero && !fv_derr[i])));
    `ASSERT(a_ent,    !live || (rmc_cif_wrob.ent_id[i] == fv_id[i] && ent_resp[i] == fv_resp[i]
                                && n_pkts[i] == fv_npkts[i] && rmc_cif_wrob.run_cnt[i] == fv_run[i]));
  end

  // ------------------------------------------------------------------------
  // covers
  // ------------------------------------------------------------------------
  `COVER(c_retire,     retire);
  `COVER(c_full,       fv_full);
  `COVER(c_slverr,     retire && RESP_RESP == 2'b10 && fv_resp[fv_head] == 2'b00); // back-end error
  `COVER(c_decerr,     retire && RESP_RESP == 2'b11);                              // front error
  `COVER(c_early,      entry_valid && fv_cmpl[fv_tail] != '0);   // completions before the entry
  `COVER(c_bothch,     cmpl_valid == '1);                        // both channels complete at once
  `COVER(c_wrap,       retire && retire_ptr == PTR_W'(ROB_DEPTH-1));
  `COVER(c_max,        retire && 32'(fv_npkts[fv_head]) == MAX_PKTS);
  `COVER(c_stall,      RESP_VALID && !RESP_READY);

`undef ASSERT
`undef COVER
`undef ASSUME
`undef NEXT
endmodule : rmc_cif_wrob_fv
