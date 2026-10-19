// rmc_cif_r_resp_fv.sv
// Formal checker for rmc_cif_r_resp (read-response assembler). Bound into the DUT by
// rmc_cif_r_resp_bind.sv, so the RTL carries no verification code.
//
// What this proves (safety, open-source slang flow, boolean properties only):
//   1. walk      : an independent re-derivation of the per-beat in-page address from the
//                  closed form (aligned addr_lo + beat*2^size) matches the DUT's line/off
//                  counters, for any ar_size (full-width or narrow). Ties the counter walk
//                  to the arithmetic it stands in for.
//   2. slot      : the RD-SRAM read targets head_ptr[ch] + (that channel's lines seen so
//                  far), with the per-channel counters matching an independent model.
//   3. handshake : rvalid holds until rready; rid/rresp/rlast are correct and stable under
//                  back-pressure; resp_ready pulses exactly on the last accepted beat; the
//                  request then releases.
// The channel reconstruction (walk_channel, the window mux) is NOT re-proved here - the
// splitter FV already proves the window mux equals the full-address decode (a_rebuild).
// This checker uses that same channel function as the oracle for the counting/stepping
// logic, which is what r_resp adds.
//
// Property style matches the other cif harnesses: boolean only (no |->, |=>, sequences).
//   a -> b  written  !a || b ;  a => b  written via `NEXT (skips first post-reset cycle).

import rmc_cfg_pkg::*;

module rmc_cif_r_resp_fv #(
  parameter int AXI_IDW   = 8,
  parameter int N_RDB     = 64,
  // DUT-derived widths, passed straight from the DUT localparams by the bind
  parameter int SLOT_W    = 6,
  parameter int OFF_W     = rmc_cfg_pkg::PKT_OFF_W,
  parameter int LINE_W    = rmc_cfg_pkg::PAGE_LSB - rmc_cfg_pkg::PKT_OFF_W,
  parameter int BEAT_LSB  = 4,
  parameter int SUBB_W    = 2,
  parameter int NCH_LOG_W = 1
)(
  input logic                              aclk,
  input logic                              aresetn,

  // DUT config + descriptor inputs (free, constrained by assumptions)
  input logic [DEC_W-1:0][MAP_SEL_W-1:0]   addr_map,
  input logic [NCH_LOG_W-1:0]              cfg_n_ch_log2,
  input logic                              resp_valid,
  input logic                              resp_ready,
  input logic [AXI_IDW-1:0]                resp_id,
  input logic [1:0]                        resp_resp,
  input logic [N_CH-1:0][SLOT_W-1:0]       resp_head_ptr,
  input logic [CH_W-1:0]                   resp_base_ch,
  input logic [PAGE_LSB-1:0]               resp_addr_lo,
  input logic [7:0]                        resp_len,
  input logic [2:0]                        resp_size,

  // R channel outputs under test
  input logic [AXI_IDW-1:0]                rid,
  input logic [1:0]                        rresp,
  input logic                              rlast,
  input logic                              rvalid,
  input logic                              rready,

  // RD-SRAM read port (address side is checked; data is the SRAM's responsibility)
  input logic [N_CH-1:0]                   rdb_re,
  input logic [N_CH-1:0][SLOT_W-1:0]       rdb_addr,

  // DUT internals (reachable through the bind)
  input logic                              run,
  input logic                              g_last_done,
  input logic                              err_q,
  input logic [AXI_IDW-1:0]                rid_q,
  input logic [7:0]                        len_q,
  input logic [2:0]                        size_q,
  input logic [7:0]                        beat_q,
  input logic [LINE_W-1:0]                 line_q,
  input logic [OFF_W-1:0]                  off_q,
  input logic [CH_W-1:0]                   bch_q,
  input logic [N_CH-1:0][SLOT_W-1:0]       head_q,
  input logic [N_CH-1:0][SLOT_W-1:0]       cnt_q,
  input logic [CH_W-1:0]                   ch,
  input logic [SUBB_W-1:0]                 chunk,
  input logic [SLOT_W-1:0]                 slot,
  input logic                              is_last,
  input logic                              off_wrap,
  input logic                              g_fire,
  input logic                              d_valid,
  input logic                              d_last,
  input logic [CH_W-1:0]                   d_ch,
  input logic [SLOT_W-1:0]                 d_slot
);

  `define ASSERT(name, expr) name: assert property (@(posedge aclk) disable iff (!aresetn) (expr))
  `define COVER(name, expr)  name: cover  property (@(posedge aclk) disable iff (!aresetn) (expr))
  `define ASSUME(name, expr) name: assume property (@(posedge aclk) (expr))
  `define NEXT(a, b)         (!$past(aresetn) || !$past(a) || (b))

  localparam int N_CH_L = 1 << CH_W;

  // channel of a page line, rebuilt from the descriptor (the window mux the DUT uses). Same
  // function as walk_channel; used here only as the oracle for the counting/stepping logic.
  function automatic logic [CH_W-1:0] ref_chan(input logic [LINE_W-1:0] ln);
    logic [PAGE_LSB-1:0] tag;
    int src;
    tag = {ln, {OFF_W{1'b0}}};
    ref_chan = '0;
    for (int k = 0; k < CH_W; k++)
      if (k < int'(cfg_n_ch_log2)) begin
        src = int'(addr_map[OFF_W + k]);
        ref_chan[k] = (src < PAGE_LSB) ? tag[(src < PAGE_LSB) ? src : 0] : bch_q[k];
      end
  endfunction

  // assumptions
  logic started = 1'b0;
  always_ff @(posedge aclk) started <= 1'b1;
  `ASSUME(m_reset_first, started || !aresetn);

  // ar_size never exceeds the full bus width (a legal AXI read)
  `ASSUME(m_size_legal, 32'(resp_size) <= BEAT_LSB);

  // legal map: each CH source bit at or above the line offset and inside the address; the
  // active-channel count matches CH_W; map stable in flight (boot CSR).
  for (genvar k = 0; k < CH_W; k++) begin : g_map_legal
    `ASSUME(m_map_ch_lo, addr_map[PKT_OFF_W + k] >= MAP_SEL_W'(PKT_OFF_W));
    `ASSUME(m_map_ch_hi, addr_map[PKT_OFF_W + k] <  MAP_SEL_W'(SYS_ADDR_W));
  end
  `ASSUME(m_cfg_nch,   cfg_n_ch_log2 == NCH_LOG_W'(CH_W));
  `ASSUME(m_map_stable, !started || $stable(addr_map));

  // the ROB holds the descriptor stable while the response is pending (resp_valid & !ready)
  `ASSUME(m_desc_stable, !(resp_valid && !resp_ready) || (
       $stable(resp_id) && $stable(resp_resp) && $stable(resp_head_ptr)
    && $stable(resp_base_ch) && $stable(resp_addr_lo) && $stable(resp_len) && $stable(resp_size)));

  // the request fits: the walk never steps past the page, and each channel's run fits the
  // RD-SRAM (so slots do not alias). Stated on the latched request.
  wire [PAGE_LSB-1:0] alo_aligned = resp_addr_lo & ~PAGE_LSB'((1 << resp_size) - 1);
  wire [16:0] span_end = 17'(alo_aligned) + ((17'(resp_len) + 17'd1) << resp_size) - 17'd1;
  `ASSUME(m_in_page, !resp_valid || span_end[16:PAGE_LSB] == '0);

  // independent reference walk: closed-form address + per-channel line counters. Latched on
  // the same cycle the DUT latches (idle & a new response), advanced on the DUT's g_fire.
  logic [7:0]                 r_beat;
  logic [LINE_W-1:0]          r_line;
  logic [OFF_W-1:0]           r_off;
  logic [2:0]                 r_size;
  logic [N_CH-1:0][SLOT_W-1:0] r_cnt;
  logic [N_CH-1:0][SLOT_W-1:0] r_head;

  wire [OFF_W:0] r_off_nxt  = {1'b0, r_off} + (OFF_W+1)'(1 << r_size);
  wire           r_off_wrap = r_off_nxt[OFF_W];
  wire           dut_latch  = !run && resp_valid;   // the DUT's start condition

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      r_beat <= '0; r_line <= '0; r_off <= '0; r_cnt <= '0;
    end else if (resp_ready && d_last && rvalid && rready) begin
      // request released; counters reset for the next one on dut_latch
      r_cnt <= '0;
    end else if (dut_latch) begin
      r_beat <= '0;
      r_line <= resp_addr_lo[PAGE_LSB-1 : OFF_W];
      r_off  <= resp_addr_lo[OFF_W-1 : 0] & ~OFF_W'((1 << resp_size) - 1);
      r_size <= resp_size;
      r_head <= resp_head_ptr;
      r_cnt  <= '0;
    end else if (g_fire && !is_last) begin
      r_beat <= r_beat + 8'd1;
      r_off  <= r_off_nxt[OFF_W-1:0];
      if (r_off_wrap) begin
        r_line        <= r_line + 1'b1;
        r_cnt[ref_chan(r_line)] <= r_cnt[ref_chan(r_line)] + 1'b1;
      end
    end
  end

  // 1. walk: the DUT counters equal the reference walk
  `ASSERT(a_beat, !run || beat_q == r_beat);
  `ASSERT(a_line, !run || line_q == r_line);
  `ASSERT(a_off,  !run || off_q  == r_off);
  `ASSERT(a_size, !run || size_q == r_size);
  `ASSERT(a_chunk, !run || chunk == off_q[OFF_W-1:BEAT_LSB]);

  // 2. slot: the DUT's channel + per-channel counters drive the right RD-SRAM address
  for (genvar c = 0; c < N_CH_L; c++) begin : g_cnt
    `ASSERT(a_cnt, !run || cnt_q[c] == r_cnt[c]);
  end
  `ASSERT(a_ch,   !run || ch == ref_chan(line_q));                 // DUT channel == oracle
  `ASSERT(a_slot, !run || slot == head_q[ch] + cnt_q[ch]);
  `ASSERT(a_rd_onehot, !(g_fire && !err_q) || (rdb_re == (N_CH'(1) << ch)));
  `ASSERT(a_rd_addr,   !(g_fire && !err_q) || rdb_addr[ch] == slot);
  `ASSERT(a_slot_rng,  !(g_fire && !err_q) || 32'(slot) < N_RDB);

  // 3. R-channel handshake + metadata
  `ASSERT(a_rvalid, rvalid == d_valid);
  `ASSERT(a_rid,    !rvalid || rid == rid_q);
  `ASSERT(a_rresp,  !rvalid || rresp == (err_q ? 2'b10 : 2'b00));
  `ASSERT(a_rlast,  rlast == (d_valid && d_last));
  `ASSERT(a_last,   is_last == (beat_q == len_q));
  `ASSERT(a_resprdy, resp_ready == (d_valid && d_last && rready));
  // a held beat stays put and stable (data excluded: it tracks the free SRAM dout)
  `ASSERT(a_hold, `NEXT(rvalid && !rready, rvalid && $stable(rid) && $stable(rresp) && $stable(rlast)));
  // the last accepted beat releases the request
  `ASSERT(a_release, `NEXT(resp_ready, !run || !$past(run)));
  // g_fire only while running and not already drained
  `ASSERT(a_gfire, !g_fire || (run && !g_last_done));

  // reset
  `ASSERT(a_reset, $past(aresetn) || (!run && !rvalid && !resp_ready));

  // covers (guard against a vacuous proof)
  `COVER(c_beat,      rvalid && rready);
  `COVER(c_last,      rvalid && rlast && rready);
  `COVER(c_stall,     rvalid && !rready);
  `COVER(c_cross,     g_fire && off_wrap && !is_last);          // a line boundary
  `COVER(c_narrow,    run && 32'(size_q) < BEAT_LSB && rvalid); // a narrow burst in flight
  `COVER(c_two_ch,    run && cnt_q[0] != '0 && cnt_q[N_CH-1] != '0); // both channels used
  `COVER(c_slverr,    rvalid && rresp == 2'b10);

  `undef ASSERT
  `undef COVER
  `undef ASSUME
  `undef NEXT

endmodule : rmc_cif_r_resp_fv
