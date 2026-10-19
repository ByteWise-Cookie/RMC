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
// The channel reconstruction is NOT re-proved here - the splitter FV proves the
// descriptor rebuild equals the full-address decode (a_rebuild). It is the oracle here.
// Every other answer (address of beat b, slot of a line, which beat is on R) comes from a
// closed form on the latched request, not from a copy of the DUT's counters.
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

  // the ROB keeps the response offered until it is taken
  `ASSUME(m_resp_hold, `NEXT(resp_valid && !resp_ready, resp_valid));

  // ------------------------------------------------------------------------
  // reference: the request as latched from the PORTS, and closed-form answers
  //   addr(b)    = align_down(addr_lo, size) + b * 2^size        (12-bit, in page)
  //   line(b)    = addr(b)[11:6]      chunk(b) = addr(b)[5:BEAT_LSB]
  //   slot(b)    = head[ch] + (lines l0 <= j < line(b) with chan(j) == ch)
  // i.e. the k-th line of a channel in the request sits k slots after that channel's
  // head - exactly how the splitter stored it. No counters are mirrored.
  // ------------------------------------------------------------------------
  wire dut_latch = !run && resp_valid;                 // the DUT's start condition

  logic [PAGE_LSB-1:0]         r_lo;                   // aligned start address in page
  logic [2:0]                  r_size;
  logic [7:0]                  r_len;
  logic [CH_W-1:0]             r_bch;
  logic [N_CH-1:0][SLOT_W-1:0] r_head;
  logic                        r_err;
  logic [AXI_IDW-1:0]          r_id;
  logic [7:0]                  r_out;                  // R beats accepted so far

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      r_out <= '0;
    end else begin
      if (rvalid && rready) r_out <= r_out + 8'd1;
      if (dut_latch) begin
        r_lo   <= resp_addr_lo & ~PAGE_LSB'((1 << resp_size) - 1);
        r_size <= resp_size;
        r_len  <= resp_len;
        r_bch  <= resp_base_ch;
        r_head <= resp_head_ptr;
        r_err  <= (resp_resp != 2'b00);
        r_id   <= resp_id;
        r_out  <= '0;
      end
    end
  end

  // channel of page line ln, from the descriptor (bits below the page from the line
  // address, bits above it from base_ch). The splitter proves this equals the full
  // address decode (a_rebuild there), so it is the oracle here.
  function automatic logic [CH_W-1:0] ref_chan(input logic [LINE_W-1:0] ln);
    logic [PAGE_LSB-1:0] tag;
    int src;
    tag = {ln, {OFF_W{1'b0}}};
    ref_chan = '0;
    for (int k = 0; k < CH_W; k++)
      if (k < int'(cfg_n_ch_log2)) begin
        src = int'(addr_map[OFF_W + k]);
        ref_chan[k] = (src < PAGE_LSB) ? tag[(src < PAGE_LSB) ? src : 0] : r_bch[k];
      end
  endfunction

  function automatic logic [PAGE_LSB-1:0] ref_addr(input logic [7:0] b);
    ref_addr = r_lo + PAGE_LSB'(32'(b) << r_size);
  endfunction

  // lines of channel c strictly between the first line and line ln
  function automatic logic [SLOT_W-1:0] ref_before(input logic [CH_W-1:0] c,
                                                   input logic [LINE_W-1:0] ln);
    logic [LINE_W-1:0] l0;
    l0 = r_lo[PAGE_LSB-1:OFF_W];
    ref_before = '0;
    for (int j = 0; j < (1 << LINE_W); j++)
      if (LINE_W'(j) >= l0 && LINE_W'(j) < ln && ref_chan(LINE_W'(j)) == c)
        ref_before = ref_before + 1'b1;
  endfunction

  function automatic logic [SLOT_W-1:0] ref_slot(input logic [LINE_W-1:0] ln);
    ref_slot = r_head[ref_chan(ln)] + ref_before(ref_chan(ln), ln);
  endfunction

  // 1. walk: the counters are the closed-form address of the current beat
  wire [PAGE_LSB-1:0] g_addr = ref_addr(beat_q);
  `ASSERT(a_walk,  !run || {line_q, off_q} == g_addr);
  `ASSERT(a_size,  !run || size_q == r_size);
  `ASSERT(a_chunk, !run || chunk == off_q[OFF_W-1:BEAT_LSB]);
  `ASSERT(a_beat_rng, !run || beat_q <= r_len);

  // 2. slot: channel and RD-SRAM slot of the beat G is issuing
  `ASSERT(a_ch,   !run || ch == ref_chan(line_q));
  `ASSERT(a_slot, !run || slot == ref_slot(line_q));
  `ASSERT(a_rd_onehot, !(g_fire && !err_q) || (rdb_re == (N_CH'(1) << ch)));
  `ASSERT(a_rd_addr,   !(g_fire && !err_q) || rdb_addr[ch] == slot);
  `ASSERT(a_slot_rng,  !(g_fire && !err_q) || 32'(slot) < N_RDB);
  for (genvar c = 0; c < N_CH; c++) begin : g_cnt
    // helper: each channel's counter = its lines already left behind
    `ASSERT(a_cnt, !run || cnt_q[c] == ref_before(CH_W'(c), line_q));
  end

  // 2b. data path: the beat on R is beat r_out of the burst, and the SRAM word under it
  //     is the right one. d_* is what D slices rdata from; the SRAM was read at d_slot on
  //     the previous edge, so rdb_dout this cycle is that slot's line.
  wire [PAGE_LSB-1:0] d_addr = ref_addr(r_out);
  logic [N_CH-1:0]             re_q;
  logic [N_CH-1:0][SLOT_W-1:0] addr_q;
  always_ff @(posedge aclk) begin
    re_q   <= rdb_re;
    addr_q <= rdb_addr;
  end
  wire [SUBB_W-1:0] d_chunk_w = rmc_cif_r_resp.d_chunk;
  wire              d_err_w   = rmc_cif_r_resp.d_err;
  `ASSERT(a_d_beat,  !rvalid || r_out <= r_len);
  `ASSERT(a_d_last,  !rvalid || rlast == (r_out == r_len));
  `ASSERT(a_d_ch,    !(rvalid && !d_err_w) || d_ch == ref_chan(d_addr[PAGE_LSB-1:OFF_W]));
  `ASSERT(a_d_chunk, !(rvalid && !d_err_w) || d_chunk_w == d_addr[OFF_W-1:BEAT_LSB]);
  `ASSERT(a_d_slot,  !(rvalid && !d_err_w) || d_slot == ref_slot(d_addr[PAGE_LSB-1:OFF_W]));
  `ASSERT(a_d_read,  !(rvalid && !d_err_w) || ($past(aresetn) && re_q[d_ch] && addr_q[d_ch] == d_slot));
  `ASSERT(a_d_rid,   !rvalid || (rid == r_id && rresp == (r_err ? 2'b10 : 2'b00)));
  // helpers: D holds a beat only while the request runs, with the request's id / error,
  // and G is at most one beat ahead of R
  `ASSERT(a_h_d,     !d_valid || (run && d_err_w == err_q && rmc_cif_r_resp.d_rid == rid_q));
  `ASSERT(a_h_req,   !run || (err_q == r_err && rid_q == r_id && len_q == r_len
                              && bch_q == r_bch && head_q == r_head));
  `ASSERT(a_h_ahead, !run || (g_last_done ? (32'(r_out) + (d_valid ? 1 : 0) == 32'(r_len) + 1)
                                          : (32'(beat_q) == 32'(r_out) + (d_valid ? 1 : 0))));
  // helpers: the latched request is legal - start aligned to the beat size, size legal,
  // and the whole burst inside the page (these hold at latch by the input assumptions)
  wire [16:0] r_end = 17'(r_lo) + ((17'(r_len) + 17'd1) << r_size) - 17'd1;
  `ASSERT(a_h_legal, !run || ((r_lo & PAGE_LSB'((1 << r_size) - 1)) == '0
                              && 32'(r_size) <= BEAT_LSB && r_end[16:PAGE_LSB] == '0));
  `ASSERT(a_h_idle,  run || (!d_valid && !g_last_done));

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
  `COVER(c_narrow_x,  rvalid && rready && rlast && 32'(r_size) < BEAT_LSB
                      && r_lo[OFF_W-1:0] != '0 && r_lo[PAGE_LSB-1:OFF_W] != d_addr[PAGE_LSB-1:OFF_W]);
                                                                  // narrow, unaligned, crosses a line
  `COVER(c_b2b,       dut_latch && $past(resp_ready));             // next request right after

  `undef ASSERT
  `undef COVER
  `undef ASSUME
  `undef NEXT

endmodule : rmc_cif_r_resp_fv
