// rmc_cif_r_resp.sv
// Read-response assembler. Drains the per-channel RD-SRAM into the AXI R channel for
// one retired request at a time, in ascending address order.
//
// A request's lines (64 B packets) were split across N_CH channels and stored, per
// channel, contiguously from head_ptr[ch]. To rebuild the R beat stream we must walk
// the lines in address order and, for each line, know which channel it came from. That
// channel is recovered from the compact descriptor the ROB kept:
//   addr_lo = ar_addr[PAGE_LSB-1:0]   (low PKT_OFF_W = trim offset, rest = start line)
//   base_ch                            (channel of line 0)
// using the per-bit window mux: a CH source bit inside the page [PKT_OFF_W,PAGE_LSB)
// varies line-to-line and is read from the walked line address; a CH source bit at or
// above PAGE_LSB is constant for the whole request and is taken from base_ch. This is
// the inverse of rmc_cfg_pkg::line_channel, specialised to the bits the descriptor keeps.
//
// The ROB only raises resp_valid once the request is fully complete (every packet in
// SRAM), so the assembler reads freely with no data-ready handshake per line.
//
// Two-stage pipeline, one beat per cycle:
//   G (addr-gen) : from the registered walk state compute the beat's in-page byte
//                  address, recover its channel, and drive the RD-SRAM read port
//                  (rdb_addr/rdb_re). The walk (beat_q, per-channel cnt_q) advances
//                  here, gated by the downstream being ready.
//   D (data/out) : one cycle later the synchronous SRAM returns the 64 B packet;
//                  slice the chunk-th AXI beat out of it and present it on R.
// A one-beat skid (d_valid) carries G's per-beat control to D so rready back-pressure
// stalls the walk without dropping or duplicating a beat. The SRAM read straddles the
// G|D register boundary - that boundary is the point of the cut.
//
// Assumptions:
//   - RD-SRAM read is SYNCHRONOUS: address registered in G, data valid in D the next
//     cycle (standard single-port ASIC macro). A different read latency changes the
//     number of D stages.
//   - INCR, full-width beats: ar_size == log2(AXI_DW/8). Unaligned start is handled by
//     addr_lo[PKT_OFF_W-1:0] (lead-in beats skipped); the beat count comes from ar_len.
//     Narrow bursts (ar_size < full) are not trimmed per byte lane yet; TODO.
//   - The burst stays inside its 4 KB page (guaranteed upstream), so the in-page byte
//     address is PAGE_LSB bits with no carry out.

import rmc_cfg_pkg::*;

module rmc_cif_r_resp #(
  parameter int AXI_IDW = 8,            // AXI id width
  parameter int N_RDB   = 64,           // RD-SRAM slots per channel

  // derived (localparam: never passed in). pkg values qualified so strict front-ends
  // resolve them in the header without leaning on the wildcard import.
  localparam int N_CH_L     = rmc_cfg_pkg::N_CH,
  localparam int CH_W_L     = rmc_cfg_pkg::CH_W,
  localparam int OFF_W_L    = rmc_cfg_pkg::PKT_OFF_W,
  localparam int PAGE_LSB_L = rmc_cfg_pkg::PAGE_LSB,
  localparam int DEC_W_L    = rmc_cfg_pkg::DEC_W,
  localparam int SEL_W_L    = rmc_cfg_pkg::MAP_SEL_W,
  localparam int AXI_DW_L   = rmc_cfg_pkg::AXI_DW,
  localparam int SRAM_W     = rmc_cfg_pkg::PKT_BYTES * 8,                 // 64 B packet / SRAM line
  localparam int SLOT_W     = (N_RDB > 1) ? $clog2(N_RDB) : 1,           // RD-SRAM slot index
  localparam int BEAT_BYTES = AXI_DW_L / 8,                              // bytes per AXI beat
  localparam int BEAT_LSB   = (BEAT_BYTES > 1) ? $clog2(BEAT_BYTES) : 1, // log2(beat bytes)
  localparam int N_SUBB     = SRAM_W / AXI_DW_L,                         // AXI beats per 64 B packet
  localparam int SUBB_W     = (N_SUBB > 1) ? $clog2(N_SUBB) : 1,
  localparam int NCH_LOG_W  = $clog2(CH_W_L + 1)
)(
  input  logic                               aclk,
  input  logic                               aresetn,

  // live address map + active-channel count (static after boot) for the channel recovery
  input  logic [DEC_W_L-1:0][SEL_W_L-1:0]    addr_map,
  input  logic [NCH_LOG_W-1:0]               cfg_n_ch_log2,

  // retired request from the ROB. Held stable while resp_ready is low; popped on the
  // final accepted beat (resp_ready pulse).
  // FROM ROB ingress
  input  logic                               resp_valid,
  output logic                               resp_ready,
  input  logic [AXI_IDW-1:0]                 resp_id,
  input  logic [1:0]                         resp_resp,      // 00 OKAY / 10 SLVERR

  // FROM ROB ingress
  input  logic [N_CH_L-1:0][SLOT_W-1:0]      resp_head_ptr,  // per-channel RD-SRAM run base
  input  logic [CH_W_L-1:0]                  resp_base_ch,   // channel of line 0
  input  logic [PAGE_LSB_L-1:0]              resp_addr_lo,   // ar_addr[PAGE_LSB-1:0]
  input  logic [7:0]                         resp_len,       // ar_len (beats - 1)
  input  logic [2:0]                         resp_size,      // ar_size (unused in first cut)

  // RD-SRAM client-side read port, one per channel (MC writes; CIF drains)
  output logic [N_CH_L-1:0]                  rdb_re,
  output logic [N_CH_L-1:0][SLOT_W-1:0]      rdb_addr,
  input  logic [N_CH_L-1:0][SRAM_W-1:0]      rdb_dout,

  // AXI R channel
  output logic [AXI_IDW-1:0]                 rid,
  output logic [AXI_DW_L-1:0]                rdata,
  output logic [1:0]                         rresp,
  output logic                               rlast,
  output logic                               rvalid,
  input  logic                               rready
);

  // Channel a single line maps to, rebuilt from the stored descriptor instead of the
  // full address (which the ROB does not keep). The address map decodes the channel as
  // a set of source bits: CH bit k = sys_addr[ amap[PKT_OFF_W + k] ] (same as
  // rmc_cfg_pkg::line_channel). Within one 4 KB request only the in-page bits move, so:
  //   - a CH source bit < PAGE_LSB (inside addr_lo) changes line-to-line -> read it from
  //     the walked line address `adr_low_tag`;
  //   - a CH source bit >= PAGE_LSB is constant for the whole request -> its value is
  //     already folded into base_ch (the channel of line 0), so take base_ch[k].
  // n_ch_log2 masks off inactive channels (CH bits at or above it are forced 0), matching
  // line_channel exactly so the rebuild can never disagree with how the split was done.
  function automatic logic [CH_W_L-1:0] walk_channel(
      input logic [PAGE_LSB_L-1:0]            adr_low_tag,
      input logic [CH_W_L-1:0]                base_ch,
      input logic [DEC_W_L-1:0][SEL_W_L-1:0]  amap,
      input logic [NCH_LOG_W-1:0]             n_ch_log2);
    logic [CH_W_L-1:0] c;
    int src, idx;
    c = '0;
    for (int k = 0; k < CH_W_L; k++)
      if (k < int'(n_ch_log2)) begin
        src    = int'(amap[OFF_W_L + k]);               // which sys_addr bit feeds CH bit k
        idx    = (src < PAGE_LSB_L) ? src : 0;          // guard the part-select when src is out-of-page
        c[k]   = (src < PAGE_LSB_L) ? adr_low_tag[idx]  // in-page: from the walked line
                                    : base_ch[k];       // out-of-page: frozen in base_ch
      end
    return c;
  endfunction

  // latched descriptor + walk state (the G, address-generation stage)
  logic                          run;         // a request is being issued (G active)
  logic                          g_last_done; // G has issued the final beat; drain D only
  logic [AXI_IDW-1:0]            rid_q;
  logic                          err_q;
  logic [N_CH_L-1:0][SLOT_W-1:0] head_q;
  logic [CH_W_L-1:0]             bch_q;
  logic [PAGE_LSB_L-1:0]         alo_q;
  logic [7:0]                    len_q;
  logic [7:0]                    beat_q;                 // current beat index (0..len_q)
  logic [N_CH_L-1:0][SLOT_W-1:0] cnt_q;                  // per-channel lines emitted so far

  // D stage: one beat of R control in flight, filled by G, drained by the R handshake.
  logic                 d_valid;
  logic [AXI_IDW-1:0]   d_rid;
  logic                 d_err;
  logic                 d_last;
  logic [CH_W_L-1:0]    d_ch;                            // channel whose SRAM dout to slice
  logic [SUBB_W-1:0]    d_chunk;                         // which AXI beat within the packet
  logic [SLOT_W-1:0]    d_slot;                          // its SRAM slot (re-read while stalled)

  // Per-beat address decomposition (G stage). byte_lo is the current beat's in-page byte
  // address, stepping BEAT_BYTES per beat from addr_lo (the request's unaligned start). The
  // 4 KB no-cross guarantee keeps it within PAGE_LSB bits, so the add never carries out.
  //
  //   byte_lo[PAGE_LSB-1 : PKT_OFF_W]  = line index  -> picks the 64 B packet / channel
  //   byte_lo[PKT_OFF_W-1 : BEAT_LSB]  = chunk        -> which AXI beat inside that packet
  //   byte_lo[BEAT_LSB-1  : 0]         = byte-in-beat (always 0 for full-width INCR)
  //
  logic [PAGE_LSB_L-1:0] byte_lo, next_lo;
  logic [CH_W_L-1:0]     ch;
  logic [SUBB_W-1:0]     chunk;
  logic [SLOT_W-1:0]     slot;
  logic                  is_last, ln_cross;

  assign byte_lo = alo_q + (PAGE_LSB_L'(beat_q)        << BEAT_LSB);
  assign next_lo = alo_q + (PAGE_LSB_L'(beat_q + 8'd1) << BEAT_LSB);  // next beat, to spot a line change
  assign ch      = walk_channel(byte_lo, bch_q, addr_map, cfg_n_ch_log2);
  assign chunk   = byte_lo[OFF_W_L-1 : BEAT_LSB];        // which AXI beat within the 64 B packet
  // Each channel's lines sit contiguously from head_ptr[ch]; the k-th time the walk lands
  // on a channel it wants that channel's k-th stored line. cnt_q[ch] counts prior landings.
  assign slot    = head_q[ch] + cnt_q[ch];
  assign is_last = (beat_q == len_q);                    // ar_len is beats-1, so last beat == len
  // the current beat is the last of its line when the next beat steps to a new line index
  assign ln_cross = next_lo[PAGE_LSB_L-1:OFF_W_L] != byte_lo[PAGE_LSB_L-1:OFF_W_L];

  // Elastic handshake G -> D. D can take a new beat when it is empty or draining this
  // cycle; G issues (and the walk advances) only when D can take it and G has beats left.
  wire d_ready = ~d_valid | rready;
  wire g_fire  = run & ~g_last_done & d_ready;

  // SRAM read drive. On an issue (g_fire) G reads the new beat's slot; dout lands in D
  // next cycle. While D is stalled (holding a beat, rready low, no issue) the in-flight
  // beat's slot is re-read so its dout stays valid without depending on the macro holding
  // Q across idle read cycles. The two never collide: g_fire requires d_ready, which when
  // d_valid means rready, i.e. the held beat is being accepted this very cycle.
  always_comb begin
    rdb_re   = '0;
    rdb_addr = '0;
    if (g_fire & ~err_q) begin
      rdb_re[ch]     = 1'b1;
      rdb_addr[ch]   = slot;
    end else if (d_valid & ~d_err) begin
      rdb_re[d_ch]   = 1'b1;
      rdb_addr[d_ch] = d_slot;
    end
  end

  // R-channel outputs (D stage). rdata takes the chunk-th AXI_DW slice of the 64 B packet.
  assign rvalid = d_valid;
  assign rid    = d_rid;
  assign rresp  = d_err ? 2'b10 : 2'b00;                 // whole burst carries the request's resp
  assign rlast  = d_valid & d_last;
  assign rdata  = d_err ? '0 : rdb_dout[d_ch][d_chunk*AXI_DW_L +: AXI_DW_L];

  // pop the ROB when the final beat is accepted on R
  assign resp_ready = d_valid & d_last & rready;

  // Idle until a request appears, then issue it in G one beat per cycle while D is ready;
  // D holds the SRAM result and presents it on R. run stays high until the last beat is
  // ACCEPTED (not just issued), so G and the descriptor are not reloaded under the drain.
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      run         <= 1'b0;
      g_last_done <= 1'b0;
      d_valid     <= 1'b0;
    end else begin
      // ---- D stage: accept a new beat from G, or drain the held one
      if (d_ready) begin
        d_valid <= g_fire;
        if (g_fire) begin
          d_rid   <= rid_q;
          d_err   <= err_q;
          d_last  <= is_last;
          d_ch    <= ch;
          d_chunk <= chunk;
          d_slot  <= slot;
        end
      end

      // ---- G stage: advance the walk on each issued beat
      if (g_fire) begin
        if (is_last) begin
          g_last_done <= 1'b1;                            // stop issuing; wait for D to drain
        end else begin
          beat_q <= beat_q + 8'd1;
          // finishing a line: bump the counter of the channel we are leaving so its next
          // visit reads the next stored slot. ch is that channel (the issued beat's).
          if (ln_cross) cnt_q[ch] <= cnt_q[ch] + 1'b1;
        end
      end

      // ---- request boundary
      if (resp_ready) begin
        run         <= 1'b0;                             // last beat accepted; ROB popped
        g_last_done <= 1'b0;
      end else if (!run && resp_valid) begin
        // latch the oldest retired request and start walking it
        run    <= 1'b1;
        rid_q  <= resp_id;
        err_q  <= (resp_resp != 2'b00);
        head_q <= resp_head_ptr;
        bch_q  <= resp_base_ch;
        alo_q  <= resp_addr_lo;
        len_q  <= resp_len;
        beat_q <= '0;
        cnt_q  <= '0;                                     // per-channel line counters start at 0
      end
    end
  end

`ifndef SYNTHESIS
  // a non-error beat being issued must address a real slot; recovered channel must be active
  always @(posedge aclk) if (aresetn && g_fire && !err_q) begin
    assert (32'(slot) < N_RDB)
      else $error("r_resp: slot %0d out of range (N_RDB=%0d)", slot, N_RDB);
    assert (int'(ch) < (1 << cfg_n_ch_log2))
      else $error("r_resp: recovered channel %0d exceeds active channels", ch);
  end
`endif

endmodule : rmc_cif_r_resp
