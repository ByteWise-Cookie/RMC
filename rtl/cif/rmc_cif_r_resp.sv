// rmc_cif_r_resp.sv
// Read-response assembler. Drains the per-channel RD-SRAM into the AXI R channel for
// one retired request at a time, in ascending address order, handling any legal
// ar_size (full-width or narrow) for an INCR burst inside one 4 KB page.
//
// WHAT IT REBUILDS
// A request's lines (64 B packets) were split across N_CH channels and stored, per
// channel, contiguously from head_ptr[ch]. The ROB kept only a compact descriptor:
//   addr_lo = ar_addr[PAGE_LSB-1:0]   (start byte in the page)
//   base_ch                            (channel of line 0)
//   len / size                         (ar_len beats-1, ar_size log2 bytes/beat)
//   head_ptr[N_CH]                     (each channel's first RD-SRAM slot)
// To stream R we walk the burst beat by beat and, for each beat, recover WHICH
// channel its line sits in and WHICH slot inside that channel.
//
// THE WALK: two small counters, not a 12-bit address adder  ("our idea")
// The channel of a line is a pure function of the line INDEX (the map's CH source
// bits all sit at or above the 64 B line offset), and on an INCR run the line index
// just steps up. So instead of re-forming a full byte address every beat and slicing
// it, we keep two counters:
//   line_q : index of the current 64 B line in the page (0..PG_LINES-1).
//            Feeds walk_channel; ++ when the in-line offset wraps past 64 B.
//   off_q  : byte offset WITHIN the current line (PKT_OFF_W bits).
//            Steps by 2^size each beat; split into {chunk, lane}:
//              off_q[PKT_OFF_W-1 : BEAT_LSB] = chunk  (which AXI beat in the packet)
//              off_q[BEAT_LSB-1  : 0]        = lane   (byte-in-beat, for narrow)
// The per-channel slot is head_q[ch] + cnt_q[ch]; cnt_q[ch] counts how many of that
// channel's lines we have already left, so the k-th landing on a channel reads its
// k-th stored line. This needs only a PKT_OFF_W-wide add + a line increment, no full
// page-address adder.
//
// NARROW BURSTS (ar_size < full)  -- slave side is simple
// For a narrow read the master takes 2^size bytes per beat off the byte lanes its
// address selects, and IGNORES the other lanes (they are don't-care). So the slave
// does NOT shift or mask: it presents the aligned AXI_DW word (the chunk slice) and
// the master extracts its lane. All that changes versus full-width is the address
// STEP (2^size, not the full beat width), which makes several beats fall on the same
// chunk/line before off_q wraps. One SRAM line therefore feeds many beats; its slot
// is simply held (re-read) until the walk crosses out of it. Beat count and rlast
// come straight from ar_len (beat_q == len_q), independent of size.
//   - ar_size == full (2^size == AXI_DW/8): off_q steps a full beat, chunk advances
//     every beat, a 64 B line = N_SUBB beats. Streams with no internal hold.
//   - ar_size <  full: off_q steps less than a beat, so the chunk (hence the SRAM
//     slot) is reused across 2^(BEAT_LSB-size) beats -- the read is "held down"
//     while the master drains that word lane by lane.
//
// FREEING RD-SRAM SLOTS
// A request's slots stay occupied for the whole burst (longest for a narrow read,
// which can be hundreds of beats). They are reclaimed when the final beat is accepted
// -- the same resp_ready pulse that pops the ROB. The RD-SRAM allocator (which issued
// head_ptr and already knows each channel's line count) reclaims that request's run
// on this pulse, so r_resp does not need to report per-channel counts. Until then the
// allocator's free slots stay low, which is the natural back-pressure a long narrow
// burst applies toward new reads / the MC.
//
// PIPELINE
// Two stages, one beat per cycle, synchronous (1-cycle) RD-SRAM:
//   G (addr-gen) : counters -> ch, chunk, slot -> drive rdb_addr/rdb_re.
//   D (data/out) : next cycle the SRAM returns the 64 B packet; slice the chunk-th
//                  AXI_DW word and present it on R.
// A one-beat skid (d_valid) carries G's control to D so rready back-pressure stalls
// the walk cleanly. While stalled the in-flight beat's slot is re-read so the SRAM
// output stays valid without relying on the macro holding Q across idle reads.
//
// Assumptions:
//   - INCR only, inside one 4 KB page (both guaranteed upstream). WRAP/FIXED TODO.
//   - RD-SRAM read is SYNCHRONOUS (address in G, data in D next cycle).

import rmc_cfg_pkg::*;

module rmc_cif_r_resp #(
  parameter int AXI_IDW = 8,            // AXI id width
  parameter int N_RDB   = 64,           // RD-SRAM slots per channel

  // derived (localparam: never passed in). pkg values qualified so strict front-ends
  // resolve them in the header without leaning on the wildcard import.
  localparam int N_CH_L     = rmc_cfg_pkg::N_CH,
  localparam int CH_W_L     = rmc_cfg_pkg::CH_W,
  localparam int OFF_W_L    = rmc_cfg_pkg::PKT_OFF_W,                     // byte-in-line bits (64B -> 6)
  localparam int PAGE_LSB_L = rmc_cfg_pkg::PAGE_LSB,                      // in-page bits (4KB -> 12)
  localparam int LINE_W_L   = PAGE_LSB_L - OFF_W_L,                       // line index bits (64 lines -> 6)
  localparam int DEC_W_L    = rmc_cfg_pkg::DEC_W,
  localparam int SEL_W_L    = rmc_cfg_pkg::MAP_SEL_W,
  localparam int AXI_DW_L   = rmc_cfg_pkg::AXI_DW,
  localparam int SRAM_W     = rmc_cfg_pkg::PKT_BYTES * 8,                 // 64 B packet / SRAM line
  localparam int SLOT_W     = (N_RDB > 1) ? $clog2(N_RDB) : 1,           // RD-SRAM slot index
  localparam int BEAT_BYTES = AXI_DW_L / 8,                              // bytes per full AXI beat
  localparam int BEAT_LSB   = (BEAT_BYTES > 1) ? $clog2(BEAT_BYTES) : 1, // log2(full beat bytes) = max ar_size
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
  // final accepted beat (resp_ready pulse), which is also the RD-SRAM free trigger.
  input  logic                               resp_valid,
  output logic                               resp_ready,
  input  logic [AXI_IDW-1:0]                 resp_id,
  input  logic [1:0]                         resp_resp,      // 00 OKAY / 10 SLVERR
  input  logic [N_CH_L-1:0][SLOT_W-1:0]      resp_head_ptr,  // per-channel RD-SRAM run base
  input  logic [CH_W_L-1:0]                  resp_base_ch,   // channel of line 0
  input  logic [PAGE_LSB_L-1:0]              resp_addr_lo,   // ar_addr[PAGE_LSB-1:0]
  input  logic [7:0]                         resp_len,       // ar_len (beats - 1)
  input  logic [2:0]                         resp_size,      // ar_size (log2 bytes/beat)

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
  // full address. CH bit k = sys_addr[ amap[PKT_OFF_W + k] ] (same as line_channel).
  // Within one 4 KB request only the in-page bits move:
  //   - a CH source bit < PAGE_LSB changes line-to-line -> read it from the walked line
  //     address `adr_low_tag` (only its line-index bits matter; the offset bits are 0);
  //   - a CH source bit >= PAGE_LSB is constant for the request -> take base_ch[k].
  // n_ch_log2 masks inactive channels, matching line_channel exactly.
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
  logic [N_CH_L-1:0][SLOT_W-1:0] head_q;      // per-channel RD-SRAM run base
  logic [CH_W_L-1:0]             bch_q;
  logic [7:0]                    len_q;        // ar_len (beats - 1)
  logic [2:0]                    size_q;       // ar_size (log2 bytes/beat)
  logic [7:0]                    beat_q;       // current beat index (0..len_q)
  logic [LINE_W_L-1:0]           line_q;       // current 64 B line index in the page
  logic [OFF_W_L-1:0]            off_q;        // byte offset within the current line
  logic [N_CH_L-1:0][SLOT_W-1:0] cnt_q;        // per-channel lines left behind so far

  // D stage: one beat of R control in flight, filled by G, drained by the R handshake.
  logic                 d_valid;
  logic [AXI_IDW-1:0]   d_rid;
  logic                 d_err;
  logic                 d_last;
  logic [CH_W_L-1:0]    d_ch;                            // channel whose SRAM dout to slice
  logic [SUBB_W-1:0]    d_chunk;                         // which AXI beat within the packet
  logic [SLOT_W-1:0]    d_slot;                          // its SRAM slot (re-read while stalled)

  // G-stage decode of the current beat from the two counters.
  logic [PAGE_LSB_L-1:0] line_tag;    // {line_q, 0}: line address for the channel decode
  logic [CH_W_L-1:0]     ch;          // channel this line maps to
  logic [SUBB_W-1:0]     chunk;       // AXI beat within the 64 B packet
  logic [SLOT_W-1:0]     slot;        // RD-SRAM slot = head + per-channel count
  logic [OFF_W_L:0]      off_nxt;     // next in-line offset (one extra bit for the wrap)
  logic                  off_wrap;    // the step crosses out of the current 64 B line
  logic                  is_last;     // last beat of the burst (ar_len reached)

  assign line_tag = {line_q, {OFF_W_L{1'b0}}};
  assign ch       = walk_channel(line_tag, bch_q, addr_map, cfg_n_ch_log2);
  assign chunk    = off_q[OFF_W_L-1 : BEAT_LSB];          // which AXI beat inside the 64 B packet
  assign slot     = head_q[ch] + cnt_q[ch];               // channel's k-th stored line
  assign off_nxt  = {1'b0, off_q} + (OFF_W_L+1)'(1 << size_q);  // step 2^size bytes
  assign off_wrap = off_nxt[OFF_W_L];                     // carried out of the 64 B line
  assign is_last  = (beat_q == len_q);                    // ar_len is beats-1

  // Elastic handshake G -> D. D can take a new beat when it is empty or draining this
  // cycle; G issues (and the walk advances) only when D can take it and G has beats left.
  wire d_ready = ~d_valid | rready;
  wire g_fire  = run & ~g_last_done & d_ready;

  // SRAM read drive. On an issue (g_fire) G reads the new beat's slot; dout lands in D
  // next cycle. While D is stalled (holding a beat, rready low, no issue) the in-flight
  // beat's slot is re-read so its dout stays valid without depending on the macro holding
  // Q across idle read cycles. For a narrow burst, consecutive same-chunk beats re-read
  // the same slot here too (off_q has not wrapped), which is exactly the intended hold.
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

  // R-channel outputs (D stage). rdata is the chunk-th AXI_DW word of the 64 B packet;
  // for a narrow beat the master reads only the lanes its address selects, so no masking
  // is done here (the other lanes carry adjacent bytes the master ignores).
  assign rvalid = d_valid;
  assign rid    = d_rid;
  assign rresp  = d_err ? 2'b10 : 2'b00;                 // whole burst carries the request's resp
  assign rlast  = d_valid & d_last;
  assign rdata  = d_err ? '0 : rdb_dout[d_ch][d_chunk*AXI_DW_L +: AXI_DW_L];

  // pop the ROB (and free this request's RD-SRAM run) when the final beat is accepted
  assign resp_ready = d_valid & d_last & rready;

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
          off_q  <= off_nxt[OFF_W_L-1:0];                 // step 2^size within the line
          if (off_wrap) begin
            line_q      <= line_q + 1'b1;                 // crossed into the next 64 B line
            cnt_q[ch]   <= cnt_q[ch] + 1'b1;              // the channel we leave advances a slot
          end
        end
      end

      // ---- request boundary
      if (resp_ready) begin
        run         <= 1'b0;                             // last beat accepted; ROB popped, slots freed
        g_last_done <= 1'b0;
      end else if (!run && resp_valid) begin
        // latch the oldest retired request and start walking it. off_q starts size-aligned
        // inside line L0 (the splitter aligns the start to ar_size, which is <= 64 B, so the
        // start line is addr_lo's line index directly).
        run    <= 1'b1;
        rid_q  <= resp_id;
        err_q  <= (resp_resp != 2'b00);
        head_q <= resp_head_ptr;
        bch_q  <= resp_base_ch;
        len_q  <= resp_len;
        size_q <= resp_size;
        beat_q <= '0;
        line_q <= resp_addr_lo[PAGE_LSB_L-1 : OFF_W_L];
        off_q  <= resp_addr_lo[OFF_W_L-1 : 0] & ~OFF_W_L'((1 << resp_size) - 1);
        cnt_q  <= '0;                                     // per-channel line counters start at 0
      end
    end
  end

`ifndef SYNTHESIS
  // a non-error beat being issued must address a real slot; recovered channel must be
  // active; ar_size must not exceed the full bus width (a legal AXI read).
  always @(posedge aclk) if (aresetn && g_fire && !err_q) begin
    assert (32'(slot) < N_RDB)
      else $error("r_resp: slot %0d out of range (N_RDB=%0d)", slot, N_RDB);
    assert (int'(ch) < (1 << cfg_n_ch_log2))
      else $error("r_resp: recovered channel %0d exceeds active channels", ch);
    assert (int'(size_q) <= BEAT_LSB)
      else $error("r_resp: ar_size %0d exceeds full-width %0d", size_q, BEAT_LSB);
  end
`endif

endmodule : rmc_cif_r_resp
