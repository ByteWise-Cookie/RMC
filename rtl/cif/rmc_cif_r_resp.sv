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
//     slot) is reused across 2^(BEAT_LSB-size) beats -- the walk holds the line
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
// Three stages, one beat per cycle, around an ASYNCHRONOUS (combinational-read) RD-SRAM:
//   G (addr-gen) : counters -> ch, chunk, slot (combinational from the walk state).
//   A (sram)     : register rdb_addr/rdb_re, so the SRAM sees a stable address and gets
//                  its own cycle; the async array returns rdb_dout combinationally.
//   D (data/out) : capture the sliced AXI_DW word in a register and drive R from it.
// Registering both the address (A) and the sliced data (D) keeps the per-cycle logic
// bounded (addr-gen | array+slice | output) and gives a clean registered R output with no
// post-SRAM combinational path to the port. The elastic ready chain (d_ready -> a_ready ->
// g_fire) stalls the walk under rready back-pressure; a stall just holds the A and D
// registers, so there is no re-read and no reliance on the macro holding its output.
// Latency: a beat reaches R two cycles after its read is issued.
//
// Assumptions:
//   - INCR only, inside one 4 KB page (both guaranteed upstream). WRAP/FIXED TODO.
//   - RD-SRAM read is ASYNCHRONOUS: rdb_dout is combinational in the address driven the
//     previous cycle (A registers the address, D registers the data).

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
  localparam int BEAT_LSB   = $clog2(BEAT_BYTES),                        // log2(full beat bytes) = max ar_size (0 if 1 byte; it is an index, not a width)
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
  output logic [N_CH_L-1:0]                  rdb_re, // SRAM read enable
  output logic [N_CH_L-1:0][SLOT_W-1:0]      rdb_addr, // SRAM read addr
  input  logic [N_CH_L-1:0][SRAM_W-1:0]      rdb_dout, // SRAM read data 64Bytes

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
      input logic [PAGE_LSB_L-1:0]            adr_low_tag, // 12bit lsb addr
      input logic [CH_W_L-1:0]                base_ch, // packet 0 ch_addr
      input logic [DEC_W_L-1:0][SEL_W_L-1:0]  amap, // addr map
      input logic [NCH_LOG_W-1:0]             n_ch_log2); // ch_sel
    logic [CH_W_L-1:0] c; //ch_ptr
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

  // A stage: the read is in flight in the (async) RD-SRAM. rdb_addr/rdb_re are registered
  // here, so the SRAM sees a stable address and gets its own cycle; a_* carry the control
  // the D stage needs to slice the returned word.
  logic                 a_valid;
  logic [AXI_IDW-1:0]   a_rid;
  logic                 a_err;
  logic                 a_last;
  logic [CH_W_L-1:0]    a_ch;                            // channel whose rdb_dout to slice
  logic [SUBB_W-1:0]    a_chunk;                         // which AXI beat within the packet

  // D stage: the sliced R beat, captured in a register (clean registered R output).
  logic                 d_valid;
  logic [AXI_IDW-1:0]   d_rid;
  logic                 d_err;
  logic                 d_last;
  logic [AXI_DW_L-1:0]  d_data;                          // the AXI_DW beat, already sliced out

  // G-stage decode of the current beat from the two counters.
  logic [PAGE_LSB_L-1:0] line_tag;    // {line_q, 0}: line address for the channel decode
  logic [CH_W_L-1:0]     ch;          // channel this line maps to
  logic [SUBB_W-1:0]     chunk;       // AXI beat within the 64 B packet
  logic [SLOT_W-1:0]     slot;        // RD-SRAM slot = head + per-channel count
  logic [OFF_W_L:0]      off_nxt;     // next in-line offset (one extra bit for the wrap)
  logic                  off_wrap;    // the step crosses out of the current 64 B line
  logic                  is_last;     // last beat of the burst (ar_len reached)

  assign line_tag = {line_q, {OFF_W_L{1'b0}}};            // current line's in-page addr: line index up top, byte offset forced 0 (the channel decode only reads line bits)
  assign ch       = walk_channel(line_tag, bch_q, addr_map, cfg_n_ch_log2); // ch = the DRAM channel this 64B line's data sits in, re-derived from the descriptor (window mux): the same channel the splitter routed the line to, so head_q[ch]/cnt_q[ch] below index the right per-channel run
  // which AXI_DW beat within the 64B packet (offset bits above one beat width). When a beat
  // is a whole packet (N_SUBB==1, i.e. AXI_DW==SRAM_W) BEAT_LSB==OFF_W and the slice would be
  // null, so chunk is a constant 0.
  if (N_SUBB > 1) begin : g_chunk
    assign chunk = off_q[OFF_W_L-1 : BEAT_LSB];
  end else begin : g_chunk1
    assign chunk = '0;
  end
  assign slot     = head_q[ch] + cnt_q[ch];               // RD-SRAM slot to read: this channel's run base + how many of its lines already streamed
  assign off_nxt  = {1'b0, off_q} + (OFF_W_L+1)'(1 << size_q);  // next in-line byte offset; extra top bit catches the carry; step = 2^ar_size bytes
  assign off_wrap = off_nxt[OFF_W_L];                     // that carry bit = the step crossed out of this 64B line
  assign is_last  = (beat_q == len_q);                    // reached ar_len (beats-1) -> this is the final beat

  // Elastic ready chain over the two registered stages. A stage can take a new read when it
  // is empty or its in-flight read moves into D this cycle; D can take a new beat when it is
  // empty or its held beat is accepted on R this cycle; G issues (and the walk advances) only
  // when A can take it and G still has beats.
  wire d_ready = ~d_valid | rready;                       // D free: empty, or its beat accepted this cycle
  wire a_ready = ~a_valid | d_ready;                      // A free: empty, or its read captured by D this cycle
  wire g_fire  = run & ~g_last_done & a_ready;            // issue a read now: running, not past the last beat, A free

  // R-channel outputs, all from the D register (no post-SRAM combinational path to the port).
  assign rvalid = d_valid;                               // R beat valid whenever D holds one
  assign rid    = d_rid;                                 // id of the request being streamed
  assign rresp  = d_err ? 2'b10 : 2'b00;                 // SLVERR if the request errored, else OKAY (whole burst carries it)
  assign rlast  = d_valid & d_last;                      // last-beat marker (the beat whose is_last was latched)
  assign rdata  = d_data;                                // the already-sliced AXI_DW beat

  // pop the ROB (and free this request's RD-SRAM run) when the final beat is accepted
  assign resp_ready = d_valid & d_last & rready;         // final beat handshakes -> release the ROB entry and its slots

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      run         <= 1'b0;                               // idle, no request in flight
      g_last_done <= 1'b0;                               // no final beat issued yet
      a_valid     <= 1'b0;                               // A stage empty (no read in flight)
      d_valid     <= 1'b0;                               // D stage empty
      rdb_re      <= '0;                                 // no SRAM read driven
    end else begin
      // D stage: capture the word the async SRAM returns for A's in-flight read
      if (d_ready) begin                                 // D is free to update (empty or accepted)
        d_valid <= a_valid;                              // hold a beat iff A had a read in flight, else bubble
        if (a_valid) begin                               // slice the addressed channel's returned packet and register it
          d_data <= a_err ? '0 : rdb_dout[a_ch][a_chunk*AXI_DW_L +: AXI_DW_L]; // chunk-th AXI_DW word (0 on error); narrow master extracts its lane
          d_rid  <= a_rid;                               // request id
          d_err  <= a_err;                               // error flag (SLVERR)
          d_last <= a_last;                              // final-beat marker -> rlast
        end
      end

      // A stage: issue a read (register the address) when A is free; hold it while A stalls so
      // the async SRAM output stays stable for D to capture.
      if (a_ready) begin                                 // A is free to update (empty or captured by D)
        a_valid <= g_fire;                               // a read goes in flight iff G issued one this cycle
        rdb_re  <= '0;                                   // default: drive no read
        if (g_fire) begin                                // register the issued beat's address + control
          a_rid   <= rid_q;                              // request id
          a_err   <= err_q;                              // error flag
          a_last  <= is_last;                            // final beat?
          a_ch    <= ch;                                 // channel, to select rdb_dout next cycle
          a_chunk <= chunk;                              // sub-beat, to slice next cycle
          if (~err_q) begin                              // real beat: drive the registered SRAM read
            rdb_re[ch]   <= 1'b1;                         // read enable on this line's channel
            rdb_addr[ch] <= slot;                         // at its slot
          end
        end
      end

      // G stage: advance the walk on each issued read
      if (g_fire) begin                                  // a read was issued this cycle
        if (is_last) begin                               // it was the final beat of the burst
          g_last_done <= 1'b1;                            // stop issuing; drain A then D
        end else begin
          beat_q <= beat_q + 8'd1;                        // next AXI beat index
          off_q  <= off_nxt[OFF_W_L-1:0];                 // advance in-line offset by 2^size
          if (off_wrap) begin                            // the step crossed into a new 64B line
            line_q      <= line_q + 1'b1;                 // move to the next line index
            cnt_q[ch]   <= cnt_q[ch] + 1'b1;              // the channel we just left advances its slot counter
          end
        end
      end

      // request boundary
      if (resp_ready) begin                              // final beat accepted this cycle
        run         <= 1'b0;                             // request done; ROB popped and slots freed
        g_last_done <= 1'b0;                             // clear for the next request
      end else if (!run && resp_valid) begin             // idle and a retired request is waiting: latch it and start
        // latch the oldest retired request and start walking it. off_q starts size-aligned
        // inside line L0 (the splitter aligns the start to ar_size, which is <= 64 B, so the
        // start line is addr_lo's line index directly).
        run    <= 1'b1;                                  // begin streaming
        rid_q  <= resp_id;                               // latch the AXI id
        err_q  <= (resp_resp != 2'b00);                  // latch error: any non-OKAY resp -> SLVERR burst
        head_q <= resp_head_ptr;                         // per-channel RD-SRAM run bases
        bch_q  <= resp_base_ch;                          // channel of line 0 (holds the frozen out-of-page CH bits)
        len_q  <= resp_len;                              // ar_len (beats - 1)
        size_q <= resp_size;                             // ar_size (log2 bytes per beat)
        beat_q <= '0;                                    // start at beat 0
        line_q <= resp_addr_lo[PAGE_LSB_L-1 : OFF_W_L];  // start line index = addr_lo's line bits
        off_q  <= resp_addr_lo[OFF_W_L-1 : 0] & ~OFF_W_L'((1 << resp_size) - 1); // start offset within the line, aligned down to ar_size
        cnt_q  <= '0;                                     // per-channel line counters start at 0
      end
    end
  end

`ifndef SYNTHESIS
  // elaboration/param guards. The per-channel slot pointer is head_q[ch] + cnt_q[ch] in
  // SLOT_W bits, so it wraps at 2^SLOT_W; that only matches the ring when N_RDB is a power
  // of two (otherwise a run that wraps reads the wrong slot). CH_W < 1 (single channel)
  // collapses every [CH_W-1:0] vector and is not supported yet.
  initial begin
    if ((1 << SLOT_W) != N_RDB)
      $fatal(1, "r_resp: N_RDB (%0d) must be a power of two", N_RDB);
    if (CH_W_L < 1)
      $fatal(1, "r_resp: single-channel build (CH_W=0) not supported");
  end

  // a non-error beat being issued must address a real slot; recovered channel must be
  // active; ar_size must not exceed the full bus width (a legal AXI read).
  always @(posedge aclk) if (aresetn && g_fire && !err_q) begin   // only on a real issued beat
    assert (32'(slot) < N_RDB)                                     // the read must land on a real RD-SRAM slot
      else $error("r_resp: slot %0d out of range (N_RDB=%0d)", slot, N_RDB);
    assert (int'(ch) < (1 << cfg_n_ch_log2))                       // the recovered channel must be one of the active channels
      else $error("r_resp: recovered channel %0d exceeds active channels", ch);
    assert (int'(size_q) <= BEAT_LSB)                              // ar_size cannot exceed the full bus width (a legal AXI read)
      else $error("r_resp: ar_size %0d exceeds full-width %0d", size_q, BEAT_LSB);
  end

  // the address map must route each active CH bit from at or above the 64B line offset; a
  // source below it would make line_tag feed a forced-0 bit and silently corrupt the channel.
  always @(posedge aclk) if (aresetn && run)
    for (int k = 0; k < CH_W_L; k++)
      if (k < int'(cfg_n_ch_log2))
        assert (int'(addr_map[OFF_W_L + k]) >= OFF_W_L)
          else $error("r_resp: CH source bit %0d is below the line offset", k);
`endif

endmodule : rmc_cif_r_resp
