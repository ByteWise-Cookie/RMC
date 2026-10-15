// rmc_cif_r_burst_spltr.sv
// READ burst splitter - top. Takes one validated AXI AR per cycle, splits it into 64B line
// packets routed to the DRAM channel each line maps to. Three parts:
//   1. precompute   : on admit, works out how many packets land on every channel
//   2. shared queue : one ring entry per in-flight request, walked by N_CH engines
//   3. head pop     : when every channel has finished the oldest entry, pulse seg_last
// The N_CH per-channel engines (rmc_cif_r_burst_spltr_ch) are instanced here.
//
// TERMS
//   line  : 64B DRAM atom (one BL16 on a 32b subchannel). Byte bits [5:0] are the in-line
//           offset, fixed; every bit above is mappable.
//   page  : 4KB. An AXI INCR burst never crosses it, so only line bits [11:6] change inside
//           one request -> at most 64 lines, indexed i = 0..63 inside the page.
//   l0    : line index of the first line = ar_addr[11:6];  l_last = line index of the last.
//   base  : first line number = ar_addr[AXI_AW-1:6].
//   CH    : channel bits taken from addr_map; each can sit at any address bit >= 6 and need
//           not be adjacent. rmc_cfg_pkg::line_channel is the single decode used here AND in
//           the engines, so per-channel counts and per-line routing can never disagree.
//
// 1. PRECOMPUTE (admit cycle, combinational from the s0 AR register)
//    Bounds (12-bit math, 4KB rule): a_al = ar_addr[11:0] with low ar_size bits cleared;
//    end = a_al + (ar_len+1)*2^ar_size - 1; l0 = ar_addr[11:6]; l_last = end[11:6].
//    n_pkts = l_last - l0 + 1. An INCR burst covers a contiguous run of lines l0..l_last, so
//    the count of lines is just last - first + 1 (inclusive), 1..64.
//    Per channel: for each page line i, in_range[i] = l0<=i<=l_last and channel(i) via
//    line_channel; ch_pkts[c] = popcount of lines with in_range & channel==c. sum == n_pkts.
//    Not validated -> all ch_pkts = 0, n_pkts reported 0 (retire SLVERR, nothing to DRAM).
//    Admit = s0 valid & rob_free & queue not full. Same cycle: drive admit_* to the ROB and
//    enqueue, atomically.
//
// 2. SHARED QUEUE (ring, QUEUE_DEPTH = ROB_DEPTH, power of 2)
//    Per entry: base, ch_pkts[N_CH], rob_idx, ch_done[N_CH], head_ptr[N_CH].
//    Enqueue at alloc_ptr: write fields, ch_done[c] = (ch_pkts[c]==0) preset. Engine c reads
//    the entry at its rd_ptr[c]; it writes head_ptr[c] on its first packet and set_done[c] on
//    its last. ch_done is per-(entry,channel) state the queue owns: engine c drives ONLY bit
//    [c], so two channels finishing one request the same cycle set different bits - no
//    conflict, no arbitration. e_ch_done is the engine's read of that bit; set_done is its
//    write-enable - the loop runs through this flop, nothing is inout. entry_valid[c] =
//    rd_ptr[c] != alloc_ptr keeps an engine off the slot being enqueued, and queue-full blocks
//    admit, so enqueue and engine writes never hit the same slot.
//
// 3. HEAD POP -> seg_last
//    Oldest entry pops when queue not empty and &ch_done[pop_ptr]. The &ch_done it reads is
//    the stored bits (set by the engines' set_done or the enqueue preset); that is how a
//    channel's set_done reaches the pop. That cycle: seg_last, seg_rob_idx = entry rob_idx,
//    seg_head_ptr = entry head_ptr[N_CH]. pop_ptr++. Pop is in order, so seg_last is in
//    program order. Pop moves <=1/cycle and skip advances an engine <=1/cycle, and &ch_done
//    means every engine already passed pop_ptr, so pop never overtakes an engine's rd_ptr.
//
// RULES
//   - 4KB crossing rejected upstream (request_validator). Parked.
//   - N_SLOTS >= MAX_PKTS (64): one request can land every line on one channel; RD-SRAM slots
//     free only at retire, so smaller N_SLOTS can self-deadlock.
//   - N_SLOTS and QUEUE_DEPTH powers of 2. addr_map / cfg_n_ch_log2 static after boot.

import rmc_cfg_pkg::*;

module rmc_cif_r_burst_spltr #(
  parameter int ROB_DEPTH   = 32,          // ROB entries (rob_idx width)            [root knob]
  parameter int MAX_PKTS    = 64,          // max lines per request (4KB / 64B)      [root knob]
  parameter int AXI_AW      = SYS_ADDR_W,  // AXI byte address width                 [root knob]
  parameter int AXI_IDW     = 8,           // AXI id width                           [root knob]
  parameter int N_SLOTS     = 64,          // RD-SRAM slots per channel (>= MAX_PKTS) [root knob]
  parameter int QUEUE_DEPTH = ROB_DEPTH,   // shared queue entries, power of 2       [knob]

  // derived (localparam: never passed in)
  localparam int LINE_LSB   = PKT_OFF_W,                                // log2(64B line) = 6
  localparam int PAGE_LSB   = rmc_cfg_pkg::PAGE_LSB,                    // log2(4KB page), value from pkg
  localparam int PG_LINES   = rmc_cfg_pkg::PG_LINES,                    // lines per page (64), value from pkg
  localparam int ROB_IDX_W  = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1, // rob_idx width
  localparam int CNT_W      = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1, // packet count 0..MAX_PKTS
  localparam int SLOT_W     = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1, // RD-SRAM slot width
  localparam int LINE_W     = AXI_AW - LINE_LSB,                        // line number width
  localparam int QIDX_W     = (QUEUE_DEPTH > 1) ? $clog2(QUEUE_DEPTH) : 1, // queue index width
  localparam int QPTR_W     = QIDX_W + 1,                               // queue ptr = index + wrap
  localparam int DATA_BYTES = AXI_DW / 8,                               // AXI beat bytes
  localparam int SIZE_MAX   = (DATA_BYTES > 1) ? $clog2(DATA_BYTES) : 0, // max ar_size
  localparam int MAX_BEATS  = MAX_REQ_BYTES / DATA_BYTES,               // max beats per request
  localparam int LEN_W      = (MAX_BEATS  > 1) ? $clog2(MAX_BEATS)  : 1, // ar_len width
  localparam int SIZE_W     = (SIZE_MAX   > 0) ? $clog2(SIZE_MAX+1) : 1, // ar_size width
  localparam int NCH_LOG_W  = $clog2(CH_W + 1),                         // cfg_n_ch_log2 width
  localparam int LN_W       = (PG_LINES > 1) ? $clog2(PG_LINES) : 1     // line-in-page index (6)
)(
  input  logic                           aclk,              // clock
  input  logic                           aresetn,           // async reset, active low

  // config (boot-time CSR, static after init)
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,         // per-dest source bit select
  input  logic [NCH_LOG_W-1:0]           cfg_n_ch_log2,     // active channels = 2^this (<= N_CH)

  // AR in (from request_validator)
  input  logic                           ar_valid,          // request offered
  output logic                           ar_ready,          // accepted (ROB + queue have room)
  input  logic [AXI_IDW-1:0]             ar_id,             // AXI id
  input  logic [AXI_AW-1:0]              ar_addr,           // start byte address, may be unaligned
  input  logic [LEN_W-1:0]               ar_len,            // beats - 1
  input  logic [SIZE_W-1:0]              ar_size,           // log2(bytes per beat)
  input  logic                           ar_validated,      // 0 -> SLVERR, nothing sent to DRAM

  // ROB admit (writes ROB meta SRAM #1 at rob_idx)
  input  logic                           rob_free,          // ROB has a free slot
  input  logic [ROB_IDX_W-1:0]           rob_idx,           // slot the ROB offers (its alloc_ptr)
  output logic                           admit_valid,       // admit fires this cycle (ROB allocs)
  output logic [AXI_IDW-1:0]             admit_id,          // -> RESP_ID
  output logic                           admit_validated,   // 0 -> SLVERR
  output logic [CNT_W-1:0]               admit_npkts,       // total lines; ROB completion target
  output logic [PAGE_LSB-1:0]            admit_addr_lo,     // ar_addr[11:0]: [5:0] trim, [11:6] l0
  output logic [LEN_W-1:0]               admit_len,         // ar_len  -> resp beat slicing
  output logic [SIZE_W-1:0]              admit_size,        // ar_size -> resp beat slicing
  output logic [CH_W-1:0]                admit_base_ch,     // channel of line 0 (resp CH walk)
  output logic [N_CH-1:0]                admit_ch_mask,     // channels with ch_pkts > 0

  // packets out, one stream per channel (from the engines)
  output logic [N_CH-1:0]                pkt_valid,         // packet offered on channel c
  input  logic [N_CH-1:0]                pkt_ready,         // channel c command FIFO accepts
  input  logic [N_CH-1:0]                pkt_dbuf_adr_ready,// channel c RD-SRAM has a free slot
  output logic [N_CH-1:0][ROB_IDX_W-1:0] pkt_rob_idx,       // completion tag
  output logic [N_CH-1:0][DADDR_W-1:0]   pkt_daddr,         // DRAM coords of the line
  output logic [N_CH-1:0][SLOT_W-1:0]    pkt_dbuf_adr,      // RD-SRAM slot the line returns to

  // split done -> ROB (writes ROB meta SRAM #2 at seg_rob_idx)
  output logic                           seg_last,          // oldest entry fully split (pulse)
  output logic [ROB_IDX_W-1:0]           seg_rob_idx,       // its ROB slot
  output logic [N_CH-1:0][SLOT_W-1:0]    seg_head_ptr,      // per-channel first RD-SRAM slot

  output logic                           idle               // queue empty, all engines idle
);

  // s0: 1-entry AR register. ar_ready drops only while s0 is held waiting to admit.
  logic               s0_valid;
  logic [AXI_IDW-1:0] s0_id;
  logic [AXI_AW-1:0]  s0_addr;
  logic [LEN_W-1:0]   s0_len;
  logic [SIZE_W-1:0]  s0_size;
  logic               s0_validated;

  // queue ring pointers (declared early; admit needs q_full)
  logic [QPTR_W-1:0]  alloc_ptr, pop_ptr;
  wire  [QIDX_W-1:0]  alloc_idx = alloc_ptr[QIDX_W-1:0];
  wire  [QIDX_W-1:0]  pop_idx   = pop_ptr[QIDX_W-1:0];
  wire                q_empty   = (alloc_ptr == pop_ptr);
  wire                q_full    = (alloc_idx == pop_idx) & (alloc_ptr[QIDX_W] != pop_ptr[QIDX_W]);

  logic admit;
  assign admit    = s0_valid & rob_free & ~q_full;
  assign ar_ready = ~s0_valid | admit;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) s0_valid <= 1'b0;
    else if (ar_ready) s0_valid <= ar_valid;
  end
  always_ff @(posedge aclk)
    if (ar_ready & ar_valid) begin
      s0_id        <= ar_id;
      s0_addr      <= ar_addr;
      s0_len       <= ar_len;
      s0_size      <= ar_size;
      s0_validated <= ar_validated;
    end

  // PRECOMPUTE: bounds (12-bit math) + per-line channel + per-channel popcount
  logic [PAGE_LSB-1:0] size_mask, a_al;
  logic [15:0]         n_bytes, end16;
  logic [LN_W-1:0]     l0, l_last;
  logic [CNT_W-1:0]    n_pkts;
  logic [LINE_W-1:0]   base_line;

  assign size_mask = PAGE_LSB'((32'd1 << s0_size) - 32'd1);
  assign a_al      = s0_addr[PAGE_LSB-1:0] & ~size_mask;
  assign n_bytes   = (16'(s0_len) + 16'd1) << s0_size;
  assign end16     = 16'(a_al) + n_bytes - 16'd1;
  assign l0        = s0_addr[PAGE_LSB-1:LINE_LSB];
  assign l_last    = end16[PAGE_LSB-1:LINE_LSB];
  assign n_pkts    = CNT_W'(l_last) - CNT_W'(l0) + CNT_W'(1);
  assign base_line = s0_addr[AXI_AW-1:LINE_LSB];

  // byte address of page line i for the s0 request
  function automatic logic [AXI_AW-1:0] s0_line_byte(input logic [LN_W-1:0] i);
    s0_line_byte = {s0_addr[AXI_AW-1:PAGE_LSB], i, {LINE_LSB{1'b0}}};
  endfunction

  logic [PG_LINES-1:0]            in_range;
  logic [PG_LINES-1:0][CH_W-1:0]  line_ch_v;
  always_comb
    for (int i = 0; i < PG_LINES; i++) begin
      in_range[i]  = (LN_W'(i) >= l0) & (LN_W'(i) <= l_last);
      line_ch_v[i] = rmc_cfg_pkg::line_channel(s0_line_byte(LN_W'(i)), addr_map, cfg_n_ch_log2);
    end

  logic [N_CH-1:0][CNT_W-1:0] ch_pkts, ch_pkts_eff;
  always_comb begin
    for (int c = 0; c < N_CH; c++) ch_pkts[c] = '0;
    for (int i = 0; i < PG_LINES; i++)
      if (in_range[i]) ch_pkts[line_ch_v[i]] = ch_pkts[line_ch_v[i]] + CNT_W'(1);
  end

  logic [CH_W-1:0] base_ch;
  assign base_ch = line_ch_v[l0];
  always_comb
    for (int c = 0; c < N_CH; c++)
      ch_pkts_eff[c] = s0_validated ? ch_pkts[c] : '0;

  // admit outputs (ROB meta SRAM #1)
  assign admit_valid     = admit;
  assign admit_id        = s0_id;
  assign admit_validated = s0_validated;
  assign admit_npkts     = s0_validated ? n_pkts : '0;
  assign admit_addr_lo   = s0_addr[PAGE_LSB-1:0];
  assign admit_len       = s0_len;
  assign admit_size      = s0_size;
  assign admit_base_ch   = base_ch;
  always_comb
    for (int c = 0; c < N_CH; c++) admit_ch_mask[c] = (ch_pkts_eff[c] != '0);

  // SHARED QUEUE storage
  logic [LINE_W-1:0]           q_base    [QUEUE_DEPTH];
  logic [N_CH-1:0][CNT_W-1:0]  q_ch_pkts [QUEUE_DEPTH];
  logic [ROB_IDX_W-1:0]        q_rob     [QUEUE_DEPTH];
  logic [N_CH-1:0]             q_done    [QUEUE_DEPTH];
  logic [N_CH-1:0][SLOT_W-1:0] q_head    [QUEUE_DEPTH];

  // ENGINES: one per channel, each walking the queue with its own rd_ptr
  logic [N_CH-1:0][QPTR_W-1:0] eng_rd;
  logic [N_CH-1:0][QIDX_W-1:0] eng_ri;
  logic [N_CH-1:0]             eng_set_done, eng_head_we, eng_idle;
  logic [N_CH-1:0][SLOT_W-1:0] eng_head;

  always_comb
    for (int c = 0; c < N_CH; c++) eng_ri[c] = eng_rd[c][QIDX_W-1:0];

  generate
    for (genvar c = 0; c < N_CH; c++) begin : g_eng
      rmc_cif_r_burst_spltr_ch #(
        .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_AW (AXI_AW),
        .N_SLOTS (N_SLOTS), .QUEUE_DEPTH (QUEUE_DEPTH)
      ) u_ch (
        .aclk, .aresetn,
        .ch_id              (CH_W'(c)),
        .addr_map           (addr_map),
        .cfg_n_ch_log2      (cfg_n_ch_log2),
        .rd_ptr             (eng_rd[c]),
        .entry_valid        (eng_rd[c] != alloc_ptr),
        .e_base             (q_base[eng_ri[c]]),
        .e_ch_pkts          (q_ch_pkts[eng_ri[c]][c]),
        .e_rob_idx          (q_rob[eng_ri[c]]),
        .e_ch_done          (q_done[eng_ri[c]][c]),
        .set_done           (eng_set_done[c]),
        .head_ptr_we        (eng_head_we[c]),
        .head_ptr           (eng_head[c]),
        .pkt_valid          (pkt_valid[c]),
        .pkt_ready          (pkt_ready[c]),
        .pkt_dbuf_adr_ready (pkt_dbuf_adr_ready[c]),
        .pkt_rob_idx        (pkt_rob_idx[c]),
        .pkt_daddr          (pkt_daddr[c]),
        .pkt_dbuf_adr       (pkt_dbuf_adr[c]),
        .idle               (eng_idle[c])
      );
    end
  endgenerate

  // HEAD POP: oldest entry retires when every channel is done with it
  wire all_done = &q_done[pop_idx];
  wire pop      = ~q_empty & all_done;
  assign seg_last     = pop;
  assign seg_rob_idx  = q_rob[pop_idx];
  assign seg_head_ptr = q_head[pop_idx];

  assign idle = (&eng_idle) & q_empty;

  // QUEUE updates: enqueue (admit), per-channel head/done writes (engines), pop.
  // admit and engine writes never target the same slot (entry_valid + q_full gates); the
  // engine writes are guarded against the alloc slot anyway so the intent is explicit.
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      alloc_ptr <= '0;
      pop_ptr   <= '0;
    end else begin
      if (admit) begin
        q_base[alloc_idx] <= base_line;
        q_rob[alloc_idx]  <= rob_idx;
        for (int c = 0; c < N_CH; c++) begin
          q_ch_pkts[alloc_idx][c] <= ch_pkts_eff[c];
          q_done[alloc_idx][c]    <= (ch_pkts_eff[c] == '0); // preset done where nothing to do
        end
        alloc_ptr <= alloc_ptr + 1'b1;
      end

      for (int c = 0; c < N_CH; c++) begin
        if (eng_head_we[c] & ~(admit & (eng_ri[c] == alloc_idx)))
          q_head[eng_ri[c]][c] <= eng_head[c];
        if (eng_set_done[c] & ~(admit & (eng_ri[c] == alloc_idx)))
          q_done[eng_ri[c]][c] <= 1'b1;
      end

      if (pop) pop_ptr <= pop_ptr + 1'b1;
    end
  end

`ifndef SYNTHESIS
  // sum of per-channel counts must equal the total packet count on every admit
  logic [CNT_W+CH_W-1:0] cnt_sum;
  always_comb begin
    cnt_sum = '0;
    for (int c = 0; c < N_CH; c++) cnt_sum = cnt_sum + (CNT_W+CH_W)'(ch_pkts_eff[c]);
  end
  always_ff @(posedge aclk)
    if (aresetn & admit) begin
      if (end16[15:PAGE_LSB] != '0)
        $error("r_burst_spltr: AR crosses 4KB (addr=%h len=%0d size=%0d)", s0_addr, s0_len, s0_size);
      if (s0_validated & (cnt_sum != (CNT_W+CH_W)'(n_pkts)))
        $error("r_burst_spltr: sum(ch_pkts)=%0d != n_pkts=%0d", cnt_sum, n_pkts);
    end
`endif

endmodule : rmc_cif_r_burst_spltr
