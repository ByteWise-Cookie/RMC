// rmc_cif_rseg.sv
// READ burst segmenter - full, pipelined. A pre-ROB queue (ring, QUEUE_DEPTH) holds
// admitted requests; N_CH per-channel engines (rmc_cif_rseg_ch) each walk the ring at
// their own pace and issue their channel's 64B packets. An entry pops (in order) when
// every channel has marked it done, pulsing seg_last to the ROB.
//
// No lockstep: a channel that owns many packets holds only its own rd_ptr on one entry;
// the other engines run ahead onto later entries. rob_idx is pre-allocated at enqueue
// and rides in the queue entry, tagging every packet so MC completions route back.
//
// Queue entry {aligned_base, offset, n_pkts, dbuf_base, rob_idx} is written at enqueue;
// n_pkts (needs the unaligned offset) is computed here, once. The offset is stored and
// ridden out on seg_offset for the ROB response trim - engines never touch it.
//
// QUEUE_DEPTH must be a power of 2 (ring index wraps on the low bits).
//
// ---- EXPECTED BEHAVIOR (for FV; bind assertions to these) ----
// Assume the contract map (ch = low CH_W line bits) and stable addr_map while busy.
//   P1 enqueue : (start_req & ready) is the only write; no enqueue when ~ready (full).
//   P2 ids     : a request's packets all carry its rob_idx; rob_idx is monotonic in
//                admit order (pre-alloc), one entry per request.
//   P3 route   : every packet engine c issues maps to channel c
//                (addrmap((aligned_base+k)<<6).ch == c).
//   P4 cover   : over a request's life each line k in [0,n_pkts) is issued exactly once
//                across all channels - no drop, no duplicate. n_pkts = ceil(span/64).
//   P5 stride  : engine c issues k = k0, k0+N_CH, ... with k0=(c-base_ch)&(N_CH-1);
//                dbuf_adr = dbuf_base + k (contiguous).
//   P6 seglast : seg_last pulses once per request, only after all its packets issued
//                (&ch_done at the head), in program order; seg_npkts == n_pkts and
//                seg_offset/len/size/head_ptr equal the admitted values.
//   P7 ring    : occ in [0,QUEUE_DEPTH]; pop only on head &ch_done; no read of a freed
//                slot (every engine rd_ptr in [pop_ptr,alloc_ptr]); engines never lap.
//   P8 flow    : a packet is issued only when its channel pkt_ready is high; a full
//                channel stalls only its own engine, never another.
//
import rmc_cfg_pkg::*;

module rmc_cif_rseg #(
  parameter int ROB_DEPTH   = 32,         // ROB entries (rob_idx width)   [root knob]
  parameter int MAX_PKTS    = 64,         // max packets per request       [root knob]
  parameter int AXI_AW      = SYS_ADDR_W, // AXI address width             [root knob]
  parameter int N_SLOTS     = 32,         // RD-SRAM slots                 [root knob]
  parameter int QUEUE_DEPTH = ROB_DEPTH,  // pre-ROB queue entries; = ROB_DEPTH so a channel
                                          // can hold every outstanding request (no premature
                                          // per-channel admit HOL; stalls only on ROB-full) [knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W  = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1,
  localparam int CNT_W      = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1,
  localparam int SLOT_W     = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1,
  localparam int LINE_W     = AXI_AW - PKT_OFF_W,                     // 64B line number
  localparam int QIDX_W     = (QUEUE_DEPTH > 1) ? $clog2(QUEUE_DEPTH) : 1,
  localparam int QPTR_W     = QIDX_W + 1,                             // extended (wrap bit)
  localparam int DATA_BYTES = AXI_DW / 8,                            // = N_CH*8 (beat width)
  localparam int SIZE_MAX   = (DATA_BYTES   > 1) ? $clog2(DATA_BYTES) : 0,
  localparam int MAX_BEATS  = MAX_REQ_BYTES / DATA_BYTES,
  localparam int LEN_W      = (MAX_BEATS    > 1) ? $clog2(MAX_BEATS)  : 1, // awlen (beats-1)
  localparam int SIZE_W     = (SIZE_MAX     > 0) ? $clog2(SIZE_MAX+1) : 1, // awsize
  localparam int SPAN_W     = $clog2(MAX_REQ_BYTES + 1)              // bytes spanned (<=4KB)
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // live address map (CSR), broadcast to every engine
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // enqueue (one request into the pre-ROB queue)
  input  logic                   start_req,
  input  logic [AXI_AW-1:0]      req_addr,
  input  logic [LEN_W-1:0]       req_len,
  input  logic [SIZE_W-1:0]      req_size,
  input  logic [SLOT_W-1:0]      req_dbuf_base,
  input  logic [ROB_IDX_W-1:0]   req_rob_idx,
  output logic                   ready,           // ~full: queue can accept a request

  // per-channel packet streams out (one req FIFO per channel -> MC)
  output logic [N_CH-1:0]                pkt_valid,
  input  logic [N_CH-1:0]                pkt_ready,
  output logic [N_CH-1:0][ROB_IDX_W-1:0] pkt_rob_idx,
  output logic [N_CH-1:0][DADDR_W-1:0]   pkt_daddr,
  output logic [N_CH-1:0][SLOT_W-1:0]    pkt_dbuf_adr,

  // to the ROB: one pulse when an entry is fully segmented (in-order head pop),
  // plus the read meta the ROB latches for the R-response trim.
  output logic                   seg_last,
  output logic [ROB_IDX_W-1:0]   seg_rob_idx,
  output logic [CNT_W-1:0]       seg_npkts,
  output logic [PKT_OFF_W-1:0]   seg_offset,       // unaligned start offset addr[5:0]
  output logic [LEN_W-1:0]       seg_len,          // ar_len  (beats-1)
  output logic [SIZE_W-1:0]      seg_size,         // ar_size (log2 beat bytes)
  output logic [SLOT_W-1:0]      seg_head_ptr      // RD-SRAM head slot (= dbuf_base)
);

  // Complete as a splitter. Remaining integration / open items live in rmc_cif.sv
  // (RD-SRAM allocator for req_dbuf_base, per-channel req FIFOs + N_CH->1 arbiter for
  // pkt_*, dbuf-addressing choice, scattered-map fallback).

  // pre-ROB queue (ring)
  logic [LINE_W-1:0]    q_aligned_base [QUEUE_DEPTH];
  logic [PKT_OFF_W-1:0] q_offset       [QUEUE_DEPTH];
  logic [CNT_W-1:0]     q_npkts        [QUEUE_DEPTH];
  logic [LEN_W-1:0]     q_len          [QUEUE_DEPTH];   // ar_len  -> resp trim
  logic [SIZE_W-1:0]    q_size         [QUEUE_DEPTH];   // ar_size -> resp trim
  logic [SLOT_W-1:0]    q_dbuf_base    [QUEUE_DEPTH];   // RD-SRAM head slot (= seg_head_ptr)
  logic [ROB_IDX_W-1:0] q_rob_idx      [QUEUE_DEPTH];
  logic [N_CH-1:0]      ch_done        [QUEUE_DEPTH];   // per-channel done bits

  logic [QPTR_W-1:0] alloc_ptr;    // enqueue (extended, wrap bit)
  logic [QPTR_W-1:0] pop_ptr;      // oldest entry (in-order free)

  wire [QIDX_W-1:0] alloc_idx = alloc_ptr[QIDX_W-1:0];
  wire [QIDX_W-1:0] pop_idx   = pop_ptr  [QIDX_W-1:0];
  wire queue_full  = (alloc_ptr - pop_ptr) == QPTR_W'(QUEUE_DEPTH);
  wire queue_empty = (alloc_ptr == pop_ptr);

  assign ready = ~queue_full;
  wire enqueue = start_req & ready;

  // n_pkts at enqueue: span from 64B line start = align_down(offset, size) +
  // (len+1)<<size; ceil(/64). AXI aligns every beat after the first to the beat size,
  // so the burst ends at align_down(addr, size) + (len+1)*2^size - the raw offset
  // would count one line too many for an unaligned start.
  logic [PKT_OFF_W-1:0] off_aligned;
  logic [SPAN_W-1:0]    span;
  logic [CNT_W-1:0]     npkts_calc;
  assign off_aligned = req_addr[PKT_OFF_W-1:0] & ~PKT_OFF_W'((SPAN_W'(1) << req_size) - 1'b1);
  assign span        = SPAN_W'(off_aligned) + ((SPAN_W'(req_len) + 1'b1) << req_size);
  assign npkts_calc = CNT_W'((span + SPAN_W'((1 << PKT_OFF_W) - 1)) >> PKT_OFF_W);

  // engines (one per channel)
  logic [QPTR_W-1:0] eng_rd_ptr   [N_CH];
  logic              eng_set_done [N_CH];

  // set mask: which ch_done bit each engine wants to set this cycle, per entry
  logic [N_CH-1:0]   set_bits [QUEUE_DEPTH];
  always_comb begin
    for (int e = 0; e < QUEUE_DEPTH; e++) set_bits[e] = '0;
    for (int c = 0; c < N_CH; c++)
      if (eng_set_done[c]) set_bits[eng_rd_ptr[c][QIDX_W-1:0]][c] = 1'b1;
  end

  genvar c;
  generate
    for (c = 0; c < N_CH; c++) begin : g_ch
      wire [QIDX_W-1:0] rdi = eng_rd_ptr[c][QIDX_W-1:0];
      rmc_cif_rseg_ch #(
        .ROB_DEPTH   (ROB_DEPTH),
        .MAX_PKTS    (MAX_PKTS),
        .AXI_AW      (AXI_AW),
        .N_SLOTS     (N_SLOTS),
        .QUEUE_DEPTH (QUEUE_DEPTH)
      ) u_ch (
        .aclk           (aclk),
        .aresetn        (aresetn),
        .ch_id          (CH_W'(c)),
        .addr_map       (addr_map),
        .rd_ptr         (eng_rd_ptr[c]),
        .entry_valid    (eng_rd_ptr[c] != alloc_ptr),
        .e_aligned_base (q_aligned_base[rdi]),
        .e_n_pkts       (q_npkts[rdi]),
        .e_dbuf_base    (q_dbuf_base[rdi]),
        .e_rob_idx      (q_rob_idx[rdi]),
        .e_ch_done      (ch_done[rdi][c]),
        .set_done       (eng_set_done[c]),
        .pkt_valid      (pkt_valid[c]),
        .pkt_ready      (pkt_ready[c]),
        .pkt_rob_idx    (pkt_rob_idx[c]),
        .pkt_daddr      (pkt_daddr[c]),
        .pkt_dbuf_adr   (pkt_dbuf_adr[c])
      );
    end
  endgenerate

  // head pop + seg_last (in order)
  wire head_all_done = ~queue_empty & (&ch_done[pop_idx]);
  assign seg_last     = head_all_done;
  assign seg_rob_idx  = q_rob_idx[pop_idx];
  assign seg_npkts    = q_npkts[pop_idx];
  assign seg_offset   = q_offset[pop_idx];
  assign seg_len      = q_len[pop_idx];
  assign seg_size     = q_size[pop_idx];
  assign seg_head_ptr = q_dbuf_base[pop_idx];

  // queue flops
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      alloc_ptr <= '0;
      pop_ptr   <= '0;
      for (int e = 0; e < QUEUE_DEPTH; e++) ch_done[e] <= '0;
    end else begin
      if (enqueue) begin
        q_aligned_base[alloc_idx] <= req_addr[AXI_AW-1:PKT_OFF_W];
        q_offset      [alloc_idx] <= req_addr[PKT_OFF_W-1:0];
        q_npkts       [alloc_idx] <= npkts_calc;
        q_len         [alloc_idx] <= req_len;
        q_size        [alloc_idx] <= req_size;
        q_dbuf_base   [alloc_idx] <= req_dbuf_base;
        q_rob_idx     [alloc_idx] <= req_rob_idx;
        alloc_ptr                 <= alloc_ptr + 1'b1;
      end
      // ch_done: fresh slot clears; otherwise OR in the engines' sets
      for (int e = 0; e < QUEUE_DEPTH; e++) begin
        if (enqueue & (alloc_idx == QIDX_W'(e)))
          ch_done[e] <= '0;
        else
          ch_done[e] <= ch_done[e] | set_bits[e];
      end
      if (head_all_done)
        pop_ptr <= pop_ptr + 1'b1;
    end
  end

endmodule : rmc_cif_rseg
