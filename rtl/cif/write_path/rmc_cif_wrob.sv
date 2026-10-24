// rmc_cif_wrob.sv
// WRITE reorder buffer. No pre-admit: the burst splitter is the sole master and the ONLY writer,
// writing each slot exactly once - entry_valid {id, resp, npkts, npkts_ch} at the last packet.
// So there is no claim/reserve: rob_free is just "not full" and rob_idx = alloc_ptr is offered
// and held until that one entry write advances it. Per-slot completion state is cleared at reset
// and at retire, so a reused slot starts clean.
//
// PER-CHANNEL completions. Each DRAM channel has its own completion (resp) FIFO, so two
// channels can complete the SAME entry in the SAME cycle. A single outflow counter cannot take
// two increments at once, so outflow is kept PER CHANNEL: completion on channel c bumps only
// outflow_cnt[idx][c], and the entry retires when every channel has matched its own target
// run_cnt[idx][c] (the per-channel packet count the splitter wrote). run_cnt doubles as the
// per-channel run freed to the frptr at retire. A back-end error on any channel (cmpl_err[c])
// sticks ent_derr and forces SLVERR; otherwise the response is the entry's stored front BRESP.
// A front error (npkts_ch all 0) retires at once with its BRESP, no DRAM write.
//
// Ring: alloc_ptr issues, retire_ptr drains, occ = fill; retire is in program order.

import rmc_cfg_pkg::*;

module rmc_cif_wrob #(
  parameter int ROB_DEPTH = 32,   // physical entries                [root knob]
  parameter int MAX_PKTS  = 64,   // max packets per request         [root knob]
  parameter int AXI_IDW   = 8,    // AXI id width                    [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int PTR_W     = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int CNT_W     = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // completion in, ONE per channel (each channel's compl async FIFO). Pulse, never back-pressures.
  input  logic [N_CH-1:0]              cmpl_valid,
  input  logic [N_CH-1:0][ROB_IDX_W-1:0] cmpl_rob_index,
  input  logic [N_CH-1:0]              cmpl_err,     // back-end error on that channel -> SLVERR

  // slot offer to the burst splitter (the only writer; no claim/reserve)
  output logic [ROB_IDX_W-1:0]   rob_idx,           // offered slot (= alloc_ptr), held for the req
  output logic                   rob_free,          // a slot is free (ROB not full)

  // single entry write-back from the splitter (at the last packet, or at once on a front error)
  input  logic                   entry_valid,
  input  logic [AXI_IDW-1:0]     entry_id,
  input  logic [1:0]             entry_resp,        // front BRESP: 00 ok / 10 SLVERR / 11 DECERR
  input  logic [CNT_W-1:0]       entry_npkts,       // total completion target (informational)
  input  logic [N_CH-1:0][CNT_W-1:0]  entry_npkts_ch, // per-channel target (= retire + free counts)

  // retire / B response
  output logic                   RESP_VALID,
  input  logic                   RESP_READY,
  output logic [AXI_IDW-1:0]     RESP_ID,
  output logic [1:0]             RESP_RESP,
  // per-channel run length to free at retire, handed to the frptr
  output logic [N_CH-1:0][CNT_W-1:0]  RESP_NPKTS_CH
);

  localparam int OCC_W = $clog2(ROB_DEPTH + 1);     // 0..ROB_DEPTH

  // slot ring
  logic [PTR_W-1:0] alloc_ptr;
  logic [PTR_W-1:0] retire_ptr;
  logic [OCC_W-1:0] occ;
  wire  rob_full  = (occ == OCC_W'(ROB_DEPTH));
  wire  rob_empty = (occ == '0);

  // per-slot entry (flops)
  logic [AXI_IDW-1:0]         ent_id        [ROB_DEPTH];
  logic [1:0]                 ent_resp      [ROB_DEPTH];   // front-end BRESP (00 if ok)
  logic                       ent_derr      [ROB_DEPTH];   // back-end (completion) error -> SLVERR
  logic                       inflow_complt [ROB_DEPTH];   // the splitter wrote the entry
  logic [CNT_W-1:0]           n_pkts        [ROB_DEPTH];   // total target (informational)
  logic [N_CH-1:0][CNT_W-1:0] run_cnt       [ROB_DEPTH];   // per-channel target + frptr free count
  logic [N_CH-1:0][CNT_W-1:0] outflow_cnt   [ROB_DEPTH];   // completions seen, per channel

  // slot offer: the splitter is the only writer, so a free slot is simply "not full"
  assign rob_idx  = alloc_ptr;
  assign rob_free = ~rob_full;

  // retire: every channel's completions have reached its per-channel target
  logic all_ch_done;
  always_comb begin
    all_ch_done = 1'b1;
    for (int c = 0; c < N_CH; c++)
      if (outflow_cnt[retire_ptr][c] != run_cnt[retire_ptr][c]) all_ch_done = 1'b0;
  end

  // retire / B response (comb). A back-end error forces SLVERR; else the stored front BRESP.
  assign RESP_VALID     = ~rob_empty & inflow_complt[retire_ptr] & all_ch_done;
  assign RESP_ID        = ent_id[retire_ptr];
  assign RESP_RESP      = ent_derr[retire_ptr] ? 2'b10 : ent_resp[retire_ptr];
  assign RESP_NPKTS_CH  = run_cnt[retire_ptr];
  wire   retire_fire    = RESP_VALID & RESP_READY;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      alloc_ptr  <= '0;
      retire_ptr <= '0;
      occ        <= '0;
      for (int s = 0; s < ROB_DEPTH; s++) begin
        inflow_complt[s] <= 1'b0;
        outflow_cnt[s]   <= '0;
        ent_derr[s]      <= 1'b0;
      end
    end else begin
      // ENTRY: the one write per slot - populate, mark complete-able, advance (reserve is
      // implicit: alloc_ptr only moves here).
      if (entry_valid) begin
        ent_id[alloc_ptr]        <= entry_id;
        ent_resp[alloc_ptr]      <= entry_resp;
        n_pkts[alloc_ptr]        <= entry_npkts;
        run_cnt[alloc_ptr]       <= entry_npkts_ch;
        inflow_complt[alloc_ptr] <= 1'b1;
        alloc_ptr <= (alloc_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : alloc_ptr + 1'b1;
      end

      // completion, per channel: bump this channel's outflow on the named entry; err sticks.
      // Two channels may hit the same entry the same cycle - they touch different counters
      // (outflow_cnt[idx][c]), and both setting ent_derr[idx] write the same value.
      for (int c = 0; c < N_CH; c++)
        if (cmpl_valid[c]) begin
          outflow_cnt[cmpl_rob_index[c]][c] <= outflow_cnt[cmpl_rob_index[c]][c] + CNT_W'(1);
          if (cmpl_err[c]) ent_derr[cmpl_rob_index[c]] <= 1'b1;
        end

      // retire: advance and clear the slot's completion state so it starts clean when reused
      if (retire_fire) begin
        inflow_complt[retire_ptr] <= 1'b0;
        outflow_cnt[retire_ptr]   <= '0;
        ent_derr[retire_ptr]      <= 1'b0;
        retire_ptr <= (retire_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : retire_ptr + 1'b1;
      end

      occ <= occ + OCC_W'(entry_valid) - OCC_W'(retire_fire);
    end
  end

endmodule : rmc_cif_wrob
