// rmc_cif_wrob.sv
// WRITE reorder buffer. The burst splitter is its only master, in two steps (mirrors the read
// ROB so a completion never lands on an un-entered slot):
//   ADMIT (request arrival): reserve the offered slot ROB_IDX (= alloc_ptr), write {id, resp},
//     clear its completion state, advance alloc_ptr. The entry is now LIVE - completions for
//     it can arrive (and count) while the splitter is still walking the request.
//   SEAL  (last packet, or at once on a front error): write the per-channel target npkts_ch and
//     set inflow_complt - the split is done. alloc_ptr already moved at admit.
//
// PER-CHANNEL completions. Each DRAM channel has its own completion FIFO, so two channels can
// complete the same entry the same cycle; outflow is per channel and an entry retires when
// every channel reaches run_cnt[c] AND it has been sealed. run_cnt doubles as RESP_NPKTS_CH,
// the per-channel run freed to the frptr at retire. A back-end error (cmpl_err[c]) sticks
// ent_derr -> SLVERR; else the stored front BRESP. A front error seals with npkts_ch all 0,
// so it retires at once with its BRESP and no DRAM write.
//
// Ring: alloc_ptr advances at admit, retire_ptr at retire, occ = fill; retire is in order.

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
  input  logic [N_CH-1:0]              cmpl_err,

  // slot offer + ADMIT (from the burst splitter, the only writer)
  output logic [ROB_IDX_W-1:0]   ROB_IDX,           // offered slot (= alloc_ptr)
  output logic                   rob_free,          // a slot is free (ROB not full)
  input  logic                   admit_valid,       // splitter captured a request onto ROB_IDX
  input  logic [AXI_IDW-1:0]     admit_id,
  input  logic [1:0]             admit_resp,        // front BRESP: 00 ok / 10 SLVERR / 11 DECERR

  // SEAL (at the last packet / at once on a front error): per-channel target + split done
  input  logic                   SEG_LAST,
  input  logic [ROB_IDX_W-1:0]   SEG_ROB_IDX,
  input  logic [N_CH-1:0][CNT_W-1:0]  SEG_NPKTS_CH,

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
  logic                       inflow_complt [ROB_DEPTH];   // sealed: split done, npkts_ch valid
  logic [N_CH-1:0][CNT_W-1:0] run_cnt       [ROB_DEPTH];   // per-channel target + frptr free count
  logic [N_CH-1:0][CNT_W-1:0] outflow_cnt   [ROB_DEPTH];   // completions seen, per channel

  wire admit_fire = admit_valid & rob_free;         // reserve the offered slot this cycle
  assign ROB_IDX  = alloc_ptr;
  assign rob_free = ~rob_full;

  // retire: sealed and every channel's completions have reached its per-channel target
  logic all_ch_done;
  always_comb begin
    all_ch_done = 1'b1;
    for (int c = 0; c < N_CH; c++)
      if (outflow_cnt[retire_ptr][c] != run_cnt[retire_ptr][c]) all_ch_done = 1'b0;
  end

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
    end else begin
      // ADMIT: reserve + enter the slot, clear its completion state, advance. The entry is
      // live now, so completions that beat the split land on a real, cleared counter.
      if (admit_fire) begin
        ent_id[alloc_ptr]        <= admit_id;
        ent_resp[alloc_ptr]      <= admit_resp;
        ent_derr[alloc_ptr]      <= 1'b0;
        inflow_complt[alloc_ptr] <= 1'b0;
        outflow_cnt[alloc_ptr]   <= '0;
        alloc_ptr <= (alloc_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : alloc_ptr + 1'b1;
      end

      // SEAL: the named slot's split is done; write its per-channel target.
      if (SEG_LAST) begin
        run_cnt[SEG_ROB_IDX]       <= SEG_NPKTS_CH;
        inflow_complt[SEG_ROB_IDX] <= 1'b1;
      end

      // completion, per channel: bump this channel's outflow on the named entry; err sticks.
      for (int c = 0; c < N_CH; c++)
        if (cmpl_valid[c]) begin
          outflow_cnt[cmpl_rob_index[c]][c] <= outflow_cnt[cmpl_rob_index[c]][c] + CNT_W'(1);
          if (cmpl_err[c]) ent_derr[cmpl_rob_index[c]] <= 1'b1;
        end

      if (retire_fire)
        retire_ptr <= (retire_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : retire_ptr + 1'b1;

      occ <= occ + OCC_W'(admit_fire) - OCC_W'(retire_fire);
    end
  end

endmodule : rmc_cif_wrob
