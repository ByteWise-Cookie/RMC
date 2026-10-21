// rmc_cif_wrob.sv
// WRITE reorder buffer. Admits write requests, tracks completion, retires in program
// order to drive the AXI B response path.
//
// Same ring / admit / completion-count / retire core as rmc_cif_rrob. Differences:
//   - response is B only: RESP_ID + RESP_RESP (BRESP). No read extras.
//   - the per-slot metadata is the per-channel WD-SRAM run base: SEG_HEAD_PTR[N_CH]
//     latched at SEG_LAST, read out at retire so the accumulator can free each
//     channel's whole run (linear per-channel rings, Option B free).
//
// Slot ring: alloc_ptr advances on SEG_LAST, retire_ptr on retire, occ = fill.
// Completion is a pulse {rob_index, err}; outflow up-counts to n_pkts.

import rmc_cfg_pkg::*;

module rmc_cif_wrob #(
  parameter int ROB_DEPTH = 32,   // physical entries (16-32)        [root knob]
  parameter int MAX_PKTS  = 64,   // max packets per request         [root knob]
  parameter int AXI_IDW   = 8,    // AXI id width                    [root knob]
  parameter int N_SLOTS   = 32,   // WD-SRAM slots (chain ptr width) [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W  = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int PTR_W      = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int CNT_W      = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int REQ_META_W = AXI_IDW + 8 + 3 + 1,            // {id,len8,size3,validated1}
  localparam int SLOT_W     = (N_SLOTS > 1) ? $clog2(N_SLOTS) : 1
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // completion in (MC compl async FIFO). Pulse, never back-pressures.
  input  logic                   cmpl_valid,
  input  logic [ROB_IDX_W-1:0]   cmpl_rob_index,
  input  logic                   cmpl_err,         // downstream error -> SLVERR

  // admit (from the pre-ROB AW_req buffer, post request_validator)
  input  logic [REQ_META_W-1:0]  AW_REQUEST,       // {A_ID, A_LEN, A_SIZE, A_VALIDATED}
  input  logic                   A_ID_VALID,
  output logic                   A_ID_READY,       // free slot, none mid-split

  // wrob -> wseg handoff
  output logic [ROB_IDX_W-1:0]   ROB_IDX,          // offered slot index (= alloc_ptr)
  input  logic                   SEG_LAST,         // last packet issued
  input  logic [CNT_W-1:0]       SEG_NPKTS,        // total packet count (= completion target)
  // per-channel WD-SRAM run descriptor from the splitter (latched at SEG_LAST): each channel's
  // first slot and how many slots it holds. Read at retire to free each run.
  input  logic [N_CH-1:0][SLOT_W-1:0] SEG_HEAD_PTR, // each channel's first slot
  input  logic [N_CH-1:0][CNT_W-1:0]  SEG_NPKTS_CH, // each channel's slot count

  // retire / B response
  output logic                   RESP_VALID,
  input  logic                   RESP_READY,
  output logic [AXI_IDW-1:0]     RESP_ID,
  output logic [1:0]             RESP_RESP,        // 00 OKAY / 10 SLVERR
  // per-channel run to free at retire (head + count), handed to the write accumulator
  output logic [N_CH-1:0][SLOT_W-1:0] RESP_HEAD_PTR,
  output logic [N_CH-1:0][CNT_W-1:0]  RESP_NPKTS_CH
);

  // slot ring
  localparam int OCC_W = $clog2(ROB_DEPTH + 1);
  logic [PTR_W-1:0] alloc_ptr;
  logic [PTR_W-1:0] retire_ptr;
  logic [OCC_W-1:0] occ;
  logic             rob_full;
  logic             rob_empty;
  assign rob_full  = (occ == OCC_W'(ROB_DEPTH));
  assign rob_empty = (occ == '0);

  // per-slot entry (flops)
  logic [AXI_IDW-1:0] ent_id        [ROB_DEPTH];
  logic               ent_err       [ROB_DEPTH];
  logic               ent_derr      [ROB_DEPTH];
  logic               inflow_complt [ROB_DEPTH];
  logic [CNT_W-1:0]   n_pkts        [ROB_DEPTH];
  logic [CNT_W-1:0]   outflow_cnt   [ROB_DEPTH];

  // admit payload field-slice
  logic [AXI_IDW-1:0] admit_id;
  logic               admit_ok;
  assign admit_id = AW_REQUEST[REQ_META_W-1 -: AXI_IDW];
  assign admit_ok = AW_REQUEST[0];

  logic pending_split;
  wire  admit_fire = A_ID_VALID & A_ID_READY;
  assign A_ID_READY = ~rob_full & ~pending_split;
  assign ROB_IDX    = alloc_ptr;

  // retire / B response (comb)
  assign RESP_VALID = ~rob_empty
                    & inflow_complt[retire_ptr]
                    & (outflow_cnt[retire_ptr] == n_pkts[retire_ptr]);
  assign RESP_ID    = ent_id[retire_ptr];
  assign RESP_RESP  = (ent_err[retire_ptr] | ent_derr[retire_ptr]) ? 2'b10 : 2'b00;
  wire retire_fire  = RESP_VALID & RESP_READY;

  // per-channel run descriptor store: {head, count} latched at SEG_LAST, read at retire so
  // the accumulator frees each channel's whole run (head..head+count) in one go.
  logic [N_CH-1:0][SLOT_W-1:0] run_head [ROB_DEPTH];
  logic [N_CH-1:0][CNT_W-1:0]  run_cnt  [ROB_DEPTH];
  always_ff @(posedge aclk)
    if (SEG_LAST) begin
      run_head[alloc_ptr] <= SEG_HEAD_PTR;
      run_cnt [alloc_ptr] <= SEG_NPKTS_CH;
    end
  assign RESP_HEAD_PTR  = run_head[retire_ptr];
  assign RESP_NPKTS_CH  = run_cnt [retire_ptr];

  // all flop updates (admit / SEG_LAST / cmpl / retire are parallel)
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      alloc_ptr     <= '0;
      retire_ptr    <= '0;
      occ           <= '0;
      pending_split <= 1'b0;
    end else begin
      if (admit_fire) begin
        ent_id[alloc_ptr]        <= admit_id;
        ent_err[alloc_ptr]       <= ~admit_ok;
        ent_derr[alloc_ptr]      <= 1'b0;
        inflow_complt[alloc_ptr] <= 1'b0;
        outflow_cnt[alloc_ptr]   <= '0;
        pending_split            <= 1'b1;
      end
      if (SEG_LAST) begin
        inflow_complt[alloc_ptr] <= 1'b1;
        n_pkts[alloc_ptr]        <= SEG_NPKTS;
        pending_split            <= 1'b0;
        alloc_ptr <= (alloc_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : alloc_ptr + 1'b1;
      end
      if (cmpl_valid) begin
        outflow_cnt[cmpl_rob_index] <= outflow_cnt[cmpl_rob_index] + 1'b1;
        if (cmpl_err) ent_derr[cmpl_rob_index] <= 1'b1;
      end
      if (retire_fire)
        retire_ptr <= (retire_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : retire_ptr + 1'b1;
      occ <= occ + OCC_W'(admit_fire) - OCC_W'(retire_fire);
    end
  end

endmodule : rmc_cif_wrob
