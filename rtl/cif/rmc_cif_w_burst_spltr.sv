// rmc_cif_w_burst_spltr.sv
// WRITE burst splitter (twin of rmc_cif_r_burst_spltr, but in order and issuing writes).
// Pure control: it moves no data and owns no slot pointers. For one admitted write
// request it walks the 64B lines in address order and emits one DRAM write command per
// line, then hands w_rob the per-channel run descriptor.
//
// Per line, paced by the write accumulator's `filled` pulse (the line's data is staged):
//   ch    = walk_channel(line)                  which DRAM channel (asserted to the alloc)
//   daddr = addrmap(line address)               DRAM coords
//   slot  = head_ptr[ch]   (from the SRAM manager, fanned to the accumulator too)
//   emit  { op=write, rob_idx, ch, daddr, slot, wstrb } -> MC request FIFO
//   pulse alloc[ch]        -> SRAM manager advances head_ptr[ch]
//   bookkeep: first hit of ch records head_ptr[ch] as that channel's run base;
//             cnt[ch]++ ; n_pkt--.
// At n_pkt==0: drive SEG_LAST + SEG_NPKTS (total) + SEG_HEAD_PTR[N_CH] + SEG_NPKTS_CH[N_CH]
// to w_rob, then take the next admitted request. Narrow/unaligned masking rides as wstrb.
//
// The walk is the same {line,off} + walk_channel as rmc_cif_r_resp. The WD-SRAM write and
// the per-channel ring pointers live in the accumulator and the SRAM manager, not here.
//
// STUB: ports + the walk are TODO, built against the accumulator / SRAM manager stubs.

import rmc_cfg_pkg::*;

module rmc_cif_w_burst_spltr #(
  parameter int ROB_DEPTH = 32,         // ROB entries             [root knob]
  parameter int MAX_PKTS  = 64,         // max packets per request [root knob]
  parameter int AXI_AW    = SYS_ADDR_W, // AXI address width       [root knob]
  parameter int N_SLOTS   = 32,         // WD-SRAM slots           [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int PKT_NUM_W = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS)   : 1,
  localparam int CNT_W     = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int SLOT_W    = (N_SLOTS   > 1) ? $clog2(N_SLOTS)    : 1,
  localparam int LINE_W    = AXI_AW - PKT_OFF_W
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // request in (from the pre-ROB AW_req buffer): address + burst params. No slot_base
  // (slots come from the free-list). Captured on req_in_valid & ROB_READY.
  input  logic                   req_in_valid,
  input  logic [AXI_AW-1:0]      req_in_addr,
  input  logic [7:0]             req_in_len,
  input  logic [2:0]             req_in_size,

  // live address map (CSR)
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // admitted slot index from w_rob + its ready
  input  logic [ROB_IDX_W-1:0]   ROB_IDX,
  input  logic                   ROB_READY,

  // WD-SRAM manager free-list: one free slot per packet
  input  logic [SLOT_W-1:0]      free_slot,        // next free WD-SRAM slot
  input  logic                   free_slot_valid,  // a free slot is available
  output logic                   free_slot_take,   // seg consumes a free slot this cycle

  // back to w_rob when the split finishes
  output logic                   SEG_LAST,
  output logic [CNT_W-1:0]       SEG_NPKTS,
  // write slot chain -> w_rob / accumulator (head latched at admit, tail per packet)
  output logic [SLOT_W-1:0]      SEG_HEAD_PTR,     // first slot of the chain
  // TODO: per-packet {slot, prev_slot} so the accumulator links next_ram[slot]

  // packet stream out, ONE PER CHANNEL
  output logic [N_CH-1:0]                packet_valid,
  input  logic [N_CH-1:0]                packet_ready,
  output logic [N_CH-1:0][ROB_IDX_W-1:0] packet_rob_index,
  output logic [N_CH-1:0][DADDR_W-1:0]   packet_daddr,
  output logic [N_CH-1:0][PKT_NUM_W-1:0] packet_pkt_num,
  output logic [N_CH-1:0]                packet_last_in_txn,
  output logic [N_CH-1:0][SLOT_W-1:0]    packet_slot,       // from the free-list, not base+k
  output logic [N_CH-1:0][PKT_OFF_W-1:0] packet_offset,
  output logic [N_CH-1:0]                packet_narrow
);

  // LEFT TO DO: mirror the pipelined read segmenter (rmc_cif_r_burst_spltr + rmc_cif_r_burst_spltr_ch):
  //   - same pre-ROB ring queue + N_CH per-channel stride engines, multi-outstanding,
  //     in-order head pop -> SEG_LAST + SEG_ROB_IDX to w_rob (wrob needs the same
  //     multi-outstanding rework rrob just got).
  //   - n_pkts / k0 / stride math is identical (contract map: ch = low CH_W line bits).
  //   WRITE-SPECIFIC difference = the slot engine: pull free_slot per issued packet
  //   (free_slot_take) instead of dbuf_base+k, build the chain (next_ram[slot]), and
  //   emit SEG_HEAD_PTR (first slot) + per-packet {slot, prev} to w_rob / accumulator.
  //   packet_op = write (tie high at the top). No read-meta sideband (mask rides WSTRB).
  assign free_slot_take     = 1'b0;
  assign SEG_LAST           = 1'b0;
  assign SEG_NPKTS          = '0;
  assign SEG_HEAD_PTR       = '0;
  assign packet_valid       = '0;
  assign packet_rob_index   = '0;
  assign packet_daddr       = '0;
  assign packet_pkt_num     = '0;
  assign packet_last_in_txn = '0;
  assign packet_slot        = '0;
  assign packet_offset      = '0;
  assign packet_narrow      = '0;

endmodule : rmc_cif_w_burst_spltr
