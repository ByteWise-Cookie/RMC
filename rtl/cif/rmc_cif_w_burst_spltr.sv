// rmc_cif_w_burst_spltr.sv
// WRITE burst splitter (twin of rmc_cif_r_burst_spltr, but in order and issuing writes).
// Pure control: it moves no data and owns no slot pointers. For one admitted write
// request it walks the 64B lines in address order and emits one DRAM write command per
// line, then hands w_rob the per-channel run descriptor.
//
// Per line, paced by the write accumulator's `filled` pulse (the line's data is staged):
//   ch    = walk_channel(line)                  which DRAM channel (asserted via ch/ch_valid)
//   daddr = addrmap(line address)               DRAM coords
//   slot  = write_in_ptr[ch]  (from the frptr block, the same value the accumulator writes at)
//   emit  packet_*[ch] = { rob_idx, daddr, slot } -> MC request FIFO (op=write; mask in SRAM)
//   pulse alloc[ch]        -> frptr advances write_in_ptr[ch]
//   bookkeep: first hit of ch records write_in_ptr[ch] as that channel's run base;
//             cnt[ch]++ ; n_pkt--.
// At n_pkt==0: drive SEG_LAST + SEG_NPKTS (total) + SEG_HEAD_PTR[N_CH] + SEG_NPKTS_CH[N_CH]
// to w_rob, then take the next admitted request. Narrow/unaligned masking rides as the
// per-slot wstrb the accumulator stored in WD-SRAM, not in the packet.
//
// The walk is the same {line,off} + walk_channel as rmc_cif_r_resp. The WD-SRAM write and
// the per-channel ring pointers live in the accumulator and the frptr blocks, not here.
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

  // request in (from the pre-ROB AW_req buffer): address + burst params. Captured with the
  // ROB index when ROB_READY.
  input  logic                   req_in_valid,
  input  logic [AXI_AW-1:0]      req_in_addr,
  input  logic [7:0]             req_in_len,
  input  logic [2:0]             req_in_size,

  // live address map (CSR)
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // admitted slot index from w_rob + its ready
  input  logic [ROB_IDX_W-1:0]   ROB_IDX,
  input  logic                   ROB_READY,

  // per-channel write pointer + space from the frptr blocks (fanned in). The slot a line
  // writes to IS write_in_ptr[ch] - the same value the accumulator writes at and the packet
  // carries. sram_ready[ch] gates issue on that channel's ring having room.
  input  logic [N_CH-1:0][SLOT_W-1:0] write_in_ptr,
  input  logic [N_CH-1:0]             sram_ready,
  output logic [N_CH-1:0]             alloc,        // one-hot: advance the issued channel's write ptr

  // accumulator handshake: filled (a 64B line is staged) in; the splitter asserts the line's
  // channel (ch + ch_valid) so the accumulator commits it to that channel's WD-SRAM.
  input  logic                   filled,
  output logic [CH_W-1:0]        ch,
  output logic                   ch_valid,

  // back to w_rob when the split finishes: total count + per-channel run descriptor
  output logic                   SEG_LAST,
  output logic [CNT_W-1:0]       SEG_NPKTS,
  output logic [N_CH-1:0][SLOT_W-1:0] SEG_HEAD_PTR,  // each channel's first slot (run base)
  output logic [N_CH-1:0][CNT_W-1:0]  SEG_NPKTS_CH,  // each channel's slot count

  // MC write packet stream, ONE PER CHANNEL. The byte mask rides in WD-SRAM (wdb_wstrb),
  // not in the packet; the MC reads WD-SRAM[ch][slot] for data + mask.
  output logic [N_CH-1:0]                packet_valid,
  input  logic [N_CH-1:0]                packet_ready,
  output logic [N_CH-1:0][ROB_IDX_W-1:0] packet_rob_index,
  output logic [N_CH-1:0][DADDR_W-1:0]   packet_daddr,
  output logic [N_CH-1:0][SLOT_W-1:0]    packet_slot    // = write_in_ptr[ch] when issued
);

  // STUB: the in-order {line,off}+walk_channel walk is TODO, built against the frptr and
  // accumulator stubs. Per line: wait filled; ch = walk_channel(line); if sram_ready[ch],
  // assert ch/ch_valid, emit packet_* [ch] with slot = write_in_ptr[ch], pulse alloc[ch];
  // record write_in_ptr[ch] as SEG_HEAD_PTR[ch] on the first hit, SEG_NPKTS_CH[ch]++, n_pkt--.
  // At n_pkt==0 drive SEG_LAST + SEG_NPKTS + SEG_HEAD_PTR[N_CH] + SEG_NPKTS_CH[N_CH].
  assign alloc            = '0;
  assign ch               = '0;
  assign ch_valid         = 1'b0;
  assign SEG_LAST         = 1'b0;
  assign SEG_NPKTS        = '0;
  assign SEG_HEAD_PTR     = '0;
  assign SEG_NPKTS_CH     = '0;
  assign packet_valid     = '0;
  assign packet_rob_index = '0;
  assign packet_daddr     = '0;
  assign packet_slot      = '0;

endmodule : rmc_cif_w_burst_spltr
