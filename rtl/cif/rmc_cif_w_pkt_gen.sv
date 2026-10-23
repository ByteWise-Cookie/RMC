// rmc_cif_w_pkt_gen.sv
// WRITE packet generator (packet_formation_unit). Write-only: the read path issues packets
// per-channel from its own engines (packet-then-data), so it is a separate, simpler unit.
// Turns a line descriptor from the burst splitter into one MC write request. TWO stages, so the
// MC-FIFO handshake never races the WD-SRAM write:
//
//   stage 1 - COMMIT: line staged (stage_valid), channel SRAM has room (sram_ready[ch]), and the
//     one packet-capture register is free. It writes WD-SRAM (wr_en[ch]), captures the packet
//     {rob_idx, daddr, slot = write_in_ptr[ch]} into pkt_capture, pulses alloc[ch] (frptr
//     advances next cycle), take (accumulator clears its line) and ld_ready (splitter walks on).
//     slot is sampled this cycle, one before the pointer moves, so the SRAM write and the packet
//     carry the same slot. It does NOT look at req_fifo_ready - a full MC FIFO just leaves the
//     captured packet in place, so the next commit stalls (back-pressure) rather than dropping
//     a packet while the slot advances (no orphan slot).
//
//   stage 2 - ISSUE: the captured packet drains to the MC request FIFO when req_fifo_ready[ch].
//     pkt_capture mirrors the accumulator's data stage; its valid is channel-gated
//     (packet_valid[ch]) through the ch select. A commit and an issue may happen the same cycle
//     (drain the old packet, capture the new) for full throughput.

import rmc_cfg_pkg::*;

module rmc_cif_w_pkt_gen #(
  parameter int ROB_DEPTH = 32,         // ROB entries               [root knob]
  parameter int N_SLOTS   = 64,         // WD-SRAM slots per channel  [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH) : 1,
  localparam int SLOT_W    = (N_SLOTS   > 1) ? $clog2(N_SLOTS)   : 1
)(
  input  logic                         aclk,
  input  logic                         aresetn,

  // line descriptor in (from the burst splitter): consumed on ld_valid & ld_ready
  input  logic                         ld_valid,
  output logic                         ld_ready,
  input  logic [CH_W-1:0]              ld_ch,
  input  logic [DADDR_W-1:0]           ld_daddr,
  input  logic [ROB_IDX_W-1:0]         ld_rob_idx,

  // accumulator: staged-line flag in, take out (clear the staged line on commit)
  input  logic                         stage_valid,
  output logic                         take,

  // frptr (per channel): slot pointer + room in, alloc pulse out
  input  logic [N_CH-1:0][SLOT_W-1:0]  write_in_ptr,
  input  logic [N_CH-1:0]              sram_ready,
  output logic [N_CH-1:0]              alloc,

  // WD-SRAM write-enable, per channel (the ch-select mux output)
  output logic [N_CH-1:0]              wr_en,

  // MC request FIFO, per channel: room in, ch-gated valid + shared payload out
  input  logic [N_CH-1:0]              req_fifo_ready,
  output logic [N_CH-1:0]              packet_valid,
  output logic [ROB_IDX_W-1:0]         packet_rob_index,
  output logic [DADDR_W-1:0]           packet_daddr,
  output logic [SLOT_W-1:0]            packet_slot
);

  // packet-capture register (stage 1 -> stage 2): one packet, channel-gated on the way out
  logic                  cap_valid;
  logic [CH_W-1:0]       cap_ch;
  logic [ROB_IDX_W-1:0]  cap_rob;
  logic [DADDR_W-1:0]    cap_daddr;
  logic [SLOT_W-1:0]     cap_slot;

  wire [CH_W-1:0] ch = ld_ch;

  // stage 2: drain the captured packet to the MC FIFO when its channel has room
  wire issue_fire = cap_valid & req_fifo_ready[cap_ch];
  // the capture register can take a new packet when empty, or being drained this cycle
  wire cap_free   = ~cap_valid | issue_fire;
  // stage 1: commit when the line is staged, the channel SRAM has room, and capture is free
  wire commit     = ld_valid & stage_valid & sram_ready[ch] & cap_free;

  // stage-1 strobes (combinational, channel-gated)
  assign ld_ready = commit;
  assign take     = commit;
  always_comb begin
    alloc = '0;
    wr_en = '0;
    if (commit) begin
      alloc[ch] = 1'b1;                 // frptr advances write_in_ptr[ch] next cycle
      wr_en[ch] = 1'b1;                 // WD-SRAM latches the staged line this cycle
    end
  end

  // stage-2 outputs: the captured packet, channel-gated valid + shared payload
  always_comb begin
    packet_valid = '0;
    if (cap_valid) packet_valid[cap_ch] = 1'b1;
  end
  assign packet_rob_index = cap_rob;
  assign packet_daddr     = cap_daddr;
  assign packet_slot      = cap_slot;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      cap_valid <= 1'b0;
    end else if (commit) begin          // load a new packet (also drains the old if issue_fire)
      cap_valid <= 1'b1;
      cap_ch    <= ch;
      cap_rob   <= ld_rob_idx;
      cap_daddr <= ld_daddr;
      cap_slot  <= write_in_ptr[ch];    // sampled now; pointer advances next cycle
    end else if (issue_fire) begin      // drained with nothing new to capture
      cap_valid <= 1'b0;
    end
  end

endmodule : rmc_cif_w_pkt_gen
