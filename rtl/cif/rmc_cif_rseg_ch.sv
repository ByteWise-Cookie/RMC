// rmc_cif_rseg_ch.sv
// READ burst segmenter - one channel, ring walker. Instanced once per DRAM channel by
// rmc_cif_rseg. Walks the shared pre-ROB queue at its own pace (own rd_ptr), issuing
// only the 64B packets that land in its channel (ch_id), and marks the entry done for
// this channel when its share is drained.
//
// The queue entry {aligned_base, n_pkts, dbuf_base, rob_idx} is presented by the
// wrapper, muxed at this engine's rd_ptr. n_pkts and the start offset are computed at
// enqueue; this engine never touches the offset bits.
//
// STRIDE (contract map: ch = low CH_W bits of the line number):
//   base_ch = aligned_base[CH_W-1:0]
//   k0      = (ch_id - base_ch) mod N_CH           first packet index of this channel
//   k       = k0 + m*N_CH                           m = packets of this channel issued so far
//   line addr = (aligned_base + k) << 6 ; addrmap(line) -> daddr. dbuf = dbuf_base + k.
// Assumes the ch field is the low contiguous line field (contract/default map).
//
// ---- EXPECTED BEHAVIOR (for FV) ----
//   E1 route : every issued packet maps to ch_id (addrmap(line).ch == ch_id).
//   E2 valid : pkt_valid only when entry_valid & ~e_ch_done & k < e_n_pkts; m (hence k)
//              advances only on pkt_valid & pkt_ready.
//   E3 done  : set_done asserts once per entry when k reaches e_n_pkts (incl. the
//              zero-packet case k0 >= n_pkts); rd_ptr advances then, m resets.
//   E4 ptr   : rd_ptr only advances (set_done | skip); never passes alloc_ptr
//              (entry_valid gates work), so it never reads a freed/unwritten slot.
//
import rmc_cfg_pkg::*;

module rmc_cif_rseg_ch #(
  parameter int ROB_DEPTH   = 32,         // ROB entries (rob_idx width)   [root knob]
  parameter int MAX_PKTS    = 64,         // max packets per request       [root knob]
  parameter int AXI_AW      = SYS_ADDR_W, // AXI address width             [root knob]
  parameter int N_SLOTS     = 32,         // RD-SRAM slots                 [root knob]
  parameter int QUEUE_DEPTH = ROB_DEPTH,  // pre-ROB queue entries; = ROB_DEPTH (pow2)  [knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1,
  localparam int CNT_W     = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1,
  localparam int SLOT_W    = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1,
  localparam int LINE_W    = AXI_AW - PKT_OFF_W,                     // 64B line number
  localparam int QIDX_W    = (QUEUE_DEPTH > 1) ? $clog2(QUEUE_DEPTH) : 1,
  localparam int QPTR_W    = QIDX_W + 1                              // extended (wrap bit)
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  input  logic [CH_W-1:0]        ch_id,           // the DRAM channel this engine serves
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // queue read: wrapper presents the entry at this engine's rd_ptr
  output logic [QPTR_W-1:0]      rd_ptr,
  input  logic                   entry_valid,     // rd_ptr != alloc_ptr (entry occupied)
  input  logic [LINE_W-1:0]      e_aligned_base,
  input  logic [CNT_W-1:0]       e_n_pkts,
  input  logic [SLOT_W-1:0]      e_dbuf_base,
  input  logic [ROB_IDX_W-1:0]   e_rob_idx,
  input  logic                   e_ch_done,       // this entry's done bit for ch_id
  output logic                   set_done,        // set that bit this cycle

  // packet stream out, this channel only
  output logic                   pkt_valid,
  input  logic                   pkt_ready,
  output logic [ROB_IDX_W-1:0]   pkt_rob_idx,
  output logic [DADDR_W-1:0]     pkt_daddr,
  output logic [SLOT_W-1:0]      pkt_dbuf_adr
);

  logic [CNT_W-1:0] m;                 // packets of this channel issued for current entry

  // first packet index for this channel (contract map; N_CH is a power of 2)
  logic [CH_W-1:0]  base_ch;
  logic [CH_W-1:0]  k0;
  logic [CNT_W-1:0] k;
  assign base_ch = e_aligned_base[CH_W-1:0];
  assign k0      = (ch_id - base_ch) & CH_W'(N_CH - 1);
  assign k       = CNT_W'(k0) + (CNT_W'(m) << CH_W);     // k0 + m*N_CH

  // current line -> DRAM coords. map_ch/map_off are the map's other outputs; this engine
  // trusts the stride for routing and leaves the write offset to the queue, so they idle.
  logic [AXI_AW-1:0]    line_byte_addr;
  logic [CH_W-1:0]      map_ch;
  logic [PKT_OFF_W-1:0] map_off;
  assign line_byte_addr = AXI_AW'(e_aligned_base + k) << PKT_OFF_W;

  rmc_cif_addrmap #(
    .AXI_AW  (AXI_AW),
    .DADDR_W (DADDR_W)
  ) u_map (
    .sys_addr (line_byte_addr),
    .addr_map (addr_map),
    .daddr    (pkt_daddr),
    .ch       (map_ch),
    .offset   (map_off)
  );

  wire have_work = entry_valid & ~e_ch_done;

  assign pkt_rob_idx  = e_rob_idx;
  assign pkt_dbuf_adr = e_dbuf_base + SLOT_W'(k);
  assign pkt_valid    = have_work & (k <  e_n_pkts);
  assign set_done     = have_work & (k >= e_n_pkts);     // my share drained (incl zero-pkt)

  wire skip = entry_valid & e_ch_done;                   // already done here -> just advance

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      rd_ptr <= '0;
      m      <= '0;
    end else if (set_done | skip) begin
      rd_ptr <= rd_ptr + 1'b1;                           // move to next entry
      m      <= '0;
    end else if (pkt_valid & pkt_ready) begin
      m      <= m + 1'b1;                                // next channel-ch_id packet
    end
  end

endmodule : rmc_cif_rseg_ch
