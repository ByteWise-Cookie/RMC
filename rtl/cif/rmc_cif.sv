// rmc_cif.sv
// Client interface (CIF) top. Sits between the client AXI4 bus and the MC core.
// Owns: address map (system -> DRAM coords), burst segmentation (<=16-beat
// packets), reorder buffer (program order + completion), same-line R/W hazard
// interlock, and the client-side ports of the WD/RD data SRAMs.
//
// Boundary to MC = two async FIFOs (external, one per direction). CIF presents a
// synchronous valid/ready stream on its side; CDC lives in the FIFO, not here.
//   out : request packets  {rob_index, op, daddr, pkt_num, last_in_txn, sram_slot}
//   in  : completions      {rob_index, pkt_num, status}   (13b, tag-only)
// Read data is written by MC straight into RD_SRAM; CIF drains it to the R channel.
//
// STUB: top ports + sub-block instances + internal nets. Logic TODO per block.

import rmc_cfg_pkg::*;

module rmc_cif #(
  // client AXI4
  parameter int AXI_IDW   = 8,
  parameter int AXI_AW    = 48,        // map needs >=41 (row ends at bit 40)
  // AXI_DW is NOT a free param: it is rmc_cfg_pkg::AXI_DW = N_CH*DDR_CHANNEL_W
  // (matched BW, inflow==outflow). Imported below; do not override per-instance.
  // reorder buffer / packetization
  parameter int ROB_DEPTH = 32,        // physical ROB entries (16-32)
  parameter int ROB_IDX_W = 8,         // tag width = CIF ROB_INDEX[7:0] contract
  parameter int MAX_PKTS  = 16,        // packets per request (<=16-beat segmentation)
  // data buffers (client-side ports; SRAM may be instanced at MC top)
  parameter int N_WDB     = 32,        // WD_SRAM slots
  parameter int N_RDB     = 32,        // RD_SRAM slots
  parameter int SRAM_W    = 512,       // one packet per line

  // derived widths (localparam: not overridable, visible to port list)
  localparam int PKT_NUM_W = (MAX_PKTS > 1) ? $clog2(MAX_PKTS) : 1,
  localparam int SLOT_W    = (($clog2(N_WDB) > $clog2(N_RDB)) ?
                              $clog2(N_WDB) : $clog2(N_RDB)),
  localparam int DADDR_W   = RANK_W + BG_W + BANK_PER_BG_W + ROW_W + COL_W
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // client AXI4 target (subset; lock/cache/prot/qos/region TODO)
  // AW
  input  logic [AXI_IDW-1:0]     awid,
  input  logic [AXI_AW-1:0]      awaddr,
  input  logic [7:0]             awlen,
  input  logic [2:0]             awsize,
  input  logic [1:0]             awburst,
  input  logic                   awvalid,
  output logic                   awready,
  // W
  input  logic [AXI_DW-1:0]      wdata,
  input  logic [AXI_DW/8-1:0]    wstrb,
  input  logic                   wlast,
  input  logic                   wvalid,
  output logic                   wready,
  // B
  output logic [AXI_IDW-1:0]     bid,
  output logic [1:0]             bresp,
  output logic                   bvalid,
  input  logic                   bready,
  // AR
  input  logic [AXI_IDW-1:0]     arid,
  input  logic [AXI_AW-1:0]      araddr,
  input  logic [7:0]             arlen,
  input  logic [2:0]             arsize,
  input  logic [1:0]             arburst,
  input  logic                   arvalid,
  output logic                   arready,
  // R
  output logic [AXI_IDW-1:0]     rid,
  output logic [AXI_DW-1:0]      rdata,
  output logic [1:0]             rresp,
  output logic                   rlast,
  output logic                   rvalid,
  input  logic                   rready,

  // request stream to MC. async_ prefix = bus crosses into the req async FIFO
  // (MC clock domain); CIF drives it synchronously on this side.
  output logic                   async_mc_req_valid,
  input  logic                   async_mc_req_ready,
  output logic [ROB_IDX_W-1:0]   async_mc_req_rob_index,
  output logic                   async_mc_req_op,          // 0=read, 1=write
  output logic [DADDR_W-1:0]     async_mc_req_daddr,       // mapped DRAM coords
  output logic [PKT_NUM_W-1:0]   async_mc_req_pkt_num,
  output logic                   async_mc_req_last_in_txn, // auto-precharge hint
  output logic [SLOT_W-1:0]      async_mc_req_sram_slot,   // dbuf_addr(RD) | wd_slot(WR)

  // completion stream from MC (via compl async FIFO)
  input  logic                   async_mc_cmpl_valid,
  output logic                   async_mc_cmpl_ready,
  // SUGGEST(import): tie this ready high permanently - completion must never
  // back-pressure (blocking a done-signal deadlocks the round trip). Only drop
  // it on a soft reset drain. open: confirm no case needs flow control here.
  input  logic [ROB_IDX_W-1:0]   async_mc_cmpl_rob_index,
  input  logic [PKT_NUM_W-1:0]   async_mc_cmpl_pkt_num,
  input  logic                   async_mc_cmpl_status,

  // WD_SRAM client-side write port (CIF writes W beats; MC reads)
  output logic                     wdb_we,
  output logic [$clog2(N_WDB)-1:0] wdb_addr,
  output logic [SRAM_W-1:0]        wdb_din,

  // RD_SRAM client-side read port (MC writes; CIF drains to R)
  output logic                     rdb_re,
  output logic [$clog2(N_RDB)-1:0] rdb_addr,
  input  logic [SRAM_W-1:0]        rdb_dout
);

  localparam int PG_W  = AXI_AW - 12;
  localparam int PTR_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH) : 1;

  // nets between sub-blocks (TODO: size/name as logic fills in)
  // daddr = DRAM coords for the req packet (hazard uses raw AXI addr, not this)
  logic [DADDR_W-1:0]  aw_daddr, ar_daddr;  // TODO: into req-packet build

  // AXI byte-range per request for the hazard TCAMs (exact, 4KB-rule).
  // last = addr[11:0] + (((len+1)<<size) - 1); page = addr[AXI_AW-1:12].
  // last byte of a burst, within its 4KB page (never overflows: AXI 4KB rule).
  function automatic logic [11:0] axi_last(input logic [11:0] start,
                                           input logic [7:0]  len,
                                           input logic [2:0]  size);
    logic [11:0] bytes;
    bytes    = (12'(len) + 12'd1) << size;   // burst size in bytes (<=4096)
    axi_last = start + (bytes - 12'd1);
  endfunction

  logic [PG_W-1:0] aw_page, ar_page;
  logic [11:0]     aw_start, aw_last, ar_start, ar_last;
  assign aw_page  = awaddr[AXI_AW-1:12];
  assign ar_page  = araddr[AXI_AW-1:12];
  assign aw_start = awaddr[11:0];
  assign ar_start = araddr[11:0];
  assign aw_last  = axi_last(aw_start, awlen, awsize);
  assign ar_last  = axi_last(ar_start, arlen, arsize);

  // hazard cross-search + stall vectors (two ROBs, one haz router)
  logic             raw_hit, war_hit;             // read-vs-write / write-vs-read
  logic [PTR_W-1:0] raw_rd_ptr, war_wr_ptr;       // requester's own slot to hold
  logic [PTR_W-1:0] raw_wr_ptr, war_rd_ptr;       // matched older-entry slot (release)
  logic [ROB_DEPTH-1:0] r_stall_vector, w_stall_vector;

  // address map: system addr -> {rank,bg,bank,row,col} (field-slice, no hash)
  logic [CH_W-1:0]      aw_ch, ar_ch;        // channel select (route to core)
  logic [PKT_OFF_W-1:0] aw_off, ar_off;      // byte-in-packet (write mask)

  rmc_cif_addrmap #(
    .AXI_AW  (AXI_AW),
    .DADDR_W (DADDR_W)
  ) u_addrmap_aw (
    .sys_addr (awaddr),
    .daddr    (aw_daddr),
    .ch       (aw_ch),
    .offset   (aw_off)
  );

  rmc_cif_addrmap #(
    .AXI_AW  (AXI_AW),
    .DADDR_W (DADDR_W)
  ) u_addrmap_ar (
    .sys_addr (araddr),
    .daddr    (ar_daddr),
    .ch       (ar_ch),
    .offset   (ar_off)
  );

  // segmentation: AXI burst -> <=16-beat packets, one 64B line each
  rmc_cif_seg #(
    .AXI_AW    (AXI_AW),
    .MAX_PKTS  (MAX_PKTS),
    .PKT_NUM_W (PKT_NUM_W)
  ) u_seg (
    .aclk    (aclk),
    .aresetn (aresetn)
    // TODO: AW/AR beat-count -> packet count; narrow/unaligned flag (stage 20)
  );

  // completion never back-pressures (haz-suggest, stage 20): tie ready high.
  // A completion only bumps a counter on an already-allocated entry, so it is
  // always absorbable in one cycle. TODO: route cmpl to r_rob/w_rob by a
  // direction bit in the tag; drop only on a soft-reset drain.
  assign async_mc_cmpl_ready = 1'b1;

  // read ROB. Its addr TCAM is probed by a NEW WRITE (WAR). haz holds reads.
  rmc_cif_rob #(
    .DIR       (0),
    .ROB_DEPTH (ROB_DEPTH),
    .ROB_IDX_W (ROB_IDX_W),
    .MAX_PKTS  (MAX_PKTS),
    .PKT_NUM_W (PKT_NUM_W),
    .AXI_AW    (AXI_AW)
  ) u_r_rob (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .cmpl_valid     (async_mc_cmpl_valid),
    .cmpl_rob_index (async_mc_cmpl_rob_index),
    .cmpl_pkt_num   (async_mc_cmpl_pkt_num),
    .cmpl_status    (async_mc_cmpl_status),
    // probed by the new write's range (WAR)
    .srch_valid     (awvalid),
    .srch_page      (aw_page),
    .srch_start     (aw_start),
    .srch_last      (aw_last),
    .srch_hit       (war_hit),
    .srch_ptr       (war_rd_ptr),    // matched older read slot (WAR release)
    .stall_vector   (r_stall_vector)
    // TODO: alloc/retire ports
  );

  // write ROB. Its addr TCAM is probed by a NEW READ (RAW); a hit stalls that
  // new read (r_stall_vector), not the write.
  rmc_cif_rob #(
    .DIR       (1),
    .ROB_DEPTH (ROB_DEPTH),
    .ROB_IDX_W (ROB_IDX_W),
    .MAX_PKTS  (MAX_PKTS),
    .PKT_NUM_W (PKT_NUM_W),
    .AXI_AW    (AXI_AW)
  ) u_w_rob (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .cmpl_valid     (async_mc_cmpl_valid),
    .cmpl_rob_index (async_mc_cmpl_rob_index),
    .cmpl_pkt_num   (async_mc_cmpl_pkt_num),
    .cmpl_status    (async_mc_cmpl_status),
    // probed by the new read's range (RAW)
    .srch_valid     (arvalid),
    .srch_page      (ar_page),
    .srch_start     (ar_start),
    .srch_last      (ar_last),
    .srch_hit       (raw_hit),
    .srch_ptr       (raw_wr_ptr),    // matched older write slot (RAW release)
    .stall_vector   (w_stall_vector)
    // TODO: alloc/retire ports
  );

  // haz router: a hit LATCHES a stall on the requester's own entry; released when
  // the matched older entry retires. Hits are 1-cycle, so haz holds the state.
  // TODO: raw_rd_ptr = new read's r_rob alloc slot; war_wr_ptr = new write's w_rob
  //       alloc slot (from alloc logic). free pulses come from ROB retire (not built).
  assign raw_rd_ptr = '0;
  assign war_wr_ptr = '0;
  rmc_cif_haz #(
    .ROB_DEPTH (ROB_DEPTH)
  ) u_haz (
    .clk            (aclk),
    .rst_n          (aresetn),
    .raw_hit        (raw_hit),
    .raw_rd_ptr     (raw_rd_ptr),
    .raw_wr_ptr     (raw_wr_ptr),
    .war_hit        (war_hit),
    .war_wr_ptr     (war_wr_ptr),
    .war_rd_ptr     (war_rd_ptr),
    .wr_free_vld    (1'b0),          // TODO: from w_rob retire
    .wr_free_ptr    ('0),
    .rd_free_vld    (1'b0),          // TODO: from r_rob retire
    .rd_free_ptr    ('0),
    .r_stall_vector (r_stall_vector),
    .w_stall_vector (w_stall_vector)
  );

  // request builder / response stubs (TODO)
  assign async_mc_req_valid       = 1'b0;
  assign async_mc_req_rob_index   = '0;
  assign async_mc_req_op          = 1'b0;
  assign async_mc_req_daddr       = '0;
  assign async_mc_req_pkt_num     = '0;
  assign async_mc_req_last_in_txn = 1'b0;
  assign async_mc_req_sram_slot   = '0;

  assign awready = 1'b0;
  assign wready  = 1'b0;
  assign bid     = '0;
  assign bresp   = 2'b00;
  assign bvalid  = 1'b0;
  assign arready = 1'b0;        // TODO: gate with ROB stall_vector for this AR
  assign rid     = '0;
  assign rdata   = '0;
  assign rresp   = 2'b00;
  assign rlast   = 1'b0;
  assign rvalid  = 1'b0;

  assign wdb_we   = 1'b0;
  assign wdb_addr = '0;
  assign wdb_din  = '0;
  assign rdb_re   = 1'b0;
  assign rdb_addr = '0;

endmodule : rmc_cif
