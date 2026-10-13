// rmc_cif.sv
// Client interface (CIF) top. Sits between the client AXI4 bus and the MC core.
// Owns: address map (system -> DRAM coords), burst segmentation (<=16-beat
// packets), reorder buffer (program order + completion), and the client-side
// ports of the WD/RD data SRAMs.
//
// Boundary to MC = two async FIFOs (external, one per direction). CIF presents a
// synchronous valid/ready stream on its side; CDC lives in the FIFO, not here.
//   out : request packets  {rob_index, op, daddr, pkt_num, last_in_txn, sram_slot}
//   in  : completions      {rob_index}  (a pulse; ROB counts them, retires at N)
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
  parameter int ROB_DEPTH = 32,        // physical ROB entries (16-32)   [root knob]
  parameter int MAX_PKTS  = 16,        // packets per request (<=16-beat) [root knob]
  // data buffers (client-side ports; SRAM may be instanced at MC top)
  parameter int N_WDB     = 32,        // WD_SRAM slots
  parameter int N_RDB     = 32,        // RD_SRAM slots

  // derived widths (localparam: not overridable, visible to port list)
  localparam int SRAM_W    = PKT_BYTES * 8,   // one 64B packet per line (= BL*DQ_W)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH) : 1,  // rob_index tag
  localparam int PKT_NUM_W = (MAX_PKTS > 1) ? $clog2(MAX_PKTS) : 1,
  localparam int CNT_W     = (MAX_PKTS > 1) ? $clog2(MAX_PKTS+1) : 1,  // packet-count width
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
  // tied high permanently: a completion only bumps an already-allocated entry's
  // counter, always absorbable in one cycle, so it must never back-pressure
  // (blocking a done-signal deadlocks the round trip). Only drop on soft-reset drain.
  input  logic [ROB_IDX_W-1:0]   async_mc_cmpl_rob_index,  // completion = pulse + this

  // WD_SRAM client-side write port (CIF writes W beats; MC reads)
  output logic                     wdb_we,
  output logic [$clog2(N_WDB)-1:0] wdb_addr,
  output logic [SRAM_W-1:0]        wdb_din,

  // RD_SRAM client-side read port (MC writes; CIF drains to R)
  output logic                     rdb_re,
  output logic [$clog2(N_RDB)-1:0] rdb_addr,
  input  logic [SRAM_W-1:0]        rdb_dout
);

  // nets between sub-blocks (TODO: size/name as logic fills in)
  // daddr = DRAM coords for the req packet
  logic [DADDR_W-1:0]  aw_daddr, ar_daddr;  // TODO: into req-packet build

  // address map: system addr -> {rank,bg,bank,row,col}, runtime CSR-programmable.
  // Live map = per-dest-bit source select; reset value = STAGE-24 default layout.
  // TODO: drive addr_map from a CSR reg (reset to default_addr_map()); constant for now.
  logic [CH_W-1:0]      aw_ch, ar_ch;        // channel select (route to core)
  logic [PKT_OFF_W-1:0] aw_off, ar_off;      // byte-in-packet (write mask)
  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map;
  assign addr_map = default_addr_map();

  rmc_cif_addrmap #(
    .AXI_AW  (AXI_AW),
    .DADDR_W (DADDR_W)
  ) u_addrmap_aw (
    .sys_addr (awaddr),
    .addr_map (addr_map),
    .daddr    (aw_daddr),
    .ch       (aw_ch),
    .offset   (aw_off)
  );

  rmc_cif_addrmap #(
    .AXI_AW  (AXI_AW),
    .DADDR_W (DADDR_W)
  ) u_addrmap_ar (
    .sys_addr (araddr),
    .addr_map (addr_map),
    .daddr    (ar_daddr),
    .ch       (ar_ch),
    .offset   (ar_off)
  );

  // request validator: tag each AXI request address ok(1)/err(0) before the ROB.
  // Addr-only; an err request still rides through (err bit) to retire with SLVERR.
  // TODO: carry aw_ok/ar_ok into ROB alloc as the per-request err bit.
  logic aw_ok, ar_ok;
  rmc_cif_reqvalidator #(.AXI_AW(AXI_AW)) u_val_aw (
    .req_addr (awaddr), .req_len (awlen), .req_size (awsize), .req_ok (aw_ok));
  rmc_cif_reqvalidator #(.AXI_AW(AXI_AW)) u_val_ar (
    .req_addr (araddr), .req_len (arlen), .req_size (arsize), .req_ok (ar_ok));

  // ====================================================================
  // READ PATH LEFT TO DO (rseg/rrob control is wired; these remain):
  //   - req_dbuf_base: the RD-SRAM slot allocator is not built; u_rseg's
  //     req_dbuf_base is stubbed. Allocate a contiguous RD-SRAM base per
  //     admitted read and feed it here.
  //   - pkt_* -> MC: u_rseg emits N_CH per-channel packet streams, but this
  //     block has a single async_mc_req_* port. Add per-channel req FIFOs +
  //     an N_CH->1 arbiter (or widen the MC req interface to per-channel).
  //   - dbuf addressing: global dbuf_base+k (per-channel SRAM sparse) vs a
  //     packed per-channel index - open; affects RD-SRAM size + resp readback.
  //   - scattered-map fallback: rseg assumes the contract map (ch = low CH_W
  //     line bits); an arbitrary CSR map needs the scan path (not built).
  //   - RESP_* from u_rrob -> the R response channel (drain RD-SRAM, trim).
  // ====================================================================

  // read segmenter <-> read ROB handoff nets
  logic                   ar_rseg_ready;    // rseg queue has room
  logic                   ar_rrob_rdy;      // rrob has a free slot (~rob_full)
  logic                   ar_admit;         // a read request is admitted this cycle
  logic [ROB_IDX_W-1:0]   ar_rob_idx;       // slot rrob hands out, rides with the request
  logic                   ar_seg_last;      // a read request finished splitting
  logic [ROB_IDX_W-1:0]   ar_seg_rob_idx;   // which entry it was
  logic [CNT_W-1:0]       ar_seg_npkts;
  logic [PKT_OFF_W-1:0]   ar_seg_offset;
  logic [7:0]             ar_seg_len;
  logic [2:0]             ar_seg_size;
  logic [SLOT_W-1:0]      ar_seg_head_ptr;

  // admit both together: the ROB slot and the segmenter queue entry are allocated on the
  // same cycle, so gate each on the other's readiness.
  assign ar_admit = arvalid & ar_rseg_ready & ar_rrob_rdy;

  // read segmenter. TODO: req_dbuf_base from the RD-SRAM slot allocator (not built) and
  // pkt_* -> the per-channel req async FIFOs (single async_mc_req_* port today; needs an
  // N_CH->1 arbiter or a per-channel FIFO interface).
  rmc_cif_rseg #(
    .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_AW (AXI_AW), .N_SLOTS (N_RDB)
  ) u_rseg (
    .aclk         (aclk),
    .aresetn      (aresetn),
    .addr_map     (addr_map),
    .start_req    (ar_admit),
    .req_addr     (araddr),
    .req_len      (arlen),
    .req_size     (arsize),
    .req_dbuf_base(/* TODO: RD-SRAM slot allocator */),
    .req_rob_idx  (ar_rob_idx),
    .ready        (ar_rseg_ready),
    .pkt_valid    (/* TODO: per-channel req FIFO */),
    .pkt_ready    (/* TODO */),
    .pkt_rob_idx  (/* TODO */),
    .pkt_daddr    (/* TODO */),
    .pkt_dbuf_adr (/* TODO */),
    .seg_last     (ar_seg_last),
    .seg_rob_idx  (ar_seg_rob_idx),
    .seg_npkts    (ar_seg_npkts),
    .seg_offset   (ar_seg_offset),
    .seg_len      (ar_seg_len),
    .seg_size     (ar_seg_size),
    .seg_head_ptr (ar_seg_head_ptr)
  );

  // write segmenter - still a stub. TODO: mirror rseg (ring + per-channel engines) with
  // free-list slots + chain; wire to u_wrob and the W/AW path.
  rmc_cif_wseg #(
    .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_AW (AXI_AW)
  ) u_wseg (
    .aclk (aclk), .aresetn (aresetn)
  );

  // completion never back-pressures: tie ready high. A completion only bumps a
  // counter on an already-allocated entry, always absorbable in one cycle.
  // TODO: route cmpl to r_rob/w_rob by a direction bit in the tag; drop only on
  // a soft-reset drain.
  assign async_mc_cmpl_ready = 1'b1;

  // PLAN (not built here yet): a pre-ROB AW_req buffer sits post request_validator
  // and before the ROB, one per direction. It holds the phys addr (which the ROB
  // does NOT store) and feeds the burst splitter directly, and it decouples up to
  // N_OUTSTANDING requests. The ROB only ADMITS: it takes {id,len,size,validated}
  // on AW_REQUEST/A_ID_VALID and backpressures via A_ID_READY. The buffer is not
  // built yet - for now admit is wired straight from the validator + AXI channel.

  // read ROB. admit direct from AR channel + validator (AR_req buffer not built yet).
  // A_ID_VALID is gated by rseg readiness so the ROB slot and the segmenter queue entry
  // allocate together; arready reflects both. RESP_* -> R response path is still TODO.
  assign arready = ar_rseg_ready & ar_rrob_rdy;
  rmc_cif_rrob #(
    .ROB_DEPTH (ROB_DEPTH),
    .MAX_PKTS  (MAX_PKTS),
    .AXI_IDW   (AXI_IDW),
    .N_SLOTS   (N_RDB)
  ) u_rrob (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .cmpl_valid     (async_mc_cmpl_valid),
    .cmpl_rob_index (async_mc_cmpl_rob_index),
    .AW_REQUEST     ({arid, arlen, arsize, ar_ok}),
    .A_ID_VALID     (arvalid & ar_rseg_ready),
    .A_ID_READY     (ar_rrob_rdy),
    .ROB_IDX        (ar_rob_idx),
    .SEG_LAST       (ar_seg_last),
    .SEG_ROB_IDX    (ar_seg_rob_idx),
    .SEG_NPKTS      (ar_seg_npkts),
    .SEG_OFFSET     (ar_seg_offset),
    .SEG_LEN        (ar_seg_len),
    .SEG_SIZE       (ar_seg_size),
    .SEG_HEAD_PTR   (ar_seg_head_ptr)
    // TODO: RESP_* -> R response path
  );

  // write ROB. admit direct from AW channel + validator (AW_req buffer not built yet).
  rmc_cif_wrob #(
    .ROB_DEPTH (ROB_DEPTH),
    .MAX_PKTS  (MAX_PKTS),
    .AXI_IDW   (AXI_IDW)
  ) u_wrob (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .cmpl_valid     (async_mc_cmpl_valid),
    .cmpl_rob_index (async_mc_cmpl_rob_index),
    .AW_REQUEST     ({awid, awlen, awsize, aw_ok}),
    .A_ID_VALID     (awvalid),
    .A_ID_READY     (awready)
    // TODO: SEG_* from u_wseg; RESP_* -> B response path
  );

  // request builder / response stubs (TODO)
  assign async_mc_req_valid       = 1'b0;
  assign async_mc_req_rob_index   = '0;
  assign async_mc_req_op          = 1'b0;
  assign async_mc_req_daddr       = '0;
  assign async_mc_req_pkt_num     = '0;
  assign async_mc_req_last_in_txn = 1'b0;
  assign async_mc_req_sram_slot   = '0;

  // awready / arready are driven by the ROBs' A_ID_READY (admit backpressure).
  assign wready  = 1'b0;
  assign bid     = '0;
  assign bresp   = 2'b00;
  assign bvalid  = 1'b0;
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
