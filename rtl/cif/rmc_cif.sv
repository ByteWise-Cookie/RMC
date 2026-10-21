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
  parameter int MAX_PKTS  = MAX_REQ_PKTS, // lines per request: 4KB / 64B = 64 [root knob]
  // data buffers (client-side ports; SRAM may be instanced at MC top)
  parameter int N_WDB     = 32,        // WD_SRAM slots
  parameter int N_RDB     = 64,        // RD_SRAM slots per channel (>= MAX_PKTS: one request
                                       // can land all its lines on one channel)

  // derived widths (localparam: not overridable, visible to port list)
  localparam int SRAM_W    = PKT_BYTES * 8,   // one 64B packet per line (= BL*DQ_W)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH) : 1,  // rob_index tag
  localparam int PKT_NUM_W = (MAX_PKTS > 1) ? $clog2(MAX_PKTS) : 1,
  localparam int CNT_W     = (MAX_PKTS > 1) ? $clog2(MAX_PKTS+1) : 1,  // packet-count width
  localparam int SPAN_W    = $clog2(MAX_REQ_BYTES + 1),                // bytes-spanned width
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
  input  logic                   async_mc_cmpl_op,         // 0 = read (-> r_rob), 1 = write (-> w_rob)

  // WD_SRAM client-side write port, one per channel (CIF/write_alloc writes; MC reads).
  // Per-channel: a line's data lives in its channel's WD-SRAM, symmetric to rdb_*.
  output logic [N_CH-1:0]                    wdb_we,
  output logic [N_CH-1:0][$clog2(N_WDB)-1:0] wdb_addr,
  output logic [N_CH-1:0][SRAM_W-1:0]        wdb_din,

  // RD_SRAM client-side read port, one per channel (MC writes; CIF drains to R).
  // Per-channel: each read engine owns its channel's RD-SRAM, and r_resp reads the
  // channel a line maps to. Parent must provide N_CH read ports.
  output logic [N_CH-1:0]                    rdb_re,
  output logic [N_CH-1:0][$clog2(N_RDB)-1:0] rdb_addr,
  input  logic [N_CH-1:0][SRAM_W-1:0]        rdb_dout
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
  // RESP_* -> R channel is now wired through u_r_resp (per-channel RD-SRAM read,
  // window-mux channel recovery, full-width INCR; narrow-burst trim still TODO).

  // read splitter <-> read ROB handoff nets. Splitter-side widths for ar_len/ar_size and
  // cfg_n_ch_log2 (mirror the splitter's own derivation so the port connections match).
  localparam int R_DATA_BYTES = AXI_DW / 8;
  localparam int R_MAX_BEATS  = MAX_REQ_BYTES / R_DATA_BYTES;
  localparam int R_LEN_W      = (R_MAX_BEATS  > 1) ? $clog2(R_MAX_BEATS)  : 1;
  localparam int R_SIZE_MAX   = (R_DATA_BYTES > 1) ? $clog2(R_DATA_BYTES) : 0;
  localparam int R_SIZE_W     = (R_SIZE_MAX   > 0) ? $clog2(R_SIZE_MAX+1) : 1;
  localparam int R_NCH_LOG_W  = $clog2(CH_W + 1);

  logic                        ar_rrob_rdy;     // rrob free slot (= ~rob_full) -> rob_free
  logic [ROB_IDX_W-1:0]        ar_rob_idx;      // slot rrob offers (= its alloc_ptr)
  logic                        ar_seg_last;     // a read request finished splitting
  logic [ROB_IDX_W-1:0]        ar_seg_rob_idx;  // which entry it closes
  logic [N_CH-1:0][SLOT_W-1:0] ar_seg_head_ptr; // per-channel RD-SRAM head slots (at seg_last)

  // read ROB retire -> read-response assembler (RESP_* descriptor)
  logic                               rsp_valid, rsp_ready;
  logic [AXI_IDW-1:0]                 rsp_id;
  logic [1:0]                         rsp_resp;
  logic [N_CH-1:0][$clog2(N_RDB)-1:0] rsp_head_ptr;
  logic [CH_W-1:0]                    rsp_base_ch;
  logic [PAGE_LSB-1:0]                rsp_addr_lo;
  logic [7:0]                         rsp_len;
  logic [2:0]                         rsp_size;

  // admit descriptor from the splitter. All known at admit, so it ships to the ROB early:
  // completions that beat the split still count, and the resp trim meta is ready up front.
  logic                   ar_admit_valid;
  logic [AXI_IDW-1:0]     ar_admit_id;
  logic                   ar_admit_validated;
  logic [CNT_W-1:0]       ar_admit_npkts;
  logic [PAGE_LSB-1:0]    ar_admit_addr_lo;      // ar_addr[PAGE_LSB-1:0]: [5:0] trim, [11:6] l0
  logic [R_LEN_W-1:0]     ar_admit_len;
  logic [R_SIZE_W-1:0]    ar_admit_size;
  logic [CH_W-1:0]        ar_admit_base_ch;
  logic [N_CH-1:0]        ar_admit_ch_mask;

  // read burst splitter. Owns the AR handshake (arready); the ROB only offers a free slot
  // (rob_free/rob_idx) and allocates on admit_valid. TODO: pkt_* -> the per-channel req async
  // FIFOs (single async_mc_req_* port today; needs an N_CH->1 arbiter) and pkt_dbuf_adr_ready
  // from the RD-SRAM allocator (not built).
  rmc_cif_r_burst_spltr #(
    .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_AW (AXI_AW), .N_SLOTS (N_RDB)
  ) u_rseg (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .addr_map       (addr_map),
    .cfg_n_ch_log2  (R_NCH_LOG_W'(CH_W)),      // all channels active (boot CSR; constant now)
    .ar_valid       (arvalid),
    .ar_ready       (arready),
    .ar_id          (arid),
    .ar_addr        (araddr),
    .ar_len         (R_LEN_W'(arlen)),
    .ar_size        (R_SIZE_W'(arsize)),
    .ar_validated   (ar_ok),
    .rob_free       (ar_rrob_rdy),
    .rob_idx        (ar_rob_idx),
    .admit_valid    (ar_admit_valid),
    .admit_id       (ar_admit_id),
    .admit_validated(ar_admit_validated),
    .admit_npkts    (ar_admit_npkts),
    .admit_addr_lo  (ar_admit_addr_lo),
    .admit_len      (ar_admit_len),
    .admit_size     (ar_admit_size),
    .admit_base_ch  (ar_admit_base_ch),
    .admit_ch_mask  (ar_admit_ch_mask),
    .pkt_valid      (/* TODO: per-channel req FIFO */),
    .pkt_ready      ({N_CH{1'b0}}),            // TODO: from the per-channel req FIFO
    .pkt_dbuf_adr_ready ({N_CH{1'b1}}),        // TODO: from RD-SRAM allocator (slot availability)
    .pkt_rob_idx    (/* TODO */),
    .pkt_daddr      (/* TODO */),
    .pkt_dbuf_adr   (/* TODO */),
    .seg_last       (ar_seg_last),
    .seg_rob_idx    (ar_seg_rob_idx),
    .seg_head_ptr   (ar_seg_head_ptr),
    .idle           (/* TODO */)
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
  // Demux by async_mc_cmpl_op: read completions go to r_rob, writes to w_rob. The
  // two ROB index spaces overlap, so the direction bit is what tells them apart.
  assign async_mc_cmpl_ready = 1'b1;
  wire cmpl_rd = async_mc_cmpl_valid & ~async_mc_cmpl_op;
  wire cmpl_wr = async_mc_cmpl_valid &  async_mc_cmpl_op;

  // PLAN (not built here yet): a pre-ROB AW_req buffer sits post request_validator
  // and before the ROB, one per direction. It holds the phys addr (which the ROB
  // does NOT store) and feeds the burst splitter directly, and it decouples up to
  // N_OUTSTANDING requests. The ROB only ADMITS: it takes {id,len,size,validated}
  // on AW_REQUEST/A_ID_VALID and backpressures via A_ID_READY. The buffer is not
  // built yet - for now admit is wired straight from the validator + AXI channel.

  // read ROB. The splitter owns arready (admit backpressure); the ROB just presents a free
  // slot (A_ID_READY -> the splitter's rob_free) and allocates when the splitter admits
  // (A_ID_VALID = admit_valid). The resp meta now lands at admit; only the per-channel head
  // pointers arrive at seg_last. RESP_* -> R response path is still TODO.
  rmc_cif_rrob #(
    .ROB_DEPTH (ROB_DEPTH),
    .MAX_PKTS  (MAX_PKTS),
    .AXI_IDW   (AXI_IDW),
    .N_SLOTS   (N_RDB)
  ) u_rrob (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .cmpl_valid     (cmpl_rd),
    .cmpl_rob_index (async_mc_cmpl_rob_index),
    .AW_REQUEST     ({ar_admit_id, ar_admit_validated}),
    .ADMIT_NPKTS    (ar_admit_npkts),
    .ADMIT_ADDR_LO  (ar_admit_addr_lo),
    .ADMIT_LEN      (8'(ar_admit_len)),
    .ADMIT_SIZE     (3'(ar_admit_size)),
    .ADMIT_BASE_CH  (ar_admit_base_ch),
    .A_ID_VALID     (ar_admit_valid),
    .A_ID_READY     (ar_rrob_rdy),
    .ROB_IDX        (ar_rob_idx),
    .SEG_LAST       (ar_seg_last),
    .SEG_ROB_IDX    (ar_seg_rob_idx),
    .SEG_HEAD_PTR   (ar_seg_head_ptr),
    .RESP_VALID     (rsp_valid),
    .RESP_READY     (rsp_ready),
    .RESP_ID        (rsp_id),
    .RESP_RESP      (rsp_resp),
    .RESP_HEAD_PTR  (rsp_head_ptr),
    .RESP_BASE_CH   (rsp_base_ch),
    .RESP_ADDR_LO   (rsp_addr_lo),
    .RESP_LEN       (rsp_len),
    .RESP_SIZE      (rsp_size)
  );

  // read-response assembler: drains the per-channel RD-SRAM into the R channel for one
  // retired request at a time, in ascending address order (recovers each line's channel
  // from the RESP descriptor via the window mux).
  rmc_cif_r_resp #(
    .AXI_IDW (AXI_IDW),
    .N_RDB   (N_RDB)
  ) u_r_resp (
    .aclk          (aclk),
    .aresetn       (aresetn),
    .addr_map      (addr_map),
    .cfg_n_ch_log2 (R_NCH_LOG_W'(CH_W)),      // all channels active (boot CSR; constant now)
    .resp_valid    (rsp_valid),
    .resp_ready    (rsp_ready),
    .resp_id       (rsp_id),
    .resp_resp     (rsp_resp),
    .resp_head_ptr (rsp_head_ptr),
    .resp_base_ch  (rsp_base_ch),
    .resp_addr_lo  (rsp_addr_lo),
    .resp_len      (rsp_len),
    .resp_size     (rsp_size),
    .rdb_re        (rdb_re),
    .rdb_addr      (rdb_addr),
    .rdb_dout      (rdb_dout),
    .rid           (rid),
    .rdata         (rdata),
    .rresp         (rresp),
    .rlast         (rlast),
    .rvalid        (rvalid),
    .rready        (rready)
  );

  // write ROB. admit direct from AW channel + validator (AW_req buffer not built yet).
  rmc_cif_wrob #(
    .ROB_DEPTH (ROB_DEPTH),
    .MAX_PKTS  (MAX_PKTS),
    .AXI_IDW   (AXI_IDW)
  ) u_wrob (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .cmpl_valid     (cmpl_wr),
    .cmpl_rob_index (async_mc_cmpl_rob_index),
    .cmpl_err       (1'b0),                   // TODO: carry a cmpl error bit from the MC
    .AW_REQUEST     ({awid, awlen, awsize, aw_ok}),
    .A_ID_VALID     (awvalid),
    .A_ID_READY     (awready),
    // SEG_* from u_wseg (not built yet) - tied off so nothing retires until writes flow
    .ROB_IDX        (/* TODO: -> u_wseg */),
    .SEG_LAST       (1'b0),
    .SEG_NPKTS      ('0),
    .SEG_HEAD_PTR   ('0),
    .SEG_NPKTS_CH   ('0),
    // retire -> AXI B response + per-channel run free to the write accumulator
    .RESP_VALID     (bvalid),
    .RESP_READY     (bready),
    .RESP_ID        (bid),
    .RESP_RESP      (bresp),
    .RESP_HEAD_PTR  (/* TODO: -> write accumulator free */),
    .RESP_NPKTS_CH  (/* TODO: -> write accumulator free */)
  );

  // request builder stubs (TODO: fed by u_wseg once the write issue path is built)
  assign async_mc_req_valid       = 1'b0;
  assign async_mc_req_rob_index   = '0;
  assign async_mc_req_op          = 1'b0;
  assign async_mc_req_daddr       = '0;
  assign async_mc_req_pkt_num     = '0;
  assign async_mc_req_last_in_txn = 1'b0;
  assign async_mc_req_sram_slot   = '0;

  // awready / arready are driven by the ROBs' A_ID_READY (admit backpressure).
  // R channel + rdb_* are driven by u_r_resp; the B channel by u_wrob (above).
  assign wready  = 1'b0;                      // TODO: W-data capture + write_alloc

  assign wdb_we   = '0;
  assign wdb_addr = '0;
  assign wdb_din  = '0;

endmodule : rmc_cif
