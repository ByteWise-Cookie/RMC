// rmc_cif.sv
// Client interface (CIF) top. Sits between the client AXI4 bus and the MC core.
// Owns: address map (system -> DRAM coords), burst split into 64 B line packets, the read/
// write reorder buffers, and the client-side ports of the WD/RD data SRAMs.
//
// Boundary to MC = PER-CHANNEL async FIFOs (external). Each DRAM channel has its own request
// and completion FIFO, per direction, so there is NO arbiter and no op demux here - read and
// write packets flow on separate per-channel ports, and completions come back the same way:
//   read  req  (per ch): {rob_index, daddr, slot=RD-SRAM dbuf}   MC writes returned data there
//   write req  (per ch): {rob_index, daddr, slot=WD-SRAM}         MC reads the data+mask there
//   read/write cmpl (per ch): pulse {rob_index, err}; the ROB counts them per channel
// CDC lives in the FIFOs, not here. Read data is written by MC into RD-SRAM; CIF drains it to R.

import rmc_cfg_pkg::*;

module rmc_cif #(
  parameter int AXI_IDW   = 8,
  parameter int AXI_AW    = 48,        // map needs >=41 (row ends at bit 40)
  // AXI_DW is rmc_cfg_pkg::AXI_DW = PKT_BYTES*8 = 64 B (one line per AXI beat).
  parameter int ROB_DEPTH = 32,        // physical ROB entries per direction  [root knob]
  parameter int MAX_PKTS  = MAX_REQ_PKTS, // lines per request: 4KB/64B = 64   [root knob]
  parameter int N_WDB     = 64,        // WD_SRAM slots per channel (>= MAX_PKTS)
  parameter int N_RDB     = 64,        // RD_SRAM slots per channel (>= MAX_PKTS)

  // derived widths (localparam: not overridable, visible to port list)
  localparam int SRAM_W    = PKT_BYTES * 8,                           // one 64 B line
  localparam int STRB_W    = AXI_DW / 8,                              // byte mask width (64)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH) : 1,
  localparam int CNT_W     = (MAX_PKTS > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int WD_SLOT_W = (N_WDB > 1) ? $clog2(N_WDB) : 1,
  localparam int RD_SLOT_W = (N_RDB > 1) ? $clog2(N_RDB) : 1,
  localparam int DADDR_W   = RANK_W + BG_W + BANK_PER_BG_W + ROW_W + COL_W
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // client AXI4 target (subset)
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

  // READ request stream to MC, per channel (into each channel's read req async FIFO)
  output logic [N_CH-1:0]                  async_mc_rd_req_valid,
  input  logic [N_CH-1:0]                  async_mc_rd_req_ready,
  output logic [N_CH-1:0][ROB_IDX_W-1:0]   async_mc_rd_req_rob_index,
  output logic [N_CH-1:0][DADDR_W-1:0]     async_mc_rd_req_daddr,
  output logic [N_CH-1:0][RD_SLOT_W-1:0]   async_mc_rd_req_slot,     // RD-SRAM slot MC writes into

  // WRITE request stream to MC, per channel (into each channel's write req async FIFO)
  output logic [N_CH-1:0]                  async_mc_wr_req_valid,
  input  logic [N_CH-1:0]                  async_mc_wr_req_ready,
  output logic [N_CH-1:0][ROB_IDX_W-1:0]   async_mc_wr_req_rob_index,
  output logic [N_CH-1:0][DADDR_W-1:0]     async_mc_wr_req_daddr,
  output logic [N_CH-1:0][WD_SLOT_W-1:0]   async_mc_wr_req_slot,     // WD-SRAM slot MC reads from

  // completions from MC, per channel per direction. Pulse {rob_index, err}; never back-pressure.
  input  logic [N_CH-1:0]                  async_mc_rd_cmpl_valid,
  input  logic [N_CH-1:0][ROB_IDX_W-1:0]   async_mc_rd_cmpl_rob_index,
  input  logic [N_CH-1:0]                  async_mc_rd_cmpl_err,
  input  logic [N_CH-1:0]                  async_mc_wr_cmpl_valid,
  input  logic [N_CH-1:0][ROB_IDX_W-1:0]   async_mc_wr_cmpl_rob_index,
  input  logic [N_CH-1:0]                  async_mc_wr_cmpl_err,

  // WD_SRAM client-side write port, one per channel (CIF writes; MC reads). data + byte mask.
  output logic [N_CH-1:0]                  wdb_we,
  output logic [N_CH-1:0][WD_SLOT_W-1:0]   wdb_addr,
  output logic [N_CH-1:0][SRAM_W-1:0]      wdb_din,
  output logic [N_CH-1:0][STRB_W-1:0]      wdb_dstrb,

  // RD_SRAM client-side read port, one per channel (MC writes; CIF drains to R).
  output logic [N_CH-1:0]                  rdb_re,
  output logic [N_CH-1:0][RD_SLOT_W-1:0]   rdb_addr,
  input  logic [N_CH-1:0][SRAM_W-1:0]      rdb_dout
);

  // ---- address map (CSR-programmable; constant default for now) ----
  logic [DADDR_W-1:0]  aw_daddr, ar_daddr;
  logic [CH_W-1:0]     aw_ch, ar_ch;
  logic [PKT_OFF_W-1:0] aw_off, ar_off;
  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map;
  assign addr_map = default_addr_map();

  rmc_cif_addrmap #(.AXI_AW (AXI_AW), .DADDR_W (DADDR_W)) u_addrmap_aw (
    .sys_addr (awaddr), .addr_map (addr_map), .daddr (aw_daddr), .ch (aw_ch), .offset (aw_off));
  rmc_cif_addrmap #(.AXI_AW (AXI_AW), .DADDR_W (DADDR_W)) u_addrmap_ar (
    .sys_addr (araddr), .addr_map (addr_map), .daddr (ar_daddr), .ch (ar_ch), .offset (ar_off));

  // ---- request validator: address ok(1)/err(0) ahead of the ROB ----
  logic aw_ok, ar_ok;
  rmc_cif_reqvalidator #(.AXI_AW(AXI_AW)) u_val_aw (
    .req_addr (awaddr), .req_len (awlen), .req_size (awsize), .req_ok (aw_ok));
  rmc_cif_reqvalidator #(.AXI_AW(AXI_AW)) u_val_ar (
    .req_addr (araddr), .req_len (arlen), .req_size (arsize), .req_ok (ar_ok));
  // TODO: distinguish DECERR (unmapped) from SLVERR; for now any fail -> SLVERR
  wire [1:0] aw_err = aw_ok ? 2'b00 : 2'b10;

  // ========================================================================
  // READ PATH
  // ========================================================================
  localparam int R_DATA_BYTES = AXI_DW / 8;
  localparam int R_MAX_BEATS  = MAX_REQ_BYTES / R_DATA_BYTES;
  localparam int R_LEN_W      = (R_MAX_BEATS  > 1) ? $clog2(R_MAX_BEATS)  : 1;
  localparam int R_SIZE_MAX   = (R_DATA_BYTES > 1) ? $clog2(R_DATA_BYTES) : 0;
  localparam int R_SIZE_W     = (R_SIZE_MAX   > 0) ? $clog2(R_SIZE_MAX+1) : 1;
  localparam int R_NCH_LOG_W  = $clog2(CH_W + 1);

  logic                        ar_rrob_rdy;
  logic [ROB_IDX_W-1:0]        ar_rob_idx;
  logic                        ar_seg_last;
  logic [ROB_IDX_W-1:0]        ar_seg_rob_idx;
  logic [N_CH-1:0][RD_SLOT_W-1:0] ar_seg_head_ptr;

  // read admit descriptor (splitter -> rrob)
  logic                   ar_admit_valid;
  logic [AXI_IDW-1:0]     ar_admit_id;
  logic                   ar_admit_validated;
  logic [CNT_W-1:0]       ar_admit_npkts;
  logic [N_CH-1:0][CNT_W-1:0] ar_admit_npkts_ch;
  logic [PAGE_LSB-1:0]    ar_admit_addr_lo;
  logic [R_LEN_W-1:0]     ar_admit_len;
  logic [R_SIZE_W-1:0]    ar_admit_size;
  logic [CH_W-1:0]        ar_admit_base_ch;
  logic [N_CH-1:0]        ar_admit_ch_mask;

  // read ROB retire -> read-response assembler (RESP_* descriptor)
  logic                            rsp_valid, rsp_ready;
  logic [AXI_IDW-1:0]              rsp_id;
  logic [1:0]                      rsp_resp;
  logic [N_CH-1:0][RD_SLOT_W-1:0]  rsp_head_ptr;
  logic [N_CH-1:0][CNT_W-1:0]      rsp_npkts_ch;    // per-channel run to free at retire
  logic [CH_W-1:0]                 rsp_base_ch;
  logic [PAGE_LSB-1:0]             rsp_addr_lo;
  logic [7:0]                      rsp_len;
  logic [2:0]                      rsp_size;

  wire rd_retire = rsp_valid & rsp_ready;            // last beat accepted -> free the RD-SRAM run

  // read burst splitter (owns arready; emits per-channel packet streams)
  rmc_cif_r_burst_spltr #(
    .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_AW (AXI_AW), .N_SLOTS (N_RDB)
  ) u_rseg (
    .aclk (aclk), .aresetn (aresetn),
    .addr_map       (addr_map),
    .cfg_n_ch_log2  (R_NCH_LOG_W'(CH_W)),
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
    .admit_npkts_ch (ar_admit_npkts_ch),
    // packets out, one stream per channel -> read request FIFO per channel
    .pkt_valid      (async_mc_rd_req_valid),
    .pkt_ready      (async_mc_rd_req_ready),
    .pkt_rob_idx    (async_mc_rd_req_rob_index),
    .pkt_daddr      (async_mc_rd_req_daddr),
    .pkt_dbuf_adr   (async_mc_rd_req_slot),
    // RD-SRAM free: at retire release each channel's run (count from rrob)
    .free_valid     ({N_CH{rd_retire}}),
    .free_burst     (rsp_npkts_ch),
    .seg_last       (ar_seg_last),
    .seg_rob_idx    (ar_seg_rob_idx),
    .seg_head_ptr   (ar_seg_head_ptr),
    .idle           (/* unused */)
  );

  // read ROB
  rmc_cif_rrob #(
    .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_IDW (AXI_IDW), .N_SLOTS (N_RDB)
  ) u_rrob (
    .aclk (aclk), .aresetn (aresetn),
    .cmpl_valid     (async_mc_rd_cmpl_valid),
    .cmpl_rob_index (async_mc_rd_cmpl_rob_index),
    .cmpl_err       (async_mc_rd_cmpl_err),
    .AW_REQUEST     ({ar_admit_id, ar_admit_validated}),
    .ADMIT_NPKTS    (ar_admit_npkts),
    .ADMIT_NPKTS_CH (ar_admit_npkts_ch),
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
    .RESP_NPKTS_CH  (rsp_npkts_ch),
    .RESP_BASE_CH   (rsp_base_ch),
    .RESP_ADDR_LO   (rsp_addr_lo),
    .RESP_LEN       (rsp_len),
    .RESP_SIZE      (rsp_size)
  );

  // read-response assembler -> R channel
  rmc_cif_r_resp #(.AXI_IDW (AXI_IDW), .N_RDB (N_RDB)) u_r_resp (
    .aclk (aclk), .aresetn (aresetn),
    .addr_map      (addr_map),
    .cfg_n_ch_log2 (R_NCH_LOG_W'(CH_W)),
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
    .rid (rid), .rdata (rdata), .rresp (rresp), .rlast (rlast), .rvalid (rvalid), .rready (rready)
  );

  // ========================================================================
  // WRITE PATH
  // ========================================================================
  // AW request buffer (sole consumer = the write splitter)
  logic                aw_buf_valid, aw_buf_pop, aw_buf_full;
  logic [AXI_IDW-1:0]  aw_buf_id;
  logic [AXI_AW-1:0]   aw_buf_addr;
  logic [7:0]          aw_buf_len;
  logic [2:0]          aw_buf_size;
  logic [1:0]          aw_buf_err;
  assign awready = ~aw_buf_full;
  rmc_cif_aw_reqbuf #(.DEPTH (ROB_DEPTH), .AXI_AW (AXI_AW), .AXI_IDW (AXI_IDW)) u_aw_reqbuf (
    .aclk (aclk), .aresetn (aresetn),
    .push (awvalid & awready), .in_id (awid), .in_addr (awaddr), .in_len (awlen),
    .in_size (awsize), .in_err (aw_err), .full (aw_buf_full),
    .valid (aw_buf_valid), .pop (aw_buf_pop),
    .out_id (aw_buf_id), .out_addr (aw_buf_addr), .out_len (aw_buf_len),
    .out_size (aw_buf_size), .out_err (aw_buf_err)
  );

  // W data FIFO (AXI W -> accumulator)
  logic                w_fifo_mvalid, w_fifo_mready, w_fifo_mlast;
  logic [AXI_DW-1:0]   w_fifo_mdata;
  logic [STRB_W-1:0]   w_fifo_mstrb;
  rmc_cif_w_data_fifo #(.DEPTH (8)) u_w_data_fifo (
    .aclk (aclk), .aresetn (aresetn),
    .s_valid (wvalid), .s_ready (wready), .s_wdata (wdata), .s_wstrb (wstrb), .s_wlast (wlast),
    .m_valid (w_fifo_mvalid), .m_ready (w_fifo_mready),
    .m_wdata (w_fifo_mdata), .m_wstrb (w_fifo_mstrb), .m_wlast (w_fifo_mlast)
  );

  // write ROB handoff + retire nets
  logic                   w_rob_free;
  logic [ROB_IDX_W-1:0]   w_rob_idx;
  logic                   w_admit_valid;
  logic [AXI_IDW-1:0]     w_admit_id;
  logic [1:0]             w_admit_resp;
  logic                   w_seg_last;
  logic [ROB_IDX_W-1:0]   w_seg_rob_idx;
  logic [N_CH-1:0][CNT_W-1:0] w_seg_npkts_ch;
  logic                   w_rsp_valid, w_rsp_ready;
  logic [AXI_IDW-1:0]     w_rsp_id;
  logic [1:0]             w_rsp_resp;
  logic [N_CH-1:0][CNT_W-1:0] w_rsp_npkts_ch;
  wire  wr_retire = w_rsp_valid & w_rsp_ready;       // B accepted -> free the WD-SRAM run

  // splitter -> accumulator shape + drop; accumulator -> splitter drain_done
  logic                   w_stage_en, w_skip, w_drain_done;
  logic [PKT_OFF_W-1:0]   w_stage_len;
  logic [2:0]             w_stage_size;
  // splitter -> pkt_gen line descriptor
  logic                   w_ld_valid, w_ld_ready;
  logic [CH_W-1:0]        w_ld_ch;
  logic [DADDR_W-1:0]     w_ld_daddr;
  logic [ROB_IDX_W-1:0]   w_ld_rob_idx;
  // accumulator <-> pkt_gen stage handshake + shared SRAM data bus
  logic                   w_stage_valid, w_take;
  logic [AXI_DW-1:0]      w_write_data;
  logic [STRB_W-1:0]      w_write_strobe;
  // pkt_gen <-> frptr (per channel)
  logic [N_CH-1:0][WD_SLOT_W-1:0] w_write_in_ptr;
  logic [N_CH-1:0]        w_sram_ready;
  logic [N_CH-1:0]        w_alloc;
  logic [N_CH-1:0]        w_wr_en;

  rmc_cif_w_burst_spltr #(
    .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_AW (AXI_AW), .AXI_IDW (AXI_IDW)
  ) u_w_burst_spltr (
    .aclk (aclk), .aresetn (aresetn),
    .aw_request_valid     (aw_buf_valid),
    .burst_splitter_ready (aw_buf_pop),
    .aw_id                (aw_buf_id),
    .aw_len               (aw_buf_len),
    .aw_size              (aw_buf_size),
    .phy_adr              (aw_buf_addr),
    .error                (aw_buf_err),
    .addr_map             (addr_map),
    .rob_idx_in           (w_rob_idx),
    .rob_free             (w_rob_free),
    .admit_valid          (w_admit_valid),
    .admit_id             (w_admit_id),
    .admit_resp           (w_admit_resp),
    .seg_last             (w_seg_last),
    .seg_rob_idx          (w_seg_rob_idx),
    .seg_npkts_ch         (w_seg_npkts_ch),
    .stage_en             (w_stage_en),
    .skip                 (w_skip),
    .stage_len            (w_stage_len),
    .stage_size           (w_stage_size),
    .drain_done           (w_drain_done),
    .ld_valid             (w_ld_valid),
    .ld_ready             (w_ld_ready),
    .ld_ch                (w_ld_ch),
    .ld_daddr             (w_ld_daddr),
    .ld_rob_idx           (w_ld_rob_idx)
  );

  rmc_cif_w_accum u_w_accum (
    .aclk (aclk), .aresetn (aresetn),
    .wvalid (w_fifo_mvalid), .wready (w_fifo_mready),
    .wdata (w_fifo_mdata), .wstrb (w_fifo_mstrb), .wlast (w_fifo_mlast),
    .stage_en (w_stage_en), .skip (w_skip), .stage_len (w_stage_len), .stage_size (w_stage_size),
    .drain_done (w_drain_done),
    .stage_valid (w_stage_valid), .take (w_take),
    .write_data (w_write_data), .write_data_strobe (w_write_strobe)
  );

  rmc_cif_w_pkt_gen #(.ROB_DEPTH (ROB_DEPTH), .N_SLOTS (N_WDB)) u_w_pkt_gen (
    .aclk (aclk), .aresetn (aresetn),
    .ld_valid (w_ld_valid), .ld_ready (w_ld_ready),
    .ld_ch (w_ld_ch), .ld_daddr (w_ld_daddr), .ld_rob_idx (w_ld_rob_idx),
    .stage_valid (w_stage_valid), .take (w_take),
    .write_in_ptr (w_write_in_ptr), .sram_ready (w_sram_ready), .alloc (w_alloc),
    .wr_en (w_wr_en),
    .req_fifo_ready   (async_mc_wr_req_ready),
    .packet_valid     (async_mc_wr_req_valid),
    .packet_rob_index (async_mc_wr_req_rob_index[0]),   // shared payload, fanned per channel below
    .packet_daddr     (async_mc_wr_req_daddr[0]),
    .packet_slot      (async_mc_wr_req_slot[0])
  );
  // pkt_gen drives one shared payload + a per-channel valid; fan the payload to every channel
  // port (the MC only looks at the channel whose valid is high).
  for (genvar c = 1; c < N_CH; c++) begin : g_wr_req_fan
    assign async_mc_wr_req_rob_index[c] = async_mc_wr_req_rob_index[0];
    assign async_mc_wr_req_daddr[c]     = async_mc_wr_req_daddr[0];
    assign async_mc_wr_req_slot[c]      = async_mc_wr_req_slot[0];
  end

  // per-channel WD-SRAM slot rings (frptr); free the run at B retire
  for (genvar c = 0; c < N_CH; c++) begin : g_wfrptr
    rmc_cif_frptr #(.N_SLOTS (N_WDB)) u_frptr (
      .aclk (aclk), .aresetn (aresetn),
      .write_in_ptr (w_write_in_ptr[c]),
      .sram_ready   (w_sram_ready[c]),
      .alloc        (w_alloc[c]),
      .free_valid   (wr_retire),
      .free_burst   (w_rsp_npkts_ch[c])
    );
  end

  // WD-SRAM write port: data+mask shared (accum), we per channel (pkt_gen), addr per channel (frptr)
  always_comb begin
    for (int c = 0; c < N_CH; c++) begin
      wdb_we[c]    = w_wr_en[c];
      wdb_addr[c]  = w_write_in_ptr[c];
      wdb_din[c]   = w_write_data;
      wdb_dstrb[c] = w_write_strobe;
    end
  end

  // write ROB
  rmc_cif_wrob #(.ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_IDW (AXI_IDW)) u_wrob (
    .aclk (aclk), .aresetn (aresetn),
    .cmpl_valid     (async_mc_wr_cmpl_valid),
    .cmpl_rob_index (async_mc_wr_cmpl_rob_index),
    .cmpl_err       (async_mc_wr_cmpl_err),
    .ROB_IDX        (w_rob_idx),
    .rob_free       (w_rob_free),
    .admit_valid    (w_admit_valid),
    .admit_id       (w_admit_id),
    .admit_resp     (w_admit_resp),
    .SEG_LAST       (w_seg_last),
    .SEG_ROB_IDX    (w_seg_rob_idx),
    .SEG_NPKTS_CH   (w_seg_npkts_ch),
    .RESP_VALID     (w_rsp_valid),
    .RESP_READY     (w_rsp_ready),
    .RESP_ID        (w_rsp_id),
    .RESP_RESP      (w_rsp_resp),
    .RESP_NPKTS_CH  (w_rsp_npkts_ch)
  );

  // B response assembler
  rmc_cif_w_resp #(.AXI_IDW (AXI_IDW)) u_w_resp (
    .aclk (aclk), .aresetn (aresetn),
    .RESP_VALID (w_rsp_valid), .RESP_READY (w_rsp_ready), .RESP_ID (w_rsp_id), .RESP_RESP (w_rsp_resp),
    .bvalid (bvalid), .bready (bready), .bid (bid), .bresp (bresp)
  );

endmodule : rmc_cif
