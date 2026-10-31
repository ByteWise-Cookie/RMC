// rmc_mc_top.sv
//
// MC core top. For every rank it stands up the RAW/WAR packet scheduler: a read and a write
// rmc_mc_packet_tcam wired as complements, plus an rmc_mc_wlre to pick the direction. Same-line
// read/write accesses come out in order; issue is out-of-order, bank-ready, round-robin per rank.
//
// Per rank the two halves are cross-wired: read queries write (q<->s) and vice versa, and each
// half's issued rd_ptr/packet_valid drives the other's compl_rd_ptr/compl_rd_fire so a stalled
// entry clears when the entry it waits on issues. The WLRE shares one mode bit across the pair.
//
// Ranks are independent (own banks, own pair), so this is a plain genvar loop over N_RANKS.
//
// TODO (not here yet): rank demux feeding the per-rank intake, the async req/compl FIFO boundary,
// the bank timing FSMs that drive bank_ready_vector, and the DFI command fabric on the issue side.

import rmc_cfg_pkg::*;

module rmc_mc_top #(
  parameter int DEPTH     = PKTS_PER_RANK,   // scheduler entries per direction, per rank
  parameter int ROB_IDX_W = 5,               // CIF rob_index tag width
  parameter int SLOT_W    = 6,               // WD/RD-SRAM slot width
  parameter int QUANTUM   = 8,               // WLRE back-to-back issues before a forced yield

  // derived
  localparam int ADDR_W = DADDR_W,
  localparam int META_W = ROB_IDX_W + SLOT_W,
  localparam int PKT_W  = META_W + ADDR_W,
  localparam int PTR_W  = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
  input  logic                 mclk,
  input  logic                 mreset,

  // per-rank read intake (from the rank demux)
  input  logic [N_RANKS-1:0]              rd_pkt_valid,
  input  logic [N_RANKS-1:0][PKT_W-1:0]   rd_pkt,
  output logic [N_RANKS-1:0]              rd_tcam_ready,

  // per-rank write intake (from the rank demux)
  input  logic [N_RANKS-1:0]              wr_pkt_valid,
  input  logic [N_RANKS-1:0][PKT_W-1:0]   wr_pkt,
  output logic [N_RANKS-1:0]              wr_tcam_ready,

  // per-rank, per-bank readiness (from the bank FSMs)
  input  logic [N_RANKS-1:0][N_BANK-1:0]  bank_ready_vector,

  // per-rank muxed issue (active direction)
  output logic [N_RANKS-1:0]              out_valid,
  output logic [N_RANKS-1:0]              out_is_read,
  output logic [N_RANKS-1:0][PKT_W-1:0]   out_packet,
  output logic [N_RANKS-1:0][N_BANK-1:0]  out_bank_vector
);

  genvar g;
  generate
    for (g = 0; g < N_RANKS; g++) begin : g_rank
      // issue + readiness from each half
      logic r_valid, w_valid;
      logic r_has_ready, w_has_ready;

      // WLRE: shared direction selector (1 = read drives, 0 = write drives)
      logic mode;
      rmc_mc_wlre #( .QUANTUM(QUANTUM) ) u_wlre (
        .mclk(mclk), .mreset(mreset),
        .rd_has_ready(r_has_ready), .wr_has_ready(w_has_ready),
        .rd_issued(r_valid),        .wr_issued(w_valid),
        .mode(mode)
      );

      // nets between the two halves (r_ = read side out, w_ = write side out)
      logic [PTR_W-1:0]  r_rd_ptr, w_rd_ptr;
      logic [ADDR_W-1:0] r_q_search, w_q_search;
      logic              r_q_valid,  w_q_valid;
      logic              r_s_hit,    w_s_hit;
      logic [PTR_W-1:0]  r_s_hit_ptr, w_s_hit_ptr;
      logic              r_s_hit_staging, w_s_hit_staging;
      logic [PKT_W-1:0]  r_packet, w_packet;
      logic [N_BANK-1:0] r_bank,   w_bank;

      // read half: queries the write CAM, answers the write side's query
      rmc_mc_packet_tcam #(
        .DIR_IS_READ(1'b1), .DEPTH(DEPTH), .ROB_IDX_W(ROB_IDX_W), .SLOT_W(SLOT_W)
      ) u_read (
        .mclk(mclk), .mreset(mreset),
        .pkt_valid(rd_pkt_valid[g]), .pkt(rd_pkt[g]), .tcam_ready(rd_tcam_ready[g]),
        .wlre_mode(mode), .bank_ready_vector(bank_ready_vector[g]),
        .packet(r_packet), .packet_valid(r_valid), .bank_valid_vector(r_bank), .has_ready(r_has_ready),
        .rd_ptr(r_rd_ptr), .compl_rd_ptr(w_rd_ptr), .compl_rd_fire(w_valid),
        // query out to the write side
        .q_search(r_q_search), .q_valid(r_q_valid),
        .q_hit(w_s_hit), .q_hit_ptr(w_s_hit_ptr), .q_hit_staging(w_s_hit_staging),
        // answer back to the write side's query
        .s_search(w_q_search), .s_valid(w_q_valid),
        .s_hit(r_s_hit), .s_hit_ptr(r_s_hit_ptr), .s_hit_staging(r_s_hit_staging)
      );

      // write half: queries the read CAM, answers the read side's query
      rmc_mc_packet_tcam #(
        .DIR_IS_READ(1'b0), .DEPTH(DEPTH), .ROB_IDX_W(ROB_IDX_W), .SLOT_W(SLOT_W)
      ) u_write (
        .mclk(mclk), .mreset(mreset),
        .pkt_valid(wr_pkt_valid[g]), .pkt(wr_pkt[g]), .tcam_ready(wr_tcam_ready[g]),
        .wlre_mode(mode), .bank_ready_vector(bank_ready_vector[g]),
        .packet(w_packet), .packet_valid(w_valid), .bank_valid_vector(w_bank), .has_ready(w_has_ready),
        .rd_ptr(w_rd_ptr), .compl_rd_ptr(r_rd_ptr), .compl_rd_fire(r_valid),
        // query out to the read side
        .q_search(w_q_search), .q_valid(w_q_valid),
        .q_hit(r_s_hit), .q_hit_ptr(r_s_hit_ptr), .q_hit_staging(r_s_hit_staging),
        // answer back to the read side's query
        .s_search(r_q_search), .s_valid(r_q_valid),
        .s_hit(w_s_hit), .s_hit_ptr(w_s_hit_ptr), .s_hit_staging(w_s_hit_staging)
      );

      // only the active half can fire, so mux it onto this rank's issue port
      assign out_valid[g]       = r_valid | w_valid;
      assign out_is_read[g]     = r_valid;
      assign out_packet[g]      = r_valid ? r_packet : w_packet;
      assign out_bank_vector[g] = r_valid ? r_bank   : w_bank;
    end
  endgenerate

endmodule : rmc_mc_top
