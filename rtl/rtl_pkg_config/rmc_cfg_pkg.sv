// rmc_cfg_pkg.sv
// Compile-time configuration for RMC. ONE DDR generation per build.
// Select the generation with DDR_GEN; structural widths/counts derive here.
// Runtime-tunable values (timing, mode registers) live in rmc_csr.sv, not here.

package rmc_cfg_pkg;

  // ---- DDR generation select (build-time) ----
  typedef enum logic [2:0] {
    DDR1 = 3'd1,
    DDR2 = 3'd2,
    DDR3 = 3'd3,
    DDR4 = 3'd4,
    DDR5 = 3'd5
  } ddr_gen_e;

  // pick the generation for this build
  parameter ddr_gen_e DDR_GEN = DDR5;

  // ---- structural counts (keep parametric, never hardcode) ----
  parameter int N_RANKS = 2;

  // per-gen structural params, resolved at elaboration from DDR_GEN
  // TODO: fill real values for DDR1..4
  function automatic int bg_count(ddr_gen_e g);
    case (g)
      DDR4:    return 4;    // 4 bank groups
      DDR5:    return 8;    // 8 bank groups
      default: return 1;    // DDR1-3: no bank groups  (TODO verify)
    endcase
  endfunction

  function automatic int banks_per_bg(ddr_gen_e g);
    case (g)
      DDR4:    return 4;
      DDR5:    return 4;
      default: return 8;    // flat bank array (DDR1-3)  TODO verify
    endcase
  endfunction

  parameter int N_BG          = bg_count(DDR_GEN);
  parameter int N_BANK_PER_BG = banks_per_bg(DDR_GEN);
  parameter int N_BANK        = N_BG * N_BANK_PER_BG;   // banks per rank

  // address geometry  (TODO: set per gen / density)
  parameter int ROW_W = 18;
  parameter int COL_W = 10;

  // data / burst  (TODO: per gen)
  parameter int BL   = 16;   // burst length (DDR5 default)
  parameter int DQ_W = 32;   // sub-channel data width

  // derived index widths
  parameter int RANK_W       = (N_RANKS       > 1) ? $clog2(N_RANKS)       : 1;
  parameter int BG_W         = (N_BG          > 1) ? $clog2(N_BG)          : 1;
  parameter int BANK_W       = (N_BANK        > 1) ? $clog2(N_BANK)        : 1; // full {bg,bank}
  parameter int BANK_PER_BG_W= (N_BANK_PER_BG > 1) ? $clog2(N_BANK_PER_BG) : 1; // bank within a BG

  // canonical DRAM-coord width {rank, bg, bank_in_bg, row, col}
  parameter int DADDR_W = RANK_W + BG_W + BANK_PER_BG_W + ROW_W + COL_W;

  // ---- packet / channel geometry ----
  parameter int PKT_BYTES = BL * DQ_W / 8;              // 64 B (BL16 x 32b subchannel)
  parameter int PKT_OFF_W = $clog2(PKT_BYTES);          // 6
  parameter int N_CH      = 2;                           // channels (1 MC core each)
  parameter int CH_W      = (N_CH > 1) ? $clog2(N_CH) : 1;

  // ---- packet interleave / packets-per-bank (STAGE 24) ----
  // A request fans into packets; P_MAX = packets mapped to one bank (row-hit train).
  // Spread the rest across BGs (rotate) so consecutive packets are different-BG (tCCD_S)
  // and ACTs hide. P_MAX sets the per-bank row-hit store / cas_fifo depth.
  parameter int MAX_REQ_BYTES = 4096;                   // 4 KB max burst
  parameter int MAX_REQ_PKTS  = MAX_REQ_BYTES / PKT_BYTES;          // 64
  parameter int PKTS_PER_RANK = MAX_REQ_PKTS / (N_CH * N_RANKS);    // 16
  parameter int P_MAX         = 4;                      // packets/bank (CSR-knob down to 2)
  parameter int BANKS_USED    = PKTS_PER_RANK / P_MAX;  // 4 banks (in 4 different BGs)
  parameter int BG_LO_W       = (BANKS_USED > 1) ? $clog2(BANKS_USED) : 1; // BGs rotated
  parameter int BG_HI_W       = BG_W - BG_LO_W;         // remaining BG bits (which BG set)
  parameter int COL_LO_W      = (P_MAX > 1) ? $clog2(P_MAX) : 1;    // row-hit revisit bits
  parameter int COL_HI_W      = COL_W - COL_LO_W;

  // ---- interleave bit positions (LSB->MSB), running sum (STAGE 24 layout) ----
  parameter int OFF_POS   = 0;                        // [PKT_OFF_W-1:0] byte-in-packet
  parameter int CH_POS    = OFF_POS   + PKT_OFF_W;    // channel  (cache-line interleave)
  parameter int RANK_POS  = CH_POS    + CH_W;         // rank
  parameter int BGLO_POS  = RANK_POS  + RANK_W;       // BG rotate (consecutive pkt -> diff BG)
  parameter int COLLO_POS = BGLO_POS  + BG_LO_W;      // col-lo = row-hit revisit (= P)
  parameter int BGHI_POS  = COLLO_POS + COL_LO_W;     // BG high (which BG set)
  parameter int BANK_POS  = BGHI_POS  + BG_HI_W;      // bank within BG
  parameter int COLHI_POS = BANK_POS  + BANK_PER_BG_W;// col-high
  parameter int ROW_POS   = COLHI_POS + COL_HI_W;     // row (MSB)

  // ---- per-bank path sizing (STAGE 24 -> STAGE 12/15) ----
  parameter int BANK_Q_DEPTH = P_MAX;    // row-hit store / cas_fifo depth per bank

endpackage : rmc_cfg_pkg
