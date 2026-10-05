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
  parameter int RANK_W = (N_RANKS > 1) ? $clog2(N_RANKS) : 1;
  parameter int BG_W   = (N_BG    > 1) ? $clog2(N_BG)    : 1;
  parameter int BANK_W = (N_BANK  > 1) ? $clog2(N_BANK)  : 1;

endpackage : rmc_cfg_pkg
