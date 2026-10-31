// rmc_mc_bank_sb.sv
//
// Per-bank timing scoreboard, one per {rank, bank}, beside rmc_bank. Tracks the intra-bank DDR
// timing as ABSOLUTE deadlines against the free-running 20-bit global cycle counter (gc), per the
// spec (rmc_theory: rmc_knowledge_base "Per-Bank Timing Eligibility Fields", scheduler_microarch
// section 4). NOT down-counters: each constraint is stored as the gc value at which it next
// becomes legal, and eligibility is a wraparound-safe sign-bit compare.
//
//   next_cas = gc + tRCD                                   on ACT
//   next_act = gc + tRP                                    on PRE
//   next_pre = MAX( ACT: gc+tRAS, RD: gc+tRTP, WR: gc+tWR_eff )   (tWR_eff = CWL+BL/2+tWR)
// next_pre takes the MAX over its writers (a later event pushes the precharge deadline out), done
// as a modular "keep the later deadline" using the same gc_ge compare.
//
//   can_cas/can_act/can_pre = gc_ge(gc, next_*) = (gc - next_*)[MSB] == 0
// which is gc >= next_* modular, safe across the gc rollover (same trick as TCP sequence numbers).
//
// Inter-bank timing (tCCD, tRRD, tFAW, tWTR, tRTW, bus turnaround) and refresh (tRFC) are NOT here:
// those are the BG / rank / global scoreboard levels above the per-bank cell.

module rmc_mc_bank_sb #(
  parameter int GC_W    = 20,               // global cycle counter width (spec)
  parameter int TIMER_W = 11                // CSR timing-value width (<= GC_W)
)(
  input  logic                mclk,
  input  logic                mreset,

  // free-running global cycle counter (shared, read-only here)
  input  logic [GC_W-1:0]     gc,

  // timing values in mclk cycles (CSR timing shadow; static after init)
  input  logic [TIMER_W-1:0]  t_rcd,        // ACT -> CAS
  input  logic [TIMER_W-1:0]  t_rp,         // PRE -> ACT
  input  logic [TIMER_W-1:0]  t_ras,        // ACT -> PRE (min row-open)
  input  logic [TIMER_W-1:0]  t_rtp,        // RD  -> PRE
  input  logic [TIMER_W-1:0]  t_wr,         // WR  -> PRE, effective = CWL + BL/2 + tWR

  // command events (the arb grants this bank acts on)
  input  logic                grant_act,
  input  logic                grant_cas,
  input  logic                grant_pre,
  input  logic                cas_is_write, // granted CAS is a write (tWR) vs a read (tRTP)

  // legality to rmc_bank
  output logic                can_act,
  output logic                can_cas,
  output logic                can_pre
);

  // absolute deadlines (gc value at which the command next becomes legal)
  logic [GC_W-1:0] next_act, next_cas, next_pre;

  // gc + timing, in GC_W width
  wire [GC_W-1:0] d_rcd = gc + GC_W'(t_rcd);
  wire [GC_W-1:0] d_rp  = gc + GC_W'(t_rp);
  wire [GC_W-1:0] d_ras = gc + GC_W'(t_ras);
  wire [GC_W-1:0] d_rtp = gc + GC_W'(t_rtp);
  wire [GC_W-1:0] d_wr  = gc + GC_W'(t_wr);

  // wraparound-safe "a >= b": (a - b) has a clear MSB within the half-range window
  function automatic logic gc_ge(input logic [GC_W-1:0] a, input logic [GC_W-1:0] b);
    logic [GC_W-1:0] diff;
    diff  = a - b;
    gc_ge = ~diff[GC_W-1];
  endfunction

  // the CAS writer's precharge deadline: read uses tRTP, write uses tWR_eff
  wire [GC_W-1:0] d_cas_pre = cas_is_write ? d_wr : d_rtp;

  always_ff @(posedge mclk or negedge mreset) begin
    if (!mreset) begin
      next_act <= '0;                       // gc=0 at reset -> gc_ge true -> idle bank may ACT
      next_cas <= '0;
      next_pre <= '0;
    end else begin
      if (grant_act) begin
        next_cas <= d_rcd;                  // tRCD to the first CAS
        next_pre <= d_ras;                  // tRAS before this row can be precharged
      end else if (grant_cas) begin
        // push the precharge deadline out if this CAS needs longer (modular max)
        if (gc_ge(d_cas_pre, next_pre)) next_pre <= d_cas_pre;
      end

      if (grant_pre) next_act <= d_rp;      // tRP before the next ACT
    end
  end

  assign can_cas = gc_ge(gc, next_cas);
  assign can_act = gc_ge(gc, next_act);
  assign can_pre = gc_ge(gc, next_pre);

endmodule : rmc_mc_bank_sb
