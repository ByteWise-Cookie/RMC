// rmc_mc_packet_tcam.sv
//
// One direction (read or write, set by DIR_IS_READ) of a rank's packet scheduler. Two of these
// pair as complements (in rmc_mc_rank_tcam): a read and a write to the same DRAM line are forced
// out in RAW/WAR order, and a packet leaves only when its bank is ready. Sits after the rank demux.
// Same-direction order (WAW, RAR) is upstream's job (AXI in-order), so only read-vs-write is here.
// Issue is out of order + round-robin, so a cleared, bank-ready entry never waits behind others.
//
// Storage, per entry:
//   valid_vector   slot occupied.
//   phy_cam        DRAM line address - the CAM the complement searches.
//   bank_id        {bg,bank}, decoded once at intake so the ready path need not re-slice phy_cam.
//   can_drain      bank_ready_vector[bank_id]: this entry's bank can take a command.
//   stall_vector   ordering dependency still pending (cam_hit).
//   dependant_ptr  complement slot this entry waits behind (cam_hit_ptr).
//   in_stage       the dependency is on a complement packet still in its staging reg (ptr = the
//                  slot it is landing in), not a committed entry.
//   rank_sram      payload: {CIF rob_idx, WD/RD-SRAM slot}.
//
// Intake: a demux beat lands in the staging reg; its address searches the complement's committed
// phy_cam AND its staging reg. A staging match is the newest access, so it wins the pointer, and is
// reported only when that staged packet commits the same cycle (its landing slot is then real).
//   committed hit : stall behind it (dependant_ptr = that slot).
//   staging  hit  : both sides staged the same line - read goes first: write stalls behind read's
//                   incoming slot, read does not stall.
//   miss          : free to go.
//
// Issue: ready = valid & ~stall & can_drain. When the shared WLRE bit is this direction, the RR
// picker takes one ready entry: payload out, bank_valid_vector one-hots its bank, slot frees, index
// goes out on rd_ptr. has_ready tells the WLRE this side could issue.
//
// Unstall: the complement watches this rd_ptr; when it issues the slot a stalled entry waits on
// (dependant_ptr), that stall clears. Every dep is cross-direction, so running the other side
// always clears blockers - the WLRE can switch on stall without deadlock (pairing in rmc_mc_rank_tcam).

import rmc_cfg_pkg::*;

module rmc_mc_packet_tcam #(
  parameter bit DIR_IS_READ = 1'b1,            // 1 = read TCAM, 0 = write TCAM (its WLRE mode)
  parameter int DEPTH       = PKTS_PER_RANK,   // table entries
  parameter int ROB_IDX_W   = 5,               // CIF rob_index tag width
  parameter int SLOT_W      = 6,               // WD/RD-SRAM slot width

  // derived
  localparam int PTR_W    = (DEPTH > 1) ? $clog2(DEPTH) : 1,
  localparam int ADDR_W   = DADDR_W,           // stored/searched DRAM address (rank..col)
  localparam int BANK_LSB = ROW_W + COL_W,     // {bg,bank} sit just above row+col in daddr
  localparam int META_W   = ROB_IDX_W + SLOT_W,// {idx, slot}
  localparam int PKT_W    = META_W + ADDR_W    // packet = {idx, slot, daddr}
)(
  input  logic                 mclk,
  input  logic                 mreset,

  // rank-demux intake
  input  logic                 pkt_valid,
  input  logic [PKT_W-1:0]     pkt,
  output logic                 tcam_ready,

  // WLRE / per-bank issue
  input  logic                 wlre_mode,          // shared 1-bit mode: 1 = read, 0 = write
  input  logic [N_BANK-1:0]    bank_ready_vector,
  output logic [PKT_W-1:0]     packet,
  output logic                 packet_valid,       // single-bit issue (selected entry leaves)
  output logic [N_BANK-1:0]    bank_valid_vector,
  output logic                 has_ready,          // any entry issuable now (mode-independent) - WLRE hint

  // read-pointer exchange with the complement TCAM (same-address ordering)
  output logic [PTR_W-1:0]     rd_ptr,             // index issued this cycle (valid when packet_valid)
  input  logic [PTR_W-1:0]     compl_rd_ptr,       // index the complement issued
  input  logic                 compl_rd_fire,      // complement issued one this cycle

  // query the complement at intake (this -> complement)
  output logic [ADDR_W-1:0]    q_search,
  output logic                 q_valid,
  input  logic                 q_hit,
  input  logic [PTR_W-1:0]     q_hit_ptr,
  input  logic                 q_hit_staging,      // hit was the complement's staging reg (not committed)

  // answer the complement's query (complement -> this), combinational
  input  logic [ADDR_W-1:0]    s_search,
  input  logic                 s_valid,
  output logic                 s_hit,
  output logic [PTR_W-1:0]     s_hit_ptr,
  output logic                 s_hit_staging       // the match is our staging reg, landing in free_sel this cycle
);

  // packet field slices: {idx, slot, daddr} from MSB
  function automatic logic [ROB_IDX_W-1:0] p_idx (input logic [PKT_W-1:0] p); return p[PKT_W-1 -: ROB_IDX_W]; endfunction
  function automatic logic [SLOT_W-1:0]    p_slot(input logic [PKT_W-1:0] p); return p[ADDR_W +: SLOT_W];     endfunction
  function automatic logic [ADDR_W-1:0]    p_adr (input logic [PKT_W-1:0] p); return p[ADDR_W-1:0];           endfunction
  // {bg,bank} index of a daddr, into bank_ready_vector / bank_valid_vector
  function automatic logic [BANK_W-1:0]    a_bank(input logic [ADDR_W-1:0] a); return a[BANK_LSB +: BANK_W];  endfunction

  // table storage
  logic [META_W-1:0]   rank_sram     [DEPTH];   // RANK_SRAM {idx, slot}
  logic [ADDR_W-1:0]   phy_cam       [DEPTH];   // phy_adr CAM (daddr, searchable by complement)
  logic [BANK_W-1:0]   bank_id       [DEPTH];   // {bg,bank} decoded once at intake (no per-cycle re-slice)
  logic                valid_vector  [DEPTH];   // entry occupied
  logic                stall_vector  [DEPTH];   // ordering dependency pending (cam_hit)
  logic [PTR_W-1:0]    dependant_ptr [DEPTH];   // complement index this entry waits behind (cam_hit_ptr)
  logic                in_stage      [DEPTH];   // dep is a complement STAGING packet (incoming slot), not committed

  // ---- occupancy / full (full = every slot valid) ----
  logic full;
  always_comb begin
    full = 1'b1;
    for (int i = 0; i < DEPTH; i++) if (!valid_vector[i]) full = 1'b0;
  end

  // ---- intake skid (S0) ----
  logic             s0_valid;
  logic [PKT_W-1:0] s0_pkt;
  wire  s1_ready  = ~full;                        // table has a free slot
  wire  s0_accept = pkt_valid & tcam_ready;       // demux beat taken into the skid
  wire  s0_fire   = s0_valid & s1_ready;          // staged beat written into the table
  assign tcam_ready = ~s0_valid | s1_ready;       // room: skid empty, or draining this cycle

  // first free slot for alloc (lowest index with ~valid)
  logic [PTR_W-1:0] free_sel;
  always_comb begin
    free_sel = '0;
    for (int i = DEPTH-1; i >= 0; i--) if (!valid_vector[i]) free_sel = PTR_W'(i);
  end

  // intake searches the complement with the staged address
  assign q_search = p_adr(s0_pkt);
  assign q_valid  = s0_valid;

  // ---- issue: pick one READY entry ----
  // can_drain[i] = this entry's bank can accept a command (bank decoded once at intake).
  // ready[i]     = occupied & no pending ordering dependency & can_drain.
  logic [DEPTH-1:0] can_drain;
  logic [DEPTH-1:0] ready;
  always_comb
    for (int i = 0; i < DEPTH; i++) begin
      can_drain[i] = bank_ready_vector[bank_id[i]];
      ready[i]     = valid_vector[i] & ~stall_vector[i] & can_drain[i];
    end

  wire mode_ok = (wlre_mode == DIR_IS_READ);
  // round-robin pick: the first ready entry at/after rr_ptr (circular). rr_ptr advances past the
  // winner each issue, so service rotates instead of always favouring the lowest index.
  logic [PTR_W-1:0] rr_ptr;
  logic             any_ready;
  logic [PTR_W-1:0] rd_sel;
  always_comb begin
    any_ready = 1'b0;
    rd_sel    = rr_ptr;
    for (int k = DEPTH-1; k >= 0; k--)
      if (ready[(int'(rr_ptr) + k) % DEPTH]) begin
        any_ready = 1'b1;
        rd_sel    = PTR_W'((int'(rr_ptr) + k) % DEPTH);
      end
  end
  wire read_fire = mode_ok & any_ready;

  assign has_ready    = any_ready;       // mode-independent: lets the WLRE batch/switch direction
  assign rd_ptr       = rd_sel;
  assign packet       = {rank_sram[rd_sel], phy_cam[rd_sel]};
  assign packet_valid = read_fire;
  always_comb begin
    bank_valid_vector = '0;
    if (read_fire) bank_valid_vector[bank_id[rd_sel]] = 1'b1;
  end

  // ---- complement search answer (combinational) ----
  // Search committed entries AND our own staging reg: the complement may still be holding a
  // same-line packet in its skid that has not yet reached phy_cam, and that packet is the NEWEST
  // of all, so it wins the pointer. It is only reported when it commits this cycle (s0_fire), so
  // free_sel is the real slot it lands in and the dependant_ptr the querier records is valid next
  // cycle. s_hit_staging flags that match so the querier can apply the read-first tie-break.
  always_comb begin
    s_hit         = 1'b0;
    s_hit_ptr     = '0;
    s_hit_staging = 1'b0;
    // committed entries
    for (int i = DEPTH-1; i >= 0; i--)
      if (s_valid && valid_vector[i] && phy_cam[i] == s_search) begin
        s_hit     = 1'b1;
        s_hit_ptr = PTR_W'(i);
      end
    // own staging reg (committing this cycle) - newest, overrides the pointer with its landing slot
    if (s_valid && s0_fire && p_adr(s0_pkt) == s_search) begin
      s_hit         = 1'b1;
      s_hit_ptr     = free_sel;
      s_hit_staging = 1'b1;
    end
  end

  // intake ordering decision (combinational), applied on s0_fire:
  //   committed complement hit  -> stall behind it (normal RAW/WAR).
  //   staging-only complement hit (both sides stage the same line this cycle) -> read goes first:
  //     the WRITE instance stalls behind the READ's incoming slot; the READ does not stall.
  wire               intake_commit_hit  = q_hit & ~q_hit_staging;
  wire               intake_staging_hit = q_hit &  q_hit_staging;
  wire               intake_stall       = intake_commit_hit
                                        | (intake_staging_hit & (DIR_IS_READ == 1'b0));
  wire [PTR_W-1:0]   intake_dep         = q_hit_ptr;

  always_ff @(posedge mclk or negedge mreset) begin
    if (!mreset) begin
      s0_valid <= 1'b0;
      rr_ptr   <= '0;
      for (int i = 0; i < DEPTH; i++) begin
        valid_vector[i] <= 1'b0;
        stall_vector[i] <= 1'b0;
        in_stage[i]     <= 1'b0;
      end
    end else begin
      // S0 skid
      if (s0_accept) s0_pkt <= pkt;
      if      (s0_accept) s0_valid <= 1'b1;
      else if (s0_fire)   s0_valid <= 1'b0;

      // intake: drop the staged packet into a free slot, capture its ordering dependency
      if (s0_fire) begin
        rank_sram[free_sel]     <= {p_idx(s0_pkt), p_slot(s0_pkt)};
        phy_cam[free_sel]       <= p_adr(s0_pkt);
        bank_id[free_sel]       <= a_bank(p_adr(s0_pkt));  // decode bank once, here
        valid_vector[free_sel]  <= 1'b1;
        stall_vector[free_sel]  <= intake_stall;                       // cam_hit
        dependant_ptr[free_sel] <= intake_stall ? intake_dep : '0;     // cam_hit_ptr
        in_stage[free_sel]      <= intake_stall & intake_staging_hit;  // dep on complement's staged slot
      end

      // dependency clear: the blocking complement entry is being issued now
      for (int i = 0; i < DEPTH; i++)
        if (stall_vector[i] && compl_rd_fire && compl_rd_ptr == dependant_ptr[i]) stall_vector[i] <= 1'b0;

      // issue: the selected entry leaves, free its slot; rotate RR past the winner
      if (read_fire) begin
        valid_vector[rd_sel] <= 1'b0;
        rr_ptr <= (int'(rd_sel) == DEPTH-1) ? '0 : rd_sel + 1'b1;
      end
    end
  end

endmodule : rmc_mc_packet_tcam
