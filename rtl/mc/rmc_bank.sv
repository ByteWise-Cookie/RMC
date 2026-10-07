// rmc_bank.sv
// One per-bank cell {rank,bank}. Open-page, single open row. Exploits the STAGE-24
// address map: a request's packets arrive as a ROW-STREAK on a bank, so same-row
// packets are contiguous. Two comparators classify the admit stream:
//
//   CMP1: admit_row == open_row (and state==OPEN)  -> HIT  -> cas_fifo
//   CMP2: admit_row == last_miss_row               -> toggle: same row keeps the
//         toggle, a new row flips it. MISS -> row_miss_fifo tagged with the toggle.
//
// The 1-bit toggle delimits row-groups in row_miss_fifo (valid because runs are
// contiguous): draining the head group = pop while toggle == cur_toggle, i.e. one
// PRE+ACT opens the row, then CAS the whole group. cas_fifo holds hits to the row
// already open. Depths = BANK_Q_DEPTH = P_MAX (the row-hit train).
//
// FSM: IDLE -ACT-> ACTING -(can_cas=tRCD)-> OPEN -(CAS drain)-> PRE -> PREING
//      -(can_act=tRP)-> IDLE ; REFING for refresh. Scoreboard can_* gate issue and
// clock the transient states; the arb grants the actual command.
//
// First cut. Open: program-order between the two fifos, same-row-streak fairness cap,
// packets admitted during ACTING re-grouping (benign re-open). Flagged below.

import rmc_cfg_pkg::*;

module rmc_bank #(
  parameter int Q_DEPTH = BANK_Q_DEPTH,   // cas_fifo / row_miss_fifo depth (= P_MAX)
  parameter int SPTR_W  = 10              // payload pointer (mem_ptr / sram slot)
)(
  input  logic              clk,
  input  logic              rst_n,

  // admit one packet into this bank (from the decoder, after the lookahead)
  input  logic              admit_valid,
  output logic              admit_ready,
  input  logic [ROW_W-1:0]  admit_row,
  input  logic [COL_W-1:0]  admit_col,
  input  logic              admit_op,       // 0 = read, 1 = write
  input  logic [SPTR_W-1:0] admit_sptr,

  // timing legality from the scoreboard (per this bank)
  input  logic              can_act,
  input  logic              can_cas,
  input  logic              can_pre,

  // maintenance engine
  input  logic              ref_pending,
  input  logic              ref_done,

  // command readiness to the arb tree
  output logic              act_rdy,
  output logic              cas_rdy,
  output logic              pre_rdy,

  // grants back (one-hot; the granted command advances the FSM / pops a fifo)
  input  logic              grant_act,
  input  logic              grant_cas,
  input  logic              grant_pre,
  input  logic              grant_ref,

  // issued-command payload (packer picks by command)
  output logic [ROW_W-1:0]  out_row,        // ACT
  output logic [COL_W-1:0]  out_col,        // CAS
  output logic              out_op,         // CAS
  output logic [SPTR_W-1:0] out_sptr        // CAS
);

  // ---- FSM ----
  typedef enum logic [2:0] { IDLE, ACTING, OPEN, PREING, REFING } bank_state_e;
  bank_state_e state;

  logic [ROW_W-1:0] open_row;      // currently open DRAM row (valid in OPEN)
  logic             cur_toggle;    // toggle of the group whose row is open
  logic [ROW_W-1:0] last_miss_row; // CMP2 target (admit-side grouping)
  logic             toggle;        // current admit-side group toggle

  // ---- cas_fifo: hits to open_row.  entry = {col, op, sptr} ----
  localparam int CAS_W = COL_W + 1 + SPTR_W;
  logic             cas_empty, cas_full;
  logic [CAS_W-1:0] cas_head;
  logic             cas_wr, cas_rd;
  logic [COL_W-1:0]  cas_col;  logic cas_op;  logic [SPTR_W-1:0] cas_sptr;
  assign {cas_col, cas_op, cas_sptr} = cas_head;

  // ---- row_miss_fifo: misses, grouped by toggle.  entry = {row,col,op,sptr,tgl} ----
  localparam int MISS_W = ROW_W + COL_W + 1 + SPTR_W + 1;
  logic              rm_empty, rm_full;
  logic [MISS_W-1:0] rm_head;
  logic              rm_wr, rm_rd;
  logic [ROW_W-1:0]  rm_row;  logic [COL_W-1:0] rm_col;  logic rm_op;
  logic [SPTR_W-1:0] rm_sptr; logic rm_tgl;
  assign {rm_row, rm_col, rm_op, rm_sptr, rm_tgl} = rm_head;

  // ---- admit classify (CMP1) ----
  logic hit, adm_fire, adm_miss, new_row, push_tgl;
  assign hit      = (state == OPEN) & (admit_row == open_row);           // CMP1
  assign admit_ready = hit ? ~cas_full : ~rm_full;
  assign adm_fire = admit_valid & admit_ready;
  assign adm_miss = adm_fire & ~hit;
  assign new_row  = adm_miss & (admit_row != last_miss_row);             // CMP2
  assign push_tgl = new_row ? ~toggle : toggle;

  assign cas_wr = adm_fire & hit;
  assign rm_wr  = adm_miss;

  // ---- CAS source: the opened group (row_miss head, matching toggle) then hits ----
  logic miss_grp_valid, cas_src_miss, cas_src_hit;
  assign miss_grp_valid = (state == OPEN) & ~rm_empty & (rm_tgl == cur_toggle);
  assign cas_src_miss   = miss_grp_valid;
  assign cas_src_hit    = (state == OPEN) & ~cas_empty & ~miss_grp_valid;

  // ---- readiness ----
  logic need_switch;
  // current row done (group drained + no queued hits) and another row waits (or ref)
  assign need_switch = (state == OPEN) & cas_empty & ~miss_grp_valid &
                       (~rm_empty | ref_pending);
  assign act_rdy = (state == IDLE) & ~rm_empty & can_act;
  assign cas_rdy = (state == OPEN) & can_cas & (cas_src_miss | cas_src_hit);
  assign pre_rdy = need_switch & can_pre;

  // ---- fifo pops ----
  assign cas_rd = grant_cas & cas_src_hit;
  assign rm_rd  = grant_cas & cas_src_miss;

  // ---- issued payload ----
  assign out_row  = rm_row;                                   // ACT opens group's row
  assign out_col  = cas_src_miss ? rm_col  : cas_col;
  assign out_op   = cas_src_miss ? rm_op   : cas_op;
  assign out_sptr = cas_src_miss ? rm_sptr : cas_sptr;

  // ---- fifos ----
  rmc_fifo #(.WIDTH(CAS_W), .DEPTH(Q_DEPTH), .FWFT(1'b1)) u_cas_fifo (
    .clk(clk), .rst_n(rst_n),
    .wr_en(cas_wr), .wr_data({admit_col, admit_op, admit_sptr}),
    .rd_en(cas_rd), .rd_data(cas_head), .rd_valid(),
    .full(cas_full), .empty(cas_empty),
    .almost_full(), .almost_empty(), .count(), .overflow(), .underflow()
  );

  rmc_fifo #(.WIDTH(MISS_W), .DEPTH(Q_DEPTH), .FWFT(1'b1)) u_row_miss_fifo (
    .clk(clk), .rst_n(rst_n),
    .wr_en(rm_wr), .wr_data({admit_row, admit_col, admit_op, admit_sptr, push_tgl}),
    .rd_en(rm_rd), .rd_data(rm_head), .rd_valid(),
    .full(rm_full), .empty(rm_empty),
    .almost_full(), .almost_empty(), .count(), .overflow(), .underflow()
  );

  // ---- admit-side grouping state ----
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      toggle        <= 1'b0;
      last_miss_row <= '0;
    end else if (adm_miss) begin
      toggle        <= push_tgl;
      last_miss_row <= admit_row;
    end
  end

  // ---- FSM ----
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state      <= IDLE;
      open_row   <= '0;
      cur_toggle <= 1'b0;
    end else begin
      unique case (state)
        IDLE: begin
          if (ref_pending & grant_ref) state <= REFING;
          else if (grant_act) begin
            state      <= ACTING;
            open_row   <= rm_row;       // open the head group's row
            cur_toggle <= rm_tgl;       // lock the group being served
          end
        end
        ACTING:  if (can_cas)  state <= OPEN;    // tRCD elapsed
        OPEN:    if (grant_pre) state <= PREING; // row miss / switch
        PREING:  if (can_act)  state <= IDLE;    // tRP elapsed
        REFING:  if (ref_done) state <= IDLE;    // tRFC elapsed
        default:               state <= IDLE;
      endcase
    end
  end

endmodule : rmc_bank
