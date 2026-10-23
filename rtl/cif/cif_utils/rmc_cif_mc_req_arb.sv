// rmc_cif_mc_req_arb.sv
// Shared MC request arbiter. Both paths produce per-channel packet streams - read from its
// engines, write from w_pkt_gen - but the CIF has ONE request port to the MC. This block
// serializes all 2*N_CH sources onto async_mc_req_*, tags each with its direction (op), and
// hands every source its ready back (= granted this cycle AND the MC req FIFO accepted).
//
// Sources are flattened: index [0 .. N_CH-1] = read channel c (op=0), index [N_CH .. 2*N_CH-1]
// = write channel c (op=1). Round-robin priority (rr_ptr) rotates past the last grant so no
// source starves. The external async req FIFO provides the buffering + back-pressure
// (async_mc_req_ready = not full); a source's ready is asserted only on the cycle it wins and
// that FIFO accepts, so a granted packet is never dropped.
//
// pkt_num / last_in_txn (auto-precharge hint) are not produced by the sources yet - tied 0
// here. TODO: carry them from the splitters when the MC scheduler needs them.

import rmc_cfg_pkg::*;

module rmc_cif_mc_req_arb #(
  parameter int ROB_IDX_W = 5,
  parameter int DADDR_W   = 32,
  parameter int SLOT_W    = 6,
  parameter int PKT_NUM_W = 6,

  // derived (localparam: never passed in)
  localparam int NSRC   = 2 * N_CH,                       // read + write, per channel
  localparam int SEL_W  = (NSRC > 1) ? $clog2(NSRC) : 1
)(
  input  logic                              aclk,
  input  logic                              aresetn,

  // read sources (one per channel), from the read splitter engines
  input  logic [N_CH-1:0]                   rd_valid,
  output logic [N_CH-1:0]                   rd_ready,
  input  logic [N_CH-1:0][ROB_IDX_W-1:0]    rd_rob_idx,
  input  logic [N_CH-1:0][DADDR_W-1:0]      rd_daddr,
  input  logic [N_CH-1:0][SLOT_W-1:0]       rd_slot,

  // write sources (one per channel), from w_pkt_gen
  input  logic [N_CH-1:0]                   wr_valid,
  output logic [N_CH-1:0]                   wr_ready,
  input  logic [N_CH-1:0][ROB_IDX_W-1:0]    wr_rob_idx,
  input  logic [N_CH-1:0][DADDR_W-1:0]      wr_daddr,
  input  logic [N_CH-1:0][SLOT_W-1:0]       wr_slot,

  // single MC request port (into the req async FIFO)
  output logic                              async_mc_req_valid,
  input  logic                              async_mc_req_ready,
  output logic [ROB_IDX_W-1:0]              async_mc_req_rob_index,
  output logic                              async_mc_req_op,          // 0=read, 1=write
  output logic [DADDR_W-1:0]                async_mc_req_daddr,
  output logic [PKT_NUM_W-1:0]              async_mc_req_pkt_num,
  output logic                              async_mc_req_last_in_txn,
  output logic [SLOT_W-1:0]                 async_mc_req_sram_slot
);

  // flatten sources: [0..N_CH-1] read, [N_CH..2N_CH-1] write
  logic [NSRC-1:0]              s_valid;
  logic [NSRC-1:0][ROB_IDX_W-1:0] s_rob;
  logic [NSRC-1:0][DADDR_W-1:0]   s_daddr;
  logic [NSRC-1:0][SLOT_W-1:0]    s_slot;
  always_comb begin
    for (int c = 0; c < N_CH; c++) begin
      s_valid[c]        = rd_valid[c];
      s_rob[c]          = rd_rob_idx[c];
      s_daddr[c]        = rd_daddr[c];
      s_slot[c]         = rd_slot[c];
      s_valid[N_CH+c]   = wr_valid[c];
      s_rob[N_CH+c]     = wr_rob_idx[c];
      s_daddr[N_CH+c]   = wr_daddr[c];
      s_slot[N_CH+c]    = wr_slot[c];
    end
  end

  // round-robin grant: first valid source at/after rr_ptr (circular)
  logic [SEL_W-1:0] rr_ptr;
  logic             grant_valid;
  logic [SEL_W-1:0] grant_idx;
  always_comb begin
    grant_valid = 1'b0;
    grant_idx   = '0;
    for (int k = 0; k < NSRC; k++) begin
      int s;
      s = (int'(rr_ptr) + k) % NSRC;
      if (!grant_valid && s_valid[s]) begin
        grant_valid = 1'b1;
        grant_idx   = SEL_W'(s);
      end
    end
  end

  wire accept = grant_valid & async_mc_req_ready;         // winner's packet taken this cycle

  // drive the MC request port from the granted source
  assign async_mc_req_valid      = grant_valid;
  assign async_mc_req_rob_index  = s_rob[grant_idx];
  assign async_mc_req_op         = (grant_idx >= SEL_W'(N_CH));   // write sources are the upper half
  assign async_mc_req_daddr      = s_daddr[grant_idx];
  assign async_mc_req_sram_slot  = s_slot[grant_idx];
  assign async_mc_req_pkt_num    = '0;                    // TODO: source-provided
  assign async_mc_req_last_in_txn= 1'b0;                  // TODO: source-provided

  // hand ready to the granted source only, on the cycle the FIFO accepts
  always_comb begin
    rd_ready = '0;
    wr_ready = '0;
    if (accept) begin
      if (grant_idx < SEL_W'(N_CH)) rd_ready[grant_idx]            = 1'b1;
      else                          wr_ready[grant_idx - SEL_W'(N_CH)] = 1'b1;
    end
  end

  // rotate priority past the winner so the next arb favours the following source
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn)    rr_ptr <= '0;
    else if (accept) rr_ptr <= (int'(grant_idx) == NSRC-1) ? '0 : grant_idx + 1'b1;
  end

endmodule : rmc_cif_mc_req_arb
