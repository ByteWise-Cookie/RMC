// rmc_mc_wrr_arb.sv
// Weighted round-robin arbiter. Grants the ready input with the highest key; ties go to the lowest
// index. In WRR mode the key is an internal aging counter: the winner resets to 0, every other
// ready-but-waiting input ages up (saturating), so a starved input eventually wins - least-recently
// -granted order. PRIORITY_EN=1 swaps the key for the external per-input prio (strict priority).

module rmc_mc_wrr_arb #(
  parameter int INPUT_WIDTH  = 4,                 // number of requesters
  parameter int WEIGHT_WIDTH = 3,                 // aging-counter width (cap = all ones)
  parameter int PRIO_WIDTH   = 3,                 // per-input priority level width
  parameter bit PRIORITY_EN  = 1'b0,              // 1 = strict priority on prio, 0 = aging WRR
  localparam int SEL_W = (INPUT_WIDTH > 1) ? $clog2(INPUT_WIDTH) : 1,
  localparam int KEY_W = PRIORITY_EN ? PRIO_WIDTH : WEIGHT_WIDTH
)(
  input  logic                                     mclk,
  input  logic                                     mreset,

  input  logic [INPUT_WIDTH-1:0]                   ready_vector,  // requester c asks when it can go
  input  logic [INPUT_WIDTH-1:0][PRIO_WIDTH-1:0]   prio,          // per-input priority (PRIORITY_EN mode)
  input  logic                                     grant_taken,   // winner consumed -> advance

  output logic                                     grant_valid,   // some input won this cycle
  output logic [INPUT_WIDTH-1:0]                   grant_oh,      // winner one-hot
  output logic [SEL_W-1:0]                         grant_idx      // winner index
);

  // the key each input is ranked by: external prio in priority mode, internal age in WRR mode
  logic [INPUT_WIDTH-1:0][KEY_W-1:0] key;

  // pick: highest key among the ready set, ties to the lowest index (first strict-max wins)
  logic [KEY_W-1:0] best_key;
  always_comb begin
    grant_valid = 1'b0;
    grant_idx   = '0;
    best_key    = '0;
    for (int i = 0; i < INPUT_WIDTH; i++)
      if (ready_vector[i] & (~grant_valid | (key[i] > best_key))) begin
        grant_valid = 1'b1;
        grant_idx   = SEL_W'(i);
        best_key    = key[i];
      end
  end

  always_comb begin
    grant_oh = '0;
    if (grant_valid) grant_oh[grant_idx] = 1'b1;
  end

  generate
    if (PRIORITY_EN) begin : g_prio
      assign key = prio;                                   // rank straight off the external priority
    end else begin : g_wrr
      // internal aging counters: winner resets, ready waiters climb (saturating)
      logic [INPUT_WIDTH-1:0][WEIGHT_WIDTH-1:0] weight;
      always_comb
        for (int i = 0; i < INPUT_WIDTH; i++) key[i] = weight[i];

      always_ff @(posedge mclk or negedge mreset) begin
        if (!mreset) begin
          for (int i = 0; i < INPUT_WIDTH; i++) weight[i] <= '0;
        end else if (grant_taken & grant_valid) begin
          for (int i = 0; i < INPUT_WIDTH; i++) begin
            if (SEL_W'(i) == grant_idx)      weight[i] <= '0;                 // served -> back of line
            else if (ready_vector[i])        weight[i] <= (&weight[i]) ? weight[i]        // saturate
                                                                       : weight[i] + 1'b1; // age up
            // not ready -> hold
          end
        end
      end
    end
  endgenerate

endmodule : rmc_mc_wrr_arb
