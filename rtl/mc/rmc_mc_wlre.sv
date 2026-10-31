// rmc_mc_wlre.sv
//
// Write/read direction selector for one rank's packet TCAM pair. Owns the single mode bit the two
// halves share (1 = read drives the command bus, 0 = write drives); only the active half may issue.
//
// Policy: stay on a direction while it keeps issuing (cuts read/write bus turnarounds), but hand
// over when the active half has nothing ready, or when it has run QUANTUM issues back to back and
// the other half has work waiting. Can't deadlock - every TCAM dependency is cross-direction, so
// running the other half always clears blockers. Can't starve - the quantum forces a handover.

module rmc_mc_wlre #(
  parameter int QUANTUM = 8,                        // max back-to-back issues before a forced yield
  localparam int GCNT_W = (QUANTUM > 1) ? $clog2(QUANTUM) : 1
)(
  input  logic mclk,
  input  logic mreset,

  // readiness + issue pulse from each half (read, write)
  input  logic rd_has_ready,
  input  logic wr_has_ready,
  input  logic rd_issued,
  input  logic wr_issued,

  output logic mode                                 // 1 = read drives, 0 = write drives
);

  logic [GCNT_W-1:0] grant_cnt;

  wire active_has_ready = mode ? rd_has_ready : wr_has_ready;
  wire other_has_ready  = mode ? wr_has_ready : rd_has_ready;
  wire active_issued    = mode ? rd_issued    : wr_issued;

  always_ff @(posedge mclk or negedge mreset) begin
    if (!mreset) begin
      mode      <= 1'b1;          // read first
      grant_cnt <= '0;
    end else if (!active_has_ready) begin
      mode      <= ~mode;         // active dry: hand the bus over
      grant_cnt <= '0;
    end else if (active_issued) begin
      if (grant_cnt == GCNT_W'(QUANTUM-1) && other_has_ready) begin
        mode      <= ~mode;       // quantum spent and the other has work: yield
        grant_cnt <= '0;
      end else begin
        grant_cnt <= grant_cnt + GCNT_W'(1);
      end
    end
  end

endmodule : rmc_mc_wlre
