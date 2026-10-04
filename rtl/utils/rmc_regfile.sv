// rmc_regfile.sv
// Generic register set: a flop array with one synchronous write port and N
// combinational (same-cycle) read ports. Fully parametric: data width, depth,
// number of read ports, reset value, and optional write-to-read bypass so a read
// of the slot being written this cycle returns the new data. Standalone.
//
// This is the comb-read store to use where a sync SRAM's +1 read latency is not
// acceptable (small, window-sized tables). For large/dense storage use rmc_sram.

module rmc_regfile #(
  parameter int              WIDTH    = 32,   // data width
  parameter int              DEPTH    = 16,   // entries
  parameter int              N_RD     = 1,    // combinational read ports
  parameter bit              WR_BYPASS= 1'b1, // 1 = forward write to same-cycle reads
  parameter logic [WIDTH-1:0] RST_VAL = '0,   // reset value per entry

  localparam int AW = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
  input  logic                          clk,
  input  logic                          rst_n,

  // write port
  input  logic                          we,
  input  logic [AW-1:0]                 waddr,
  input  logic [WIDTH-1:0]              wdata,

  // N combinational read ports
  input  logic [N_RD-1:0][AW-1:0]       raddr,
  output logic [N_RD-1:0][WIDTH-1:0]    rdata
);

  logic [WIDTH-1:0] rf [DEPTH];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < DEPTH; i++) rf[i] <= RST_VAL;
    end else if (we) begin
      rf[waddr] <= wdata;
    end
  end

  // combinational reads, with optional write-first bypass
  always_comb begin
    for (int p = 0; p < N_RD; p++) begin
      if (WR_BYPASS && we && (waddr == raddr[p]))
        rdata[p] = wdata;
      else
        rdata[p] = rf[raddr[p]];
    end
  end

endmodule : rmc_regfile
