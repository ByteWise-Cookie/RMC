// rmc_sram.sv
// Generic single-clock simple-dual-port RAM (one write port, one read port).
// Fully parametric: data width, depth, read latency, and same-address collision
// mode. Standalone.
//   RD_LAT = 0  combinational read ("0th clk") - infers a flop/LUT array, not BRAM
//   RD_LAT = 1  synchronous read (1 cycle)
//   RD_LAT = 2  synchronous read + output register (2 cycles)
//   COLLIDE     WRITE_FIRST | READ_FIRST | NO_CHANGE on same-address wr+rd

module rmc_sram #(
  parameter int    WIDTH   = 32,
  parameter int    DEPTH   = 64,
  parameter int    RD_LAT  = 1,
  parameter string COLLIDE = "WRITE_FIRST",

  localparam int AW = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
  input  logic             clk,

  // write port
  input  logic             we,
  input  logic [AW-1:0]    waddr,
  input  logic [WIDTH-1:0] wdata,

  // read port
  input  logic             re,
  input  logic [AW-1:0]    raddr,
  output logic [WIDTH-1:0] rdata
);

  logic [WIDTH-1:0] mem [DEPTH];

  always_ff @(posedge clk)
    if (we) mem[waddr] <= wdata;

  generate
    if (RD_LAT == 0) begin : g_comb
      // combinational read (0th clk), write-first bypass on same address
      always_comb begin
        if (COLLIDE == "WRITE_FIRST" && we && (waddr == raddr))
          rdata = wdata;
        else
          rdata = mem[raddr];
      end
    end else begin : g_sync
      logic [WIDTH-1:0] rd_s1;
      always_ff @(posedge clk) begin
        if (re) begin
          if (COLLIDE == "WRITE_FIRST" && we && (waddr == raddr))
            rd_s1 <= wdata;                 // forward just-written data
          else if (COLLIDE == "NO_CHANGE" && we && (waddr == raddr))
            rd_s1 <= rd_s1;                 // hold old output
          else
            rd_s1 <= mem[raddr];            // READ_FIRST
        end
      end
      if (RD_LAT >= 2) begin : g_outreg
        logic [WIDTH-1:0] rd_s2;
        always_ff @(posedge clk) rd_s2 <= rd_s1;
        assign rdata = rd_s2;
      end else begin : g_flow
        assign rdata = rd_s1;
      end
    end
  endgenerate

endmodule : rmc_sram
