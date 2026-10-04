// rmc_sram.sv
// Generic single-clock simple-dual-port RAM (one write port, one read port).
// Fully parametric: data width, depth (any value), synchronous read with an
// optional extra output register, and a selectable same-address collision mode.
// Maps to a block-RAM or a latch/flop array depending on the tool. Standalone.

module rmc_sram #(
  parameter int    WIDTH   = 32,          // data width
  parameter int    DEPTH   = 64,          // entries
  parameter bit    OUTREG  = 1'b0,        // 0 = 1-cycle read, 1 = 2-cycle (extra reg)
  parameter string COLLIDE = "WRITE_FIRST", // same-addr wr+rd: WRITE_FIRST|READ_FIRST|NO_CHANGE

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
  logic [WIDTH-1:0] rd_s1;

  // write
  always_ff @(posedge clk)
    if (we) mem[waddr] <= wdata;

  // read (stage 1), with same-address collision handling
  always_ff @(posedge clk) begin
    if (re) begin
      if (COLLIDE == "WRITE_FIRST" && we && (waddr == raddr))
        rd_s1 <= wdata;                 // forward the just-written data
      else if (COLLIDE == "NO_CHANGE" && we && (waddr == raddr))
        rd_s1 <= rd_s1;                 // hold old output on collision
      else
        rd_s1 <= mem[raddr];            // READ_FIRST (old memory contents)
    end
  end

  // optional output register
  generate
    if (OUTREG) begin : g_outreg
      logic [WIDTH-1:0] rd_s2;
      always_ff @(posedge clk) rd_s2 <= rd_s1;
      assign rdata = rd_s2;
    end else begin : g_flow
      assign rdata = rd_s1;
    end
  endgenerate

endmodule : rmc_sram
