// rmc_fifo.sv
// Generic synchronous FIFO. Single clock. Fully parametric: data width, depth
// (any value >= 2, not just powers of two), first-word-fall-through vs standard
// read, almost-full/empty thresholds, and optional overflow/underflow guards.
// No package dependency - a standalone library block.

module rmc_fifo #(
  parameter int  WIDTH      = 32,   // data width
  parameter int  DEPTH      = 16,   // entries (>= 2)
  parameter bit  FWFT       = 1'b1, // 1 = first-word-fall-through, 0 = standard
  parameter int  AF_THRESH  = 1,    // almost_full  when free slots <= AF_THRESH
  parameter int  AE_THRESH  = 1,    // almost_empty when count     <= AE_THRESH
  parameter bit  PROT       = 1'b1, // 1 = drop writes when full / reads when empty

  localparam int AW    = (DEPTH > 1) ? $clog2(DEPTH)   : 1,
  localparam int CNT_W = $clog2(DEPTH + 1)
)(
  input  logic             clk,
  input  logic             rst_n,

  input  logic             wr_en,
  input  logic [WIDTH-1:0] wr_data,
  input  logic             rd_en,
  output logic [WIDTH-1:0] rd_data,
  output logic             rd_valid,   // FWFT: ~empty; standard: pop happened last cycle

  output logic             full,
  output logic             empty,
  output logic             almost_full,
  output logic             almost_empty,
  output logic [CNT_W-1:0] count,
  output logic             overflow,   // sticky: write attempted while full  (PROT)
  output logic             underflow   // sticky: read  attempted while empty (PROT)
);

  logic [WIDTH-1:0] mem [DEPTH];
  logic [AW-1:0]    wr_ptr, rd_ptr;
  logic [CNT_W-1:0] cnt;

  logic do_wr, do_rd;
  assign do_wr = wr_en & (~full  | ~PROT);
  assign do_rd = rd_en & (~empty | ~PROT);

  assign full         = (cnt == CNT_W'(DEPTH));
  assign empty        = (cnt == '0);
  assign almost_full  = (cnt >= CNT_W'(DEPTH - AF_THRESH));
  assign almost_empty = (cnt <= CNT_W'(AE_THRESH));
  assign count        = cnt;

  // write / read pointers (wrap at DEPTH-1 so DEPTH need not be a power of two)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr <= '0;
      rd_ptr <= '0;
      cnt    <= '0;
    end else begin
      if (do_wr) begin
        mem[wr_ptr] <= wr_data;
        wr_ptr      <= (wr_ptr == AW'(DEPTH-1)) ? '0 : wr_ptr + 1'b1;
      end
      if (do_rd)
        rd_ptr <= (rd_ptr == AW'(DEPTH-1)) ? '0 : rd_ptr + 1'b1;

      case ({do_wr, do_rd})
        2'b10:   cnt <= cnt + 1'b1;
        2'b01:   cnt <= cnt - 1'b1;
        default: cnt <= cnt;        // both or neither: unchanged
      endcase
    end
  end

  // read data path
  generate
    if (FWFT) begin : g_fwft
      assign rd_data  = mem[rd_ptr];   // head always visible
      assign rd_valid = ~empty;
    end else begin : g_std
      logic [WIDTH-1:0] rd_data_q;
      logic             rd_valid_q;
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          rd_data_q  <= '0;
          rd_valid_q <= 1'b0;
        end else begin
          if (do_rd) rd_data_q <= mem[rd_ptr];
          rd_valid_q <= do_rd;
        end
      end
      assign rd_data  = rd_data_q;
      assign rd_valid = rd_valid_q;
    end
  endgenerate

  // sticky overflow / underflow (only meaningful with PROT)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      overflow  <= 1'b0;
      underflow <= 1'b0;
    end else begin
      if (wr_en & full)  overflow  <= 1'b1;
      if (rd_en & empty) underflow <= 1'b1;
    end
  end

endmodule : rmc_fifo
