// rmc_async_fifo.sv
// Dual-clock (CDC) FIFO, Gray-code pointers + multi-FF synchronizers. The clean
// crossing for the CIF<->MC boundary. Fully parametric: width, depth, synchronizer
// depth, and optional (conservative) almost-full/almost-empty flags.
//
// DEPTH MUST be a power of two - Gray-code pointer wrap depends on it. The pointers
// are AW+1 bits (the extra MSB is what lets a pure pointer compare tell full from
// empty - no separate counter can cross the clock boundary). full/empty are
// REGISTERED (glitch-free, pessimistic by one cycle) - standard async-FIFO form.
// FWFT read: rd_data shows the head whenever ~rempty. Standalone, no pkg dep.

module rmc_async_fifo #(
  parameter int WIDTH       = 32,
  parameter int DEPTH       = 16,     // power of two
  parameter int SYNC_STAGES = 2,      // >=2 flop stages per crossing
  parameter bit GEN_ALMOST  = 1'b1,   // generate almost_full / almost_empty
  parameter int AF_THRESH   = 1,      // almost_full  when fill  >= DEPTH-AF_THRESH
  parameter int AE_THRESH   = 1,      // almost_empty when fill  <= AE_THRESH

  localparam int AW = (DEPTH > 1) ? $clog2(DEPTH) : 1,
  localparam int PW = AW + 1          // extra MSB to tell full from empty
)(
  // write domain
  input  logic             wclk,
  input  logic             wrst_n,
  input  logic             wr_en,
  input  logic [WIDTH-1:0] wr_data,
  output logic             wfull,
  output logic             walmost_full,

  // read domain
  input  logic             rclk,
  input  logic             rrst_n,
  input  logic             rd_en,
  output logic [WIDTH-1:0] rd_data,
  output logic             rd_valid,
  output logic             rempty,
  output logic             ralmost_empty
);

  // storage (written on wclk, read combinationally on rclk)
  logic [WIDTH-1:0] mem [DEPTH];

  // pointers: binary + Gray, PW bits
  logic [PW-1:0] wbin, wgray, wbin_nxt, wgray_nxt;
  logic [PW-1:0] rbin, rgray, rbin_nxt, rgray_nxt;
  logic          wfull_val, rempty_val;
  logic          winc, rinc;            // 1-bit advance enables (kept out of casts)

  // cross-domain synchronizers
  logic [PW-1:0] w_rgray [SYNC_STAGES];  // read Gray ptr, synced into wclk
  logic [PW-1:0] r_wgray [SYNC_STAGES];  // write Gray ptr, synced into rclk
  logic [PW-1:0] rgray_at_w, wgray_at_r;
  assign rgray_at_w = w_rgray[SYNC_STAGES-1];
  assign wgray_at_r = r_wgray[SYNC_STAGES-1];

  function automatic logic [PW-1:0] gray2bin(input logic [PW-1:0] g);
    logic [PW-1:0] b;
    b[PW-1] = g[PW-1];
    for (int i = PW-2; i >= 0; i--) b[i] = b[i+1] ^ g[i];
    return b;
  endfunction

  //write domain
  // increment uses the REGISTERED wfull (no comb loop through wfull_val)
  assign winc      = wr_en & ~wfull;
  assign wbin_nxt  = wbin + PW'(winc);
  assign wgray_nxt = wbin_nxt ^ (wbin_nxt >> 1);

  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
      wbin  <= '0;
      wgray <= '0;
    end else begin
      wbin  <= wbin_nxt;
      wgray <= wgray_nxt;
    end
  end

  always_ff @(posedge wclk)
    if (winc) mem[wbin[AW-1:0]] <= wr_data;

  // sync read Gray pointer into write domain
  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
      for (int i = 0; i < SYNC_STAGES; i++) w_rgray[i] <= '0;
    end else begin
      w_rgray[0] <= rgray;
      for (int i = 1; i < SYNC_STAGES; i++) w_rgray[i] <= w_rgray[i-1];
    end
  end

  // full: next write Gray == read Gray with the top two bits inverted
  assign wfull_val = (wgray_nxt ==
                      {~rgray_at_w[PW-1:PW-2], rgray_at_w[PW-3:0]});
  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) wfull <= 1'b0;
    else         wfull <= wfull_val;
  end

  //read domain
  assign rinc      = rd_en & ~rempty;
  assign rbin_nxt  = rbin + PW'(rinc);
  assign rgray_nxt = rbin_nxt ^ (rbin_nxt >> 1);

  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
      rbin  <= '0;
      rgray <= '0;
    end else begin
      rbin  <= rbin_nxt;
      rgray <= rgray_nxt;
    end
  end

  // sync write Gray pointer into read domain
  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
      for (int i = 0; i < SYNC_STAGES; i++) r_wgray[i] <= '0;
    end else begin
      r_wgray[0] <= wgray;
      for (int i = 1; i < SYNC_STAGES; i++) r_wgray[i] <= r_wgray[i-1];
    end
  end

  // empty: next read Gray == synced write Gray (resets asserted)
  assign rempty_val = (rgray_nxt == wgray_at_r);
  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) rempty <= 1'b1;
    else         rempty <= rempty_val;
  end

  assign rd_data  = mem[rbin[AW-1:0]];   // FWFT
  assign rd_valid = ~rempty;

  // optional almost flags (conservative)
  // write side sees a lagging read ptr -> underestimates free -> early almost_full
  // read side sees a lagging write ptr -> underestimates fill -> early almost_empty
  generate
    if (GEN_ALMOST) begin : g_almost
      logic [PW-1:0] rbin_at_w, wbin_at_r, occ_w, occ_r;
      assign rbin_at_w = gray2bin(rgray_at_w);
      assign wbin_at_r = gray2bin(wgray_at_r);
      assign occ_w = wbin - rbin_at_w;           // fill seen by write domain
      assign occ_r = wbin_at_r - rbin;           // fill seen by read domain
      assign walmost_full  = (occ_w >= PW'(DEPTH - AF_THRESH));
      assign ralmost_empty = (occ_r <= PW'(AE_THRESH));
    end else begin : g_no_almost
      assign walmost_full  = 1'b0;
      assign ralmost_empty = 1'b0;
    end
  endgenerate

endmodule : rmc_async_fifo
