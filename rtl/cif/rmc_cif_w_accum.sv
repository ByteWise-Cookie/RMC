// rmc_cif_w_accum.sv
// Write accumulator. Consumes the AXI W data stream and stages it into one 64 B line
// {data, strobe}, held until pkt_gen commits (take). On a dropped request (skip) it drains
// the burst's beats without staging and pulses drain_done. Does NOT own the channel (pkt_gen's
// wr_en) or the address (frptr's selected_ptr); drives only the shared WD-SRAM data + mask bus.
//
// Each W beat merges into the line buffer at its in-line byte offset; the MASK is the
// accumulated wstrb over the 64 B line, NOT derived from the beat size. Size only paces the
// offset walk (where a line ends); wstrb alone says which bytes are valid, so the DRAM write
// is byte-masked (no read-modify-write).
//   off  = (stage_len + beat*2^size)[5:0]       byte position in the 64 B line
//   for each byte b with wstrb[b]: data[off..][b] <= wdata[b] ; strobe |= wstrb
// AXI_DW is 64 B so each beat's wstrb is already line-wide (a straight merge/OR). A line is
// done when the offset walk steps out of it (off wraps past 64 B) or on wlast; it is then
// staged and held until pkt_gen takes it, which clears the buffer for the next line.
//
// skip (front-end error): accept every beat to drain the burst off the shared W channel, merge
// and stage nothing, and pulse drain_done at wlast.

import rmc_cfg_pkg::*;

module rmc_cif_w_accum #(
  // derived (localparam: pkg-sourced so strict front-ends resolve them in the header)
  localparam int OFF_W  = rmc_cfg_pkg::PKT_OFF_W,     // byte-in-line bits (64 B -> 6)
  localparam int DW     = rmc_cfg_pkg::AXI_DW,        // 512 b (one 64 B line)
  localparam int STRB_W = DW / 8                      // 64
)(
  input  logic                aclk,
  input  logic                aresetn,

  // AXI W data stream (from w_data_fifo / data_buffer): raw beats only, no control
  input  logic                wvalid,
  output logic                wready,
  input  logic [DW-1:0]       wdata,
  input  logic [STRB_W-1:0]   wstrb,
  input  logic                wlast,

  // request shape from the burst splitter (held for this request's W stream)
  input  logic                stage_en,           // request active
  input  logic                skip,               // drop/drain this burst (front-end error)
  input  logic [OFF_W-1:0]    stage_len,          // in-line start byte offset
  input  logic [2:0]          stage_size,         // beat size (log2 bytes)
  output logic                drain_done,          // pulse: dropped burst fully drained

  // stage handshake to packet_formation_unit
  output logic                stage_valid,         // a 64 B line is staged
  input  logic                take,                // committed this cycle -> clear the line

  // shared WD-SRAM write bus (we + addr driven elsewhere: pkt_gen + frptr)
  output logic [DW-1:0]       write_data,
  output logic [STRB_W-1:0]   write_data_strobe
);

  // line buffer: one 64 B line plus its accumulated byte mask and the in-line byte offset
  // (used only to find where a line ends).
  logic [DW-1:0]     line_data;
  logic [STRB_W-1:0] line_strobe;
  logic [OFF_W-1:0]  off;                 // offset of lines after the first (first comes from shape)
  logic [2:0]        size_q;
  logic              drop_q;              // this request's beats are being discarded
  logic              in_req;              // mid-request: shape (off/size/drop) latched
  logic              staged;              // a completed line is waiting for pkt_gen to take
  logic              last_q;              // the staged line is the request's last (wlast)

  // first beat of a request takes offset/size/drop from the shape ports; later beats use the
  // latched values. The start offset is aligned down to the beat size.
  wire [2:0]       size_cur = in_req ? size_q : stage_size;
  wire             drop_cur = in_req ? drop_q : skip;
  wire [OFF_W-1:0] off_cur  = in_req ? off
                                     : (stage_len & OFF_W'(~((1 << stage_size) - 1)));
  wire [OFF_W:0]   off_nxt  = {1'b0, off_cur} + (OFF_W+1)'(1 << size_cur);
  wire             off_wrap = off_nxt[OFF_W];        // step crosses out of the 64 B line

  // in drop, accept every beat to drain fast; else accept only while no staged line waits
  assign wready     = stage_en & (drop_cur | ~staged);
  wire   beat_fire  = wvalid & wready;
  wire   line_done  = beat_fire & (off_wrap | wlast);
  assign drain_done = beat_fire & drop_cur & wlast;

  assign stage_valid       = staged;
  assign write_data        = line_data;
  assign write_data_strobe = line_strobe;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      line_data   <= '0;
      line_strobe <= '0;
      off         <= '0;
      size_q      <= '0;
      drop_q      <= 1'b0;
      in_req      <= 1'b0;
      staged      <= 1'b0;
      last_q      <= 1'b0;
    end else begin
      if (beat_fire) begin
        if (~in_req) begin                            // first beat: latch the request's shape
          in_req <= 1'b1;
          size_q <= stage_size;
          drop_q <= skip;
        end
        if (drop_cur) begin                           // discard: merge/stage nothing
          if (wlast) in_req <= 1'b0;                  // request drained, ready for the next
        end else begin
          for (int b = 0; b < STRB_W; b++)            // merge bytes by their own wstrb lanes
            if (wstrb[b]) line_data[b*8 +: 8] <= wdata[b*8 +: 8];
          line_strobe <= line_strobe | wstrb;
          if (line_done) begin
            staged <= 1'b1;                            // hold the completed line for pkt_gen
            last_q <= wlast;
          end else begin
            off <= off_nxt[OFF_W-1:0];                 // keep filling this line
          end
        end
      end

      // pkt_gen took the staged line: clear the buffer (aligned at 0); request ends on its last
      if (take) begin
        line_data   <= '0;
        line_strobe <= '0;
        off         <= '0;
        staged      <= 1'b0;
        if (last_q) in_req <= 1'b0;
      end
    end
  end

endmodule : rmc_cif_w_accum
