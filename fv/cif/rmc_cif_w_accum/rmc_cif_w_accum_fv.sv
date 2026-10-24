// rmc_cif_w_accum_fv.sv
// Formal checker for rmc_cif_w_accum. Bound by rmc_cif_w_accum_bind.sv. Scaffold: staging
// handshake safety (one line held until taken), the wready gate, output wiring, and that
// drain_done only fires while dropping. A full mask-accumulation reference model (bytes land
// by wstrb, mask = OR of wstrb over the line) is left as an extension.

import rmc_cfg_pkg::*;

module rmc_cif_w_accum_fv #(
  parameter int OFF_W  = 6,
  parameter int DW     = 512,
  parameter int STRB_W = 64
)(
  input logic               aclk,
  input logic               aresetn,
  input logic               wvalid,
  input logic               wready,
  input logic               wlast,
  input logic               stage_en,
  input logic               skip,
  input logic               take,
  input logic               drain_done,
  input logic               stage_valid,
  input logic [DW-1:0]      write_data,
  input logic [STRB_W-1:0]  write_data_strobe,
  // internals
  input logic [DW-1:0]      line_data,
  input logic [STRB_W-1:0]  line_strobe,
  input logic               in_req,
  input logic               drop_q,
  input logic               staged
);
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  wire drop_cur = in_req ? drop_q : skip;

  // output wiring
  as_stage_valid: assert property (@(posedge aclk) disable iff (!aresetn) stage_valid == staged);
  as_wdata:  assert property (@(posedge aclk) disable iff (!aresetn) write_data == line_data);
  as_wstrb:  assert property (@(posedge aclk) disable iff (!aresetn) write_data_strobe == line_strobe);

  // wready gate: no accept outside an active request; a staged (non-drop) line blocks accept
  as_wready: assert property (@(posedge aclk) disable iff (!aresetn)
    wready == (stage_en & (drop_cur | ~staged)));

  // a staged line is held until pkt_gen takes it (never silently dropped)
  as_staged_hold: assert property (@(posedge aclk) disable iff (!aresetn)
    !(en & $past(en) & $past(staged) & ~$past(take)) || (staged));

  // drain_done only while draining a dropped burst, on its last beat
  as_drain_in_drop: assert property (@(posedge aclk) disable iff (!aresetn)
    !(drain_done) || ((drop_cur & wvalid & wready & wlast)));

  // never stage a line for a dropped request
  as_no_stage_on_drop: assert property (@(posedge aclk) disable iff (!aresetn)
    !((in_req & drop_q)) || (~staged));

  // helper: a staged line belongs to a live, kept (not dropped) request
  as_h_staged: assert property (@(posedge aclk) disable iff (!aresetn)
    !staged || (in_req && !drop_q));

  // AXI: a beat is never wider than the 64 B bus
  m_size_legal: assume property (@(posedge aclk) (1 << rmc_cif_w_accum.stage_size) <= (1 << OFF_W));

  // pkt_gen: take (= its commit) only fires on a staged line (commit needs stage_valid)
  m_take_staged: assume property (@(posedge aclk) !take || staged);

  // ---- reference: where each beat lands, and what a staged line must hold ----
  // In-line offset of beat b of a request = (align_down(stage_len, size) + b * 2^size) mod 64.
  // One byte lane j (a constant the solver picks) is followed through the line: the last
  // beat that strobed lane j since the line was cleared decides that byte.
  wire       wfire  = wvalid & wready;
  wire [2:0] size_l = rmc_cif_w_accum.size_cur;
  logic [OFF_W-1:0] r_start;               // aligned in-line start of the request
  logic [2:0]       r_size;
  logic [7:0]       r_b;                   // beats of this request accepted so far
  logic [5:0]       j;                     // watched byte lane
  logic             e_val;                 // lane j strobed since the line was cleared
  logic [7:0]       e_byte;                // its latest value
  always_ff @(posedge aclk) j <= j;
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      r_b   <= '0;
      e_val <= 1'b0;
    end else begin
      if (wfire) begin
        if (!in_req) begin
          r_start <= rmc_cif_w_accum.stage_len & OFF_W'(~((1 << rmc_cif_w_accum.stage_size) - 1));
          r_size  <= rmc_cif_w_accum.stage_size;
          r_b     <= 8'd1;
        end else r_b <= r_b + 8'd1;
        if (!drop_cur && rmc_cif_w_accum.wstrb[j]) begin
          e_val  <= 1'b1;
          e_byte <= rmc_cif_w_accum.wdata[j*8 +: 8];
        end
      end
      if (take) e_val <= 1'b0;
    end
  end
  wire [31:0] exp_off = (32'(r_start) + (32'(r_b) << r_size)) & ((1 << OFF_W) - 1);

  // the beat being accepted lands at the closed-form offset (first beat: aligned start)
  as_beat_off: assert property (@(posedge aclk) disable iff (!aresetn)
    !(wfire && in_req && !drop_q) || 32'(rmc_cif_w_accum.off_cur) == exp_off);
  as_first_off: assert property (@(posedge aclk) disable iff (!aresetn)
    !(wfire && !in_req) || rmc_cif_w_accum.off_cur
      == (rmc_cif_w_accum.stage_len & OFF_W'(~((1 << rmc_cif_w_accum.stage_size) - 1))));
  // a line is closed exactly when the next beat would leave it, or on wlast
  as_line_end: assert property (@(posedge aclk) disable iff (!aresetn)
    !(wfire && !drop_cur) || (rmc_cif_w_accum.line_done ==
      (wlast || (32'(rmc_cif_w_accum.off_cur) + (32'(1) << size_l) >= (1 << OFF_W)))));
  // the staged line holds every strobed byte of its beats, and only those
  as_lane: assert property (@(posedge aclk) disable iff (!aresetn)
    !staged || (line_strobe[j] == e_val && (!e_val || line_data[j*8 +: 8] == e_byte)));
  // helpers for the lane model: between lines the buffer is clear; mid-line it matches
  as_h_lane: assert property (@(posedge aclk) disable iff (!aresetn)
    line_strobe[j] == e_val && (!e_val || line_data[j*8 +: 8] == e_byte));
  as_h_req: assert property (@(posedge aclk) disable iff (!aresetn)
    !(in_req && !drop_q) || (rmc_cif_w_accum.size_q == r_size && (staged ||
      32'(rmc_cif_w_accum.off) == ((32'(r_start) + (32'(r_b) << r_size)) & ((1 << OFF_W) - 1)))));

  as_h_legal: assert property (@(posedge aclk) disable iff (!aresetn)
    !(in_req && !drop_q) || ((1 << r_size) <= (1 << OFF_W)
      && (r_start & OFF_W'((1 << r_size) - 1)) == '0));

  // helper: a staged line that is not the request's last ended exactly at a line boundary
  as_h_full_line: assert property (@(posedge aclk) disable iff (!aresetn)
    !(staged && !rmc_cif_w_accum.last_q) || exp_off == 0);

  // drain: a dropped request stages nothing and reports drain_done once, on its wlast
  as_drain_once: assert property (@(posedge aclk) disable iff (!aresetn)
    !drain_done || (drop_cur && wlast));

  // covers
  c_stage:   cover property (@(posedge aclk) disable iff (!aresetn) take);
  c_narrow:  cover property (@(posedge aclk) disable iff (!aresetn)
               take && r_size < 3'd6 && r_b >= 8'd3);       // several narrow beats in one line
  c_unalign: cover property (@(posedge aclk) disable iff (!aresetn)
               wfire && !in_req && rmc_cif_w_accum.stage_len != '0 && !skip);
  c_drain:   cover property (@(posedge aclk) disable iff (!aresetn) drain_done);
  c_two_req: cover property (@(posedge aclk) disable iff (!aresetn)
               take && rmc_cif_w_accum.last_q && $past(drain_done, 2));
  c_lane:    cover property (@(posedge aclk) disable iff (!aresetn) take && e_val);

  // the proof starts in reset (without this the first state is unconstrained)
  logic fv_started = 1'b0;
  always_ff @(posedge aclk) fv_started <= 1'b1;
  m_reset_first: assume property (@(posedge aclk) fv_started || !aresetn);

endmodule : rmc_cif_w_accum_fv
