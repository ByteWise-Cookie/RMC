// rmc_cif_aw_reqbuf_fv.sv
// Formal checker for rmc_cif_aw_reqbuf. Bound by rmc_cif_aw_reqbuf_bind.sv. Scaffold:
// occupancy safety + flag correctness + pointer advance. In-order descriptor integrity is
// left as an extension.

import rmc_cfg_pkg::*;

module rmc_cif_aw_reqbuf_fv #(
  parameter int DEPTH = 32,
  parameter int ENT_W = 64,
  // DUT-derived widths (passed by the bind)
  parameter int PTR_W = 5,
  parameter int OCC_W = 6
)(
  input logic             aclk,
  input logic             aresetn,
  input logic             push,
  input logic             full,
  input logic             valid,
  input logic             pop,
  // internals
  input logic [PTR_W-1:0] wptr,
  input logic [PTR_W-1:0] rptr,
  input logic [OCC_W-1:0] occ
);
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  wire do_push = push & ~full;
  wire do_pop  = pop  & valid;

  as_occ_range: assert property (@(posedge aclk) disable iff (!aresetn)
    occ <= OCC_W'(DEPTH));
  as_full:  assert property (@(posedge aclk) disable iff (!aresetn)
    full == (occ == OCC_W'(DEPTH)));
  as_valid: assert property (@(posedge aclk) disable iff (!aresetn)
    valid == (occ != '0));
  as_wptr: assert property (@(posedge aclk) disable iff (!aresetn)
    !(en & $past(en) & $past(do_push)) || (wptr == (($past(wptr) == PTR_W'(DEPTH-1)) ? '0 : $past(wptr) + 1'b1)));
  as_rptr: assert property (@(posedge aclk) disable iff (!aresetn)
    !(en & $past(en) & $past(do_pop)) || (rptr == (($past(rptr) == PTR_W'(DEPTH-1)) ? '0 : $past(rptr) + 1'b1)));

  // ---- data order: one watched entry ----
  localparam int ENT_W_L = ENT_W;
  // fv_n is a constant the solver picks: the fv_n-th push is watched. It must come out
  // of the FIFO exactly when every entry ahead of it has left, unchanged.
  logic [15:0]      fv_n, push_cnt;
  logic             trk;                 // watched entry is inside the FIFO
  logic [PTR_W-1:0] t_slot;              // slot it was written to
  logic [OCC_W-1:0] t_pos;               // entries ahead of it
  logic [ENT_W_L-1:0] t_data;
  always_ff @(posedge aclk) fv_n <= fv_n;
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      push_cnt <= '0;
      trk      <= 1'b0;
    end else begin
      if (do_push) push_cnt <= push_cnt + 1'b1;
      if (trk && do_pop) begin
        if (t_pos == '0) trk <= 1'b0;
        else             t_pos <= t_pos - 1'b1;
      end
      if (do_push && push_cnt == fv_n && !trk) begin
        trk    <= 1'b1;
        t_slot <= wptr;
        t_pos  <= occ - OCC_W'(do_pop);
        t_data <= {rmc_cif_aw_reqbuf.in_id, rmc_cif_aw_reqbuf.in_addr, rmc_cif_aw_reqbuf.in_len, rmc_cif_aw_reqbuf.in_size, rmc_cif_aw_reqbuf.in_err};
      end
    end
  end
  as_order:  assert property (@(posedge aclk) disable iff (!aresetn)
    !(trk && do_pop && t_pos == '0) || {rmc_cif_aw_reqbuf.out_id, rmc_cif_aw_reqbuf.out_addr, rmc_cif_aw_reqbuf.out_len, rmc_cif_aw_reqbuf.out_size, rmc_cif_aw_reqbuf.out_err} == t_data);
  // helpers: the watched entry sits t_pos slots after the read pointer, untouched
  as_h_trk:  assert property (@(posedge aclk) disable iff (!aresetn)
    !trk || (t_pos < occ && rmc_cif_aw_reqbuf.mem[t_slot] == t_data
             && t_slot == PTR_W'((32'(rptr) + 32'(t_pos)) % DEPTH)));
  as_h_ptr:  assert property (@(posedge aclk) disable iff (!aresetn)
    32'(wptr) == (32'(rptr) + 32'(occ)) % DEPTH && 32'(rptr) < DEPTH && occ <= OCC_W'(DEPTH));
  c_full:    cover property (@(posedge aclk) disable iff (!aresetn) occ == OCC_W'(DEPTH));
  c_order:   cover property (@(posedge aclk) disable iff (!aresetn) trk && do_pop && t_pos == '0 && $past(occ) > 1);
  c_wrap:    cover property (@(posedge aclk) disable iff (!aresetn) do_pop && 32'(rptr) == DEPTH-1);
  c_pushpop: cover property (@(posedge aclk) disable iff (!aresetn) do_push && do_pop);

  // the proof starts in reset (without this the first state is unconstrained)
  logic fv_started = 1'b0;
  always_ff @(posedge aclk) fv_started <= 1'b1;
  m_reset_first: assume property (@(posedge aclk) fv_started || !aresetn);

endmodule : rmc_cif_aw_reqbuf_fv
