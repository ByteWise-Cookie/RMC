// tut_cnt_fv.sv
// Checker for tut_cnt. FV_HELPER adds the invariant that makes induction pass.

module tut_cnt_fv (
  input logic       clk,
  input logic       rst_n,
  input logic       en,
  input logic [3:0] cnt,
  input logic       tick
);

  // assumption: the run starts in reset
  logic started = 1'b0;
  always_ff @(posedge clk) started <= 1'b1;
  m_reset_first: assume property (@(posedge clk) started || !rst_n);

  // the property we care about
  a_never15: assert property (@(posedge clk) disable iff (!rst_n) cnt != 4'd15);

  // next-cycle check: after an enabled 9 the counter is 0
  a_wrap: assert property (@(posedge clk) disable iff (!rst_n)
    !$past(rst_n) || !$past(en && cnt == 4'd9) || cnt == 4'd0);

`ifdef FV_HELPER
  // helper invariant: rules out the unreachable states 10..15
  a_range: assert property (@(posedge clk) disable iff (!rst_n) cnt <= 4'd9);
`endif

  // reachability
  c_tick: cover property (@(posedge clk) disable iff (!rst_n) tick);

endmodule : tut_cnt_fv

bind tut_cnt tut_cnt_fv u_fv (.*);
