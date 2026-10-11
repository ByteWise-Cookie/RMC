// tut_cnt.sv
// Tutorial DUT for the fv course: decade counter with enable. Not part of RMC.

module tut_cnt (
  input  logic       clk,
  input  logic       rst_n,
  input  logic       en,
  output logic [3:0] cnt,    // 0..9
  output logic       tick    // pulses on the 9 -> 0 step
);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)  cnt <= '0;
    else if (en) cnt <= (cnt == 4'd9) ? '0 : cnt + 1'b1;
  end

  assign tick = en & (cnt == 4'd9);

endmodule : tut_cnt
