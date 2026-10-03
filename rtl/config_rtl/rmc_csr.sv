// rmc_csr.sv
// Runtime configuration / status registers for RMC, accessed over APB.
// Build-time structure lives in rmc_cfg_pkg.sv, not here.
// STUB: ports + APB handshake skeleton. Register map TODO.

import rmc_cfg_pkg::*;

module rmc_csr #(
  parameter int APB_AW = 12,
  parameter int APB_DW = 32
)(
  input  logic              pclk,
  input  logic              presetn,

  // APB target
  input  logic              psel,
  input  logic              penable,
  input  logic              pwrite,
  input  logic [APB_AW-1:0] paddr,
  input  logic [APB_DW-1:0] pwdata,
  output logic [APB_DW-1:0] prdata,
  output logic              pready,
  output logic              pslverr

  // TODO: config outputs to scheduler (timing regs, enables, MR shadows)
  // TODO: status inputs (init_done, error flags)
);

  // APB handshake (zero-wait stub)
  assign pready  = 1'b1;
  assign pslverr = 1'b0;

  // TODO: register file + address decode
  //   write: (psel & penable & pwrite)
  //   read : (psel & ~pwrite) -> prdata
  always_comb prdata = '0;   // TODO

endmodule : rmc_csr
