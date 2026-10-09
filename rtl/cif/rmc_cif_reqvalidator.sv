// rmc_cif_reqvalidator.sv
// Pre-ROB request validator. Each incoming AXI request (AW or AR) is checked for a
// decodable / in-range address and tagged ok(1)/err(0) BEFORE it enters the ROB.
// Validation is ADDRESS-ONLY here; R/W ordering hazards are handled in the MC core,
// not in the CIF.
//
// An err-tagged request is NOT dropped: it rides through with its err bit so the
// ROB still admits it, its packets skip CAS at the MC, and it retires with an error
// response (SLVERR) and frees its slot. One validator instance per channel (aw/ar).
//
// STUB: in-range check done; size / burst-end checks are TODO.

module rmc_cif_reqvalidator import rmc_cfg_pkg::*; #(
  parameter int AXI_AW = SYS_ADDR_W    // sys_addr width (== CIF AXI_AW)
)(
  input  logic [AXI_AW-1:0] req_addr,
  input  logic [7:0]        req_len,   // AXI awlen/arlen (beats-1)  (burst-end check TODO)
  input  logic [2:0]        req_size,  // AXI awsize/arsize          (burst-end check TODO)
  output logic              req_ok     // 1 = address in range / decodable, 0 = error
);

  // In range iff no address bit at or above MAP_ADDR_W is set: every address in
  // [0, 2^MAP_ADDR_W) maps to a DRAM coord; anything above aliases -> error.
  if (MAP_ADDR_W >= AXI_AW) assign req_ok = 1'b1;                   // whole bus mapped
  else                      assign req_ok = (req_addr[AXI_AW-1:MAP_ADDR_W] == '0);

  // TODO: also reject an illegal size (beat wider than the subchannel) and a burst
  //       whose last byte req_addr + ((req_len+1) << req_size) - 1 leaves the range.

endmodule : rmc_cif_reqvalidator
