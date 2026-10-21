// rmc_cif_wd_frptr.sv
// WD-SRAM free/write pointer tracker, ONE PER CHANNEL. Owns no SRAM port - it only hands
// out the pointer and the space flag. The write accumulator owns the actual WD-SRAM write
// port (we/addr/din/wstrb) and uses write_in_ptr as the address; the burst splitter uses the
// same write_in_ptr as the packet slot. alloc advances the write pointer once per issued
// line; at retire a whole run is freed (free_head + free_burst), advancing the free pointer.
// sram_ready = the ring is not full (write pointer has not caught the free pointer).
//
// STUB: ports fixed; the ring (write ptr, free ptr, occupancy) is TODO.

import rmc_cfg_pkg::*;

module rmc_cif_wd_frptr #(
  parameter int N_WDB = 64,                              // slots in this channel's ring

  // derived (localparam: pkg-sourced so strict front-ends resolve them in the header)
  localparam int MAX_PKTS = rmc_cfg_pkg::MAX_REQ_PKTS,
  localparam int CNT_W    = (MAX_PKTS > 1) ? $clog2(MAX_PKTS + 1) : 1,
  localparam int SLOT_W   = (N_WDB    > 1) ? $clog2(N_WDB)        : 1
)(
  input  logic              aclk,
  input  logic              aresetn,

  // to the accumulator (write address) and the splitter (packet slot): one value, fanned out
  output logic [SLOT_W-1:0] write_in_ptr,   // next slot to write; advances on alloc
  output logic              sram_ready,      // ring has space (not full)

  // a line was issued on this channel -> write_in_ptr++
  input  logic              alloc,

  // free a whole run at retire (from u_wrob RESP_*, sliced to this channel)
  input  logic              free_valid,
  input  logic [SLOT_W-1:0] free_head,       // run base   (= RESP_HEAD_PTR[c])
  input  logic [CNT_W-1:0]  free_burst       // run length (= RESP_NPKTS_CH[c])
);

  // STUB body: safe defaults so it elaborates. TODO: real write/free pointer ring.
  assign write_in_ptr = '0;
  assign sram_ready   = 1'b1;

endmodule : rmc_cif_wd_frptr
