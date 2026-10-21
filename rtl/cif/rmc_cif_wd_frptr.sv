// rmc_cif_wd_frptr.sv
// WD-SRAM free/write pointer tracker, ONE PER CHANNEL. Owns no SRAM port - it only hands
// out the pointer and the space flag. The write accumulator owns the actual WD-SRAM write
// port (we/addr/din/wstrb) and uses write_in_ptr as the address; the burst splitter uses the
// same write_in_ptr as the packet slot. alloc advances the write pointer once per issued
// line; at retire a whole run is freed (free_head + free_burst), advancing the free pointer.
// sram_ready = the ring is not full (there is room for at least one more line).
//
// The ring is tracked by a write pointer (wptr = write_in_ptr) and an occupancy counter
// (occ). alloc advances wptr by one and occ up by one; a retire frees a whole run and drops
// occ by free_burst (the tail advances by that much). Because admit is in order and a request
// frees its whole run at retire, the freed slots are always the oldest, so only the count
// matters - free_head is carried for an assertion, not used in the pointer math.

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

  localparam int OCC_W = $clog2(N_WDB + 1);         // 0..N_WDB

  logic [SLOT_W-1:0] wptr;                          // alloc pointer (= write_in_ptr)
  logic [OCC_W-1:0]  occ;                           // slots in use

  assign write_in_ptr = wptr;
  assign sram_ready   = (occ != OCC_W'(N_WDB));     // room for at least one more line

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      wptr <= '0;
      occ  <= '0;
    end else begin
      if (alloc)                                    // a line was issued -> take a slot
        wptr <= (wptr == SLOT_W'(N_WDB - 1)) ? '0 : wptr + 1'b1;
      // occupancy: +1 on alloc, -free_burst on a retire-time free (both may land together)
      occ <= occ + OCC_W'(alloc ? 1 : 0)
                 - (free_valid ? OCC_W'(free_burst) : '0);
    end
  end

`ifndef SYNTHESIS
  // the freed run is the oldest in the ring: its base must be the current tail
  // (wptr - occ, mod N_WDB), and it cannot release more than is in use.
  always @(posedge aclk) if (aresetn && free_valid) begin
    assert (OCC_W'(free_burst) <= occ)
      else $error("wd_frptr: free_burst %0d exceeds occupancy %0d", free_burst, occ);
    assert (free_head == SLOT_W'((32'(wptr) - 32'(occ)) % N_WDB))
      else $error("wd_frptr: free_head %0d is not the ring tail", free_head);
  end
`endif

endmodule : rmc_cif_wd_frptr
