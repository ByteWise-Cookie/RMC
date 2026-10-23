// rmc_cif_frptr.sv
// Per-channel SRAM slot ring (free/write pointer tracker). Owns no SRAM port - it only hands
// out the slot pointer and the space flag. Used on BOTH paths, one instance per channel:
//   write (WD-SRAM): write_in_ptr = the slot the accumulator's line is written to and the write
//     packet carries; alloc = w_pkt_gen committed a line; free = w_rob retire.
//   read  (RD-SRAM): write_in_ptr = the slot the MC writes returned data into and the read
//     packet carries; alloc = a read packet was issued; free = r_resp retire.
// In both, the value is "where data gets written into the SRAM next", hence write_in_ptr.
//
// Sizing: sram_ready is per-line (room for >= 1). Nothing pre-reserves a whole request's run,
// so the oldest unretired request must be able to fit its per-channel run in the ring, i.e.
// N_SLOTS >= max per-channel lines in one request - else it can stall with nothing older to
// free. (= OPEN item 5.)
//
// The ring is a write pointer (wptr = write_in_ptr) and an occupancy counter (occ). alloc
// advances wptr by one and occ up by one; a retire frees a whole run and drops occ by
// free_burst. Admit/retire are in order, so a request's run is always the ring tail - only
// the count is needed to free it, not the run base. sram_ready = not full.

import rmc_cfg_pkg::*;

module rmc_cif_frptr #(
  parameter int N_SLOTS = 64,                              // slots in this channel's ring

  // derived (localparam: pkg-sourced so strict front-ends resolve them in the header)
  localparam int MAX_PKTS = rmc_cfg_pkg::MAX_REQ_PKTS,
  localparam int CNT_W    = (MAX_PKTS > 1) ? $clog2(MAX_PKTS + 1) : 1,
  localparam int SLOT_W   = (N_SLOTS    > 1) ? $clog2(N_SLOTS)        : 1
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
  input  logic [CNT_W-1:0]  free_burst       // run length (= RESP_NPKTS_CH[c])
);

  localparam int OCC_W = $clog2(N_SLOTS + 1);         // 0..N_SLOTS

  logic [SLOT_W-1:0] wptr;                          // alloc pointer (= write_in_ptr)
  logic [OCC_W-1:0]  occ;                           // slots in use

  assign write_in_ptr = wptr;
  assign sram_ready   = (occ != OCC_W'(N_SLOTS));     // room for at least one more line

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      wptr <= '0;
      occ  <= '0;
    end else begin
      if (alloc)                                    // a line was issued -> take a slot
        wptr <= (wptr == SLOT_W'(N_SLOTS - 1)) ? '0 : wptr + 1'b1;
      // occupancy: +1 on alloc, -free_burst on a retire-time free (both may land together)
      occ <= occ + OCC_W'(alloc ? 1 : 0)
                 - (free_valid ? OCC_W'(free_burst) : '0);
    end
  end

endmodule : rmc_cif_frptr
