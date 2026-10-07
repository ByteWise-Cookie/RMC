// rmc_cif_haz.sv
// Same-line R/W hazard router for the two ROBs (r_rob, w_rob). CIF owns RAW + WAR
// (STAGE 14); WAW can't occur (AXI write-data ordering). Each ROB owns an address
// TCAM; a new request probes the OPPOSITE ROB and reports a hit for one cycle.
//
//   RAW: a new READ probes w_rob  -> raw_hit  (hold the new read)
//   WAR: a new WRITE probes r_rob -> war_hit  (hold the new write)
//
// IMPORTANT: raw_hit / war_hit are ONE-CYCLE pulses (the search is per-admit; the
// hit deasserts so the next request can probe). So the stall is NOT combinational -
// it is LATCHED here: SET on the hit, HELD, and RELEASED when the conflicting older
// entry (in the opposite ROB) retires and frees its slot. Each stalled entry records
// the opposite-ROB slot it waits on; a matching free clears it.

module rmc_cif_haz #(
  parameter int ROB_DEPTH = 32,
  localparam int PTR_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH) : 1
)(
  input  logic                 clk,
  input  logic                 rst_n,

  // RAW: new read hit an older write (1-cycle pulse)
  input  logic                 raw_hit,
  input  logic [PTR_W-1:0]     raw_rd_ptr,   // the new READ's own r_rob slot  -> hold it
  input  logic [PTR_W-1:0]     raw_wr_ptr,   // the matched older WRITE's w_rob slot -> wait on

  // WAR: new write hit an older read (1-cycle pulse)
  input  logic                 war_hit,
  input  logic [PTR_W-1:0]     war_wr_ptr,   // the new WRITE's own w_rob slot -> hold it
  input  logic [PTR_W-1:0]     war_rd_ptr,   // the matched older READ's r_rob slot -> wait on

  // release: the waited-on entry retired (frees its slot)
  input  logic                 wr_free_vld,
  input  logic [PTR_W-1:0]     wr_free_ptr,  // a WRITE slot retired -> free reads waiting on it
  input  logic                 rd_free_vld,
  input  logic [PTR_W-1:0]     rd_free_ptr,  // a READ slot retired  -> free writes waiting on it

  // held stall vectors (one bit per ROB entry)
  output logic [ROB_DEPTH-1:0] r_stall_vector,  // -> r_rob: these reads are held
  output logic [ROB_DEPTH-1:0] w_stall_vector   // -> w_rob: these writes are held
);

  // per stalled entry: which opposite-ROB slot it is waiting on
  logic [PTR_W-1:0] r_wait [ROB_DEPTH];   // read  i waits on write slot r_wait[i]
  logic [PTR_W-1:0] w_wait [ROB_DEPTH];   // write i waits on read  slot w_wait[i]

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      r_stall_vector <= '0;
      w_stall_vector <= '0;
    end else begin
      // reads held by RAW: set on hit, clear when the waited write frees
      for (int i = 0; i < ROB_DEPTH; i++) begin
        automatic logic set_i = raw_hit & (raw_rd_ptr == PTR_W'(i));
        automatic logic clr_i = r_stall_vector[i] & wr_free_vld & (r_wait[i] == wr_free_ptr);
        // a just-set read whose waited write frees this same cycle: never stall
        if (set_i & wr_free_vld & (raw_wr_ptr == wr_free_ptr)) clr_i = 1'b1;
        if      (clr_i) r_stall_vector[i] <= 1'b0;   // release wins
        else if (set_i) r_stall_vector[i] <= 1'b1;
        if      (set_i) r_wait[i]         <= raw_wr_ptr;
      end
      // writes held by WAR: set on hit, clear when the waited read frees
      for (int i = 0; i < ROB_DEPTH; i++) begin
        automatic logic set_i = war_hit & (war_wr_ptr == PTR_W'(i));
        automatic logic clr_i = w_stall_vector[i] & rd_free_vld & (w_wait[i] == rd_free_ptr);
        if (set_i & rd_free_vld & (war_rd_ptr == rd_free_ptr)) clr_i = 1'b1;
        if      (clr_i) w_stall_vector[i] <= 1'b0;
        else if (set_i) w_stall_vector[i] <= 1'b1;
        if      (set_i) w_wait[i]         <= war_rd_ptr;
      end
    end
  end

endmodule : rmc_cif_haz
