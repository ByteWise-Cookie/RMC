// rmc_lookahead.sv
// Generic look-ahead / bank-stall bypass window. A DEPTH-entry age-ordered window
// (index 0 = oldest = head). Each entry carries an address key (adr) that probes a
// readiness vector (rdy_vec); the oldest ELIGIBLE entry (valid & rdy_vec[adr])
// issues, so a younger entry to a ready resource bypasses a stalled older one while
// same-resource order is preserved (they share the same rdy_vec bit).
//
// Payload storage is configurable:
//   PAYLOAD_IN_REG = 1  carry the payload in the window register (wide shift, no RAM)
//   PAYLOAD_IN_REG = 0  keep the payload in a comb-read meta store (rmc_sram with
//                       RD_LAT=0, WRITE_FIRST) indexed by a per-entry slot ptr; the
//                       window shifts only {adr, slot}. A free-list allocs/frees
//                       slots (out-of-order departure safe).
//
// Same-cycle arrive+issue: when nothing resident is eligible but din is, din issues
// live this cycle (bypass) and is not stored. Standalone except for rmc_sram.

module rmc_lookahead #(
  parameter int DEPTH          = 4,      // window entries
  parameter int N_RDY          = 32,     // readiness vector width (e.g. banks)
  // SKIP/ASK: "FIX: ISSUE PATHS" unclear - N_RDY is the readiness width, not an
  // issue-path count (this block has ONE issue port). Say what you want: more
  // than one issue/cycle, or a rename? Then I'll apply.
  parameter int PAYLOAD_W      = 40,     // wide payload
  parameter bit PAYLOAD_IN_REG = 1'b0,   // 1 = payload in window reg, 0 = payload in rmc_sram meta store

  localparam int ADR_W = (N_RDY > 1) ? $clog2(N_RDY) : 1,
  localparam int PTR_W = (DEPTH > 1) ? $clog2(DEPTH) : 1,
  localparam int SFW   = PAYLOAD_IN_REG ? PAYLOAD_W : PTR_W   // slot-field width
)(
  input  logic                 clk,
  input  logic                 rst_n,

  // ingress
  input  logic                 din_valid,
  output logic                 din_rdy_out,
  input  logic [ADR_W-1:0]     din_adr,
  input  logic [PAYLOAD_W-1:0] din_payload,

  // readiness (one bit per resource; adr indexes it)
  input  logic [N_RDY-1:0]     rdy_vec,

  // egress
  output logic                 out_valid,
  input  logic                 out_ready,
  output logic [ADR_W-1:0]     out_adr,
  output logic [PAYLOAD_W-1:0] out_payload
);

  // ---- window state (index 0 = oldest; valids kept compacted as a prefix) ----
  logic              ent_vld [DEPTH];
  logic [ADR_W-1:0]  ent_adr [DEPTH];
  logic [SFW-1:0]    ent_sf  [DEPTH];   // payload (reg mode) or slot ptr (store mode)

  // ---- occupancy ----
  logic [PTR_W:0] fill;
  always_comb begin
    fill = '0;
    for (int i = 0; i < DEPTH; i++)
      if (ent_vld[i]) fill = fill + 1'b1;
  end

  // ---- eligibility + oldest-eligible winner (lowest index) ----
  logic [DEPTH-1:0] elig;
  logic             en_res;
  logic [PTR_W-1:0] widx;
  always_comb begin
    for (int i = 0; i < DEPTH; i++)
      elig[i] = ent_vld[i] & rdy_vec[ent_adr[i]];
    en_res = 1'b0;
    widx   = '0;
    for (int i = DEPTH-1; i >= 0; i--)       // last write wins -> lowest index
      if (elig[i]) begin en_res = 1'b1; widx = PTR_W'(i); end
  end

  // ---- din bypass + issue/admit control ----
  logic din_elig, issue_din, do_issue_res, do_issue_din, do_admit, space;
  assign din_elig     = din_valid & rdy_vec[din_adr];
  assign issue_din    = ~en_res & din_elig;          // no resident ready -> din live
  assign out_valid    = en_res | issue_din;
  assign do_issue_res = en_res   & out_ready;
  assign do_issue_din = issue_din & out_ready;
  assign space        = (fill < (PTR_W+1)'(DEPTH)) | do_issue_res; // room (post-removal)
  assign do_admit     = din_valid & ~do_issue_din & space;
  assign din_rdy_out  = do_issue_din | space;

  // ---- meta store / free-list (store mode only) ----
  logic [SFW-1:0]    admit_sf;          // what gets written into the appended entry
  logic [PAYLOAD_W-1:0] rd_payload;     // resident payload for the winner

  generate
    if (PAYLOAD_IN_REG) begin : g_reg
      assign admit_sf   = din_payload;
      assign rd_payload = ent_sf[widx];       // payload carried in the reg
    end else begin : g_store
      logic [DEPTH-1:0]     mfree;            // 1 = slot free
      logic [PTR_W-1:0]     alloc_slot;
      logic [PTR_W-1:0]     rf_raddr;
      logic [PAYLOAD_W-1:0] rf_rdata;

      // lowest free slot
      always_comb begin
        alloc_slot = '0;
        for (int i = DEPTH-1; i >= 0; i--) if (mfree[i]) alloc_slot = PTR_W'(i);
      end
      assign admit_sf   = alloc_slot;
      assign rf_raddr   = en_res ? ent_sf[widx] : '0;
      assign rd_payload = rf_rdata;

      // meta store = rmc_sram in combinational-read mode (RD_LAT=0 = 0th clk),
      // write-first so a same-slot wr+rd forwards the live payload.
      rmc_sram #(
        .WIDTH   (PAYLOAD_W),
        .DEPTH   (DEPTH),
        .RD_LAT  (0),
        .COLLIDE ("WRITE_FIRST")
      ) u_meta (
        .clk   (clk),
        .we    (do_admit),
        .waddr (alloc_slot),
        .wdata (din_payload),
        .re    (en_res),
        .raddr (rf_raddr),
        .rdata (rf_rdata)
      );

      // free-list: free the issued slot, claim the allocated one
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          mfree <= '1;
        end else begin
          if (do_issue_res) mfree[ent_sf[widx]] <= 1'b1;
          if (do_admit)     mfree[alloc_slot]   <= 1'b0;
        end
      end
    end
  endgenerate

  // ---- outputs ----
  assign out_adr     = en_res ? ent_adr[widx] : din_adr;
  assign out_payload = en_res ? rd_payload    : din_payload;  // din bypass = live

  // ---- next window: compact out the winner, append din ----
  logic             nxt_vld [DEPTH];
  logic [ADR_W-1:0] nxt_adr [DEPTH];
  logic [SFW-1:0]   nxt_sf  [DEPTH];
  always_comb begin
    int j;
    for (int i = 0; i < DEPTH; i++) begin
      nxt_vld[i] = 1'b0;
      nxt_adr[i] = '0;
      nxt_sf[i]  = '0;
    end
    j = 0;
    for (int i = 0; i < DEPTH; i++) begin
      if (ent_vld[i] && !(do_issue_res && (PTR_W'(i) == widx))) begin
        nxt_vld[j] = 1'b1;
        nxt_adr[j] = ent_adr[i];
        nxt_sf[j]  = ent_sf[i];
        j = j + 1;
      end
    end
    if (do_admit) begin
      nxt_vld[j] = 1'b1;
      nxt_adr[j] = din_adr;
      nxt_sf[j]  = admit_sf;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < DEPTH; i++) begin
        ent_vld[i] <= 1'b0;
        ent_adr[i] <= '0;
        ent_sf[i]  <= '0;
      end
    end else begin
      for (int i = 0; i < DEPTH; i++) begin
        ent_vld[i] <= nxt_vld[i];
        ent_adr[i] <= nxt_adr[i];
        ent_sf[i]  <= nxt_sf[i];
      end
    end
  end

endmodule : rmc_lookahead
