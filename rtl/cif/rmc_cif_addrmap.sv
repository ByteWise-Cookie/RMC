// rmc_cif_addrmap.sv
// System byte address -> DRAM coords {rank, bg, bank, row, col}, plus ch (channel
// select, routes to the per-channel core) and offset (byte-in-64B-packet, for the
// write mask / narrow-access byte lanes). Pure field-slice, no hash (stage-7 lock).
//
// Interleave (STAGE 24, P_MAX packets/bank): consecutive packets rotate BGs
// (different-BG = tCCD_S, writes safe), then revisit the same bank's next column
// (row-hit train of P_MAX). BG and col are SPLIT fields (low part rotates, high
// part selects the set), assembled here. Bit positions come from rmc_cfg_pkg.
//
//   [ row | col_hi | bank | bg_hi | col_lo(P) | bg_lo | rank | ch | offset ]
//    MSB                                                              LSB
// daddr = {rank,bg,bank,row,col} (DRAM coords only); ch + offset are separate
// (ch is routing, offset is a byte lane - neither is a DRAM coordinate).

import rmc_cfg_pkg::*;

module rmc_cif_addrmap #(
  parameter int AXI_AW  = 48,        // >= 41 (STAGE-24 map: row ends at bit 40)
  parameter int DADDR_W = 1          // = rmc_cfg_pkg::DADDR_W (passed from top)
)(
  input  logic [AXI_AW-1:0]     sys_addr,
  output logic [DADDR_W-1:0]    daddr,
  output logic [CH_W-1:0]       ch,       // channel select (-> per-channel core)
  output logic [PKT_OFF_W-1:0]  offset    // byte-in-64B-packet (write mask)
);

  logic [RANK_W-1:0]        rank;
  logic [BG_W-1:0]          bg;      // {bg_hi, bg_lo}
  logic [BANK_PER_BG_W-1:0] bank;
  logic [ROW_W-1:0]         row;
  logic [COL_W-1:0]         col;     // {col_hi, col_lo}

  always_comb begin
    offset = sys_addr[OFF_POS   +: PKT_OFF_W];
    ch     = sys_addr[CH_POS    +: CH_W];
    rank   = sys_addr[RANK_POS  +: RANK_W];
    bg     = { sys_addr[BGHI_POS +: BG_HI_W],
               sys_addr[BGLO_POS +: BG_LO_W] };
    bank   = sys_addr[BANK_POS  +: BANK_PER_BG_W];
    col    = { sys_addr[COLHI_POS +: COL_HI_W],
               sys_addr[COLLO_POS +: COL_LO_W] };
    row    = sys_addr[ROW_POS   +: ROW_W];
  end

  // pack {rank, bg, bank, row, col} (matches rmc_cfg_pkg::DADDR_W order)
  assign daddr = { rank, bg, bank, row, col };

endmodule : rmc_cif_addrmap
