// rmc_cif_w_burst_spltr_fv.sv
// Formal checker for rmc_cif_w_burst_spltr. Bound by rmc_cif_w_burst_spltr_bind.sv. Scaffold:
// capture gating (one request at a time), the descriptor + accumulator drives, and the single
// ROB entry-write condition. A full per-line walk reference model (line count = address span,
// per-channel counts) is left as an extension.

import rmc_cfg_pkg::*;

module rmc_cif_w_burst_spltr_fv #(
  parameter int ROB_DEPTH = 32,
  parameter int MAX_PKTS  = 64,
  parameter int AXI_AW    = 48,
  parameter int AXI_IDW   = 8,
  parameter int ROB_IDX_W = 5,
  parameter int CNT_W     = 7,
  parameter int LINE_W    = 6
)(
  input logic                   aclk,
  input logic                   aresetn,
  input logic                   aw_request_valid,
  input logic                   burst_splitter_ready,
  input logic [AXI_IDW-1:0]     aw_id,
  input logic [1:0]             error,
  input logic [ROB_IDX_W-1:0]   rob_idx_in,
  input logic                   rob_free,
  input logic                   admit_valid,
  input logic [AXI_IDW-1:0]     admit_id,
  input logic [1:0]             admit_resp,
  input logic                   seg_last,
  input logic [ROB_IDX_W-1:0]   seg_rob_idx,
  input logic [N_CH-1:0][CNT_W-1:0]  seg_npkts_ch,
  input logic                   stage_en,
  input logic                   skip,
  input logic [PKT_OFF_W-1:0]   stage_len,
  input logic [2:0]             stage_size,
  input logic                   drain_done,
  input logic                   ld_valid,
  input logic                   ld_ready,
  input logic [ROB_IDX_W-1:0]   ld_rob_idx,
  // internals
  input logic                   run,
  input logic                   drop_q,
  input logic [AXI_AW-1:0]      s_addr,
  input logic [2:0]             s_size,
  input logic [ROB_IDX_W-1:0]   s_rob,
  input logic [CNT_W-1:0]       n_left
);
  logic past_rst;
  always_ff @(posedge aclk or negedge aresetn)
    if (!aresetn) past_rst <= 1'b0; else past_rst <= 1'b1;
  wire en = aresetn & past_rst;

  wire capture = burst_splitter_ready & aw_request_valid;
  wire is_last = (n_left == CNT_W'(1));
  wire fire    = ld_valid & ld_ready;

  // capture gating: idle + ROB slot free; one request at a time (no ready while running)
  as_ready: assert property (@(posedge aclk) disable iff (!aresetn)
    burst_splitter_ready == (~run & rob_free));

  // line descriptor + accumulator control
  as_ld_valid:  assert property (@(posedge aclk) disable iff (!aresetn) ld_valid == (run & ~drop_q));
  as_ld_rob:    assert property (@(posedge aclk) disable iff (!aresetn) ld_rob_idx == s_rob);
  as_stage_en:  assert property (@(posedge aclk) disable iff (!aresetn) stage_en == run);
  as_skip:      assert property (@(posedge aclk) disable iff (!aresetn) skip == drop_q);
  as_stage_len: assert property (@(posedge aclk) disable iff (!aresetn) stage_len == s_addr[PKT_OFF_W-1:0]);
  as_stage_sz:  assert property (@(posedge aclk) disable iff (!aresetn) stage_size == s_size);

  // ADMIT at capture: reserve the offered slot with the request's id + verdict
  as_admit:     assert property (@(posedge aclk) disable iff (!aresetn) admit_valid == capture);
  as_admit_id:  assert property (@(posedge aclk) disable iff (!aresetn) admit_id   == aw_id);
  as_admit_rsp: assert property (@(posedge aclk) disable iff (!aresetn) admit_resp == error);
  // SEAL: last consumed line (good) or drain_done (dropped)
  as_seg:       assert property (@(posedge aclk) disable iff (!aresetn)
    seg_last == ((fire & is_last) | drain_done));
  as_seg_rob:   assert property (@(posedge aclk) disable iff (!aresetn) seg_rob_idx == s_rob);

  // capture latches the request and starts the run
  as_capture_run: assert property (@(posedge aclk) disable iff (!aresetn)
    !(en & $past(en) & $past(capture)) || (run & (drop_q == $past(error != 2'b00))
      & (s_rob == $past(rob_idx_in))));

  // ------------------------------------------------------------------------
  // reference walk: a captured request must produce exactly its lines, in address order,
  // each routed and addressed by the map, then ONE ROB entry with the right counts.
  // ------------------------------------------------------------------------
  localparam int NCH   = rmc_cfg_pkg::N_CH;
  localparam int CHW   = rmc_cfg_pkg::CH_W;
  localparam int PG    = rmc_cfg_pkg::PAGE_LSB;
  localparam int OFW   = rmc_cfg_pkg::PKT_OFF_W;
  localparam int DW_D  = rmc_cfg_pkg::DADDR_W;
  localparam int W     = AXI_AW + 1;
  wire [rmc_cfg_pkg::DEC_W-1:0][rmc_cfg_pkg::MAP_SEL_W-1:0] amap = rmc_cif_w_burst_spltr.addr_map;
  wire [AXI_AW-1:0] phy_adr = rmc_cif_w_burst_spltr.phy_adr;
  wire [7:0]        aw_len  = rmc_cif_w_burst_spltr.aw_len;
  wire [2:0]        aw_size = rmc_cif_w_burst_spltr.aw_size;

  function automatic logic [rmc_cfg_pkg::DEC_W-1:0] exp_dec(input logic [AXI_AW-1:0] a);
    for (int b = 0; b < rmc_cfg_pkg::DEC_W; b++) exp_dec[b] = a[amap[b]];
  endfunction
  // lines from the first byte to the last byte of an AXI INCR burst
  function automatic logic [W-1:0] exp_lines(input logic [AXI_AW-1:0] a, input logic [7:0] l,
                                             input logic [2:0] z);
    logic [W-1:0] st, la;
    st = (W'(a) >> z) << z;
    la = st + ((W'(l) + 1'b1) << z) - 1'b1;
    return (la >> OFW) - (W'(a) >> OFW) + 1'b1;
  endfunction

  logic fv_go = 1'b0;
  always_ff @(posedge aclk) fv_go <= 1'b1;
  // map: static CSR, every select a real address bit, CH bits at or above the line offset
  m_map_stable: assume property (@(posedge aclk) !fv_go || $stable(amap));
  for (genvar b = 0; b < rmc_cfg_pkg::DEC_W; b++) begin : g_map
    m_map_range: assume property (@(posedge aclk) 32'(amap[b]) < AXI_AW);
  end
  for (genvar k = 0; k < CHW; k++) begin : g_chlo
    m_map_ch: assume property (@(posedge aclk) 32'(amap[OFW + k]) >= OFW);
  end
  // request: inside its 4 KB page, legal beat size, fits MAX_PKTS
  wire cap = burst_splitter_ready & aw_request_valid;
  wire [W-1:0] cap_n = exp_lines(phy_adr, aw_len, aw_size);
  m_req: assume property (@(posedge aclk) !cap || ((1 << aw_size) <= (1 << OFW)
           && ((((W'(phy_adr) >> aw_size) << aw_size) + ((W'(aw_len) + 1'b1) << aw_size) - 1'b1) >> PG)
              == (W'(phy_adr) >> PG)
           && cap_n <= W'(MAX_PKTS)));
  // accumulator: drain_done only for the running dropped request (its wready needs stage_en)
  m_drain: assume property (@(posedge aclk) !drain_done || (run && drop_q));

  // model
  logic                 r_on, r_drop;
  logic [AXI_AW-1:0]    r_addr;
  logic [W-1:0]         r_n;          // lines of the request
  logic [W-1:0]         r_k;          // lines handed to pkt_gen so far
  logic [ROB_IDX_W-1:0] r_rob;
  logic [AXI_IDW-1:0]   r_id;
  logic [1:0]           r_err;
  logic [NCH-1:0][CNT_W-1:0] r_cnt;
  wire  lfire = ld_valid & ld_ready;
  wire [CHW-1:0] f_ch = rmc_cif_w_burst_spltr.ld_ch;
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) r_on <= 1'b0;
    else begin
      if (seg_last) r_on <= 1'b0;
      if (lfire) begin
        r_k <= r_k + 1'b1;
        r_cnt[f_ch] <= r_cnt[f_ch] + 1'b1;
      end
      if (cap) begin
        r_on   <= 1'b1;
        r_drop <= (error != 2'b00);
        r_addr <= phy_adr;
        r_n    <= cap_n;
        r_k    <= '0;
        r_rob  <= rob_idx_in;
        r_id   <= aw_id;
        r_err  <= error;
        r_cnt  <= '0;
      end
    end
  end
  wire [AXI_AW-1:0] k_addr = {r_addr[AXI_AW-1:OFW] + (AXI_AW-OFW)'(r_k), {OFW{1'b0}}};
  wire [rmc_cfg_pkg::DEC_W-1:0] k_dec = exp_dec(k_addr);

  // each line handed out is line k of the request: right channel, right DRAM coords, tag
  as_w_line:  assert property (@(posedge aclk) disable iff (!aresetn)
    !ld_valid || (r_on && !r_drop && r_k < r_n
      && f_ch == k_dec[OFW +: CHW]
      && rmc_cif_w_burst_spltr.ld_daddr == k_dec[OFW + CHW +: DW_D]
      && ld_rob_idx == r_rob));
  // exactly one SEAL per request, after its last line (or its drain), with the counts
  as_w_entry: assert property (@(posedge aclk) disable iff (!aresetn)
    !seg_last || (r_on && (r_drop ? drain_done : (lfire && r_k + 1'b1 == r_n))));
  // seal carries the admitted id/resp (admit side) + the per-channel counts summing to the run
  logic [W-1:0] seg_sum;
  always_comb begin
    seg_sum = '0;
    for (int c = 0; c < NCH; c++) seg_sum = seg_sum + W'(seg_npkts_ch[c]);
  end
  as_w_count: assert property (@(posedge aclk) disable iff (!aresetn)
    !seg_last || (seg_sum == (r_drop ? '0 : r_n)));
  for (genvar c = 0; c < NCH; c++) begin : g_cnt
    as_w_chcnt: assert property (@(posedge aclk) disable iff (!aresetn)
      !seg_last || seg_npkts_ch[c]
        == r_cnt[c] + ((lfire && f_ch == CHW'(c)) ? 1'b1 : 1'b0));
    as_h_cnt: assert property (@(posedge aclk) disable iff (!aresetn)
      !run || rmc_cif_w_burst_spltr.cnt_q[c] == r_cnt[c]);
  end
  // a dropped request hands out nothing
  as_w_drop:  assert property (@(posedge aclk) disable iff (!aresetn) !(r_on && r_drop) || !ld_valid);
  // helpers
  as_h_run:   assert property (@(posedge aclk) disable iff (!aresetn)
    run == r_on && (!run || (drop_q == r_drop && s_addr == r_addr && s_rob == r_rob
      && r_n >= 1 && r_n <= W'(MAX_PKTS)
      && 32'(r_addr[PG-1:OFW]) + 32'(r_n) <= (1 << (PG - OFW))
      && (r_drop || (r_k < r_n && W'(n_left) == r_n - r_k
          && 32'(rmc_cif_w_burst_spltr.line_q) == 32'(r_addr[PG-1:OFW]) + 32'(r_k))))));
  // helper: the per-channel counts add up to the lines handed out so far
  logic [W-1:0] r_sum;
  always_comb begin
    r_sum = '0;
    for (int c = 0; c < NCH; c++) r_sum = r_sum + W'(r_cnt[c]);
  end
  as_h_sum:   assert property (@(posedge aclk) disable iff (!aresetn) !run || (r_sum == r_k && (!r_drop || r_k == 0)));
  as_h_size:  assert property (@(posedge aclk) disable iff (!aresetn)
    !run || ((1 << s_size) <= (1 << OFW)));

  c_req:      cover property (@(posedge aclk) disable iff (!aresetn) seg_last && !r_drop && r_n > 2);
  c_drop:     cover property (@(posedge aclk) disable iff (!aresetn) seg_last && r_drop);
  c_two_ch:   cover property (@(posedge aclk) disable iff (!aresetn)
                seg_last && seg_npkts_ch[0] != '0 && seg_npkts_ch[NCH-1] != '0);
  c_stall:    cover property (@(posedge aclk) disable iff (!aresetn) ld_valid && !ld_ready);
  c_b2b:      cover property (@(posedge aclk) disable iff (!aresetn) cap && $past(seg_last));

  // the proof starts in reset (without this the first state is unconstrained)
  logic fv_started = 1'b0;
  always_ff @(posedge aclk) fv_started <= 1'b1;
  m_reset_first: assume property (@(posedge aclk) fv_started || !aresetn);

endmodule : rmc_cif_w_burst_spltr_fv
