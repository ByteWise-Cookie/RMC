# RMC

Reconfigurable Memory Controller for DDR1-5.

## Layout

| Path | Contents |
|------|----------|
| `rtl/cif/` | CIF block |
| `rtl/mc/` | MC core (scheduler, DFI fabric, data path) |
| `rtl/utils/` | generic parametric blocks (FIFO, SRAM, regfile, lookahead) |
| `rtl/csr_config/` | CSR / mode registers, APB target |
| `rtl/rtl_pkg_config/` | compile-time packages (DDR-gen select, structural params) |
| `tb/phy_model/` | PHY behavioral model (local, not committed) |
| `tb/mem_model/` | DRAM memory model (local, not committed) |
| `tb/config_tb/` | test knobs, sim config, register-init vectors |
| `tb/tests/` | testcases |
| `sim/` | run scripts (podman iverilog / verilator) |
| `scripts/` | helper / automation scripts |
| `filelists/` | `.f` compile lists |
| `docs/` | documentation |

## Config interface

Register block accessed over APB (target in `rtl/csr_config/`). Compile-time
structure (DDR generation, counts, widths) lives in `rtl/rtl_pkg_config/`.

## Address map (STAGE 24)

64 B packet (BL16 x 32b). Interleave (LSB->MSB): `offset | ch | rank | BG_lo(rot) |
col_lo(=P) | BG_hi | bank | col_hi | row`. Consecutive packets rotate BGs (diff-BG
CAS = tCCD_S, writes safe), then `col_lo` revisits a bank's next column = row-hit.
`P = packets/bank` (max 4 at a 4 KB burst; 2 KB -> 2, 1 KB/64 B -> 1). `P` sets the
per-bank TCAM / cas_fifo depth.

## Matched-bandwidth AXI width

`AXI_DW = N_DDR_CHANNELS * DDR_CHANNEL_WIDTH` (`rmc_cfg_pkg::AXI_DW`), so the AXI
ingress rate equals the DDR egress rate. This keeps the WD/RD SRAM and the ROB
inflow/outflow balanced (inflow == outflow, no buffer blow-up). Derived, not a free
per-instance parameter.

Caveat: bandwidth = width x frequency, so the width-only form is exact only when
`aclk == DDR data rate`. If the clocks differ, scale by the ratio:
`AXI_DW = N_CH * DDR_CHANNEL_W * (f_ddr / f_aclk)` — otherwise AXI bottlenecks and
the buffers drift. (A `CLK_RATIO` param is a TODO.)

## Simulation

Run via rootless podman (no native iverilog):

```
podman run --rm -v "$PWD":/w:Z -w /w docker.io/hdlc/iverilog:latest \
  sh -c 'iverilog -g2012 -o sim.out -f filelists/tb.f && vvp sim.out'
```

Keep all counts parametric (`N_RANKS`, bank/BG counts) — never hardcode.
