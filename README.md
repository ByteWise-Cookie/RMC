# RMC

Memory controller for DDR1-5.

## Layout

| Path | Contents |
|------|----------|
| `rtl/cif/` | CIF block |
| `rtl/mc/` | MC core (scheduler, DFI fabric, data path) |
| `rtl/config_rtl/` | CSR / mode registers, APB target, parametric defaults |
| `tb/phy_model/` | PHY behavioral model (local, not committed) |
| `tb/mem_model/` | DRAM memory model (local, not committed) |
| `tb/config_tb/` | test knobs, sim config, register-init vectors |
| `tb/tests/` | testcases |
| `sim/` | run scripts (podman iverilog / verilator) |
| `scripts/` | helper / automation scripts |
| `filelists/` | `.f` compile lists |
| `docs/` | documentation |

## Config interface

Register block accessed over APB (target in `rtl/config_rtl/`).

## Simulation

Run via rootless podman (no native iverilog):

```
podman run --rm -v "$PWD":/w:Z -w /w docker.io/hdlc/iverilog:latest \
  sh -c 'iverilog -g2012 -o sim.out -f filelists/tb.f && vvp sim.out'
```

Keep all counts parametric (`N_RANKS`, bank/BG counts) — never hardcode.
