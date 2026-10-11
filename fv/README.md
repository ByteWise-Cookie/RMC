# Formal verification

SymbiYosys (sby) flow. One directory per module, mirroring `rtl/`.

```
fv/
  Makefile
  cif/rmc_cif_rob/
    rmc_cif_rob_fv.sv     checker: reference model, assumptions, asserts, covers
    rmc_cif_rob_bind.sv   binds the checker into the DUT (RTL stays untouched)
    rmc_cif_rob.sby       tasks: which config, which mode, which solver
    rmc_cif_rob_props.txt every assume / assert / cover, one line each; add yours at the bottom
    rob_test.sby          read + write ROB pair (tb/tests/module_level/rob)
  work/                   run output (ignored by git)
```

## Tools

OSS CAD Suite (sby, yosys + slang frontend, bitwuzla, gtkwave), unpacked to
`~/.local/opt/oss-cad-suite`. No install step, no root. Elsewhere: `make OSS_CAD=<dir>`.

## Run

```
cd fv
make rob                              # every task
make rob T=d4_prove                   # one task
make traces M=rob T=d4_cover          # which trace shows what
make wave   M=rob T=d4_cover N=4      # open it
```

Result per task: `PASS`, `FAIL` (real trace from reset, `trace.vcd`), or
`UNKNOWN` (bounded check passed, induction did not: `trace_induct.vcd`).

## Frontend limits (yosys-slang)

- Properties must be boolean: no `|->`, `|=>`, sequences, `default clocking`.
  Write `!a || b`, and `$past` for next-cycle checks. `$stable`, `$countones` work.
- Read with `--no-implicit-memories` so unpacked arrays can be used in properties.
- Async-reset flops need `async2sync` after `prep`.

## Adding a module

1. `fv/<area>/<module>/` with `<module>_fv.sv`, `<module>_bind.sv`, `<module>.sby`.
2. Add `SBY_<name>` and the name to `MODULES` in the Makefile.
3. Start with a small parameter config, get covers reachable, then prove.
