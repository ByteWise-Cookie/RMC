# CIF formal — status & log

Owned by the FV session. Keep this file current: flip status as proofs pass/fail,
note depth/assume caveats, and APPEND run logs at the bottom (newest last). RTL/
integration sessions only add new harness rows here; they do not run the proofs.

Run from `fv/`:  `make <target> [T=<task>]`  (OSS-CAD at ~/.local/opt/oss-cad-suite).
Tasks per block are in its `.sby` ([tasks]); typically `<cfg>_prove|cover|bmc`.

## Status

Legend: elab = verilator --assert elaborates ; prove/cover/bmc = sby task result.
(todo = not run, ok = clean, FAIL = counterexample, n/a = no such task)

| target  | module                | elab | prove | cover | bmc  | notes                         |
|---------|-----------------------|------|-------|-------|------|-------------------------------|
| rrob    | rmc_cif_rrob          | ok   | ok    | ok    | ok   | live d3/d4/full ok            |
| rspltr  | rmc_cif_r_burst_spltr | ok   | ok    | ok*   | ok   | *d4_cover >20 min; d2_cover ok |
| rresp   | rmc_cif_r_resp        | ok   | ok    | ok    | ok   | symbolic SRAM data check      |
| frptr   | rmc_cif_frptr         | ok   | ok    | ok    | ok   | shared read+write             |
| awbuf   | rmc_cif_aw_reqbuf     | ok   | ok    | ok    | ok   | data-order scoreboard         |
| wdfifo  | rmc_cif_w_data_fifo   | ok   | ok    | ok    | ok   | data-order scoreboard         |
| waccum  | rmc_cif_w_accum       | ok   | ok    | ok    | ok   | byte-lane/offset-walk model   |
| wspltr  | rmc_cif_w_burst_spltr | ok   | ok    | ok    | ok   |                               |
| wrob    | rmc_cif_wrob          | ok   | ok    | ok    | ok   | per-channel completion        |
| wpktgen | rmc_cif_w_pkt_gen     | ok   | ok    | ok    | ok   | two-stage commit              |
| wresp   | rmc_cif_w_resp        | ok   | ok    | ok    | ok   | B register slice              |

mc_req_arb removed (per-channel MC boundary; no arbiter).

## Open
- rspltr d4_cover: c_w_stall needs 16 lines on one channel, >20 min. Covered by
  d2_cover (8 slots, step 12). Drop c_w_stall from d4 or skip d4_cover in `make`.
- Do not run sby tasks in parallel from one work/ dir: the shared status db
  corrupts (sqlite "database disk image is malformed") and tasks die/hang.
- No checker for rmc_cif_addrmap / rmc_cif_reqvalidator.
- Checkers are reference-model where fleshed; confirm the neighbour `assume`s are
  not over-constraining (a vacuous pass hides bugs) — check the cover traces.
- rrob/wrob liveness (FV_LIVE) assumes fair RESP_READY / completion cadence; tune
  FV_*_MAX if the bound is too tight.
- No top-level (rmc_cif) formal; block-level only.

## Log
(append: `YYYY-MM-DD  make <target> T=<task>  -> result / notes`)
2026-10-25  make (all tasks, main @ 9495ad5)  -> all prove/cover/bmc PASS except
            rrob d3/d4/full_live UNKNOWN (a_head_live induction) and
            rspltr d4_cover TIMEOUT
2026-10-25  rrob: add a_run_sum (sum per-channel run == npkts per live slot);
            induction start had runs 3+3 with npkts 0  -> d3/d4/full_live PASS,
            rrob prove/cover tasks still PASS
