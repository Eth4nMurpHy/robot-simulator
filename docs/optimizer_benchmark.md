# Optimizer benchmark

Measured on 2026-10-14 with the default map/configuration and task seeds 1, 2, 3.
Python 3.14.8; OR-Tools 9.15.6755. Both optimizers use seed 0 and one worker.

Offline CP-SAT: `time_limit=30`, default deterministic budget 1.5. Rolling: 2 seconds / 0.1 deterministic time per solve, 60-second replans, 600-second work horizon. Runtime below is end-to-end seconds. Budgets are upper limits, not forced run durations.

Tardiness is the unweighted sum; J uses priority-weighted tardiness. Wait is intersection plus charger queue seconds. Energy is total positive consumption in battery percentage points.

| Seed/tasks | Strategy | J | Makespan s | Tardiness s | Wait s | Energy % | Runtime s |
|---|---|---:|---:|---:|---:|---:|---:|
| 1/20 | fcfs | 41791.6 | 2310.7 | 6582.4 | 1501.6 | 241.61 | 0.01 |
| 1/20 | deadline | 21221.9 | 1870.6 | 4123.9 | 19.0 | 202.26 | 0.01 |
| 1/20 | joint | 13832.4 | 1669.6 | 2104.8 | 0.0 | 175.00 | 0.03 |
| 1/20 | cpsat | 1274.4 | 848.2 | 13.0 | 0.0 | 193.60 | 5.21 |
| 1/20 | rolling | 6758.2 | 1412.4 | 1043.0 | 0.0 | 191.60 | 2.99 |
| 1/40 | fcfs | 161922.6 | 3600.1 | 30747.2 | 1206.3 | 424.22 | 0.02 |
| 1/40 | deadline | 167471.1 | 3975.4 | 33210.6 | 1525.1 | 433.88 | 0.02 |
| 1/40 | joint | 122523.9 | 2897.0 | 23097.1 | 1701.4 | 364.27 | 0.09 |
| 1/40 | cpsat | 122523.9 | 2897.0 | 23097.1 | 1701.4 | 364.27 | 7.27 |
| 1/40 | rolling | 164071.1 | 3237.7 | 27342.6 | 1660.8 | 438.94 | 15.02 |
| 2/20 | fcfs | 24007.9 | 2261.3 | 4613.5 | 766.6 | 227.96 | 0.01 |
| 2/20 | deadline | 27215.9 | 2273.4 | 4381.0 | 666.6 | 221.62 | 0.01 |
| 2/20 | joint | 12430.3 | 1665.6 | 2106.8 | 25.0 | 177.08 | 0.04 |
| 2/20 | cpsat | 5390.4 | 1514.6 | 1172.2 | 10.0 | 177.03 | 4.53 |
| 2/20 | rolling | 2492.4 | 1130.6 | 312.2 | 15.0 | 176.45 | 2.69 |
| 2/40 | fcfs | 230975.1 | 3812.1 | 42207.9 | 1723.2 | 496.14 | 0.02 |
| 2/40 | deadline | 201836.1 | 3716.4 | 35265.0 | 1573.4 | 468.44 | 0.02 |
| 2/40 | joint | 171186.1 | 3256.7 | 31124.0 | 1856.3 | 434.99 | 0.10 |
| 2/40 | cpsat | 171186.1 | 3256.7 | 31124.0 | 1856.3 | 434.99 | 8.93 |
| 2/40 | rolling | 157176.3 | 3469.7 | 27189.1 | 1152.8 | 438.64 | 8.94 |
| 3/20 | fcfs | 23940.1 | 1802.6 | 3912.6 | 10.0 | 218.83 | 0.01 |
| 3/20 | deadline | 36628.4 | 2313.3 | 7689.1 | 1417.6 | 264.53 | 0.01 |
| 3/20 | joint | 20148.4 | 1743.6 | 3245.4 | 0.0 | 197.60 | 0.04 |
| 3/20 | cpsat | 11660.7 | 1743.6 | 1880.6 | 647.6 | 206.36 | 5.08 |
| 3/20 | rolling | 18272.2 | 1576.6 | 3061.2 | 0.0 | 202.20 | 6.66 |
| 3/40 | fcfs | 130462.1 | 3089.0 | 24517.7 | 1503.1 | 397.41 | 0.02 |
| 3/40 | deadline | 173971.3 | 3365.0 | 34392.5 | 1576.0 | 448.65 | 0.02 |
| 3/40 | joint | 100759.8 | 2729.2 | 17376.1 | 1517.1 | 346.46 | 0.10 |
| 3/40 | cpsat | 100750.2 | 2729.2 | 17376.1 | 1498.1 | 346.39 | 9.65 |
| 3/40 | rolling | 95088.9 | 2805.2 | 17502.6 | 1057.2 | 381.32 | 16.01 |

All 30 runs completed and passed the simulator validator. Offline CP-SAT returned FEASIBLE in all six cases, with no decoder fallbacks. It improved all 20-task instances; 40-task seeds 1 and 2 retained the joint incumbent. Seed 3 at 40 tasks improved only slightly. The reported model gaps span 56.2%–99.5%, so these runs provide no large-instance optimality proof.

Rolling was worse than joint and FCFS for seed 1 with 40 tasks (J = 164071.1, versus 122523.9 and 161922.6). It cannot anticipate future orders, freezes complete visits, and retains conservative terminal energy reserves. Its other five runs beat all three heuristics on J. This small deterministic matrix is not a statistical performance claim.

The 162 rolling solves used 59 local fallbacks: 27 exhausted their budget before obtaining a CP-SAT incumbent, 29 retained a lower-J physical joint hint, and 3 failed the additional terminal reserve check. All selected schedules were valid; no complete run failed.

Validation after the final code changes:

- `python3 -m unittest discover -s tests`: 58 tests, 12 optimizer tests skipped, 5.088 seconds.
- `.venv/bin/python -m unittest discover -s tests`: all 58 tests passed, 23.273 seconds.
- The optional optimizer file remains comfortably below the three-minute target.

Reproduce the raw measurements with `.venv/bin/python scripts/benchmark_optimizer.py`. [Raw JSON](optimizer_benchmark.json) includes separate wait totals, complete solver diagnostics, and rolling decision audits. Runtime varies with machine load; fixed-budget solver decisions are reproducible while the wall cap remains nonbinding.
