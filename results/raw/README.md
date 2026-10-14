# Raw experiment runs

Merged by `experiments.py --reuse raw/merged.csv --reuse-strategies fcfs staged deadline joint cpsat cpsat-nores rolling`
into `../runs.csv`, `../summary.csv`, `../results.md` and the figures. Seeds 1–10, all settings.

| File | Strategies | Code | Machine |
|---|---|---|---|
| `light.csv` | fcfs, staged, deadline, joint, rolling | 75afc09 | Colab standard CPU VM (2 vCPU); deterministic, machine-independent |
| `cpsat_nores_*.csv` | CP-SAT without terminal reserve (run as `cpsat` before the option existed, relabelled) | 75afc09 | 4 × Colab high-RAM VM, Intel Xeon @ 2.20 GHz, 8 workers, 120 s |
| `cpsat_fair_*.csv` | CP-SAT with terminal reserve and 0.5 % margin (`cpsat`) | c87fb9f | 3 × Colab high-RAM VM, Intel Xeon @ 2.20 GHz, 8 workers, 120 s |

`*.log` are the matching console logs. OR-Tools 9.15.6755, Python 3.13 on Colab.
