# LoopLab · dual-loop robot simulator

A working browser simulator based on the EE6228 M02/M04 proposal: task assignment, exclusive intersection access, shared charging, battery safety, and experiment visualization.

## Run

Python 3.10+ is the only runtime dependency. No package installation, API key, or external service is needed.

```bash
cd /workspace/robot-simulator
python3 app.py --host 0.0.0.0 --port 8000
```

When running on your own computer, open `http://localhost:8000` in a browser. For cloud tasks, use the platform's supported application access mechanism. The server is a development server; it has no authentication and is not a public production deployment.

The UI loads 40 reproducible tasks using seed 42. Click **Run simulation**, then play, pause, seek, step to the next event, or reset the replay. Task library supports editing and CSV/JSON import. Analytics shows Gantt charts, battery curves, reservations, delivery outcomes, and side-by-side strategy comparisons on an identical input snapshot. The layout adapts to mobile screens.

## Task input

See [examples/tasks.csv](examples/tasks.csv) for a ready-to-import 40-task workload. CSV requires the following header; JSON accepts an array of task objects or an object containing a `tasks` array:

```csv
task_id,release_time,pickup,dropoff,load_time,unload_time,due_time,priority
T001,0,A1,A2,15,15,600,1
T002,10,B3,B1,15,20,700,2
```

- Times are **seconds**; energy and SOC are **battery percentage points**.
- IDs must be unique, nonempty, and at most 64 characters; maximum 200 tasks.
- Stations are A1–A4 and B1–B4. Pickup and dropoff must differ and belong to the same loop.
- R1/R2 serve A; R3/R4 serve B. Each robot carries one task at a time.
- Release time: 0–86,400; due time: 0–100,000, no earlier than release. Handling times: 0–300. Priority: 1–5; larger numbers receive greater lateness weight in the joint heuristic.
- Uploaded tasks and edits live in the current browser session. Export the task CSV to keep your changes.

## Scheduling and simulation

The event engine preplans deterministic, piecewise-linear movement and battery updates. Replay and charts read the returned action log; animation does not run an independent scheduler.

Three **heuristics**, with no global optimality claim:

1. **FCFS:** task release order, selecting the compatible robot with the earliest predicted completion. Below 20% SOC, recharge before taking a task; all strategies also apply the stronger safety reserve.
2. **Deadline + reservations:** earliest deadline first; priority breaks deadline ties. Each candidate includes resource reservations and energy checks.
3. **Joint greedy:** evaluate up to eight next tasks and both compatible robots. Minimize completion seconds + 3 × priority × lateness seconds + 0.5 × resource wait seconds + 2 × consumed battery percentage points.

Each loop follows `X → 1 → 2 → 3 → 4 → W → X`. Charging branches from X through H to a bay. The bay is vacated immediately after charging; robots return through H and X. Ordinary edges have fixed travel time and energy, and crossings have their own duration with proportional energy consumption.

Intersection bookings are exclusive with a gap of twice the configured per-side buffer. Charger bookings are also exclusive. Waiting occurs at WA/WB or H, never inside the intersection or an occupied charging bay. Powered-down standby between tasks consumes no energy; queue waiting does. Loading and unloading each consume 0.5%. Charging is linear to 95%.

Before accepting a task, the planner checks completion, travel to charging, and the actual reserved charging queue. It also keeps conservative battery headroom for three other robots each needing a full recharge, which can cause earlier charging. If needed, charge before the task. Configurations that the heuristic cannot schedule safely return a descriptive error rather than an invalid schedule. A failed heuristic is not proof that no possible schedule exists.

An independent validator recomputes action energy and checks task uniqueness, pickup/delivery precedence, release times, route continuity, robot action overlaps, resource exclusivity, crossing buffers, wait locations, charger departure, and SOC bounds.

### Metrics and exports

- **Makespan:** final delivery completion time.
- **Tardiness:** `max(0, completion − deadline)`; rate = late tasks / total tasks.
- **Average resource wait:** total intersection + charging queue time / task count.
- **Energy:** sum of positive consumption across robots, excluding battery replenishment.
- **Utilization:** occupied time / makespan; charging divides by bay count as well.
- **Violations:** independent validation errors. Invalid schedules are not returned as successful runs.

Export full results as JSON (tasks, configuration, actions, events, reservations, validation, metrics, and comparisons). Download task, schedule, event log, and summary CSVs individually. The exported summary contains all three strategies after running a comparison. Comparisons use the exact tasks and settings of the most recent successful run, regardless of later UI edits.

## Tests

Required engine and API checks use the standard library:

```bash
python3 -m unittest discover -s tests -v
```

The tests include hand-calculated schedules, release/deadline behavior, reproducibility, invalid inputs, battery infeasibility, one-bay congestion, thresholds, unbalanced workloads, 200 tasks, tampered schedules, and functional HTTP requests.

Optional browser checks require Playwright and a Chromium installation. With the server running:

```bash
python3 -m pip install playwright
python3 tests/browser_smoke.py --chromium /usr/bin/chromium
```

Alternatively install Playwright's browser with `python3 -m playwright install chromium` and omit `--chromium`. Add `--screenshots /tmp/looplab-check` to save desktop, analytics, and mobile screenshots. Browser checks cover replay controls, charts, comparison, imports/exports, task editing, input failures, mobile overflow, and HTML escaping.

## Files

- `simulator.py`: world, task generation/validation, reservation calendars, battery model, heuristics, event records, and independent validation.
- `app.py`: static UI and JSON endpoints (`/api/health`, `/api/generate`, `/api/simulate`, `/api/compare`).
- `static/`: responsive dashboard, SVG replay/charts, task editor, CSV handling, and downloads.
- `tests/`: engine/API suite and optional real-browser smoke check.

## Scope

This implements the proposal's minimum version with three scheduling rules. It does **not** implement CP-SAT, global or rolling optimization, live dynamic orders, nonlinear charging, physical traffic, or repeated-seed statistical reports. Ordinary loop edges are not exclusive; robots may share them, so the safety validator covers the shared intersection/chargers and battery state rather than geometric collision avoidance. All four robots start with the user-selected SOC. Strategy improvements depend on the workload; the UI reports measured results without assuming the joint heuristic always wins.
