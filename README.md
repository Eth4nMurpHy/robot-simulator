# LoopLab · dual-loop robot simulator

A working browser simulator based on the EE6228 M02/M04 proposal: task assignment, exclusive intersection access, shared charging, battery safety, and experiment visualization.

## Run

Python 3.10+ is the only runtime dependency of the simulator, the dashboard and the four heuristics. No API key or external service is needed.

```bash
python3 app.py --host 0.0.0.0 --port 8000
```

The CP-SAT optimizers and the experiment figures additionally need OR-Tools and matplotlib (`requirements.txt`). Use a virtual environment and start the server with its interpreter to enable the two optimizer strategies in the dashboard:

```bash
python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt
.venv/bin/python app.py --port 8000
```

When running on your own computer, open `http://localhost:8000` in a browser. For cloud tasks, use the platform's supported application access mechanism. The server is a development server; it has no authentication and is not a public production deployment.

The UI loads 40 reproducible tasks using seed 42 and an example command plan (move, then wait). The default mode is **My command plan**. To schedule all generated tasks automatically for a reference experiment, select one of the automatic strategies. Click **Run simulation**, then play, pause, seek, step to the next event, or reset the replay. Task library supports editing and CSV/JSON import. Analytics shows Gantt charts, battery curves, reservations, delivery outcomes, and side-by-side strategy comparisons on an identical input snapshot. The layout adapts to mobile screens.

## Task input

**Generate** creates a reproducible workload from the seed: count, loop A share, mean release gap (default 12 s per task), optional tight deadlines (release + 4–8 min instead of 8–15 min) and an optional share of urgent orders (`U…` IDs, priority 4–5, due 3–6 min after release). Urgent orders come from a separate random stream, so adding them never changes the base tasks of the same seed. **Random per robot** under Battery gives each robot a seeded initial SOC of 50–100 % (`robot_soc` in the config); otherwise all robots start at the slider value.

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

## Edit your map

1. Open **Map editor** in the sidebar.
2. Change an edge's **Length (m)** in the table. Each direction is separate: `H → C1` and `C1 → H` can have different lengths. Click a length on the diagram to select its table row.
3. Set robot speed (m/s) and energy consumption (% per meter).
4. Click **Apply map to next run**, return to Simulation, and click **Run simulation**.

Travel time is `length_m / speed_m_per_s`; energy consumption is `length_m * energy_per_m`. This also applies to the three edges entering X, so their lengths change intersection occupancy. Lengths must be 0.1–200 m, speed 0.1–10 m/s, and consumption 0.001–1% per meter. Very long routes or high consumption may be impossible to schedule with the selected battery settings.

**Import map JSON** and **Export map JSON** let you share maps. Start with [examples/map.json](examples/map.json). Files use `version: 1`, `speed_m_per_s`, `energy_per_m`, and an `edges` array of objects with `from`, `to`, and `length_m`. All 18 directed edges must be present exactly once. The current topology remains the dual loop: changing lengths does not add/remove stations, change connectivity, or rearrange the diagram. The drawing is schematic, not to scale; labels show the configured physical lengths.

Applying a map overrides the original global edge-time, crossing-time, and edge-energy controls, which become disabled. Click **Use original global timing settings** to restore them. **Reset defaults** resets the map editor draft; click Apply to activate it.

## Your strategy is a robot command timeline

The simulator **does not decide a strategy for you** in command mode. Import the command output of an external planner, or enter commands yourself in **My strategy**. You specify the robot, exact start time, and action. There are no scoring weights, automatic task assignments, automatic charging decisions, or conflict-repair queues.

Quick example:

```json
{
  "name": "My robot plan",
  "commands": [
    {"robot_id": "R1", "at": 0, "action": "execute_task", "task_id": "T001"},
    {"robot_id": "R3", "at": 0, "action": "execute_task", "task_id": "T002"},
    {"robot_id": "R1", "at": 60, "action": "charge", "bay": "C1", "target_soc": 90},
    {"robot_id": "R1", "at": 650, "action": "move_to", "node": "H"}
  ]
}
```

To try this complete example, import **[examples/command_tasks.csv](examples/command_tasks.csv)** into Task library and **[examples/strategy.json](examples/strategy.json)** into My strategy. Use the default map and battery settings. The two deliveries finish at 50 seconds; charging and the explicit departure command extend the plan to 670 seconds. Edited lengths/speeds change action durations and can make a formerly valid plan conflict.

### Command fields

Each command requires `robot_id` (`R1`–`R4`), `at` (absolute simulation seconds, 0–1,000,000), and `action`, plus only the parameters for that action:

| Action | Parameters | Execution |
|---|---|---|
| `execute_task` | `task_id` | Travel to the task's pickup, load, travel to its dropoff, and unload. No reassignment, automatic charging, or release-time delay. |
| `move_to` | `node` | Follow the fixed directed route to a station or shared node. No route optimization. |
| `charge` | `bay`, `target_soc` | Travel to the named bay, then charge to the target (1–100%, not below arrival SOC). Remain parked there until explicitly moved away. |
| `wait` | `duration` | Stay at the current node for 0.001–86,400 seconds, consuming the configured waiting energy. |

Robot-to-loop compatibility still applies. C2 cannot be used when only one bay is enabled. `at` starts the entire command, including travel: a charge at 60 seconds **starts travel at 60**, not charging at 60. Commands are executed chronologically; original array positions remain the command numbers shown in errors. Maximum 1000 commands.

### Strict errors, not automatic repairs

- A command arriving before its robot's preceding command ends is **ROBOT_BUSY**. It is invalid, not queued.
- Overlapping X occupancy or insufficient crossing safety buffer is **INTERSECTION_CONFLICT**.
- Two robots in the same charging bay is **CHARGER_CONFLICT**. Parked robots occupy a bay even after charging ends, including the final arrival instant of the plan.
- Stopping in X is **INTERSECTION_BLOCKED**. A `move_to X` must be immediately followed by a departure at its exact arrival time. No waiting/parking inside the intersection.
- Commands for unknown tasks, repeated tasks, another loop's tasks, or tasks not yet released are invalid.
- Any action that would put SOC below the configured safety threshold is **BATTERY_UNSAFE**. No charging command is inserted automatically. Later commands for a robot whose battery action failed are marked dependent/unvalidated.

The app rejects the whole plan when errors occur. It displays command numbers, robots, simulation times, resource conflicts, and reasons, with an **Error report JSON** download. It does not return the invalid plan as a successful replay. A previous valid result remains clearly identified as the previous run. Independent errors are collected where possible; malformed definitions and commands depending on failed actions can prevent downstream validation.

Uncommanded tasks are **unexecuted**, not silently assigned and not counted as completed. Analytics and exports list them. A partial command plan is allowed, including movement-only plans without tasks. Every command records its requested start and computed end in result JSON.

### Editor and persistence

Use **My strategy** to add/edit/delete command rows, or import/paste JSON directly. Click **Apply & select command plan**, then **Run simulation**. Applying validates the JSON structure; simulation checks timing, resources, tasks, and batteries against the actual task list/map. Export JSON to save or share the plan.

Applied maps and command plans persist in this browser's local storage. Unapplied drafts and task edits are session-only. Maps from the previous rules editor are retained, but old weight-rule JSON is deliberately rejected as a command strategy. Export files for use in another browser/computer. Existing results retain the map and plan used for that run.

## Automatic strategies

The event engine uses deterministic, piecewise-linear movement and battery updates. Replay and charts read the returned action log; animation does not run an independent scheduler.

Four **heuristics** (standard library only, no optimality claim):

1. **FCFS:** task release order, selecting the compatible robot with the earliest predicted completion. Below 20% SOC, recharge before taking a task; all strategies also apply the stronger safety reserve.
2. **Staged:** assign first, then reserve the intersection, then charge. Assignment is list scheduling on nominal travel times that ignores X, chargers and battery; each robot's fixed sequence is then executed with intersection reservations, and a charge is inserted only when the next task would otherwise be unsafe.
3. **Deadline + reservations:** earliest deadline first; priority breaks deadline ties. Each candidate includes resource reservations and energy checks.
4. **Joint greedy:** evaluate up to eight next tasks and both compatible robots. Minimize completion seconds + 3 × priority × lateness seconds + 0.5 × resource wait seconds + 2 × consumed battery percentage points.

Two **optimizers** (`optimizer.py`, need OR-Tools; formulation in [docs/optimizer_model.md](docs/optimizer_model.md)):

5. **CP-SAT joint optimisation:** one model decides task assignment, per-robot order (circuit constraints), intersection slots (optional intervals in one no-overlap constraint with the buffer), charging visits, bay choice, charging start and amount, with SOC bounds after every event. Like the heuristics, every robot that executes orders must end the horizon able to reach a charger behind a full queue (`terminal_reserve`, default on); without it terminal SOC may fall below that reserve, and under load the optimum then often ends near the safety floor and skips charging. It minimises the normalised objective J below, is warm-started with the joint greedy schedule, and is realised through the same simulator and validator, so it never returns a schedule worse than that warm start. Default 60 s wall time with up to 8 parallel workers (the dashboard default is 20 s); parallel search is not bit-for-bit reproducible.
6. **Rolling-horizon CP-SAT:** a task is invisible until its release time. Every 60 s the known, unstarted tasks within about 10 min of work are re-optimised (single worker; a deterministic budget of 0.1 per solve, about 0.5–1 s, binds instead of the 30 s wall-time cap, so with the same OR-Tools version and options the schedules are identical across machines unless the cap binds); actions that start before the next re-plan are committed and their reservations frozen. Results report the number of re-plans and how often the heuristic incumbent was kept.

Options for the optimizers are passed as the fifth `simulate()` argument, e.g. `simulate(tasks, config, 'cpsat', None, {'time_limit': 30, 'workers': 8})`; `{'terminal_reserve': False}` removes the end-of-horizon reserve (offline CP-SAT only).

Each loop follows `X → 1 → 2 → 3 → 4 → W → X`. Charging branches from X through H to a bay. The bay is vacated immediately after charging; robots return through H and X. With default settings, ordinary edges use the global travel time/energy and crossings use their own duration. With an applied map, every edge uses its physical length, robot speed, and energy per meter.

Intersection bookings are exclusive with a gap of twice the configured per-side buffer. Charger bookings are also exclusive. Waiting occurs at WA/WB or H, never inside the intersection or an occupied charging bay. Powered-down standby between tasks consumes no energy; queue waiting does. Loading and unloading each consume 0.5%. Automatic examples charge linearly to 95%. Command mode uses only explicitly requested charging targets.

For automatic examples only, before accepting a task, the planner checks completion, travel to charging, and the actual reserved charging queue. It also keeps conservative battery headroom for three other robots each needing a full recharge, which can cause earlier charging. If needed, charge before the task. Configurations that the heuristic cannot schedule safely return a descriptive error rather than an invalid schedule. A failed heuristic is not proof that no possible schedule exists.

An independent validator recomputes action energy and checks task uniqueness, pickup/delivery precedence, release times, route continuity, robot action overlaps, resource exclusivity, crossing buffers, wait locations, charger departure, and SOC bounds.

### Metrics and exports

- **Plan duration (command mode):** end of the final command, including charging/waiting. `task_makespan` separately records the final delivery completion.
- **Makespan (automatic examples):** final delivery completion time.
- **Tardiness:** `max(0, completion − deadline)`; rate = late tasks / total tasks.
- **Average resource wait:** total intersection + charging queue time / task count.
- **Energy:** sum of positive consumption across robots, excluding battery replenishment.
- **Utilization:** occupied time / makespan; charging divides by bay count as well. In command mode, charging utilization measures bay occupation, including parked time; active charging sessions are separately logged.
- **Violations:** independent validation errors. Invalid schedules are not returned as successful runs.
- **Objective J:** `0.25 × (Cmax / Cmax_ref + Σ priority·T / ref + (W_cross + W_charge) / ref + E / E_ref)`, where each reference is the FCFS value on the same instance (floors of 1 s, 60 s, 60 s and 1 % avoid division by zero). FCFS therefore scores 1 (slightly less if one of its terms is below the floor); lower is better. Normalisation stops tardiness seconds from dominating. `objective_terms` and `objective_reference` in the summary give the raw components. Battery safety is a hard constraint, not a term.

Export full results as JSON (tasks, configuration, actions, events, reservations, validation, metrics, and comparisons). Download task, schedule, event log, robots and summary CSVs individually. Comparisons contain the four heuristics, plus both optimizers when OR-Tools is available (CP-SAT uses the time limit field). Manual command plans are not benchmarked against automatically assigned workloads; their run reports exact commands and unexecuted tasks. Full results also retain the exact map and command plan definition; schedule records and summary include distance in meters. Comparisons use the exact tasks and settings of the most recent successful run, regardless of later UI edits.

## Experiments

`experiments.py` runs the proposal's experiment plan on repeated seeds with identical inputs for every strategy (plus `cpsat-nores`, an ablation of CP-SAT without the terminal reserve): algorithm comparison, load (20/40/60 tasks), chargers (1/2), zone imbalance (1:1, 2:1, 3:1), safety threshold (10/20/30 %) and dynamic urgent orders (+0/10/20 %). All runs use seeded random initial SOC.

```bash
.venv/bin/python experiments.py --seeds 10 --time-limit 120 --out results
python3 experiments.py --seeds 3 --strategies fcfs staged deadline joint   # heuristics only, no OR-Tools
```

It writes `runs.csv` (every run), `summary.csv` (mean, std, median and worst per group, plus the paired change of J against FCFS), `results.md` and PNG figures. Each distinct instance is solved once even if it appears in several experiments; CP-SAT runs one instance at a time with its own workers, the other strategies run in a process pool. A heuristic that cannot keep every robot safe is recorded as a failure (an "energy deadlock"), not dropped.

## Tests

Required engine and API checks use the standard library:

```bash
python3 -m unittest discover -s tests -v
```

With `.venv/bin/python` the optimizer tests also run (otherwise they are skipped): proven optima on small chains, charge amounts, fractional maps, determinism, warm-start feasibility and rolling release/commitment audits. The tests include hand-calculated schedules, release/deadline behavior, reproducibility, invalid inputs, battery infeasibility, one-bay congestion, thresholds, unbalanced workloads, 200 tasks, tampered schedules, and functional HTTP requests.

Optional browser checks require Playwright and a Chromium installation. With the server running:

```bash
python3 -m pip install playwright
python3 tests/browser_smoke.py --chromium /usr/bin/chromium
```

Alternatively install Playwright's browser with `python3 -m playwright install chromium` and omit `--chromium`. Add `--screenshots /tmp/looplab-check` to save desktop, analytics, and mobile screenshots. Browser checks cover replay controls, charts, comparisons, task/map/strategy editing and import/export, local-storage restoration, input failures, mobile overflow, and HTML escaping.

## Files

- `simulator.py`: world, task generation/validation, reservation calendars, battery model, heuristics, event records, and independent validation.
- `optimizer.py`: CP-SAT joint model, decoder through the simulator, and rolling-horizon re-planning.
- `experiments.py`: repeated-seed experiment runner, tables and figures.
- `docs/optimizer_model.md`: mathematical formulation of the optimizer.
- `report/`: course report.
- `command_plan.py`: strict command schema, execution, occupancy tracking, and structured errors.
- `customization.py`: map schemas and edge metrics (also retains legacy rule helpers for compatibility testing).
- `simulator_types.py`: shared input validation.
- `app.py`: static UI and JSON endpoints (`/api/health`, `/api/options`, `/api/generate`, `/api/validate-customization`, `/api/simulate`, `/api/compare`).
- `static/`: responsive dashboard, SVG replay/charts, task editor, CSV handling, and downloads.
- `tests/`: engine/API suite and optional real-browser smoke check.

## Scope

Implemented: the proposal's minimum version plus the staged baseline, CP-SAT joint optimisation, rolling-horizon re-optimisation with tasks revealed at release, urgent dynamic orders, per-robot random initial SOC and repeated-seed experiments. Not implemented: nonlinear charging, multiple intersections, a variable number of robots per loop, physical traffic and continuous kinematics. Ordinary loop edges are not exclusive; robots may share them, so the safety validator covers the shared intersection/chargers and battery state rather than geometric collision avoidance. The heuristics see the whole task list in advance (they still never start a task before its release); only the rolling strategy is truly online. Strategy improvements depend on the workload; reported results are measured, not assumed.
