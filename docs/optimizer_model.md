# Joint CP-SAT scheduling and rolling replanning

`optimizer.py` models task assignment, order, intersection reservations, optional
charging visits, bay selection, charge timing and charge amount together. The
simulator remains the authority for the returned times, energy, and validation.
Both strategies are available in the dashboard and `/api/compare` when the server
runs with OR-Tools.

## Calling and reproducibility

```python
from simulator import generate_tasks, simulate
result = simulate(generate_tasks(40, 1), {}, 'cpsat', None,
                  {'time_limit': 60, 'workers': 8})
online = simulate(generate_tasks(40, 1), {}, 'rolling', None,
                  {'replan_seconds': 60, 'horizon_seconds': 600})
```

Use `.venv/bin/python`. `simulator` lazily imports the optimizer only for these
two strategies and raises `InputError` with installation guidance if OR-Tools is
unavailable. `STRATEGIES` enumerates the four heuristics; `OPTIMIZER_STRATEGIES`
supplies the two optimizer labels.

| Option | Offline default | Rolling default | Meaning |
|---|---:|---:|---|
| `time_limit` | 60 | 30 (safety cap) | Wall-clock limit in seconds **per CP-SAT call** |
| `deterministic_time` | 3600 with several workers, else `0.05 * time_limit` | 0.1 (binding) | CP-SAT deterministic work limit |
| `workers` | `min(8, cpu_count)` | 1 | CP-SAT search workers |
| `seed` | 0 | 0 | CP-SAT random seed |
| `replan_seconds` | 60 | 60 | Period between online decision epochs |
| `horizon_seconds` | 600 | 600 | Estimated work window per robot |
| `max_charges_per_robot` | automatic | automatic | Charge-visit slots per robot **per solve** |

Automatic charge slots are the maximum of the hint's number of visits and
`max(1, compatible_task_count // 5)`. Explicit caps may exclude the heuristic
hint. Such a hint cannot be returned as a fallback violating the requested cap.
Frozen online visits are outside the next solve's cap. The cap is a modeling
restriction, not a proved sufficient bound on the number of charges.

One worker, stable variable ordering (including sorted bays), the same seed,
and the deterministic limit give repeatable search on the same OR-Tools version
when the wall-clock limit does not bind first. A wall-clock interruption or
multiple workers can change the incumbent. For reproducibility experiments set
an explicit deterministic limit and a generous wall limit. Runtime includes
model construction, the heuristic, solving, decoding and validation; solver
`wall_time` measures CP-SAT only. A 30-second limit can therefore finish much
sooner when a single-worker deterministic budget is exhausted.

Offline solves use parallel workers by default because a single worker barely
moves away from the warm start on 40 or more tasks: on 40 tasks (seed 1) the
joint greedy J was not improved by one worker within a deterministic budget of
40 (about 120 s), while 8 workers reduced the raw objective from 55 092 to 34 882
in 30 s and to 2 010 in 120 s. The portfolio's large-neighbourhood-search workers
do most of the improvement. Parallel runs are therefore not bit-for-bit
reproducible; rolling solves keep one worker and a deterministic budget.

## Sets and parameters

Let $I$ be the input tasks and $K=\{1,2,3,4\}$ the robots. $I_k$ contains tasks
on robot $k$'s loop. $P=\{C1,\ldots,C_{\mathrm{chargers}}\}$ are enabled bays.
Robot $k$ has optional visits $Q_k$ and vertices $V_k=\{0\}\cup I_k\cup Q_k$;
vertex 0 represents its initial or frozen terminal state.

Task $i$ has release $r_i$, due date $d_i$, priority $p_i$, pickup $u_i$, dropoff
$v_i$, and load/unload durations. Travel is directed and fixed by `LOOPS`.
Each edge has duration $\tau_{ab}$ and consumption $g_{ab}$ from
`edge_metrics(world, a, b)`. Handling consumes $g_h$ per operation, independent
of its duration. Waiting drain is $\rho_w=\texttt{wait_energy_per_min}/60$ and
charging gain is $\rho_c=\texttt{charge_per_min}/60$. Initial SOC is
`cfg.start_soc(k)`, safety is $S_{min}$, capacity is 100, and the X clearance gap
is $b=2\,\texttt{buffer_seconds}$.

An incoming arc to a task travels from the predecessor's final location to its
pickup. Its internal path loads, travels to dropoff, and unloads. An incoming
arc to a charge visit travels to X and then H; the internal operation queues at
H, travels to its chosen bay, charges, and immediately returns to H. The next
arc from H includes H→X. Thus a task beginning after a charge includes that
crossing in its own travel, exactly as in `Planner.task`.

## Decision variables

* $x_{ik}\in\{0,1\}$ selects robot $k$ for task $i$ (`x_i_k`).
* $a_{kab}\in\{0,1\}$ selects a predecessor/successor arc (`arc_k_a_b`).
* $y_{kq}$ selects a charging visit; $b_{kqp}$ selects its bay.
* $s_i,c_i$ are task start and completion (`s_i`, `c_i`). Each robot copy or
  charging vertex has `start`, `ready` and `end`; ready means pickup arrival
  for a task and H arrival for a charge.
* $e_{kr}$ is a selected crossing's entry time (`e_kr`). Its exit is the affine
  expression $l_{kr}=e_{kr}+\tau_r$; no independent exit variable is needed.
  $w_{kr}\geq0$ is its queue time (`w_cross`).
* $u_{kq},D_{kq},w^C_{kq}$ are charging start, duration and H queue time
  (`p_k_q`, `d_k_q`, `queue_k_q`). `target_k_q` is SOC after charging.
* $soc_{kn}$ comprises `arrival_soc_k_n` and `soc_k_n`, respectively the SOC
  after incoming travel and after internal service. Arc predecessors supply
  the SOC at job start.
* $T_i\geq0$ is tardiness and $C_{max}$ is final delivery completion.

## Constraints

**Assignment and robot order.** $\sum_{k:i\in I_k}x_{ik}=1$. Each robot uses
`AddCircuit` with a dummy depot, optional-vertex self loops for absence and a
depot self loop for an idle robot. Arcs returning to 0 end the sequence and do
not imply physical travel. Positive travel durations and temporal precedence
exclude disconnected nontrivial circuits. Charge vertices cannot terminate a
sequence. Consecutive charge visits are allowed and include the actual H→X→H
travel between visits. Charge slots are ordered and prefix-present to eliminate visit-label symmetry.

For a selected $a\to n$, $s_{kn}\geq c_{ka}$. Task copies share the assigned
$s_i,c_i$, and $s_i\geq r_i$. Slack between jobs is powered-down standby with
zero consumption. Deliberately postponing a task before travel is allowed.
Travel toward a pickup never precedes that task's recorded release/start.

**Travel and intersection.** Walk each fixed path from its start. Ordinary
operations advance time by their duration; an X entry adds an explicit wait:

$$e_{kr}=\mathrm{arrival}_{kr}+w_{kr},\qquad l_{kr}=e_{kr}+\tau_r.$$

Optional intervals $[e_{kr},l_{kr}+b)$, present exactly when their arc or task
is selected, share one `NoOverlap`. This imposes at least $b$ between one
crossing's exit and the next entry. All entries WA→X, WB→X and H→X are included,
with their own edited-map durations. The wait is at the incoming source, never
inside X. Internal travel has equality timing; arbitrary slack cannot become
free waiting inside a task. Inactive crossing times and waits are zero.

**Charging.** If bay $p$ is selected, with $h$ the H arrival and $S_H$ its SOC:

$$u_{kq}=h+w^C_{kq}+\tau_{Hp},\quad
 S^-_{kq}=S_H-\rho_w w^C_{kq}-g_{Hp}\geq S_{min},$$
$$S^+_{kq}=S^-_{kq}+\rho_cD_{kq}\leq100,\quad
 c_{kq}=u_{kq}+D_{kq}+\tau_{pH},\quad
 soc_{kq}=S^+_{kq}-g_{pH}\geq S_{min}.$$

$\sum_p b_{kqp}=y_{kq}$. Each bay has `NoOverlap` over optional
$[u_{kq},u_{kq}+D_{kq})$ intervals. The departure H edge starts exactly at charge
completion, so bay occupancy equals charging duration. Active visits charge for
at least one time tick. There is no time-indexed $z_{kph}$ variable and no fixed
95% target constraint.

**Energy and safety.** For every selected path segment:

$$soc_{out}=soc_{in}-\sum g_{ab}-\sum g_h-\rho_w\sum w.$$

SOC variables stay in $[S_{min},100]$, with a separate lower bound immediately
before charging. Between replenishments SOC only decreases, so checking the
ends of incoming travel and internal service checks every intermediate action
as well. Initial SOC is robot-specific. Standby costs zero. Offline scheduling
does not require spare energy after the final delivery. Online terminal tasks
add enough reserve to travel to a bay, including the heuristic's conservative
queue allowance for three other full recharges; decoded online candidates also
undergo `Planner.ensure_reserve`. This is stronger than physical safety alone
and can sacrifice short-term objective quality to keep later solves usable.

Frozen reservations become mandatory intervals in subsequent online models.
Frozen robot terminal time, location and SOC form the new depot state.

## Objective and integer approximation

The objective is the simulator's normalised objective. Each raw term is divided
by its value $R_k$ in the FCFS schedule of the same instance (with floors
$f=(1\,\mathrm{s}, 60\,\mathrm{s}, 60\,\mathrm{s}, 1\,\%)$) and weighted equally:

$$\min J=\tfrac14\left(\frac{C_{max}}{\hat R_C}+\frac{\sum_i p_iT_i}{\hat R_T}
+\frac{\sum w_{kr}+\sum w^C_{kq}}{\hat R_W}+\frac{E}{\hat R_E}\right),
\qquad \hat R_k=\max(R_k,f_k),$$
$$T_i=\max(0,c_i-d_i),\qquad C_{max}=\max_i c_i.$$

The references are constants computed before the solve, so $J$ stays linear.
FCFS scores 1 unless one of its terms is below the floor. Without normalisation
the priority-weighted tardiness (tens of thousands of seconds under load) would
dominate the energy and waiting terms by orders of magnitude. Rolling re-plans
use online references: FCFS on the tasks known at that epoch, continued from the
current partial plan, so no coefficient depends on unreleased tasks. Only the
final, retrospective score uses the full-instance references.

$E$ counts positive edge, handling and queue consumption, never replenishment.
`AddMaxEquality` models tardiness and makespan. Optional resource intervals,
arc-conditioned equalities, bay-conditioned charging equations and absent-zero
waits avoid products of two decision variables and avoid hand-chosen big-M
constraints. Frozen objective contributions are constants; each rolling solve
includes completed deliveries in its makespan and lateness calculations.

The time grid is 1 ms (`TIME=1000`); SOC has 60,000,000 units per percentage point
(`SOC`). Edge/handling durations and release times round up, deadlines round
down, consumption and wait-drain coefficients round up, and charging gain and
initial SOC round down. Many default-map instances are exact on these grids.
Charge duration is an integer number of ticks; SOC target is a decision, so
charge amount is not restricted to integer percentages.

On paths with non-grid edge/handling times, each crossing and charging approach
also reserves 20 time ticks of drain as a conservative energy guard. A path has
fewer than 20 rounded operations before its next reserved event. This covers
the additional queue energy when the continuous simulator arrives slightly
earlier than the rounded model. These guards enter modeled $E$ as well. They
are **not** fabricated simulator actions. Arbitrary fractional parameters can
still make decoding differ from the model; independent validation and fallback
are mandatory, not an assumption of exact correspondence.

The solver objective is expressed directly in J units using floating objective
coefficients, which OR-Tools scales internally. Times and SOC constraints are
integer. Small bound/optimality tolerances from objective scaling should not be
interpreted as a continuous-time proof. `OPTIMAL` means optimal for the finite
charge-slot, finite-horizon integer model. It is not a global proof for unlimited
charging or arbitrary continuous times. The tiny test family separately proves
physical lower bounds: every delivery costs at least 50 seconds and 3 SOC, so
four balanced tasks need $C_{max}\geq50$ and five or six need $C_{max}\geq100$
(with three tasks on a loop, one of its two robots performs two deliveries),
and $E\geq3n$. The constructed chains attain both bounds without extra travel,
charging, waiting or tardiness, so they are optimal for any positive weights; the
tests check these raw terms and that the proven bound equals the objective.

## Warm start, decoding and result metadata

`plan_tasks` runs the unchanged joint greedy rule from the current Planner state.
Its independently validated physical schedule is retained immediately as an
incumbent outside CP-SAT. The complete assignment, visit order, bays, times,
crossings and SOC are replayed on the integer grid to supply `AddHint` values,
including inactive variables. Grid replay adjusts queue and charging duration
consistently. A hint is guidance, not a fixed assignment: all modeled decisions
remain free. Tests fix the entire projected hint and check its feasibility on a
fractional map. If a hint cannot be projected or the heuristic fails, the model
can still solve without it. Presolve can consume a very short budget before
CP-SAT registers the hint; the external physical incumbent still exists.

The decoder follows selected circuits. It uses `Planner.wait(..., 'idle')`,
`Planner.task` and `Planner.charge`. Per-robot crossing hints provide earliest X
entry times; charging adds explicit bay, target SOC and earliest charge start.
Actual edge durations and energy always come from Planner and the provided map.
The existing reservation allocator retains its conflict checks. The shared
`build_result` produces the same schema and calls `validate_result`.

Invalid decoded candidates are rejected. If the original physical joint hint
has lower J than the decoded candidate, it is retained. Thus an unrestricted
offline run never returns worse J than a feasible joint hint on the same input.
If neither candidate nor hint is valid, `InputError` reports failure; no invalid
schedule is returned. A failure is not proof that the physical instance is
infeasible. Heuristic reserve failure does not prevent CP-SAT from trying.

`solver` includes raw CP-SAT `status`, `objective` and `best_bound` in J units,
relative `gap`, CP-SAT `wall_time`, `deterministic_time`, `num_solves`, seed,
workers, hint objective and fallback reason. `decoded_objective`,
`objective_difference` (returned J minus solver J) and `decoded_differs` describe
the **returned** schedule, including fallback. Missing solver incumbents use
`null`, never a fabricated solver objective. Raw `OPTIMAL` plus a fallback does
not claim that the returned physical schedule is optimal.

Rolling results include all solve records under `replans`, each with its own
online normalisation. The top-level `objective` is therefore the final schedule's
full-instance J, not a solver value, and `decoded_differs`/`objective_difference`
are `null`. There is no single global online bound or gap: those fields are `null`. Local bounds remain in individual solve
records. `fallback_count` distinguishes local fallbacks from failure of the run.

## Rolling algorithm

The scheduler sees only tasks released by the current epoch. Future task data
is used solely to advance the clock when no known task exists, not to select,
assign or rank work. Task execution and charge visits are nonpreemptive.

```text
committed := initial Planner; pending := input stream; t := 0
while unfinished tasks remain:
    known := released by t and not committed
    if known empty: advance to the next release-aligned epoch; continue
    if all compatible robots are busy through the next epoch: advance; continue
    idle free robots until t
    capacity per loop := sum(max(0, t + horizon - robot.available_time))
    window := known tasks in due/priority/id order within estimated capacity
              (include at least one task per nonempty loop for progress)
    incumbent := joint heuristic from committed state, using window only
    solve joint CP-SAT with frozen states and fixed resource reservations
    decode, validate, check terminal reserve; retain the better safe candidate
    commit each robot's prefix of visits whose start < t + replan_seconds
    freeze each selected visit in full, even if it ends after the next epoch
    remove committed tasks; record audit; t += replan_seconds
```

Work estimates count handling plus pickup-to-dropoff travel. The horizon is a
capacity budget, **not** a hard completion cutoff; positioning, queues and
charging can extend completion beyond it. A long individual task is never
starved solely by the window. Unstarted tasks after the commitment boundary are
reoptimized. A charge committed now does not force commitment of a task whose
start lies beyond that boundary. Every future reservation belonging to a
committed visit remains fixed, including reservations far beyond the next tick.

Each audit entry records `replan_time`, `known_tasks`, `optimized_tasks`,
`committed_tasks`, visit commitments and copies of future frozen reservations.
Tests check release visibility, that commitments begin within the next epoch,
that frozen reservations appear unchanged in the final result, and that adding
an unreleased order does not change earlier decisions.

## Complexity, limitations and validation

With $n_k$ compatible tasks and $q_k$ charge slots, each robot has
$O((n_k+q_k)^2)$ arc literals and optional crossing intervals. Fixed loop paths
have bounded length. The shared intersection and bays couple both loops.
Assignment/order/resource scheduling is combinatorial; 40–60 tasks frequently
remain far from a proof within short budgets. A large number of charge slots
increases symmetry and model size. 200 tasks are accepted by the input schema
but may be expensive to model or solve; model construction is outside the solve
time limit.

Rolling replanning may be worse than an offline heuristic: it cannot use future
orders, waits for periodic decision epochs, preserves entire committed visits,
limits its workload window, and imposes a terminal reserve. The offline
optimizer promises dominance over its feasible joint hint, not over FCFS or
deadline. No model adds exclusivity to ordinary loop edges, matching simulator
semantics. Charging is linear, traffic is deterministic, and routes are fixed.

Run both compatibility and optional suites:

```sh
python3 -m unittest discover -s tests
.venv/bin/python -m unittest discover -s tests
.venv/bin/python scripts/benchmark_optimizer.py
```

The optional tests cover seed/task-count/bay/SOC/safety combinations, edited maps,
hint dominance, proven tiny optima, optimized partial charging, heuristic
failure, grid-hint feasibility, short-budget fallback, online audit and frozen
reservations, future-order noninterference and cross-process determinism.

The design uses the circuit/transition and optional-interval patterns described
in the official [OR-Tools scheduling guide](https://github.com/google/or-tools/blob/stable/ortools/sat/docs/scheduling.md)
and [transition scheduling example](https://github.com/google/or-tools/blob/stable/examples/contrib/scheduling_with_transitions_sat.py).
The [Python API reference](https://or-tools.github.io/docs/pdoc/ortools/sat/python/cp_model)
documents the circuit, hint and interval primitives. Simulator behavior and the
implementation in this repository determine the specific constraints above.

Repeated-seed results for every strategy are produced by `experiments.py` (see `results/`).
