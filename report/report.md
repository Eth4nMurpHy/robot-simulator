# Joint Scheduling of Intersection Access and Shared Charging Resources for Delivery Robots in a Dual-Loop Network

EE6228 course project (topics M02 + M04). Pure-software 2-D online simulation.

## 1. Case introduction

Two delivery zones, A and B, are each served by a closed one-way loop. The loops meet at a single
mutually exclusive intersection X, and both zones share one charging area (entrance H, bays C1 and C2)
that can only be reached through X. Four virtual robots work in the network: R1 and R2 serve zone A,
R3 and R4 serve zone B. Pick-up-and-delivery orders arrive over time, each with a release time,
pick-up station, drop-off station, handling times, a due time and a priority.

The two shared resources are coupled. A robot that needs to charge must cross X twice (out and back),
so intersection congestion delays charging; a charging queue in turn shifts every later intersection
request of that robot and of the robots behind it. Batteries make the problem harder still: a robot may
only accept an order if, after completing it, it can still reach a charger without falling below the
safety threshold. The project studies how to coordinate **task assignment, task order, intersection
time slots, charger choice and charging duration** so that the fleet stays safe and on time under
high load.

Scope: self-designed map and data; robots move along fixed directed routes (no continuous kinematics
or obstacle avoidance); battery, handling and charging are software states. The emphasis is on
scheduling and resource conflicts.

## 2. System description

### 2.1 Network

Each loop is the directed cycle $X \to Z_1 \to Z_2 \to Z_3 \to Z_4 \to W_Z \to X$ for
$Z \in \{A, B\}$. $W_A$ and $W_B$ are the only places where a robot may wait for the intersection.
Every edge that enters X ($W_A \to X$, $W_B \to X$, $H \to X$) is a *crossing*; X can hold at most
one robot, and two consecutive crossings must be separated by a safety buffer on each side.
The charging branch is $X \to H \to C_p \to H \to X$; robots queue at H, never inside X or an
occupied bay, and leave the bay as soon as charging ends.

Because a robot travels around its loop, every lap passes through X: the intersection is not only on
the way to the chargers but on every delivery whose drop-off lies "behind" its pick-up, and on every
empty repositioning that wraps around the loop.

### 2.2 Default parameters

| Parameter | Base setting | Tested range |
|---|---|---|
| Robots | 4 (2 per loop) | — |
| Orders | 40 | 20, 40, 60 |
| Initial SOC | uniform 50–100 % per robot (seeded) | — |
| Safety threshold | 10 % | 10, 20, 30 % |
| Ordinary edge | 20 m, 20 s, 2 % | editable lengths |
| Crossing into X | 6 s | 5–8 s |
| Safety buffer | 2 s each side (4 s between crossings) | 1–4 s |
| Waiting drain | 0.2 %/min | 0–0.5 %/min |
| Load / unload | 10–20 s each, 0.5 % each | — |
| Charging | linear, 5 %/min | 3–8 %/min |
| Charging bays | 2 | 1, 2 |
| Due time | release + 8–15 min (tight: 4–8 min) | — |
| Priority | 1 (50 %), 2, 3 | urgent orders 4–5 |
| Zone A share | 50 % | 1:1, 2:1, 3:1 |
| Urgent orders | 0 | +10 %, +20 % (due in 3–6 min) |

Orders are released uniformly over $[0, 12n)$ seconds for $n$ orders. The generator is seeded, and
urgent orders and initial SOC come from separate random streams, so adding urgent orders never changes
the base workload of the same seed. All strategies are evaluated on the identical instance.

### 2.3 Objective

The proposal's objective adds terms with different units: seconds of makespan, seconds of
priority-weighted lateness, seconds of queueing and percentage points of energy. Under load the
lateness term reaches tens of thousands of seconds and would swamp everything else. Each term is
therefore normalised by its value in the FCFS schedule of the same instance and the four normalised
terms are weighted equally:

$$J = \tfrac14\left(\frac{C_{\max}}{\hat R_C} + \frac{\sum_i p_i T_i}{\hat R_T}
+ \frac{W_{\text{cross}} + W_{\text{charge}}}{\hat R_W} + \frac{E}{\hat R_E}\right),
\qquad T_i = \max(0, C_i - d_i),\quad \hat R_k = \max(R_k^{\text{FCFS}}, f_k)$$

where $C_{\max}$ is the completion time of the last order (s), $p_i$ the priority, $T_i$ the
tardiness (s), $W$ the total intersection and charger queueing time (s) and $E$ the total energy
consumed (% SOC). The floors $f = (1\,\text{s}, 60\,\text{s}, 60\,\text{s}, 1\,\%)$ only matter when
FCFS has (almost) no lateness or queueing. FCFS therefore scores $J = 1$ (slightly less when one of
its terms is below the floor), and $J = 0.6$ reads as "40 % better than FCFS on this weighted
balance". Battery safety is **not** traded off: falling below the safety threshold is a hard
constraint, checked by an independent validator, so $N_{\text{unsafe}} = 0$ in every reported
schedule.

## 3. Simulator

The simulator is a dependency-free Python web application (standard-library HTTP server, SVG front
end). Its core is a deterministic discrete-event engine with reservation calendars:

- **World and energy model** (`simulator.py`, `customization.py`): directed edges with length,
  travel time and energy; per-action SOC updates for moving, crossing, waiting, handling and charging.
- **Reservation calendars**: one for X (with the $2 \times$ buffer gap) and one per bay. A robot
  requesting a resource receives the earliest feasible slot and the waiting time is recorded as an
  explicit `cross_wait` or `charge_wait` action at an allowed waiting point.
- **Independent validator** (`validate_result`): recomputes from the returned action log alone the
  energy of every action, edge legality, route continuity, pickup-before-delivery, release times,
  action overlap per robot, exclusive and buffered resource use, waiting locations, immediate charger
  departure and SOC bounds. A schedule that fails validation is never returned as a result.
- **Replay and analytics** (`static/`): the 2-D animation reads only the event log (it never
  recomputes timing), plus a Gantt chart, battery curves, reservation timelines for X/C1/C2, a
  delivery table and a strategy comparison. Tasks, schedule, events, robots and summary export to CSV.
- **Command mode**: an external plan (robot, start time, action) can be executed strictly; timing,
  resource and battery conflicts are reported instead of being repaired.

## 4. Scheduling strategies

### 4.1 Baselines

1. **FCFS** — orders in release order; each goes to the compatible robot with the earliest predicted
   completion; a robot below 20 % SOC charges before its next order; intersection slots are granted in
   booking order (first come, first served).
2. **Staged** — the proposal's "assign, then reserve the intersection, then charge" decomposition.
   Stage 1 assigns orders by list scheduling on nominal travel times, ignoring X, chargers and
   battery. Stage 2 executes each robot's fixed sequence with intersection reservations. Stage 3
   inserts a charge only when the next order would otherwise be unsafe.
3. **Deadline + reservations** — earliest due date first (priority breaks ties), with the resource
   reservations and energy checks of the full engine.
4. **Joint greedy** — evaluates up to eight upcoming orders on both compatible robots, including the
   resulting intersection and charger reservations and charging insertions, and picks the candidate
   with the lowest score using the weights of $J$ (with the order's own completion time in place of
   $C_{\max}$).

All automatic strategies keep a battery reserve before accepting an order: the robot must be able to
finish it, reach H and survive the currently booked charger queue plus a conservative allowance for
the other three robots.

### 4.2 CP-SAT joint optimisation

The full formulation is in `docs/optimizer_model.md`; this section summarises it. The key
observation is that routes are fixed, so for every pair of consecutive jobs of a robot the travel
time, energy and the **set of crossings of X** are known in advance. Each robot $k$ gets a graph whose
vertices are a depot (its current state), every order of its loop, and $|Q_k|$ optional charging
visits.

*Decision variables.* $x_{ik}$ assigns order $i$ to robot $k$; arc literals $a_{kab}$ choose the
successor of each vertex; $s_i, c_i$ are order start and completion; for each crossing $r$, the entry
time $e_{kr}$ and the wait $w_{kr}$ in front of X (exit $l_{kr} = e_{kr} + \tau_r$); for each charging
visit $q$ a presence literal, a bay literal $b_{kqp}$, the H queue $w^C_{kq}$, charging start $u_{kq}$
and duration $D_{kq}$ (and thus the charged amount); SOC after incoming travel and after service at
every vertex; tardiness $T_i$ and $C_{\max}$.

*Constraints.*

- **Assignment and order:** $\sum_k x_{ik} = 1$; one `AddCircuit` per robot, with self-loops for
  absent vertices, so each robot follows a single sequence starting at its depot.
- **Timing:** if arc $a \to b$ is chosen, $b$ starts after $a$ ends and its arrival follows the
  fixed path; $s_i \ge r_i$. Idle time between jobs costs no energy, as in the simulator.
- **Intersection:** every crossing on every arc and inside every order is an optional interval
  $[e_{kr}, l_{kr} + 2\beta)$, present only if its arc is chosen. All of them share one `NoOverlap`,
  which enforces both mutual exclusion and the $2\beta$ buffer. The wait $w_{kr}$ happens at the
  incoming waiting point ($W_A$, $W_B$ or H), never inside X.
- **Chargers:** for the chosen bay $p$, the bay interval $[u_{kq}, u_{kq} + D_{kq})$ joins that bay's
  `NoOverlap`; the robot leaves the bay as soon as charging ends. SOC before charging must be
  $\ge S_{\min}$, and SOC after charging, $S^- + \rho_c D_{kq}$, must be $\le 100$. The charged amount
  is a decision, unlike the heuristics' fixed 95 % target.
- **Energy:** along every chosen path SOC decreases by the edge, handling and waiting consumption and
  must stay $\ge S_{\min}$ at every vertex; since SOC only decreases between charges, this checks
  every intermediate action.

Time is scaled to milliseconds and SOC to integer units, with every duration and consumption rounded
**up** so that integer-feasible schedules remain feasible in the exact simulator. The objective is
exactly the normalised $J$ of Section 2.3, whose references are constants computed before the solve.
The model is warm-started with the joint greedy schedule (`AddHint`).

*Decoding.* The solver's choices (sequence, crossing entry times, bay, charging start and target SOC)
are replayed through the simulator's own `Planner`, so times and energy in the returned schedule come
from the simulator, and the result must pass the independent validator. If the decoded schedule is
worse than the warm start or fails validation, the warm start is returned and the fallback is
recorded, so CP-SAT never reports a schedule worse than joint greedy.

*Search.* CP-SAT is run with 8 parallel workers. With a single worker, the solver did not improve the
warm start on 40-order instances even after about two minutes, while the parallel portfolio, whose
large-neighbourhood-search workers do most of the work, improved it substantially within the same
time. On small instances (4–6 orders) the model reaches proven optimality and matches the analytical
lower bound (unit tests). On realistic sizes the lower bound stays weak (large reported gaps), so the
results are good incumbents, not proven optima.

### 4.3 Rolling-horizon re-optimisation

Offline optimisation assumes all orders are known at time 0. The rolling strategy removes that
assumption: an order becomes visible only at its release time.

```
t ← 0
while orders remain:
    known ← orders released by t and not yet started
    window ← known orders, most urgent first, until ≈ 600 s of work per loop
    plan ← CP-SAT on window from the robots' current states,
           with all already-committed X and bay reservations as fixed intervals
           (1 worker, deterministic budget ≈ 0.5 s; warm start = joint greedy on the window)
    commit every job of plan that starts before t + 60 s; freeze its reservations
    t ← t + 60 s
```

Committed actions are never revised. Each window solve additionally requires the last planned job
of every robot to leave enough energy to reach a charger even if the bays are busy, so that the next
window is never left in an energy dead end. Even the normalisation is online: at each re-plan the
references come from an FCFS schedule of only the orders known so far, continued from the robots'
current state. (An earlier version used full-instance references; the review showed that this
changed decisions, i.e. it leaked future information.) The final schedule is scored with the
full-instance $J$ like every other strategy.

## 5. Experiments

*(to be completed from `results/`)*

## 6. Discussion

*(to be completed)*

## 7. Conclusion

*(to be completed)*
