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

Following the proposal, all strategies are compared on one scalar objective

$$J = w_1 C_{\max} + w_2 \sum_i p_i T_i + w_3 (W_{\text{cross}} + W_{\text{charge}}) + w_4 E,
\qquad T_i = \max(0, C_i - d_i)$$

with $w = (1, 3, 0.5, 2)$, where $C_{\max}$ is the completion time of the last order (s), $p_i$ the
priority, $T_i$ the tardiness (s), $W$ the total intersection and charger queueing time (s) and $E$
the total energy consumed (% SOC). Battery safety is **not** traded off: falling below the safety
threshold is a hard constraint, checked by an independent validator, so $N_{\text{unsafe}} = 0$ in
every reported schedule.

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

*(to be completed from `docs/optimizer_model.md`)*

### 4.3 Rolling-horizon re-optimisation

*(to be completed)*

## 5. Experiments

*(to be completed from `results/`)*

## 6. Discussion

*(to be completed)*

## 7. Conclusion

*(to be completed)*
