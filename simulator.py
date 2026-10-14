"""Deterministic discrete-event reservation simulator. Times are seconds, SOC percent."""
from __future__ import annotations
import random
import time
from dataclasses import dataclass, asdict
from simulator_types import InputError, number
from customization import LOOPS, parse_map, parse_rules, edge_metrics, ordered_tasks, custom_score

NODES = {'X': [500, 270], 'A1': [365, 165], 'A2': [175, 165],
         'A3': [175, 375], 'A4': [365, 375], 'WA': [440, 325],
         'B1': [635, 375], 'B2': [825, 375], 'B3': [825, 165],
         'B4': [635, 165], 'WB': [560, 215], 'H': [500, 105],
         'C1': [450, 50], 'C2': [550, 50]}
# Weights of the report objective J = Cmax + 3*sum(priority_i*T_i) + 0.5*(W_cross + W_charge) + 2*E.
# Battery safety is a hard constraint enforced by the validator, never a weighted term.
OBJECTIVE_WEIGHTS = {'makespan': 1, 'tardiness': 3, 'wait': .5, 'energy': 2}
STRATEGIES = {'fcfs': 'First come, first served', 'deadline': 'Deadline + reservations',
              'joint': 'Joint greedy scheduling'}
OPTIMIZER_STRATEGIES = {'cpsat': 'CP-SAT joint optimisation',
                        'rolling': 'Rolling-horizon CP-SAT'}


@dataclass
class Config:
    chargers: int = 2
    safety_soc: float = 10
    initial_soc: float = 65
    edge_seconds: float = 20
    edge_energy: float = 2
    wait_energy_per_min: float = .2
    handling_energy: float = .5
    charge_per_min: float = 5
    crossing_seconds: float = 6
    buffer_seconds: float = 2
    charge_target: float = 95
    robot_soc: list | None = None  # optional per-robot initial SOC for R1-R4; overrides initial_soc

    def start_soc(self, index):
        return self.robot_soc[index] if self.robot_soc else self.initial_soc

    @classmethod
    def parse(cls, data):
        if not isinstance(data, dict):
            raise InputError('config must be an object.')
        bounds = {'chargers': (1, 2), 'safety_soc': (5, 30), 'initial_soc': (40, 100),
                  'edge_seconds': (5, 60), 'edge_energy': (.1, 4),
                  'wait_energy_per_min': (0, .5), 'handling_energy': (0, 1),
                  'charge_per_min': (3, 8), 'crossing_seconds': (5, 8),
                  'buffer_seconds': (1, 4), 'charge_target': (80, 100)}
        unknown = set(data) - set(bounds) - {'robot_soc'}
        if unknown:
            raise InputError('Unknown config fields: ' + ', '.join(sorted(unknown)))
        kwargs = {k: number(v, k, *bounds[k]) for k, v in data.items() if k != 'robot_soc'}
        if data.get('robot_soc') is not None:
            socs = data['robot_soc']
            if not isinstance(socs, list) or len(socs) != 4:
                raise InputError('robot_soc must list four initial SOC values for R1-R4.')
            kwargs['robot_soc'] = [number(v, f'robot_soc[{i}]', 40, 100) for i, v in enumerate(socs)]
        if 'chargers' in kwargs:
            if kwargs['chargers'] != int(kwargs['chargers']):
                raise InputError('chargers must be 1 or 2.')
            kwargs['chargers'] = int(kwargs['chargers'])
        return cls(**kwargs)


def validate_tasks(rows):
    if not isinstance(rows, list) or not 1 <= len(rows) <= 200:
        raise InputError('Provide between 1 and 200 tasks.')
    out, seen = [], set()
    for i, row in enumerate(rows):
        if not isinstance(row, dict):
            raise InputError(f'Task {i + 1} must be an object.')
        tid = str(row.get('task_id', '')).strip()
        if not tid or len(tid) > 64 or tid in seen:
            raise InputError(f'Task {i + 1}: task_id must be unique and 1–64 characters.')
        seen.add(tid)
        pickup, dropoff = row.get('pickup'), row.get('dropoff')
        valid = {f'{area}{j}' for area in 'AB' for j in range(1, 5)}
        if not isinstance(pickup, str) or not isinstance(dropoff, str) or pickup not in valid or dropoff not in valid:
            raise InputError(f'{tid}: pickup and dropoff must be A1–A4 or B1–B4.')
        if pickup[0] != dropoff[0] or pickup == dropoff:
            raise InputError(f'{tid}: use distinct pickup/dropoff stations on the same loop.')
        task = dict(task_id=tid, pickup=pickup, dropoff=dropoff)
        for key, default, low, high in [('release_time', 0, 0, 86400),
                                      ('load_time', 15, 0, 300), ('unload_time', 15, 0, 300),
                                      ('due_time', None, 0, 100000), ('priority', 1, 1, 5)]:
            value = row.get(key, default)
            if value is None:
                raise InputError(f'{tid}: {key} is required.')
            task[key] = number(value, f'{tid}: {key}', low, high)
        if task['due_time'] < task['release_time']:
            raise InputError(f'{tid}: due_time cannot precede release_time.')
        out.append(task)
    return out


def generate_tasks(count=40, seed=42, a_share=.5):
    count = number(count, 'count', 1, 200)
    seed = number(seed, 'seed', 0, 2**32 - 1)
    if int(count) != count or int(seed) != seed:
        raise InputError('count and seed must be integers.')
    a_share = number(a_share, 'a_share', 0, 1)
    rng = random.Random(int(seed))
    tasks = []
    for i in range(int(count)):
        area = 'A' if rng.random() < a_share else 'B'
        pickup, dropoff = rng.sample(range(1, 5), 2)
        release = rng.randrange(0, max(1, int(count) * 12), 10)
        tasks.append(dict(task_id=f'T{i + 1:03}', release_time=release,
                          pickup=f'{area}{pickup}', dropoff=f'{area}{dropoff}',
                          load_time=rng.choice([10, 15, 20]), unload_time=rng.choice([10, 15, 20]),
                          due_time=release + rng.randint(480, 900), priority=rng.choice([1, 1, 2, 3])))
    return sorted(tasks, key=lambda t: (t['release_time'], t['task_id']))


def objective(tasks, makespan, waits, energy, weights=OBJECTIVE_WEIGHTS):
    """Scalar objective J used to compare every strategy, including the optimizers."""
    return (weights['makespan'] * makespan + weights['wait'] * waits + weights['energy'] * energy
            + weights['tardiness'] * sum(t['priority'] * t['tardiness'] for t in tasks))


def random_robot_soc(seed=42, low=50, high=100):
    """Reproducible per-robot initial SOC, independent of the task stream for the same seed."""
    seed = number(seed, 'seed', 0, 2**32 - 1)
    rng = random.Random(f'soc-{int(seed)}')
    return [round(rng.uniform(low, high), 1) for _ in range(4)]


def slot(calendar, earliest, duration, gap=0):
    t = earliest
    for reservation in sorted(calendar, key=lambda x: x['start_time']):
        a, b = reservation['start_time'], reservation['end_time']
        if t + duration + gap <= a + 1e-9:
            break
        if t < b + gap - 1e-9:
            t = b + gap
    return t


class Planner:
    def __init__(self, config, world=None, rules=None):
        self.c = config
        self.world = parse_map(world, config)
        self.rules = rules
        self.robots = [dict(robot_id=f'R{i + 1}', loop=area, node=f'{area}1',
                            time=0., soc=config.start_soc(i)) for i, area in enumerate('AABB')]
        self.calendars = {'X': [], **{f'C{i + 1}': [] for i in range(config.chargers)}}
        self.actions = []
        self.results = []

    def clone(self):
        # Completed records are immutable; copy only mutable planner containers.
        plan = object.__new__(Planner)
        plan.c = self.c
        plan.world = self.world
        plan.rules = self.rules
        plan.robots = [dict(r) for r in self.robots]
        plan.calendars = {k: list(v) for k, v in self.calendars.items()}
        plan.actions = list(self.actions)
        plan.results = list(self.results)
        return plan

    def action(self, robot, action, duration, energy=0., to=None, task_id='', resource=''):
        node = robot['node']
        dest = to or node
        start_soc = robot['soc']
        robot['soc'] -= energy
        if robot['soc'] < self.c.safety_soc - 1e-8:
            raise InputError(f"{robot['robot_id']} would fall below the safety threshold. Increase initial SOC/charging speed, or reduce energy demand.")
        item = dict(robot_id=robot['robot_id'], task_id=task_id, action=action,
                    node=dest, from_node=node, to_node=dest, start_time=robot['time'],
                    end_time=robot['time'] + duration, start_soc=start_soc,
                    end_soc=robot['soc'], energy=energy, resource=resource,
                    distance_m=edge_metrics(self.world, node, dest)[2] if action in ('move', 'cross') else 0)
        self.actions.append(item)
        robot.update(time=item['end_time'], node=dest)
        return item

    def wait(self, r, until, kind, tid=''):
        if until > r['time'] + 1e-9:
            duration = until - r['time']
            self.action(r, kind, duration, 0 if kind == 'idle' else duration / 60 * self.c.wait_energy_per_min, task_id=tid)

    def crossing(self, r, tid):
        duration, energy, _ = edge_metrics(self.world, r['node'], 'X')
        earliest = r['time']
        hints = getattr(self, 'crossing_hints', {}).get(r['robot_id'], [])
        if hints:
            earliest = max(earliest, hints.pop(0))
        start = slot(self.calendars['X'], earliest, duration, 2 * self.c.buffer_seconds)
        self.wait(r, start, 'cross_wait', tid)
        item = self.action(r, 'cross', duration, energy,
                           to='X', task_id=tid, resource='X')
        self.calendars['X'].append(item)

    def move_edge(self, r, dest, tid):
        if dest == 'X':
            self.crossing(r, tid)
        else:
            duration, energy, _ = edge_metrics(self.world, r['node'], dest)
            self.action(r, 'move', duration, energy, dest, tid)

    def travel(self, r, dest, tid=''):
        loop = LOOPS[r['loop']]
        if r['node'].startswith('C'):
            self.move_edge(r, 'H', tid)
            self.move_edge(r, 'X', tid)
        if r['node'] == 'H':
            self.move_edge(r, 'X', tid)
        while r['node'] != dest:
            nxt = loop[(loop.index(r['node']) + 1) % len(loop)]
            self.move_edge(r, nxt, tid)

    def charge(self, r, bay_choice=None, target_soc=None, earliest_start=None):
        self.travel(r, 'X')
        self.move_edge(r, 'H', '')
        # Queue at H (outside the bays); account for the travel H -> bay.
        target = self.c.charge_target if target_soc is None else number(target_soc, 'target_soc', self.c.safety_soc, 100)
        if bay_choice is not None and bay_choice not in self.calendars.keys() - {'X'}:
            raise InputError('Unavailable charging bay.')
        rate = self.c.charge_per_min / 60
        drain = self.c.wait_energy_per_min / 60
        choices = []
        for bay, calendar in self.calendars.items():
            if bay == 'X' or (bay_choice is not None and bay != bay_choice):
                continue
            travel_time, travel_energy, _ = edge_metrics(self.world, 'H', bay)
            arrival = r['time'] + travel_time
            arrival_soc = r['soc'] - travel_energy
            if arrival_soc < self.c.safety_soc - 1e-8:
                continue
            duration = max(0, (target - arrival_soc) / rate)
            start = slot(calendar, max(arrival, earliest_start or 0), duration)
            # Wait increases charging duration. Recompute monotonically until stable.
            for _ in range(len(calendar) + 3):
                soc = arrival_soc - (start - arrival) * drain
                duration = max(0, (target - soc) / rate)
                nxt = slot(calendar, start, duration)
                if abs(nxt - start) < 1e-8:
                    break
                start = nxt
            if soc >= self.c.safety_soc - 1e-8:
                choices.append((start + duration, start, duration, bay, travel_time))
        if not choices:
            raise InputError('Insufficient energy to reach and queue for a charger safely.')
        _, start, duration, bay, travel_time = min(choices)
        self.wait(r, start - travel_time, 'charge_wait')
        self.move_edge(r, bay, '')
        item = self.action(r, 'charge', duration, -duration * rate, resource=bay)
        self.calendars[bay].append(item)
        # Vacate the bay immediately; standby must not block an unreserved bay.
        self.move_edge(r, 'H', '')

    def task(self, r, task):
        tid = task['task_id']
        self.wait(r, task['release_time'], 'idle', tid)
        start = r['time']
        self.travel(r, task['pickup'], tid)
        self.action(r, 'load', task['load_time'], self.c.handling_energy, task_id=tid)
        self.travel(r, task['dropoff'], tid)
        self.action(r, 'unload', task['unload_time'], self.c.handling_energy, task_id=tid)
        result = dict(**task, robot_id=r['robot_id'], start_time=start,
                      completion_time=r['time'], tardiness=max(0, r['time'] - task['due_time']))
        self.results.append(result)

    def ensure_reserve(self, robot_index):
        check = self.clone()
        r = check.robots[robot_index]
        check.travel(r, 'X')
        check.move_edge(r, 'H', '')
        # Conservative headroom for three other robots each needing a full
        # recharge. Also test the actual booked calendar below.
        queue_seconds = 3 * (self.c.charge_target - self.c.safety_soc) / self.c.charge_per_min * 60
        check.wait(r, r['time'] + queue_seconds, 'charge_wait')
        bay = min((f'C{i+1}' for i in range(self.c.chargers)), key=lambda b: edge_metrics(self.world, 'H', b)[1])
        check.move_edge(r, bay, '')
        actual = self.clone()
        actual.charge(actual.robots[robot_index])

    def candidate(self, robot_index, task, strategy):
        plan = self.clone()
        r = plan.robots[robot_index]
        # Baseline threshold plus a mandatory safe-completion reserve for all strategies.
        should_charge = (strategy == 'fcfs' and r['soc'] < 20) or (self.rules is not None and r['soc'] < self.rules['charge_below_soc'])
        if not should_charge:
            trial = plan.clone()
            try:
                trial.task(trial.robots[robot_index], task)
                trial.ensure_reserve(robot_index)
                return trial
            except InputError:
                should_charge = True
        if should_charge:
            plan.charge(r)
            plan.task(r, task)
            plan.ensure_reserve(robot_index)
        return plan


def validate_result(result):
    """Independent checks of returned task/action/resource records, not planner state."""
    errors = []
    cfg = result['config']
    world = parse_map(result.get('map'), Config.parse(cfg))
    completed = result['tasks']
    if len({t['task_id'] for t in completed}) != len(completed):
        errors.append('Duplicate task assignment')
    for task in completed:
        actions = [a for a in result['actions'] if a['task_id'] == task['task_id']]
        loads = [a for a in actions if a['action'] == 'load']
        unloads = [a for a in actions if a['action'] == 'unload']
        if len(loads) != 1 or len(unloads) != 1:
            errors.append(f"{task['task_id']}: expected one pickup and delivery")
            continue
        if loads[0]['node'] != task['pickup'] or unloads[0]['node'] != task['dropoff']:
            errors.append(f"{task['task_id']}: wrong station")
        if loads[0]['end_time'] > unloads[0]['start_time'] + 1e-8 or task['start_time'] < task['release_time']:
            errors.append(f"{task['task_id']}: precedence/release violation")
        if any(a['robot_id'] != task['robot_id'] for a in actions):
            errors.append(f"{task['task_id']}: inconsistent robot assignment")
    for resource, calendar in result['reservations'].items():
        ordered = sorted(calendar, key=lambda a: a['start_time'])
        gap = 2 * cfg['buffer_seconds'] if resource == 'X' else 0
        if any((a.get('open_ended', False) or a['end_time'] + gap > b['start_time'] + 1e-8) for a, b in zip(ordered, ordered[1:])):
            errors.append(f'{resource}: overlapping reservations')
    for robot in result['robots']:
        actions = sorted([a for a in result['actions'] if a['robot_id'] == robot['robot_id']], key=lambda a: a['start_time'])
        last_end, soc, node = 0., robot['initial_soc'], robot['start_node']
        for index, a in enumerate(actions):
            if a['start_time'] < last_end - 1e-8 or a['end_time'] < a['start_time']:
                errors.append(f"{robot['robot_id']}: overlapping/reversed actions")
            if abs(a['start_soc'] - soc) > 1e-6 or abs(a['end_soc'] - (a['start_soc'] - a['energy'])) > 1e-6:
                errors.append(f"{robot['robot_id']}: SOC continuity")
            duration = a['end_time'] - a['start_time']
            action = a['action']
            if action in ('move', 'cross') and not any(e['from'] == a['from_node'] and e['to'] == a['to_node'] for e in world['edges']):
                errors.append(f"{robot['robot_id']}: invalid directed edge")
                continue
            if action == 'move':
                edge_time, expected_energy, distance = edge_metrics(world, a['from_node'], a['to_node'])
                if abs(duration - edge_time) > 1e-6 or abs(a.get('distance_m', distance) - distance) > 1e-6:
                    errors.append(f"{robot['robot_id']}: incorrect edge duration")
                loop = LOOPS[robot['loop']]
                source, dest = a['from_node'], a['to_node']
                valid_edge = (source in loop and dest == loop[(loop.index(source) + 1) % len(loop)] and dest != 'X') or (source == 'X' and dest == 'H') or (source == 'H' and dest.startswith('C')) or (source.startswith('C') and dest == 'H')
                if not valid_edge:
                    errors.append(f"{robot['robot_id']}: invalid directed edge")
            elif action == 'cross':
                edge_time, expected_energy, distance = edge_metrics(world, a['from_node'], a['to_node'])
                if a['from_node'] not in ('W' + robot['loop'], 'H') or a['to_node'] != 'X' or abs(duration - edge_time) > 1e-6 or abs(a.get('distance_m', distance) - distance) > 1e-6:
                    errors.append(f"{robot['robot_id']}: invalid crossing")
            elif action in ('cross_wait', 'charge_wait'):
                expected_energy = duration / 60 * cfg['wait_energy_per_min']
                allowed = ('W' + robot['loop'], 'H') if action == 'cross_wait' else ('H',)
                if a['from_node'] not in allowed or a['to_node'] != a['from_node']:
                    errors.append(f"{robot['robot_id']}: invalid queue location")
            elif action == 'charge':
                departure = actions[index + 1] if index + 1 < len(actions) else None
                if result.get('mode') != 'commands' and (departure is None or departure['action'] != 'move' or departure['to_node'] != 'H' or abs(departure['start_time'] - a['end_time']) > 1e-6):
                    errors.append(f"{robot['robot_id']}: charger not vacated immediately")
                expected_energy = -duration / 60 * cfg['charge_per_min']
                if a['from_node'] != a['resource'] or a['to_node'] != a['resource']:
                    errors.append(f"{robot['robot_id']}: charge outside bay")
            elif action in ('load', 'unload'):
                expected_energy = cfg['handling_energy']
                task = next((t for t in completed if t['task_id'] == a['task_id']), None)
                if task is None or abs(duration - task['load_time' if action == 'load' else 'unload_time']) > 1e-6:
                    errors.append(f"{robot['robot_id']}: incorrect handling duration")
            elif action == 'wait' and result.get('mode') == 'commands':
                expected_energy = duration / 60 * cfg['wait_energy_per_min']
                if a['from_node'] != a['to_node'] or a['from_node'] == 'X':
                    errors.append(f"{robot['robot_id']}: invalid wait location")
            elif action == 'idle':
                expected_energy = 0
            else:
                expected_energy = None
                errors.append(f"{robot['robot_id']}: unknown action")
            if expected_energy is not None and abs(expected_energy - a['energy']) > 1e-6:
                errors.append(f"{robot['robot_id']}: incorrect energy model")
            booked = a in result['reservations'].get(a['resource'], [])
            if action == 'charge' and result.get('mode') == 'commands':
                booked = any(b['robot_id'] == a['robot_id'] and b['start_time'] <= a['start_time'] + 1e-8 and b['end_time'] >= a['end_time'] - 1e-8 for b in result['reservations'].get(a['resource'], []))
            if action in ('cross', 'charge') and not booked:
                errors.append(f"{robot['robot_id']}: missing resource reservation")
            if a['from_node'] != node:
                errors.append(f"{robot['robot_id']}: route continuity")
            if min(a['start_soc'], a['end_soc']) < cfg['safety_soc'] - 1e-6 or max(a['start_soc'], a['end_soc']) > 100 + 1e-6:
                errors.append(f"{robot['robot_id']}: unsafe SOC")
            last_end, soc, node = a['end_time'], a['end_soc'], a['to_node']
    return list(dict.fromkeys(errors))


def simulate(tasks, config=None, strategy='joint', world=None, custom_strategy=None):
    if isinstance(strategy, str) and strategy in OPTIMIZER_STRATEGIES:
        try:
            from optimizer import optimize
        except ImportError as exc:
            raise InputError('CP-SAT strategies require OR-Tools; run with .venv/bin/python or install ortools.') from exc
        return optimize(tasks, config, strategy, world, custom_strategy)
    if strategy == 'custom':
        from command_plan import execute_commands
        return execute_commands(tasks, config, world, custom_strategy)
    started = time.perf_counter()
    tasks = validate_tasks(tasks)
    cfg = Config.parse(config or {})
    if not isinstance(strategy, str) or strategy not in (*STRATEGIES, 'rules'):
        raise InputError('Unknown strategy.')
    rules = parse_rules(custom_strategy) if strategy == 'rules' else None
    if rules:
        cfg.charge_target = rules['charge_target']
    plan = Planner(cfg, world, rules)
    plan = plan_tasks(plan, tasks, strategy)
    return build_result(plan, tasks, strategy, started)


def plan_tasks(plan, tasks, strategy='joint'):
    rules = plan.rules
    pending = list(tasks)
    while pending:
        # Joint heuristic chooses among the next 8 released/future jobs; no optimality claim.
        if rules:
            ordered = ordered_tasks(pending, rules)
        elif strategy == 'deadline':
            ordered = sorted(pending, key=lambda t: (t['due_time'], -t['priority'], t['release_time'], t['task_id']))[:1]
        else:
            ordered = sorted(pending, key=lambda t: (t['release_time'], t['task_id']))[:8 if strategy == 'joint' else 1]
        choices, failures = [], []
        for task in ordered:
            for i, robot in enumerate(plan.robots):
                if robot['loop'] != task['pickup'][0]:
                    continue
                try:
                    candidate = plan.candidate(i, task, strategy)
                    done = candidate.results[-1]
                    new = candidate.actions[len(plan.actions):]
                    waits = sum(a['end_time'] - a['start_time'] for a in new if a['action'] in ('cross_wait', 'charge_wait'))
                    energy = sum(max(0, a['energy']) for a in new)
                    distance = sum(a['distance_m'] for a in new)
                    if rules:
                        score = custom_score(task, done, waits, energy, distance, rules)
                    elif strategy == 'joint':
                        score = done['completion_time'] + 3 * task['priority'] * done['tardiness'] + .5 * waits + 2 * energy
                    else:
                        score = done['completion_time']
                    rank = rules['preferred_robots'].index(robot['robot_id']) if rules else robot['robot_id']
                    choices.append((score, task['task_id'], rank, candidate))
                except InputError as exc:
                    failures.append(str(exc))
        if not choices:
            raise InputError('No energy-safe schedule found for the next tasks. ' + (failures[0] if failures else 'Check task configuration.'))
        _, tid, _, plan = min(choices, key=lambda x: x[:3])
        pending = [t for t in pending if t['task_id'] != tid]
    return plan


def build_result(plan, tasks, strategy, started):
    cfg, rules = plan.c, plan.rules
    actions = sorted(plan.actions, key=lambda a: (a['start_time'], a['robot_id']))
    makespan = max(t['completion_time'] for t in plan.results)
    cross_wait = sum(a['end_time'] - a['start_time'] for a in actions if a['action'] == 'cross_wait')
    charge_wait = sum(a['end_time'] - a['start_time'] for a in actions if a['action'] == 'charge_wait')
    occupied = lambda key: sum(a['end_time'] - a['start_time'] for a in plan.calendars[key])
    result = dict(strategy=strategy, strategy_label=rules['name'] if rules else {**STRATEGIES, **OPTIMIZER_STRATEGIES}[strategy], config=asdict(cfg), nodes=NODES,
                  map=plan.world, custom_strategy=rules,
                  robots=[dict(robot_id=r['robot_id'], loop=r['loop'], start_node=f"{r['loop']}1", initial_soc=cfg.start_soc(i)) for i, r in enumerate(plan.robots)],
                  tasks=sorted(plan.results, key=lambda t: (t['release_time'], t['task_id'])), actions=actions,
                  reservations={k: sorted(v, key=lambda a: a['start_time']) for k, v in plan.calendars.items()})
    events = []
    for a in actions:
        for boundary in ('start', 'end'):
            events.append(dict(time=a[f'{boundary}_time'], robot_id=a['robot_id'],
                               event=a['action'] + '_' + boundary, node=a['from_node'] if boundary == 'start' else a['to_node'],
                               task_id=a['task_id'], soc=a[f'{boundary}_soc'], resource=a['resource'],
                               wait_time=a['end_time'] - a['start_time'] if boundary == 'end' and a['action'].endswith('_wait') else 0))
    result['events'] = sorted(events, key=lambda e: (e['time'], e['robot_id']))
    errors = validate_result(result)
    result['validation'] = dict(passed=not errors, errors=errors, checks=['Task uniqueness and precedence', 'Exclusive intersection with safety buffers', 'Exclusive charging bays', 'Robot action and SOC continuity', 'Battery safety'])
    result['summary'] = dict(makespan=makespan, completed=len(tasks),
        late_tasks=sum(t['tardiness'] > 1e-8 for t in plan.results),
        tardiness_rate=sum(t['tardiness'] > 1e-8 for t in plan.results) / len(tasks),
        total_tardiness=sum(t['tardiness'] for t in plan.results), cross_wait=cross_wait, charge_wait=charge_wait,
        average_wait=(cross_wait + charge_wait) / len(tasks),
        energy=sum(max(0, a['energy']) for a in actions),
        distance_m=sum(a['distance_m'] for a in actions),
        min_soc=min(a['end_soc'] for a in actions), violations=len(errors),
        cross_utilization=occupied('X') / makespan,
        charge_utilization=sum(occupied(f'C{i + 1}') for i in range(cfg.chargers)) / (makespan * cfg.chargers),
        charging_sessions=sum(a['action'] == 'charge' for a in actions), runtime=time.perf_counter() - started)
    result['summary']['objective'] = objective(plan.results, makespan, cross_wait + charge_wait, result['summary']['energy'])
    if errors:
        raise RuntimeError('Schedule validation failed: ' + '; '.join(errors))
    return result
