"""Joint circuit/interval CP-SAT model; all output actions are made by Planner."""
import math
import os
import time
from ortools.sat.python import cp_model
from customization import LOOPS, edge_metrics
from simulator import Config, Planner, build_result, normaliser, objective, objective_terms, plan_tasks, slot, validate_tasks
from simulator_types import InputError, number

TIME = 1000
SOC = 60_000_000
# The optimum drives terminal SOC exactly onto the reserve boundary; the simulator's reserve check also
# drains while queueing for X on the way to H, which the model ignores. 0.5 % (about 150 s of queue
# drain) keeps decoded plans from being rejected by a hair.
RESERVE_MARGIN = .5


def ceil_int(value):
    return math.ceil(value - 1e-7)


def options_for(options, rolling=False):
    if options is None:
        options = {}
    if not isinstance(options, dict):
        raise InputError('Optimizer options must be an object.')
    # Offline CP-SAT relies on parallel LNS workers; one worker barely improves the warm start on
    # 40+ tasks. Rolling solves stay single-worker and deterministic-time bounded (reproducible).
    # Rolling: the deterministic budget binds (0.1 per solve, about 0.5 s); the 30 s wall limit is only a
    # safety cap, so results do not depend on machine speed or load.
    defaults = dict(time_limit=30 if rolling else 60, workers=1 if rolling else min(8, os.cpu_count() or 1), seed=0,
                    replan_seconds=60, horizon_seconds=600,
                    max_charges_per_robot=None, deterministic_time=.1 if rolling else None,
                    # Same end-of-horizon battery reserve the heuristics keep (reach a charger even behind a
                    # full queue). Without it offline CP-SAT drains batteries to the threshold and skips charging.
                    terminal_reserve=True)
    if set(options) - set(defaults):
        raise InputError('Unknown optimizer options: ' + ', '.join(sorted(set(options) - set(defaults))))
    out = {**defaults, **options}
    if not isinstance(out['terminal_reserve'], bool):
        raise InputError('terminal_reserve must be true or false.')
    out['terminal_reserve'] |= rolling
    for key, low, high in [('time_limit', .001, 3600), ('workers', 1, 32),
                           ('seed', 0, 2**31-1), ('replan_seconds', .001, 86400),
                           ('horizon_seconds', 1, 100000), ('max_charges_per_robot', 0, 200),
                           ('deterministic_time', .000001, 3600)]:
        if out[key] is None and key in ('max_charges_per_robot', 'deterministic_time'):
            continue
        out[key] = number(out[key], key, low, high)
        if key in ('workers', 'seed', 'max_charges_per_robot'):
            if out[key] != int(out[key]):
                raise InputError(f'{key} must be an integer.')
            out[key] = int(out[key])
    if out['deterministic_time'] is None:
        # Single worker: a deterministic budget makes runs reproducible. Parallel: wall time binds.
        out['deterministic_time'] = out['time_limit'] * .05 if out['workers'] == 1 else 3600
    return out


def route(loop, source, dest):
    edges = []
    if source == 'H':
        edges.append(('H', 'X'))
        source = 'X'
    while source != dest:
        nxt = LOOPS[loop][(LOOPS[loop].index(source) + 1) % len(LOOPS[loop])]
        edges.append((source, nxt))
        source = nxt
    return edges


def jobs_from_plan(plan, base):
    """Recover task and charge visits from the immutable simulator trace."""
    jobs = [[] for _ in range(4)]
    done = {t['task_id']: t for t in plan.results}
    for k, robot in enumerate(plan.robots):
        actions = [a for a in plan.actions[len(base.actions):] if a['robot_id'] == robot['robot_id']]
        for a in actions:
            if a['action'] == 'idle':
                continue
            key = a['task_id'] or None
            previous = jobs[k][-1]['actions'][-1] if jobs[k] else None
            departed = previous and previous['from_node'].startswith('C') and previous['to_node'] == 'H'
            if not jobs[k] or jobs[k][-1]['key'] != key or (key is None and departed):
                jobs[k].append(dict(key=key, actions=[]))
            jobs[k][-1]['actions'].append(a)
        for job in jobs[k]:
            aa = job['actions']
            job['start'] = done[job['key']]['start_time'] if job['key'] else aa[0]['start_time']
            job['end'] = aa[-1]['end_time']
            job['task'] = done.get(job['key'])
    return jobs


class JointModel:
    def __init__(self, base, tasks, hint, opts, rolling=False):
        self.base, self.tasks, self.opts = base, tasks, opts
        self.cfg, self.world = base.c, base.world
        self.model = m = cp_model.CpModel()
        self.hints = {}
        self.vars = []
        self.resources = {r: [] for r in base.calendars}
        self.energy, self.waits, self.nodes, self.arcs = [], [], [], []
        self.seed_maps = []
        self.trace = jobs_from_plan(hint, base) if hint else [[] for _ in range(4)]
        last = max([r['time'] for r in base.robots] + [t['release_time'] for t in tasks])
        # Finite, deliberately generous horizon, independent of a heuristic's success.
        travel = max(edge_metrics(self.world, e['from'], e['to'])[0] for e in self.world['edges'])
        self.limit = ceil_int((last + (len(tasks) + 8) * (20 * travel + 5000)) * TIME)
        self.safe = ceil_int(self.cfg.safety_soc * SOC)
        self.drain = ceil_int(self.cfg.wait_energy_per_min * SOC / (60 * TIME))
        self.rate = math.floor(self.cfg.charge_per_min * SOC / (60 * TIME) + 1e-7)
        self.gap = ceil_int(2 * self.cfg.buffer_seconds * TIME)
        self.fractional = any(abs(edge_metrics(self.world, e['from'], e['to'])[0] * TIME - round(edge_metrics(self.world, e['from'], e['to'])[0] * TIME)) > 1e-6 for e in self.world['edges'])
        self.fractional |= any(abs(t[field] * TIME - round(t[field] * TIME)) > 1e-6 for t in tasks for field in ('load_time', 'unload_time'))
        self.guard = 20 * self.drain if self.fractional else 0
        for resource, calendar in base.calendars.items():
            for a in calendar:
                start = math.floor(a['start_time'] * TIME + 1e-7)
                end = ceil_int(a['end_time'] * TIME) + (self.gap if resource == 'X' else 0)
                self.resources[resource].append(m.new_fixed_size_interval_var(start, end-start, 'frozen'))
        self.starts = [self.var(f's_{i}') for i in range(len(tasks))]
        self.ends = [self.var(f'c_{i}') for i in range(len(tasks))]
        assignments = [[] for _ in tasks]
        for k, robot in enumerate(base.robots):
            nodes = {0: dict(end=ceil_int(robot['time'] * TIME), soc=math.floor(robot['soc'] * SOC + 1e-7), location=robot['node'])}
            mapping = {}
            charge_count = sum(j['key'] is None for j in self.trace[k])
            max_charges = opts['max_charges_per_robot']
            if max_charges is None:
                max_charges = max(charge_count, max(1, sum(t['pickup'][0] == robot['loop'] for t in tasks) // 5))
            for i, task in enumerate(tasks):
                if task['pickup'][0] != robot['loop']:
                    continue
                node = self.node(k, len(nodes), task['dropoff'], task, i)
                assignments[i].append(node['use'])
                mapping[task['task_id']] = len(nodes)
                nodes[len(nodes)] = node
                m.add(self.starts[i] == node['start']).only_enforce_if(node['use'])
                m.add(self.ends[i] == node['end']).only_enforce_if(node['use'])
                m.add(node['start'] >= ceil_int(task['release_time'] * TIME)).only_enforce_if(node['use'])
                internal = [('load', task['load_time'], self.cfg.handling_energy)]
                internal += route(robot['loop'], task['pickup'], task['dropoff'])
                internal += [('unload', task['unload_time'], self.cfg.handling_energy)]
                finish, consumption, crosses, waits = self.path(node['ready'], internal, node['use'])
                m.add(node['end'] == finish).only_enforce_if(node['use'])
                m.add(node['soc'] == node['arrival_soc'] - consumption).only_enforce_if(node['use'])
                node.update(crosses=crosses, waits=waits, internal=internal)
            charge_nodes = []
            for q in range(max_charges):
                idx = len(nodes)
                node = self.node(k, idx, 'H', None, None)
                nodes[idx] = node
                charge_nodes.append(idx)
                node['charge_start'] = self.var(f'p_{k}_{q}')
                node['duration'] = self.var(f'd_{k}_{q}', 0, ceil_int(100 * SOC / self.rate))
                node['target'] = self.var(f'target_{k}_{q}', self.safe, 100 * SOC, self.safe)
                node['queue'] = self.var(f'queue_{k}_{q}')
                self.waits.append(node['queue'])
                self.energy.append(self.drain * node['queue'])
                node['bays'] = {}
                for bay in sorted(self.resources.keys() - {'X'}):
                    use = self.boolean(f'bay_{k}_{q}_{bay}')
                    node['bays'][bay] = use
                    dt, de = self.metrics('H', bay)
                    rt, re = self.metrics(bay, 'H')
                    m.add(node['charge_start'] == node['ready'] + node['queue'] + dt).only_enforce_if(use)
                    before = node['arrival_soc'] - de - self.drain * node['queue'] - self.guard
                    m.add(before >= self.safe).only_enforce_if(use)
                    m.add(node['target'] == before + self.rate * node['duration']).only_enforce_if(use)
                    m.add(node['end'] == node['charge_start'] + node['duration'] + rt).only_enforce_if(use)
                    m.add(node['soc'] == node['target'] - re).only_enforce_if(use)
                    self.energy.append((de + re + self.guard) * use)
                    self.resources[bay].append(m.new_optional_interval_var(node['charge_start'], node['duration'], node['end'] - rt, use, f'bay_interval_{k}_{q}_{bay}'))
                m.add(sum(node['bays'].values()) == node['use'])
                for key in ('charge_start', 'duration', 'queue'):
                    m.add(node[key] == 0).only_enforce_if(~node['use'])
                m.add(node['duration'] >= 1).only_enforce_if(node['use'])
                if q:
                    prev = nodes[charge_nodes[-2]]
                    m.add(prev['use'] >= node['use'])
                    m.add(node['start'] >= prev['end']).only_enforce_if(node['use'])
            arcs, details = [], {}
            empty = self.boolean(f'empty_{k}')
            arcs.append((0, 0, empty))
            for b, nb in nodes.items():
                if b:
                    arcs.append((b, b, ~nb['use']))
            for a, na in nodes.items():
                for b, nb in nodes.items():
                    if a == b:
                        continue
                    if b == 0 and na.get('task') is None:
                        continue  # No final charge visits: automatic makespan ends at delivery.
                    lit = self.boolean(f'arc_{k}_{a}_{b}')
                    arcs.append((a, b, lit))
                    details[a, b] = dict(use=lit)
                    if b == 0:
                        continue
                    m.add(nb['start'] >= na['end']).only_enforce_if(lit)
                    dest = nb['task']['pickup'] if nb['task'] else 'X'
                    edges = route(robot['loop'], na['location'], dest)
                    if nb['task'] is None:
                        edges.append(('X', 'H'))
                    finish, consumption, crosses, waits = self.path(nb['start'], edges, lit)
                    m.add(nb['ready'] == finish).only_enforce_if(lit)
                    m.add(nb['arrival_soc'] == na['soc'] - consumption).only_enforce_if(lit)
                    details[a, b].update(crosses=crosses, waits=waits, edges=edges)
            m.add_circuit(arcs)
            # Terminal reachability reserve: needed by the next online solve, and for a fair offline comparison.
            if opts['terminal_reserve']:
                for a, node in nodes.items():
                    if not a or node['task'] is None:
                        continue
                    reserve = sum(self.metrics(*e)[1] for e in route(robot['loop'], node['location'], 'X'))
                    reserve += self.metrics('X', 'H')[1] + min(self.metrics('H', b)[1] for b in sorted(self.resources) if b != 'X')
                    reserve += ceil_int((3 * (self.cfg.charge_target-self.cfg.safety_soc) / self.cfg.charge_per_min * self.cfg.wait_energy_per_min + RESERVE_MARGIN) * SOC)
                    m.add(node['soc'] >= self.safe + reserve).only_enforce_if(details[a, 0]['use'])
            self.nodes.append(nodes)
            self.arcs.append(details)
            self.seed_maps.append((mapping, charge_nodes, empty))
        for copies in assignments:
            m.add_exactly_one(copies)
        for intervals in self.resources.values():
            m.add_no_overlap(intervals)
        makespan = self.var('Cmax')
        previous = max([t['completion_time'] for t in base.results] + [0])
        m.add_max_equality(makespan, self.ends + [ceil_int(previous * TIME)])
        tardiness, c = [], opts['norm']['coefficients']
        for i, task in enumerate(tasks):
            late = self.var(f'T_{i}')
            m.add_max_equality(late, [0, self.ends[i] - math.floor(task['due_time'] * TIME + 1e-7)])
            tardiness.append(c['tardiness'] * task['priority'] / TIME * late)
        fixed_energy = sum(max(0, a['energy']) for a in base.actions)
        fixed_wait = sum(a['end_time']-a['start_time'] for a in base.actions if a['action'].endswith('_wait'))
        fixed_late = sum(t['priority']*t['tardiness'] for t in base.results)
        # Same normalised J as the reports: coefficient w_k / FCFS reference_k per unit of each term.
        m.minimize(c['makespan'] / TIME * makespan + sum(tardiness) + c['wait'] / TIME * sum(self.waits)
                   + c['energy'] / SOC * sum(self.energy)
                   + c['energy']*fixed_energy + c['wait']*fixed_wait + c['tardiness']*fixed_late)
        self.hint_feasible = False
        if hint:
            self.hint_feasible = self.seed_schedule(hint)
        if self.hint_feasible:
            self.set_hint(makespan, max([self.hints[v.index] for v in self.ends] + [ceil_int(previous * TIME)]))
            for i, task in enumerate(tasks):
                late = next(v for v in self.vars if v.name == f'T_{i}')
                self.set_hint(late, max(0, self.hints[self.ends[i].index] - math.floor(task['due_time'] * TIME + 1e-7)))
            for var in self.vars:
                m.add_hint(var, self.hints[var.index])

    def var(self, name, low=0, high=None, hint=0):
        var = self.model.new_int_var(low, self.limit if high is None else high, name)
        self.vars.append(var)
        self.hints[var.index] = hint
        return var

    def boolean(self, name):
        return self.var(name, 0, 1)

    def set_hint(self, var, value):
        self.hints[var.index] = round(value)

    def metrics(self, a, b):
        duration, energy, _ = edge_metrics(self.world, a, b)
        return ceil_int(duration * TIME), ceil_int(energy * SOC)

    def node(self, k, idx, location, task, task_index):
        node = dict(use=self.boolean(f'x_{task_index}_{k}' if task else f'charge_{k}_{idx}'),
                    location=location, task=task, task_index=task_index)
        for key in ('start', 'ready', 'end'):
            node[key] = self.var(f'{key}_{k}_{idx}')
            self.model.add(node[key] == 0).only_enforce_if(~node['use'])
        for key in ('arrival_soc', 'soc'):
            node[key] = self.var(f'{key}_{k}_{idx}', self.safe, 100 * SOC, self.safe)
        return node

    def path(self, start, edges, use):
        at, energy, crosses, waits = start, 0, [], []
        for edge in edges:
            if len(edge) == 3:
                dt, de = ceil_int(edge[1] * TIME), ceil_int(edge[2] * SOC)
            else:
                dt, de = self.metrics(*edge)
            if len(edge) == 2 and edge[1] == 'X':
                wait = self.var('w_cross')
                entry = self.var('e_kr')
                self.model.add(entry == at + wait).only_enforce_if(use)
                self.model.add(entry == 0).only_enforce_if(~use)
                self.model.add(wait == 0).only_enforce_if(~use)
                self.resources['X'].append(self.model.new_optional_fixed_size_interval_var(entry, dt+self.gap, use, 'X'))
                at = entry
                energy += self.drain * wait
                de += self.guard
                self.waits.append(wait)
                self.energy.append(self.drain * wait)
                crosses.append(entry)
                waits.append(wait)
            at = at + dt
            energy += de
            self.energy.append(de * use)
        return at, energy, crosses, waits

    def seed_schedule(self, hint):
        # Replay the joint heuristic's decisions on the conservative integer grid.
        calendars = {r: [dict(start_time=math.floor(a['start_time']*TIME+1e-7),
                              end_time=ceil_int(a['end_time']*TIME)) for a in aa]
                     for r, aa in self.base.calendars.items()}
        states = [dict(time=n[0]['end'], soc=n[0]['soc'], previous=0, job=0, charge=0) for n in self.nodes]
        for k, (_, _, empty) in enumerate(self.seed_maps):
            self.set_hint(empty, not self.trace[k])
        for task in hint.results[len(self.base.results):]:
            k = int(task['robot_id'][1:])-1
            state = states[k]
            mapping, charges, _ = self.seed_maps[k]
            while state['job'] < len(self.trace[k]):
                job = self.trace[k][state['job']]
                if job['key']:
                    idx = mapping[job['key']]
                else:
                    if state['charge'] >= len(charges):
                        return False
                    idx = charges[state['charge']]
                    state['charge'] += 1
                node = self.nodes[k][idx]
                arc = self.arcs[k].get((state['previous'], idx))
                if arc is None:
                    return False
                self.set_hint(node['use'], 1)
                self.set_hint(arc['use'], 1)
                state['time'] = max(state['time'], ceil_int(job['start']*TIME))
                self.set_hint(node['start'], state['time'])
                self.seed_path(arc, arc['edges'], state, calendars)
                self.set_hint(node['ready'], state['time'])
                self.set_hint(node['arrival_soc'], state['soc'])
                if state['soc'] < self.safe:
                    return False
                if node['task']:
                    self.seed_path(node, node['internal'], state, calendars)
                    self.set_hint(self.starts[node['task_index']], self.hints[node['start'].index])
                    self.set_hint(self.ends[node['task_index']], state['time'])
                else:
                    original = next(a for a in job['actions'] if a['action']=='charge')
                    bay = original['resource']
                    self.set_hint(node['bays'][bay], 1)
                    dt, de = self.metrics('H', bay)
                    rt, re = self.metrics(bay, 'H')
                    arrival, soc = state['time']+dt, state['soc']-de-self.guard
                    target = min(100*SOC, ceil_int(original['end_soc']*SOC))
                    start = arrival
                    for _ in range(len(calendars[bay])+3):
                        before = soc - self.drain*(start-arrival)
                        duration = max(1, ceil_int((target-before)/self.rate))
                        duration = min(duration, (100*SOC-before)//self.rate)
                        nxt = slot(calendars[bay], start, duration)
                        if nxt == start:
                            break
                        start = nxt
                    if before < self.safe or duration < 1:
                        return False
                    target = before + duration*self.rate
                    self.set_hint(node['queue'], start-arrival)
                    self.set_hint(node['charge_start'], start)
                    self.set_hint(node['duration'], duration)
                    self.set_hint(node['target'], target)
                    calendars[bay].append(dict(start_time=start, end_time=start+duration))
                    state.update(time=start+duration+rt, soc=target-re)
                if state['soc'] < self.safe or state['time'] > self.limit:
                    return False
                self.set_hint(node['end'], state['time'])
                self.set_hint(node['soc'], state['soc'])
                state['previous'] = idx
                state['job'] += 1
                if job['key'] == task['task_id']:
                    break
        for k, state in enumerate(states):
            if state['previous']:
                self.set_hint(self.arcs[k][state['previous'], 0]['use'], 1)
        return True

    def seed_path(self, section, edges, state, calendars):
        crossing = 0
        for edge in edges:
            dt, de = (ceil_int(edge[1]*TIME), ceil_int(edge[2]*SOC)) if len(edge)==3 else self.metrics(*edge)
            if len(edge)==2 and edge[1]=='X':
                start = slot(calendars['X'], state['time'], dt, self.gap)
                wait = start-state['time']
                self.set_hint(section['crosses'][crossing], start)
                self.set_hint(section['waits'][crossing], wait)
                crossing += 1
                calendars['X'].append(dict(start_time=start, end_time=start+dt))
                state['time'] = start
                de += self.drain*wait + self.guard
            state['time'] += dt
            state['soc'] -= de

    def decode(self, solver):
        plan = self.base.clone()
        plan.crossing_hints = {}
        for k, nodes in enumerate(self.nodes):
            r, previous = plan.robots[k], 0
            while True:
                nxt = next((b for (a, b), arc in self.arcs[k].items() if a == previous and solver.value(arc['use'])), 0)
                if nxt == 0:
                    break
                node, arc = nodes[nxt], self.arcs[k][previous, nxt]
                crossings = arc['crosses'] + node.get('crosses', [])
                plan.crossing_hints[r['robot_id']] = [solver.value(v) / TIME for v in crossings]
                plan.wait(r, solver.value(node['start']) / TIME, 'idle', node['task']['task_id'] if node['task'] else '')
                if node['task']:
                    plan.task(r, node['task'])
                else:
                    bay = next(b for b, use in node['bays'].items() if solver.value(use))
                    plan.charge(r, bay_choice=bay, target_soc=solver.value(node['target']) / SOC,
                                earliest_start=solver.value(node['charge_start']) / TIME)
                if plan.crossing_hints[r['robot_id']]:
                    raise InputError('Decoder did not consume every crossing decision.')
                previous = nxt
        del plan.crossing_hints
        return plan


def score(plan, norm):
    return objective(objective_terms(plan), norm)


def solve(base, tasks, opts, rolling=False):
    started = time.perf_counter()
    hint, hint_error = None, None
    try:
        hint = plan_tasks(base.clone(), tasks)
    except InputError as exc:
        hint_error = str(exc)
    joint = JointModel(base, tasks, hint, opts, rolling)
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = opts['time_limit']
    solver.parameters.max_deterministic_time = opts['deterministic_time']
    solver.parameters.num_search_workers = opts['workers']
    solver.parameters.random_seed = opts['seed']
    solver.parameters.hint_conflict_limit = 10000
    status = solver.solve(joint.model)
    feasible = status in (cp_model.FEASIBLE, cp_model.OPTIMAL)
    metadata = dict(status=solver.status_name(status), objective=solver.objective_value if feasible else None,
                    best_bound=solver.best_objective_bound if status != cp_model.MODEL_INVALID else None,
                    wall_time=solver.wall_time, deterministic_time=solver.response_proto.deterministic_time,
                    num_solves=1, fallback=None, proof_scope='finite charge-visit, integer-grid model', hint_objective=score(hint, opts['norm']) if hint else None,
                    hint_error=hint_error, grid_hint=joint.hint_feasible, workers=opts['workers'], seed=opts['seed'])
    cap = opts['max_charges_per_robot']
    selected = hint if cap is None or all(sum(j['key'] is None for j in jobs) <= cap for jobs in joint.trace) else None
    if feasible:
        stage = 'decoder'
        try:
            decoded = joint.decode(solver)
            build_result(decoded, base.results + tasks, 'cpsat', started, opts['norm'])
            if opts['terminal_reserve']:
                stage = 'terminal reserve'
                active = {t['robot_id'] for t in decoded.results[len(base.results):]}
                for k in range(4):
                    if decoded.robots[k]['robot_id'] in active:
                        decoded.ensure_reserve(k)
            if selected is None or score(decoded, opts['norm']) <= score(selected, opts['norm']) + 1e-7:
                selected = decoded
            else:
                metadata['fallback'] = 'joint hint has a lower decoded objective'
        except (InputError, RuntimeError) as exc:
            metadata['fallback'] = stage + ' rejected incumbent: ' + str(exc)
    else:
        metadata['fallback'] = 'no CP-SAT incumbent within budget'
        if status == cp_model.MODEL_INVALID:
            raise RuntimeError('Invalid CP-SAT model: ' + joint.model.validate())
    if selected is None:
        raise InputError('No energy-safe optimizer schedule found. ' + (hint_error or metadata['status']))
    actual = score(selected, opts['norm'])
    metadata['decoded_objective'] = actual
    metadata['objective_difference'] = actual - metadata['objective'] if feasible else None
    metadata['decoded_differs'] = abs(metadata['objective_difference']) > 1e-5 if feasible else None
    metadata['gap'] = max(0, (metadata['objective']-metadata['best_bound']) / max(1e-9, abs(metadata['objective']))) if feasible else None
    metadata['elapsed'] = time.perf_counter()-started
    return selected, metadata


def optimize(tasks, config=None, strategy='cpsat', world=None, options=None):
    started = time.perf_counter()
    tasks = validate_tasks(tasks)
    cfg = Config.parse(config or {})
    opts = options_for(options, strategy == 'rolling')
    opts['norm'] = normaliser(tasks, cfg, world)
    base = Planner(cfg, world)
    if strategy == 'cpsat':
        plan, metadata = solve(base, tasks, opts)
    else:
        plan, metadata = rolling_plan(base, tasks, opts)
    result = build_result(plan, tasks, strategy, started, opts['norm'])
    result['solver'] = metadata
    return result


def rolling_plan(base, tasks, opts):
    pending = list(tasks)
    tick, period = 0., opts['replan_seconds']
    history = []
    while pending:
        known = [t for t in pending if t['release_time'] <= tick]
        if not known:
            tick = max(tick + period, math.ceil(min(t['release_time'] for t in pending) / period) * period)
            continue
        loops = {t['pickup'][0] for t in known}
        if all(r['time'] >= tick + period for r in base.robots if r['loop'] in loops):
            tick += period
            continue
        for r in base.robots:
            base.wait(r, tick, 'idle')
        window, work = [], dict(A=0, B=0)
        capacity = {area: sum(max(0, tick+opts['horizon_seconds']-r['time'])
                              for r in base.robots if r['loop']==area) for area in 'AB'}
        for task in sorted(known, key=lambda t: (t['due_time'], -t['priority'], t['task_id'])):
            estimate = task['load_time']+task['unload_time']
            estimate += sum(edge_metrics(base.world, *e)[0] for e in route(task['pickup'][0], task['pickup'], task['dropoff']))
            area = task['pickup'][0]
            if work[area] and work[area] + estimate > capacity[area]:
                continue
            window.append(task)
            work[area] += estimate
        frozen = {resource: [dict(a) for a in calendar if a['end_time'] >= tick]
                  for resource, calendar in base.calendars.items()}
        # Online decisions are normalised with FCFS on the known tasks only (no future information);
        # the full-instance reference in opts['norm'] is used solely for the final evaluation.
        online = dict(opts, norm=normaliser(known, base.c, start=base))
        candidate, info = solve(base, window, online, rolling=True)
        jobs = jobs_from_plan(candidate, base)
        committed, actions, commitments = [], [], []
        for k, robot_jobs in enumerate(jobs):
            last_action = None
            for job in robot_jobs:
                if job['start'] >= tick + period - 1e-8:
                    break
                last_action = job['actions'][-1]
                commitments.append(dict(robot_id=base.robots[k]['robot_id'], task_id=job['key'],
                                        start_time=job['start'], end_time=job['end']))
                if job['key']:
                    committed.append(job['key'])
            if last_action is not None:
                for a in candidate.actions[len(base.actions):]:
                    if a['robot_id'] != base.robots[k]['robot_id']:
                        continue
                    actions.append(a)
                    if a is last_action:
                        break
                last = last_action
                base.robots[k].update(time=last['end_time'], node=last['to_node'], soc=last['end_soc'])
        for a in actions:
            base.actions.append(a)
            if a['action'] in ('cross', 'charge'):
                base.calendars[a['resource']].append(a)
        base.results.extend(t for t in candidate.results if t['task_id'] in committed)
        pending = [t for t in pending if t['task_id'] not in committed]
        info.update(replan_time=tick, known_tasks=[t['task_id'] for t in known],
                    optimized_tasks=[t['task_id'] for t in window], committed_tasks=committed,
                    commitments=commitments, frozen_reservations=frozen)
        history.append(info)
        tick += period
    # Each re-plan used its own online normalisation, so solver objectives are not comparable with
    # the final full-instance J; report the evaluated J only (per-solve values stay in `replans`).
    final = score(base, opts['norm'])
    metadata = dict(status='ROLLING', objective=final, best_bound=None, gap=None,
                    wall_time=sum(h['wall_time'] for h in history), num_solves=len(history),
                    decoded_objective=final, decoded_differs=None, objective_difference=None,
                    fallback_count=sum(h['fallback'] is not None for h in history), replans=history,
                    workers=opts['workers'], seed=opts['seed'])
    return base, metadata
