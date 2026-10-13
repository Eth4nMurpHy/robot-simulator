"""Strict, data-only commands. No task assignment, auto charging, or rescheduling."""
import time
from dataclasses import asdict
from simulator_types import InputError, number
from customization import LOOPS, edge_metrics

DEFAULT_COMMAND_PLAN = {'name': 'My robot commands', 'commands': [
    {'robot_id': 'R1', 'at': 0, 'action': 'move_to', 'node': 'A2'},
    {'robot_id': 'R1', 'at': 25, 'action': 'wait', 'duration': 10}]}


class CommandPlanError(InputError):
    def __init__(self, errors):
        self.errors = sorted(errors, key=lambda e: (e.get('time', 0), e.get('command_index', 0), e['code']))
        super().__init__(f'Command plan rejected: {len(errors)} error(s). ' + self.errors[0]['message'])


def parse_command_plan(data):
    if not isinstance(data, dict) or set(data) != {'name', 'commands'}:
        raise InputError('Command strategy must contain only name and commands. Scoring/weight rules are not robot commands.')
    if not isinstance(data['name'], str) or not 1 <= len(data['name'].strip()) <= 64:
        raise InputError('Plan name must be 1–64 characters.')
    rows = data['commands']
    if not isinstance(rows, list) or not 1 <= len(rows) <= 1000:
        raise InputError('Provide 1–1000 commands.')
    clean, errors = [], []
    fields = {'execute_task': {'task_id'}, 'move_to': {'node'}, 'charge': {'bay', 'target_soc'}, 'wait': {'duration'}}
    for index, row in enumerate(rows, 1):
        try:
            if not isinstance(row, dict):
                raise InputError('Command must be an object.')
            action = row.get('action')
            if not isinstance(action, str) or action not in fields:
                raise InputError('action must be execute_task, move_to, charge, or wait.')
            if set(row) != {'robot_id', 'at', 'action'} | fields[action]:
                raise InputError(f'{action} requires robot_id, at, action, and ' + ', '.join(sorted(fields[action])) + '; no other fields.')
            rid = row['robot_id']
            if rid not in ('R1', 'R2', 'R3', 'R4'):
                raise InputError('robot_id must be R1, R2, R3, or R4.')
            command = dict(robot_id=rid, at=number(row['at'], 'at', 0, 1_000_000), action=action)
            if action == 'execute_task':
                if not isinstance(row['task_id'], str) or not 1 <= len(row['task_id']) <= 64:
                    raise InputError('task_id must be a nonempty string of at most 64 characters.')
                command['task_id'] = row['task_id']
            elif action == 'move_to':
                area = 'A' if rid in ('R1', 'R2') else 'B'
                if row['node'] not in (*LOOPS[area], 'H', 'C1', 'C2'):
                    raise InputError('node must belong to this robot’s loop or shared area.')
                command['node'] = row['node']
            elif action == 'charge':
                if row['bay'] not in ('C1', 'C2'):
                    raise InputError('bay must be C1 or C2.')
                command.update(bay=row['bay'], target_soc=number(row['target_soc'], 'target_soc', 1, 100))
            else:
                command['duration'] = number(row['duration'], 'duration', .001, 86400)
            clean.append(command)
        except InputError as exc:
            errors.append(dict(code='INVALID_COMMAND', command_index=index,
                               robot_id=row.get('robot_id', '') if isinstance(row, dict) else '',
                               time=0, message=f'Command #{index}: {exc}'))
    if errors:
        raise CommandPlanError(errors)
    return {'name': data['name'].strip(), 'commands': clean}


def execute_commands(tasks, config, world, data):
    # Imported lazily: the automatic simulator dispatches here for custom mode.
    from simulator import Planner, Config, NODES, validate_tasks, validate_result
    started = time.perf_counter()
    tasks = [] if tasks is None or tasks == [] else validate_tasks(tasks)
    cfg = Config.parse(config or {})
    definition = parse_command_plan(data)

    class Executor(Planner):
        command_index = 0

        def clone(self):
            plan = super().clone()
            plan.__class__ = Executor
            plan.command_index = self.command_index
            return plan

        def action(self, *args, **kwargs):
            robot, action, duration = args[:3]
            energy = args[3] if len(args) > 3 else kwargs.get('energy', 0)
            end_soc = robot['soc'] - energy
            if end_soc < self.c.safety_soc - 1e-8:
                error = InputError(f"{robot['robot_id']} battery would reach {end_soc:.3f}% at {robot['time'] + duration:.3f}s, below safety {self.c.safety_soc:g}%.")
                error.unsafe_time = robot['time'] + duration
                raise error
            item = super().action(*args, **kwargs)
            item['command_index'] = self.command_index
            return item

        def crossing(self, r, tid):
            duration, energy, _ = edge_metrics(self.world, r['node'], 'X')
            item = self.action(r, 'cross', duration, energy, to='X', task_id=tid, resource='X')
            self.calendars['X'].append(item)

        def travel_to(self, r, dest, tid=''):
            if r['node'] == dest:
                return
            if r['node'].startswith('C'):
                self.move_edge(r, 'H', tid)
            if dest == 'H' or dest.startswith('C'):
                if r['node'] != 'H':
                    self.travel(r, 'X', tid)
                    self.move_edge(r, 'H', tid)
                if dest != 'H':
                    self.move_edge(r, dest, tid)
            else:
                self.travel(r, dest, tid)

    plan = Executor(cfg, world)
    by_id = {t['task_id']: t for t in tasks}
    assigned, blocked, issues, command_results = set(), set(), [], []
    commands = sorted(enumerate(definition['commands'], 1), key=lambda item: (item[1]['at'], item[0]))
    for index, cmd in commands:
        rid, at = cmd['robot_id'], cmd['at']
        robot_index = int(rid[1]) - 1
        r = plan.robots[robot_index]
        code, message, issue_time = '', '', at
        if rid in blocked:
            code, message = 'DEPENDENT_COMMAND', 'An earlier command failed; this robot’s later commands cannot be validated.'
        elif at < r['time'] - 1e-8:
            code, message = 'ROBOT_BUSY', f'{rid} is busy until {r["time"]:.3f}s; command starts at {at:.3f}s. It was not queued.'
        elif cmd['action'] == 'execute_task':
            task = by_id.get(cmd['task_id'])
            if task is None:
                code, message = 'UNKNOWN_TASK', f'Task {cmd["task_id"]} does not exist.'
            elif task['task_id'] in assigned:
                code, message = 'DUPLICATE_TASK', f'Task {task["task_id"]} was already assigned by another command.'
            elif task['pickup'][0] != r['loop']:
                code, message = 'WRONG_LOOP', f'{rid} cannot execute a task on loop {task["pickup"][0]}.'
            elif at < task['release_time'] - 1e-8:
                code, message = 'TASK_NOT_RELEASED', f'Task {task["task_id"]} is not released until {task["release_time"]:.3f}s.'
        if not code and (cmd.get('bay') == 'C2' or cmd.get('node') == 'C2') and cfg.chargers == 1:
            code, message = 'DISABLED_CHARGER', 'C2 is unavailable because only one charging bay is enabled.'
        if not code:
            trial = plan.clone()
            trial.command_index = index
            tr = trial.robots[robot_index]
            try:
                trial.wait(tr, at, 'idle')
                action_start = len(trial.actions)
                if cmd['action'] == 'execute_task':
                    trial.task(tr, by_id[cmd['task_id']])
                elif cmd['action'] == 'move_to':
                    trial.travel_to(tr, cmd['node'])
                elif cmd['action'] == 'wait':
                    trial.action(tr, 'wait', cmd['duration'], cmd['duration'] / 60 * cfg.wait_energy_per_min)
                else:
                    trial.travel_to(tr, cmd['bay'])
                    if cmd['target_soc'] < tr['soc'] - 1e-8:
                        code, message = 'INVALID_CHARGE_TARGET', f'Target {cmd["target_soc"]}% is below the arrival battery {tr["soc"]:.3f}%.'
                    else:
                        duration = (cmd['target_soc'] - tr['soc']) / cfg.charge_per_min * 60
                        trial.action(tr, 'charge', max(0, duration), -(cmd['target_soc'] - tr['soc']), resource=cmd['bay'])
                        # Remain in the chosen bay. Only a later move command vacates it.
                if not code:
                    plan = trial
                    if cmd['action'] == 'execute_task':
                        assigned.add(cmd['task_id'])
                    command_results.append(dict(command_index=index, **cmd, start_time=at, end_time=tr['time'],
                                                start_soc=trial.actions[action_start]['start_soc'] if len(trial.actions) > action_start else tr['soc'],
                                                end_soc=tr['soc']))
            except InputError as exc:
                code, message = 'BATTERY_UNSAFE', str(exc) + ' No automatic charging was inserted.'
                issue_time = getattr(exc, 'unsafe_time', at)
                blocked.add(rid)
        if code:
            issues.append(dict(code=code, command_index=index, robot_id=rid, time=issue_time,
                               message=f'Command #{index} ({rid}, {at:g}s): {message}'))

    horizon = max([a['end_time'] for a in plan.actions] + [c['end_time'] for c in command_results] + [0])
    # Charging bay occupation includes parked/idle time, not just active charging.
    bays = {f'C{i+1}': [] for i in range(cfg.chargers)}
    for robot in plan.robots:
        held = None
        for action in (a for a in plan.actions if a['robot_id'] == robot['robot_id']):
            if held and action['from_node'] == held['resource'] and action['to_node'] != held['resource']:
                held['end_time'] = action['start_time']
                bays[held['resource']].append(held)
                held = None
            if action['to_node'] in bays and action['from_node'] != action['to_node']:
                held = dict(robot_id=robot['robot_id'], resource=action['to_node'], start_time=action['end_time'],
                            end_time=horizon, command_index=action['command_index'], action='occupy',
                            start_soc=action['end_soc'], end_soc=action['end_soc'])
        if held:
            held['open_ended'] = True
            bays[held['resource']].append(held)
    reservations = {'X': list(plan.calendars['X']), **bays}
    # Stopping at X blocks the intersection; commands must leave it immediately.
    for robot in plan.robots:
        robot_actions = [a for a in plan.actions if a['robot_id'] == robot['robot_id']]
        for i, a in enumerate(robot_actions):
            if a['action'] != 'cross':
                continue
            departure = next((b for b in robot_actions[i+1:] if b['from_node'] == 'X' and b['to_node'] != 'X'), None)
            leave = departure['start_time'] if departure else horizon
            if departure is None or leave > a['end_time'] + 1e-8:
                issues.append(dict(code='INTERSECTION_BLOCKED', command_index=a['command_index'], robot_id=robot['robot_id'],
                                   time=a['end_time'], resource='X', message=f'Command #{a["command_index"]}: {robot["robot_id"]} stops in X at {a["end_time"]:.3f}s. It must leave immediately.'))
                a['end_time'] = max(a['end_time'], leave)  # diagnostic reservation only; plan will be rejected
    for resource, calendar in reservations.items():
        ordered = sorted(calendar, key=lambda a: (a['start_time'], a['end_time']))
        gap = 2 * cfg.buffer_seconds if resource == 'X' else 0
        for i, a in enumerate(ordered):
            for b in ordered[i+1:]:
                if not a.get('open_ended') and b['start_time'] >= a['end_time'] + gap - 1e-8:
                    break
                if (resource != 'X' and a['robot_id'] == b['robot_id']) or (a['end_time'] <= a['start_time'] + 1e-8 and not a.get('open_ended')) or (b['end_time'] <= b['start_time'] + 1e-8 and not b.get('open_ended')):
                    continue
                code = 'INTERSECTION_CONFLICT' if resource == 'X' else 'CHARGER_CONFLICT'
                issues.append(dict(code=code, command_index=b['command_index'], robot_id=b['robot_id'],
                    other_command_index=a['command_index'], other_robot_id=a['robot_id'], resource=resource,
                    time=b['start_time'], start_time=b['start_time'], end_time=min(a['end_time'] + gap, b['end_time']),
                    message=f'{resource} conflict: {a["robot_id"]} command #{a["command_index"]} [{a["start_time"]:.3f}, {a["end_time"]:.3f})s and {b["robot_id"]} command #{b["command_index"]} [{b["start_time"]:.3f}, {b["end_time"]:.3f})s' + (f' require {gap:g}s separation.' if gap else ' overlap (a parked robot holds its bay until commanded to leave).')))
    if issues:
        raise CommandPlanError(issues)

    actions = sorted(plan.actions, key=lambda a: (a['start_time'], a['robot_id']))
    completed = plan.results
    late = sum(t['tardiness'] > 1e-8 for t in completed)
    total = len(tasks)
    occupied = lambda resource: sum(a['end_time'] - a['start_time'] for a in reservations[resource])
    result = dict(mode='commands', strategy='custom', strategy_label=definition['name'], config=asdict(cfg),
                  nodes=NODES, map=plan.world, custom_strategy=definition, commands=command_results,
                  robots=[dict(robot_id=r['robot_id'], loop=r['loop'], start_node=f'{r["loop"]}1', initial_soc=cfg.initial_soc) for r in plan.robots],
                  tasks=sorted(completed, key=lambda t: (t['release_time'], t['task_id'])), actions=actions,
                  unexecuted_tasks=[t['task_id'] for t in tasks if t['task_id'] not in assigned],
                  reservations={k: sorted(v, key=lambda a: a['start_time']) for k, v in reservations.items()})
    result['events'] = sorted([dict(time=a[f'{boundary}_time'], robot_id=a['robot_id'], event=a['action']+'_'+boundary,
        node=a['from_node'] if boundary == 'start' else a['to_node'], task_id=a['task_id'], soc=a[f'{boundary}_soc'],
        resource=a['resource'], wait_time=0, command_index=a['command_index'])
        for a in actions for boundary in ('start', 'end')], key=lambda e: (e['time'], e['robot_id']))
    errors = validate_result(result)
    if errors:
        raise CommandPlanError([dict(code='VALIDATION_ERROR', time=0, message=e) for e in errors])
    result['validation'] = dict(passed=True, errors=[], checks=['Exact command timing', 'Task uniqueness and precedence',
        'Exclusive intersection and charging bays', 'Robot action and SOC continuity', 'Battery safety'])
    result['summary'] = dict(makespan=horizon, timeline_end=horizon, task_makespan=max([t['completion_time'] for t in completed]+[0]),
        completed=len(completed), total_tasks=total, unexecuted=total-len(completed), late_tasks=late,
        tardiness_rate=late/max(1,len(completed)), total_tardiness=sum(t['tardiness'] for t in completed),
        cross_wait=0, charge_wait=0, average_wait=0, energy=sum(max(0,a['energy']) for a in actions),
        distance_m=sum(a['distance_m'] for a in actions), min_soc=min([a['end_soc'] for a in actions]+[cfg.initial_soc]),
        violations=0, cross_utilization=occupied('X')/max(1,horizon),
        charge_utilization=sum(occupied(b) for b in bays)/(max(1,horizon)*cfg.chargers),
        charging_sessions=sum(a['action']=='charge' for a in actions), runtime=time.perf_counter()-started)
    return result
