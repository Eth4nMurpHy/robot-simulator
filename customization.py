"""Validated, data-only map and scheduling rule definitions."""
from simulator_types import InputError, number

LOOPS = {'A': ['X', 'A1', 'A2', 'A3', 'A4', 'WA'],
         'B': ['X', 'B1', 'B2', 'B3', 'B4', 'WB']}
EDGE_PAIRS = [(a, loop[(i + 1) % len(loop)]) for loop in LOOPS.values() for i, a in enumerate(loop)]
EDGE_PAIRS += [('X', 'H'), ('H', 'X'), ('H', 'C1'), ('C1', 'H'), ('H', 'C2'), ('C2', 'H')]
DEFAULT_RULES = {'name': 'My custom strategy', 'task_order': 'release', 'lookahead': 8,
                 'charge_below_soc': 20, 'charge_target': 95,
                 'weights': {'completion': 1, 'tardiness': 3, 'wait': .5, 'energy': 2,
                             'distance': 0, 'priority': 0},
                 'preferred_robots': ['R1', 'R2', 'R3', 'R4']}


def default_map(config=None):
    seconds = config.edge_seconds if config else 20
    crossing = config.crossing_seconds if config else 6
    energy = config.edge_energy if config else 2
    speed = 20 / seconds
    return {'version': 1, 'speed_m_per_s': speed, 'energy_per_m': energy / 20,
            'edges': [{'from': a, 'to': b, 'length_m': crossing * speed if b == 'X' else 20}
                      for a, b in EDGE_PAIRS]}


def parse_map(data, config=None):
    if data is None:
        return default_map(config)
    if not isinstance(data, dict) or set(data) - {'version', 'speed_m_per_s', 'energy_per_m', 'edges'}:
        raise InputError('Map must be an object with version, speed_m_per_s, energy_per_m, and edges.')
    if type(data.get('version')) is not int or data['version'] != 1:
        raise InputError('Map version must be 1.')
    speed = number(data.get('speed_m_per_s'), 'map speed_m_per_s', .1, 10)
    energy = number(data.get('energy_per_m'), 'map energy_per_m', .001, 1)
    rows = data.get('edges')
    if not isinstance(rows, list) or len(rows) != len(EDGE_PAIRS):
        raise InputError(f'Map must define all {len(EDGE_PAIRS)} directed edges exactly once.')
    lengths = {}
    for row in rows:
        if not isinstance(row, dict) or set(row) != {'from', 'to', 'length_m'}:
            raise InputError('Each map edge needs from, to, and length_m.')
        a, b = row['from'], row['to']
        if not isinstance(a, str) or not isinstance(b, str) or (a, b) not in EDGE_PAIRS or (a, b) in lengths:
            raise InputError('Map contains an unknown or duplicate directed edge.')
        lengths[a, b] = number(row['length_m'], f'{a} → {b} length_m', .1, 200)
    return {'version': 1, 'speed_m_per_s': speed, 'energy_per_m': energy,
            'edges': [{'from': a, 'to': b, 'length_m': lengths[a, b]} for a, b in EDGE_PAIRS]}


def edge_metrics(world, source, dest):
    edge = next(e for e in world['edges'] if e['from'] == source and e['to'] == dest)
    length = edge['length_m']
    return length / world['speed_m_per_s'], length * world['energy_per_m'], length


def parse_rules(data):
    if not isinstance(data, dict) or set(data) != set(DEFAULT_RULES):
        raise InputError('Strategy JSON must contain name, task_order, lookahead, charge_below_soc, charge_target, weights, and preferred_robots.')
    name = data['name']
    if not isinstance(name, str) or not 1 <= len(name.strip()) <= 64:
        raise InputError('Strategy name must be 1–64 characters.')
    if data['task_order'] not in ('release', 'deadline', 'priority', 'shortest_handling'):
        raise InputError('task_order must be release, deadline, priority, or shortest_handling.')
    lookahead = number(data['lookahead'], 'lookahead', 1, 40)
    if lookahead != int(lookahead):
        raise InputError('lookahead must be an integer.')
    weights = data['weights']
    if not isinstance(weights, dict) or set(weights) != set(DEFAULT_RULES['weights']):
        raise InputError('weights must contain completion, tardiness, wait, energy, distance, and priority.')
    weights = {k: number(v, f'weight {k}', 0, 1000) for k, v in weights.items()}
    if not any(weights.values()):
        raise InputError('At least one strategy weight must be positive.')
    robots = data['preferred_robots']
    if not isinstance(robots, list) or len(robots) != 4 or not all(isinstance(r, str) for r in robots) or set(robots) != {'R1', 'R2', 'R3', 'R4'}:
        raise InputError('preferred_robots must list R1, R2, R3, R4 exactly once.')
    return {'name': name.strip(), 'task_order': data['task_order'], 'lookahead': int(lookahead),
            'charge_below_soc': number(data['charge_below_soc'], 'charge_below_soc', 0, 80),
            'charge_target': number(data['charge_target'], 'charge_target', 80, 100),
            'weights': weights, 'preferred_robots': list(robots)}


def ordered_tasks(tasks, rules):
    key = {'release': lambda t: (t['release_time'], t['task_id']),
           'deadline': lambda t: (t['due_time'], -t['priority'], t['task_id']),
           'priority': lambda t: (-t['priority'], t['release_time'], t['task_id']),
           'shortest_handling': lambda t: (t['load_time'] + t['unload_time'], t['release_time'], t['task_id'])}[rules['task_order']]
    return sorted(tasks, key=key)[:rules['lookahead']]


def custom_score(task, done, waits, energy, distance, rules):
    w = rules['weights']
    return (w['completion'] * done['completion_time'] + w['tardiness'] * task['priority'] * done['tardiness']
            + w['wait'] * waits + w['energy'] * energy + w['distance'] * distance - w['priority'] * task['priority'])
