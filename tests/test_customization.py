import copy
import unittest
from customization import default_map, parse_map, parse_rules, DEFAULT_RULES
from simulator import InputError, generate_tasks, simulate, validate_result


def task(tid='one', **changes):
    return dict(task_id=tid, release_time=0, pickup='A1', dropoff='A2', load_time=15,
                unload_time=15, due_time=600, priority=1, **changes)


def edge(world, source, dest):
    return next(e for e in world['edges'] if (e['from'], e['to']) == (source, dest))


class CustomizationTests(unittest.TestCase):
    def test_longer_edge_changes_time_energy_and_distance(self):
        world = default_map()
        edge(world, 'A1', 'A2')['length_m'] = 40
        result = simulate([task()], world=world)
        self.assertEqual(result['summary']['makespan'], 70)
        self.assertEqual(result['summary']['energy'], 5)
        self.assertEqual(result['summary']['distance_m'], 40)
        self.assertEqual(result['map'], world)
        self.assertEqual(validate_result(result), [])

    def test_speed_changes_time_not_energy(self):
        world = default_map()
        world['speed_m_per_s'] = 2
        result = simulate([task()], world=world)
        self.assertEqual(result['summary']['makespan'], 40)
        self.assertEqual(result['summary']['energy'], 3)

    def test_defaults_preserve_legacy_settings(self):
        config = {'edge_seconds': 30, 'crossing_seconds': 8, 'edge_energy': 1.5}
        tasks = generate_tasks(20)
        from simulator import Config
        a = simulate(tasks, config)
        b = simulate(tasks, config, world=default_map(Config.parse(config)))
        self.assertEqual(a['actions'], b['actions'])

    def test_asymmetric_charging_edges_and_variable_crossing(self):
        world = default_map()
        edge(world, 'H', 'C1')['length_m'] = 35
        edge(world, 'H', 'C2')['length_m'] = 5
        edge(world, 'C2', 'H')['length_m'] = 12
        edge(world, 'WA', 'X')['length_m'] = 9
        result = simulate(generate_tasks(40, 42), world=world)
        self.assertEqual(validate_result(result), [])
        for a in result['actions']:
            if a['action'] in ('move', 'cross'):
                distance = edge(world, a['from_node'], a['to_node'])['length_m']
                self.assertAlmostEqual(a['end_time'] - a['start_time'], distance)
                self.assertAlmostEqual(a['energy'], distance * .1)

    def test_robot_preference_changes_assignment(self):
        rules = copy.deepcopy(DEFAULT_RULES)
        rules['preferred_robots'] = ['R2', 'R1', 'R4', 'R3']
        result = simulate([task()], strategy='rules', custom_strategy=rules)
        self.assertEqual(result['tasks'][0]['robot_id'], 'R2')
        self.assertEqual(result['custom_strategy'], rules)

    def test_priority_order_and_bonus_change_task_selection(self):
        rules = copy.deepcopy(DEFAULT_RULES)
        rules.update(task_order='priority', lookahead=1)
        tasks = [task('low'), {**task('high'), 'priority': 5}]
        result = simulate(tasks, strategy='rules', custom_strategy=rules)
        load = next(a for a in result['actions'] if a['action'] == 'load')
        self.assertEqual(load['task_id'], 'high')
        rules.update(task_order='release', lookahead=8)
        rules['weights']['priority'] = 100
        result = simulate(tasks, strategy='rules', custom_strategy=rules)
        self.assertEqual(next(a for a in result['actions'] if a['action'] == 'load')['task_id'], 'high')

    def test_charge_threshold_and_target(self):
        rules = copy.deepcopy(DEFAULT_RULES)
        rules.update(charge_below_soc=80, charge_target=80)
        result = simulate([task()], strategy='rules', custom_strategy=rules)
        charges = [a for a in result['actions'] if a['action'] == 'charge']
        self.assertEqual(len(charges), 1)
        self.assertAlmostEqual(charges[0]['end_soc'], 80)
        self.assertEqual(result['config']['charge_target'], 80)
        self.assertEqual(result['summary']['violations'], 0)

    def test_combined_custom_map_and_strategy(self):
        world = default_map()
        for e in world['edges']:
            if e['to'] != 'X':
                e['length_m'] *= 1.2
        for chargers in (1, 2):
            result = simulate(generate_tasks(40), {'chargers': chargers, 'initial_soc': 80}, 'rules', world, DEFAULT_RULES)
            self.assertEqual(result['summary']['completed'], 40)
            self.assertEqual(validate_result(result), [])

    def test_reject_bad_maps(self):
        world = default_map()
        cases = [[], {**world, 'version': True}, {**world, 'speed_m_per_s': 0},
                 {**world, 'energy_per_m': float('nan')}, {**world, 'edges': world['edges'][:-1]}]
        duplicate = copy.deepcopy(world)
        duplicate['edges'][-1] = duplicate['edges'][0]
        cases.append(duplicate)
        negative = copy.deepcopy(world)
        negative['edges'][0]['length_m'] = -1
        cases.append(negative)
        for data in cases:
            with self.subTest(data=data), self.assertRaises(InputError):
                parse_map(data)

    def test_reject_bad_rules(self):
        cases = [None, {}, {**DEFAULT_RULES, 'lookahead': 1.5}, {**DEFAULT_RULES, 'task_order': 'eval()'},
                 {**DEFAULT_RULES, 'charge_target': 20}, {**DEFAULT_RULES, 'preferred_robots': ['R1']},
                 {**DEFAULT_RULES, 'weights': {k: 0 for k in DEFAULT_RULES['weights']}},
                 {**DEFAULT_RULES, 'weights': {**DEFAULT_RULES['weights'], 'completion': -1}}]
        for data in cases:
            with self.subTest(data=data), self.assertRaises(InputError):
                parse_rules(data)


if __name__ == '__main__':
    unittest.main()
