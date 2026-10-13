import copy
import unittest
from simulator import Config, InputError, STRATEGIES, generate_tasks, simulate, slot, validate_result, validate_tasks


def simple_task(**changes):
    task = dict(task_id='one', release_time=0, pickup='A1', dropoff='A2',
                load_time=15, unload_time=15, due_time=600, priority=1)
    return {**task, **changes}


class SimulatorTests(unittest.TestCase):
    def test_reproducible_generation(self):
        self.assertEqual(generate_tasks(40, 42), generate_tasks(40, 42))
        self.assertNotEqual(generate_tasks(40, 42), generate_tasks(40, 43))
        self.assertTrue(all(t['pickup'].startswith('A') for t in generate_tasks(20, 42, 1)))

    def test_hand_calculated_delivery(self):
        result = simulate([simple_task()], strategy='fcfs')
        self.assertEqual(result['summary']['makespan'], 50)  # load 15 + edge 20 + unload 15
        self.assertEqual(result['summary']['energy'], 3)  # 0.5 + 2 + 0.5
        self.assertEqual(result['summary']['completed'], 1)
        self.assertEqual(result['tasks'][0]['robot_id'], 'R1')
        self.assertEqual(result['summary']['min_soc'], 62)

    def test_release_and_lateness(self):
        result = simulate([simple_task(release_time=100, due_time=120)])
        task = result['tasks'][0]
        self.assertEqual(task['start_time'], 100)
        self.assertEqual(task['completion_time'], 150)
        self.assertEqual(task['tardiness'], 30)
        self.assertEqual(result['summary']['tardiness_rate'], 1)

    def test_slot_inserts_gaps_and_guards(self):
        calendar = [dict(start_time=20, end_time=30), dict(start_time=50, end_time=60)]
        self.assertEqual(slot(calendar, 0, 6, 4), 0)
        self.assertEqual(slot(calendar, 15, 6, 4), 34)
        self.assertEqual(slot(calendar, 35, 15, 4), 64)

    def test_same_robot_not_overlapping_and_resources_exclusive(self):
        for strategy in STRATEGIES:
            result = simulate(generate_tasks(40, 42), strategy=strategy)
            self.assertEqual(validate_result(result), [])
            self.assertEqual(len(result['tasks']), 40)
            self.assertGreater(result['summary']['charging_sessions'], 0)
            self.assertTrue(result['validation']['passed'])
            self.assertLessEqual(result['summary']['cross_utilization'], 1)
            self.assertLessEqual(result['summary']['charge_utilization'], 1)
            self.assertEqual(set(t['task_id'] for t in result['tasks']), set(t['task_id'] for t in generate_tasks()))

    def test_single_charger_stress(self):
        for seed in (0, 3, 5, 6, 7, 12, 13):
            for strategy in STRATEGIES:
                with self.subTest(seed=seed, strategy=strategy):
                    result = simulate(generate_tasks(60, seed), {'chargers': 1}, strategy)
                    self.assertEqual(result['summary']['violations'], 0)
                    self.assertGreaterEqual(result['summary']['min_soc'], 10)
                    self.assertGreater(result['summary']['charge_wait'], 0)
                    self.assertEqual(set(result['reservations']), {'X', 'C1'})

    def test_safety_thresholds_and_unbalanced_workload(self):
        for safety in (10, 20, 30):
            for share in (0, .75, 1):
                result = simulate(generate_tasks(40, 99, share), {'safety_soc': safety, 'initial_soc': 80})
                self.assertEqual(result['summary']['completed'], 40)
                self.assertGreaterEqual(result['summary']['min_soc'], safety)
                self.assertTrue(all(t['pickup'][0] == ('A' if t['robot_id'] in ('R1', 'R2') else 'B') for t in result['tasks']))

    def test_maximum_task_count(self):
        result = simulate(generate_tasks(200, 42))
        self.assertEqual(result['summary']['completed'], 200)
        self.assertEqual(validate_result(result), [])

    def test_inputs_not_mutated_and_repeatable_plan(self):
        tasks = generate_tasks(20)
        before = copy.deepcopy(tasks)
        a, b = simulate(tasks), simulate(tasks)
        self.assertEqual(tasks, before)
        self.assertEqual(a['actions'], b['actions'])
        self.assertEqual(a['tasks'], b['tasks'])

    def test_reject_bad_task_data(self):
        cases = [[], [simple_task(), simple_task()], [simple_task(pickup='Z1')],
                 [simple_task(dropoff='B1')], [simple_task(dropoff='A1')],
                 [simple_task(release_time=-1)], [simple_task(release_time=200, due_time=100)],
                 [simple_task(priority=6)], [simple_task(load_time=float('nan'))],
                 [simple_task(task_id='')], [None]]
        for tasks in cases:
            with self.subTest(tasks=tasks), self.assertRaises(InputError):
                validate_tasks(tasks)

    def test_reject_bad_config_and_strategy(self):
        for cfg in ({'chargers': 1.5}, {'edge_seconds': 0}, {'safety_soc': float('inf')}, {'bogus': 1}, {'chargers': True}):
            with self.subTest(config=cfg), self.assertRaises(InputError):
                Config.parse(cfg)
        with self.assertRaises(InputError):
            simulate([simple_task()], strategy='optimal')

    def test_report_energy_infeasibility(self):
        with self.assertRaisesRegex(InputError, 'No energy-safe schedule'):
            simulate([simple_task(pickup='A3', dropoff='A4')], {'initial_soc': 40, 'safety_soc': 30})

    def test_validator_detects_tampering(self):
        original = simulate(generate_tasks(40))
        changed = copy.deepcopy(original)
        cross = changed['reservations']['X']
        cross[1]['start_time'] = cross[0]['start_time']
        self.assertTrue(any('overlapping reservations' in e for e in validate_result(changed)))
        changed = copy.deepcopy(original)
        changed['actions'][0]['energy'] += 1
        self.assertTrue(any('energy' in e or 'SOC' in e for e in validate_result(changed)))
        changed = copy.deepcopy(original)
        changed['tasks'][0]['pickup'] = 'A4' if changed['tasks'][0]['pickup'] != 'A4' else 'A3'
        self.assertTrue(any('wrong station' in e for e in validate_result(changed)))

    def test_chargers_vacated_immediately(self):
        result = simulate(generate_tasks(60))
        for robot in result['robots']:
            actions = [a for a in result['actions'] if a['robot_id'] == robot['robot_id']]
            for i, action in enumerate(actions):
                if action['action'] == 'charge':
                    departure = actions[i + 1]
                    self.assertEqual(departure['action'], 'move')
                    self.assertEqual(departure['to_node'], 'H')
                    self.assertEqual(departure['start_time'], action['end_time'])


if __name__ == '__main__':
    unittest.main()
