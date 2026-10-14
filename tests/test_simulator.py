import copy
import unittest
from simulator import Config, InputError, STRATEGIES, generate_tasks, normaliser, objective, random_robot_soc, simulate, slot, validate_result, validate_tasks


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

    def test_per_robot_initial_soc(self):
        socs = random_robot_soc(7)
        self.assertEqual(socs, random_robot_soc(7))
        self.assertTrue(all(50 <= v <= 100 for v in socs))
        result = simulate(generate_tasks(20, 7), {'robot_soc': socs}, 'joint')
        self.assertEqual([r['initial_soc'] for r in result['robots']], socs)
        for robot, soc in zip(result['robots'], socs):
            first = min((a for a in result['actions'] if a['robot_id'] == robot['robot_id']), key=lambda a: a['start_time'])
            self.assertEqual(first['start_soc'], soc)
        tampered = copy.deepcopy(result)
        tampered['robots'][0]['initial_soc'] += 5
        self.assertIn('R1: SOC continuity', validate_result(tampered))
        for bad in ([80, 80, 80], [80, 80, 80, 101], 'high'):
            with self.assertRaises(InputError):
                Config.parse({'robot_soc': bad})

    def test_generator_options_keep_base_workload(self):
        base = generate_tasks(40, 5)
        rush = generate_tasks(40, 5, urgent=.2)
        self.assertEqual([t for t in rush if t['task_id'].startswith('T')], base)
        urgent = [t for t in rush if t['task_id'].startswith('U')]
        self.assertEqual(len(urgent), 8)
        self.assertTrue(all(t['priority'] >= 4 and 180 <= t['due_time'] - t['release_time'] <= 360 for t in urgent))
        self.assertTrue(all(240 <= t['due_time'] - t['release_time'] <= 480 for t in generate_tasks(40, 5, tight=True)))
        self.assertLess(max(t['release_time'] for t in base), max(t['release_time'] for t in generate_tasks(40, 5, release_gap=60)))
        with self.assertRaises(InputError):
            generate_tasks(200, 5, urgent=.5)

    def test_staged_assignment_ignores_resources(self):
        tasks = generate_tasks(30, 11)
        result = simulate(tasks, {}, 'staged')
        self.assertTrue(result['validation']['passed'])
        self.assertEqual(result['summary']['completed'], 30)
        # Stage 1 balances work on nominal times only, so both robots of each loop are used.
        self.assertEqual({t['robot_id'] for t in result['tasks']}, {'R1', 'R2', 'R3', 'R4'})

    def test_objective_is_normalised_by_fcfs(self):
        tasks = generate_tasks(40, 1)
        fcfs, joint = simulate(tasks, {}, 'fcfs')['summary'], simulate(tasks, {}, 'joint')['summary']
        self.assertAlmostEqual(fcfs['objective'], 1)  # every FCFS term is above its floor here
        self.assertEqual(joint['objective_reference'], dict(strategy='fcfs', terms=fcfs['objective_terms']))
        terms, ref = joint['objective_terms'], fcfs['objective_terms']
        self.assertAlmostEqual(terms['makespan'], joint['makespan'])
        self.assertAlmostEqual(terms['wait'], joint['cross_wait'] + joint['charge_wait'])
        expected = sum(.25 * terms[k] / max(ref[k], floor) for k, floor in
                       (('makespan', 1), ('tardiness', 60), ('wait', 60), ('energy', 1)))
        self.assertAlmostEqual(joint['objective'], expected)
        # Floors keep a zero reference term (one easy task: no lateness or queueing) finite.
        norm = normaliser([simple_task()], Config())
        self.assertEqual(norm['coefficients']['tardiness'], .25 / 60)
        self.assertAlmostEqual(objective(dict(makespan=50, tardiness=0, wait=0, energy=3), norm), .5)

    def test_validator_rejects_tampered_traces(self):
        result = simulate([simple_task(pickup='A4', dropoff='A1'), simple_task(task_id='two', pickup='B4', dropoff='B1')], strategy='fcfs')
        self.assertEqual(validate_result(result), [])
        # A crossing booked under a bay label must still count as physical use of X.
        relabel = copy.deepcopy(result)
        cross = next(a for a in relabel['actions'] if a['action'] == 'cross' and a['robot_id'] == 'R3')
        cross['resource'] = 'C1'
        self.assertIn('R3: invalid crossing', validate_result(relabel))
        # Executing before release while keeping the task record's start time.
        early = copy.deepcopy(simulate([simple_task(release_time=100)], strategy='fcfs'))
        early['actions'] = [a for a in early['actions'] if a['action'] != 'idle']
        for a in early['actions']:
            a['start_time'] -= 100
            a['end_time'] -= 100
        self.assertIn('one: precedence/release violation', validate_result(early))
        forged = copy.deepcopy(result)
        forged['tasks'][0]['completion_time'] -= 10
        self.assertIn(f"{forged['tasks'][0]['task_id']}: completion/tardiness does not match the action trace", validate_result(forged))
        teleport = copy.deepcopy(result)
        last = max((a for a in teleport['actions'] if a['robot_id'] == 'R2'), key=lambda a: a['end_time'], default=None)
        teleport['actions'].append(dict(robot_id='R2', task_id='', action='idle', node='A4', from_node='A1', to_node='A4',
                                        start_time=0, end_time=10, start_soc=65, end_soc=65, energy=0, resource='', distance_m=0))
        self.assertIsNone(last)
        self.assertIn('R2: stationary action moves the robot or stops in X', validate_result(teleport))
        lying = copy.deepcopy(result)
        lying['robots'][0]['initial_soc'] = 90
        self.assertIn('R1: initial SOC does not match the configuration', validate_result(lying))

    def test_validator_rejects_forged_occupancy_and_capacity(self):
        result = simulate([simple_task(pickup='A4', dropoff='A1')], strategy='fcfs')
        ghost = copy.deepcopy(result)
        for item in ghost['actions'] + ghost['tasks']:
            item['robot_id'] = 'R9'
        self.assertIn('Robot roster must be exactly R1-R4 and every task/action must use it', validate_result(ghost))
        linger = copy.deepcopy(result)
        cross = next(a for a in linger['actions'] if a['action'] == 'cross')
        for a in linger['actions']:
            if a['start_time'] >= cross['end_time']:
                a['start_time'] += 100
                a['end_time'] += 100
        self.assertIn('R1: does not leave X immediately after crossing', validate_result(linger))
        double = copy.deepcopy(simulate([simple_task()], strategy='fcfs'))
        unload = next(a for a in double['actions'] if a['action'] == 'unload')
        unload.update(action='load', task_id='other')
        self.assertIn('R1: loads a second task while carrying one', validate_result(double))
        busy = simulate(generate_tasks(40, 1), {'chargers': 1}, 'joint')
        charges = sorted((a for a in busy['actions'] if a['action'] == 'charge'), key=lambda a: a['start_time'])
        first = charges[0]
        later = next(a for a in charges if a['robot_id'] != first['robot_id'])
        tampered = copy.deepcopy(busy)
        departure = next(a for a in tampered['actions'] if a['robot_id'] == first['robot_id'] and a['from_node'] == 'C1'
                         and a['start_time'] >= first['end_time'] - 1e-9)
        departure['start_time'], departure['end_time'] = later['start_time'] + 1, later['start_time'] + 21
        tampered['reservations']['C1'] = []
        self.assertIn('C1: physically overlapping use', validate_result(tampered))
        two_bays = simulate(generate_tasks(40, 1), {}, 'joint')
        self.assertTrue(any(a['to_node'] == 'C2' for a in two_bays['actions']))
        disabled = copy.deepcopy(two_bays)
        disabled['config']['chargers'] = 1
        self.assertIn('Robot entered a disabled charging bay', validate_result(disabled))

    def test_min_soc_counts_idle_robots(self):
        result = simulate([simple_task()], {'robot_soc': [100, 40, 40, 40]}, 'fcfs')
        self.assertEqual(result['summary']['min_soc'], 40)

    def test_generator_rejects_release_window_beyond_schema(self):
        with self.assertRaises(InputError):
            generate_tasks(200, 42, release_gap=600)
        validate_tasks(generate_tasks(144, 42, release_gap=600))


if __name__ == '__main__':
    unittest.main()
