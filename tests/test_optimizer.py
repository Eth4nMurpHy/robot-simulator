"""Optional OR-Tools integration and small-instance optimality checks."""
import copy
import importlib.util
import json
import os
import subprocess
import sys
import unittest

from customization import default_map
from simulator import Config, InputError, Planner, generate_tasks, random_robot_soc, simulate, validate_result

HAS_ORTOOLS = importlib.util.find_spec('ortools') is not None


def chain_tasks(count):
    tasks = []
    for i in range(count):
        area = 'AB'[i % 2]
        pickup, dropoff = (1, 2) if i < 4 else (2, 3)
        tasks.append(dict(task_id=f'T{i}', pickup=f'{area}{pickup}', dropoff=f'{area}{dropoff}',
                          release_time=0, load_time=15, unload_time=15, due_time=1000, priority=1))
    return tasks


@unittest.skipUnless(HAS_ORTOOLS, 'OR-Tools is optional; use .venv/bin/python')
class OptimizerTests(unittest.TestCase):
    def check_result(self, result, tasks):
        self.assertTrue(result['validation']['passed'])
        self.assertEqual(validate_result(result), [])
        self.assertEqual({t['task_id'] for t in tasks}, {t['task_id'] for t in result['tasks']})
        for key in ('strategy', 'strategy_label', 'config', 'nodes', 'map', 'robots', 'tasks',
                    'actions', 'reservations', 'events', 'validation', 'summary', 'solver'):
            self.assertIn(key, result)
        self.assertIn('objective', result['summary'])
        self.assertGreaterEqual(result['summary']['min_soc']+1e-6, result['config']['safety_soc'])

    def test_seed_map_soc_safety_matrix_and_hint_dominance(self):
        for n, seed, chargers, safety in [(20,1,1,10), (40,1,2,20), (20,2,2,30),
                                          (40,2,1,10), (20,3,1,20), (40,3,2,30)]:
            with self.subTest(n=n, seed=seed, chargers=chargers, safety=safety):
                tasks = generate_tasks(n, seed)
                cfg = dict(chargers=chargers, safety_soc=safety, robot_soc=random_robot_soc(seed,70,100))
                world = default_map() if seed == 3 else None
                if world:
                    world['speed_m_per_s'] = 1.37
                    for i, edge in enumerate(world['edges']):
                        edge['length_m'] += .017 * i
                before = copy.deepcopy((tasks, cfg, world))
                hint = simulate(tasks, cfg, 'joint', world)
                result = simulate(tasks, cfg, 'cpsat', world, dict(time_limit=3, seed=seed))
                self.check_result(result, tasks)
                self.assertLessEqual(result['summary']['objective'], hint['summary']['objective']+1e-6)
                self.assertEqual((tasks, cfg, world), before)

    def test_tiny_proven_optima_match_lower_bound(self):
        for count in (4,5,6):
            with self.subTest(count=count):
                tasks = chain_tasks(count)
                result = simulate(tasks, strategy='cpsat', custom_strategy=dict(time_limit=10))
                # Every job needs >=50 seconds and >=3 SOC. With >2 A jobs,
                # one of two A robots must do two jobs, hence Cmax >=100.
                # These chains achieve the bound with no positioning or waiting, and every
                # job uses exactly 3 SOC, so the optimum attains each term's own lower bound.
                terms = result['summary']['objective_terms']
                self.check_result(result, tasks)
                self.assertEqual(result['solver']['status'], 'OPTIMAL')
                self.assertIsNone(result['solver']['fallback'])
                self.assertAlmostEqual(terms['makespan'], 50 if count == 4 else 100, places=5)
                self.assertAlmostEqual(terms['energy'], 3*count, places=5)
                self.assertEqual((terms['tardiness'], terms['wait']), (0, 0))
                self.assertAlmostEqual(result['solver']['best_bound'], result['summary']['objective'], places=5)

    def test_charge_amount_is_decided_and_bay_is_vacated(self):
        tasks = [dict(task_id='one', pickup='A4', dropoff='A3', due_time=2000)]
        # Without the terminal reserve the optimum charges just enough for the delivery itself.
        result = simulate(tasks, dict(initial_soc=43,safety_soc=30,chargers=1), 'cpsat',
                          custom_strategy=dict(time_limit=10, terminal_reserve=False))
        self.check_result(result, tasks)
        self.assertEqual(result['solver']['status'], 'OPTIMAL')
        self.assertAlmostEqual(result['solver']['best_bound'], result['summary']['objective'], places=5)
        charges = [a for a in result['actions'] if a['action']=='charge']
        self.assertEqual(len(charges), 1)
        self.assertAlmostEqual(charges[0]['end_soc'], 50.2, places=5)
        self.assertLess(result['summary']['objective'], result['solver']['hint_objective'])
        actions = [a for a in result['actions'] if a['robot_id']==charges[0]['robot_id']]
        departure = actions[actions.index(charges[0])+1]
        self.assertEqual(departure['to_node'], 'H')
        self.assertEqual(departure['start_time'], charges[0]['end_time'])

    def test_charger_arrival_at_safety_uses_action_tolerance(self):
        plan = Planner(Config())
        robot = plan.robots[0]
        robot['soc'] = 22.6 - 1e-14  # Exactly safety after the 12.6% trip, apart from roundoff.
        plan.charge(robot,bay_choice='C1',target_soc=50)
        self.assertGreaterEqual(min(a['end_soc'] for a in plan.actions),10-1e-8)
        self.assertEqual(robot['node'],'H')

    def test_can_solve_when_conservative_heuristic_has_no_hint(self):
        tasks = [dict(task_id='one', pickup='A3', dropoff='A4', due_time=600)]
        cfg = dict(initial_soc=40,safety_soc=30)
        with self.assertRaises(InputError):
            simulate(tasks,cfg,'joint')
        result = simulate(tasks,cfg,'cpsat',None,dict(terminal_reserve=False))
        self.check_result(result,tasks)
        self.assertEqual(result['solver']['status'],'OPTIMAL')
        self.assertIsNone(result['solver']['hint_objective'])

    def test_terminal_reserve_matches_heuristic_reserve(self):
        tasks = [dict(task_id='one', pickup='A4', dropoff='A3', due_time=2000)]
        cfg = dict(initial_soc=43, safety_soc=30, chargers=1)
        free = simulate(tasks, cfg, 'cpsat', None, dict(time_limit=10, terminal_reserve=False))
        fair = simulate(tasks, cfg, 'cpsat', None, dict(time_limit=10))
        self.check_result(fair, tasks)
        last = lambda r: max((a for a in r['actions'] if a['robot_id'] == r['tasks'][0]['robot_id']), key=lambda a: a['end_time'])['end_soc']
        # The default keeps the heuristics' end-of-horizon reserve, so it must finish with more charge.
        self.assertGreater(last(fair), last(free) + 5)
        from optimizer import options_for
        with self.assertRaises(InputError):
            options_for(dict(terminal_reserve='yes'))
        self.assertTrue(options_for(dict(terminal_reserve=False), rolling=True)['terminal_reserve'])

    def test_integer_grid_hint_is_complete_and_feasible(self):
        from optimizer import JointModel, options_for
        from ortools.sat.python import cp_model
        from simulator import normaliser, plan_tasks, validate_tasks
        world = default_map()
        world['speed_m_per_s'] = 1.37
        tasks = validate_tasks(generate_tasks(20,2))
        base = Planner(Config(chargers=1),world)
        opts = options_for(None)
        opts['norm'] = normaliser(tasks, base.c, world)
        hint = plan_tasks(base.clone(),tasks)
        model = JointModel(base,tasks,hint,opts)
        self.assertTrue(model.hint_feasible)
        solver = cp_model.CpSolver()
        solver.parameters.num_search_workers = 1
        solver.parameters.fix_variables_to_their_hinted_value = True
        self.assertEqual(solver.solve(model.model),cp_model.OPTIMAL)

    def test_rolling_decisions_ignore_unreleased_tasks(self):
        # Review regression: full-instance normalisation once changed early commitments
        # when orders released later were added.
        base = generate_tasks(5, 1, a_share=1)
        for task in base:
            task.update(release_time=0, due_time=100 + task['due_time'] % 200)
        later = [dict(task_id=f'F{i}', release_time=600, pickup='A4', dropoff='A1', load_time=10,
                      unload_time=10, due_time=600, priority=5) for i in range(5)]
        options = dict(workers=1, time_limit=5, deterministic_time=.2)
        early = lambda tasks: [(h['replan_time'], h['committed_tasks']) for h in
                               simulate(tasks, {}, 'rolling', None, options)['solver']['replans'] if h['replan_time'] < 600]
        self.assertEqual(early(base), early(base + later))

    def test_rolling_release_audit_and_frozen_reservations(self):
        tasks = generate_tasks(40,2)
        release = {t['task_id']:t['release_time'] for t in tasks}
        result = simulate(tasks,dict(chargers=1),'rolling',custom_strategy=dict(time_limit=1))
        self.check_result(result,tasks)
        history = result['solver']['replans']
        self.assertEqual(result['solver']['num_solves'],len(history))
        committed = set()
        has_frozen = False
        for step in history:
            tick = step['replan_time']
            self.assertEqual(tick % 60,0)
            self.assertTrue(set(step['optimized_tasks']) <= set(step['known_tasks']))
            for tid in step['known_tasks']:
                self.assertLessEqual(release[tid],tick)
                self.assertNotIn(tid,committed)
            for tid in step['committed_tasks']:
                self.assertIn(tid,step['optimized_tasks'])
            committed.update(step['committed_tasks'])
            for job in step['commitments']:
                self.assertGreaterEqual(job['start_time']+1e-7,tick)
                self.assertLess(job['start_time'],tick+60)
            for resource, frozen in step['frozen_reservations'].items():
                has_frozen |= bool(frozen)
                for reservation in frozen:
                    self.assertIn(reservation,result['reservations'][resource])
        self.assertTrue(has_frozen)
        self.assertEqual(committed,set(release))
        self.assertIsNone(result['solver']['best_bound'])  # Local bounds are not global online bounds.

    def test_rolling_edited_map_and_soc_thresholds(self):
        for seed, safety, chargers in [(1,20,2),(3,30,1)]:
            world = default_map()
            world['speed_m_per_s'] = 1.37
            tasks = generate_tasks(20,seed)
            cfg = dict(safety_soc=safety,chargers=chargers,robot_soc=random_robot_soc(seed,70,100))
            result = simulate(tasks,cfg,'rolling',world,dict(time_limit=1,horizon_seconds=300))
            self.check_result(result,tasks)

    def test_zero_duration_actions_at_commit_boundary(self):
        tasks = [{**t, 'load_time':0, 'unload_time':0} for t in chain_tasks(6)]
        result = simulate(tasks,strategy='rolling',custom_strategy=dict(replan_seconds=20))
        self.check_result(result,tasks)
        self.assertEqual(sum(a['action']=='load' for a in result['actions']),6)

    def test_future_orders_do_not_change_earlier_decisions(self):
        first = chain_tasks(4)
        future = dict(task_id='future',pickup='A4',dropoff='A1',release_time=600,due_time=900)
        a = simulate(first,strategy='rolling')
        b = simulate(first+[future],strategy='rolling')
        self.assertEqual(a['solver']['replans'][0]['commitments'],b['solver']['replans'][0]['commitments'])
        self.assertEqual(a['tasks'],[t for t in b['tasks'] if t['task_id']!='future'])

    def test_deterministic_across_hash_seeds(self):
        code = """import json
from simulator import simulate,generate_tasks
r=simulate(generate_tasks(20,2),strategy='cpsat',custom_strategy=dict(time_limit=30,deterministic_time=.15,seed=7))
print(json.dumps([r['tasks'],r['actions'],r['solver']['objective'],r['solver']['best_bound']],sort_keys=True))
"""
        outputs = [subprocess.check_output([sys.executable,'-c',code],env={**os.environ,'PYTHONHASHSEED':str(seed)},text=True) for seed in (1,2)]
        self.assertEqual(json.loads(outputs[0]),json.loads(outputs[1]))

    def test_limits_fallback_and_invalid_options(self):
        tasks = generate_tasks(20,1)
        result = simulate(tasks,strategy='cpsat',custom_strategy=dict(time_limit=.001))
        self.check_result(result,tasks)
        self.assertIsNotNone(result['solver']['fallback'])
        self.assertEqual(result['summary']['objective'],result['solver']['hint_objective'])
        for options in ([],dict(workers=1.5),dict(time_limit=0),dict(seed=True),dict(unknown=1)):
            with self.subTest(options=options), self.assertRaises(InputError):
                simulate(chain_tasks(4),strategy='cpsat',custom_strategy=options)
        result = simulate(chain_tasks(4),strategy='cpsat',custom_strategy=dict(max_charges_per_robot=0))
        self.assertEqual(result['summary']['charging_sessions'],0)


if __name__ == '__main__':
    unittest.main()
