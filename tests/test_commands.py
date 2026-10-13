import copy
import unittest
from command_plan import CommandPlanError, parse_command_plan, DEFAULT_COMMAND_PLAN
from customization import default_map, DEFAULT_RULES
from simulator import simulate, validate_result, InputError


def command(robot='R1', at=0, action='execute_task', **extra):
    return dict(robot_id=robot, at=at, action=action, **extra)


def task(tid='T001', **extra):
    return {**dict(task_id=tid, release_time=0, pickup='A1', dropoff='A2', load_time=15,
                   unload_time=15, due_time=600, priority=1), **extra}


def execute(commands, tasks=None, **kwargs):
    return simulate(tasks if tasks is not None else [task()], strategy='custom',
                    custom_strategy={'name': 'Manual plan', 'commands': commands}, **kwargs)


class CommandTests(unittest.TestCase):
    def errors(self, commands, **kwargs):
        with self.assertRaises(CommandPlanError) as context:
            execute(commands, **kwargs)
        return context.exception.errors

    def test_exact_task_assignment_and_time(self):
        result = execute([command('R2', 100, task_id='T001')])
        self.assertEqual(result['commands'][0]['start_time'], 100)
        self.assertEqual(result['tasks'][0]['robot_id'], 'R2')
        self.assertEqual(result['tasks'][0]['start_time'], 100)
        self.assertEqual(result['tasks'][0]['completion_time'], 150)
        self.assertEqual(result['summary']['energy'], 3)
        self.assertEqual(validate_result(result), [])

    def test_robot_busy_is_not_queued(self):
        errors = self.errors([command(task_id='T001'), command(at=10, action='move_to', node='A3')])
        self.assertEqual(errors[0]['code'], 'ROBOT_BUSY')
        self.assertEqual(errors[0]['command_index'], 2)
        self.assertEqual(errors[0]['time'], 10)
        self.assertIn('50.000', errors[0]['message'])

    def test_command_at_exact_completion_is_valid(self):
        result = execute([command(task_id='T001'), command(at=50, action='move_to', node='A3')])
        self.assertEqual(result['commands'][1]['start_time'], 50)
        self.assertEqual(result['commands'][1]['end_time'], 70)

    def test_intersection_overlap_and_buffer_errors(self):
        for offset in (0, 7):
            errors = self.errors([command('R1', 0, 'move_to', node='H'), command('R3', offset, 'move_to', node='H')])
            conflict = next(e for e in errors if e['code'] == 'INTERSECTION_CONFLICT')
            self.assertEqual(conflict['resource'], 'X')
            self.assertEqual(conflict['other_command_index'], 1)
            self.assertEqual(conflict['command_index'], 2)
        result = execute([command('R1', 0, 'move_to', node='H'), command('R3', 10, 'move_to', node='H')])
        self.assertEqual(validate_result(result), [])

    def test_same_robot_must_respect_crossing_buffer(self):
        world = default_map()
        for e in world['edges']:
            if (e['from'], e['to']) in (('X', 'H'), ('H', 'X')):
                e['length_m'] = .1
        errors = self.errors([command(at=0, action='move_to', node='H'), command(at=86.1, action='move_to', node='A1')], world=world)
        conflict = next(e for e in errors if e['code'] == 'INTERSECTION_CONFLICT')
        self.assertEqual(conflict['robot_id'], 'R1')
        self.assertEqual(conflict['other_robot_id'], 'R1')

    def test_charger_conflict_reports_both_robots(self):
        errors = self.errors([command('R1', 0, 'charge', bay='C1', target_soc=90), command('R2', 20, 'charge', bay='C1', target_soc=90)])
        conflict = next(e for e in errors if e['code'] == 'CHARGER_CONFLICT')
        self.assertEqual(conflict['time'], 146)
        self.assertEqual({conflict['robot_id'], conflict['other_robot_id']}, {'R1', 'R2'})

    def test_parked_bay_remains_exclusive_until_departure(self):
        errors = self.errors([command('R1', 0, 'move_to', node='C1'), command('R2', 200, 'move_to', node='C1')])
        self.assertTrue(any(e['code'] == 'CHARGER_CONFLICT' for e in errors))
        result = execute([command('R1', 0, 'move_to', node='C1'), command('R1', 150, 'move_to', node='H'), command('R2', 200, 'move_to', node='C1')])
        self.assertEqual(result['reservations']['C1'][0]['end_time'], 150)
        self.assertEqual(validate_result(result), [])

    def test_charging_is_explicit_and_stays_in_requested_bay(self):
        result = execute([command('R1', 0, 'charge', bay='C2', target_soc=90)])
        charge = next(a for a in result['actions'] if a['action'] == 'charge')
        self.assertEqual(charge['resource'], 'C2')
        self.assertAlmostEqual(charge['end_soc'], 90)
        self.assertEqual(result['actions'][-1]['to_node'], 'C2')
        result = execute([command(task_id='T001')], config={'initial_soc': 40})
        self.assertFalse(any(a['action'] == 'charge' for a in result['actions']))

    def test_disabled_bay_and_target_below_arrival(self):
        self.assertEqual(self.errors([command(action='charge', bay='C2', target_soc=90)], config={'chargers': 1})[0]['code'], 'DISABLED_CHARGER')
        self.assertEqual(self.errors([command(action='charge', bay='C1', target_soc=10)])[0]['code'], 'INVALID_CHARGE_TARGET')

    def test_low_battery_and_dependent_commands(self):
        world = default_map()
        next(e for e in world['edges'] if e['from']=='A1' and e['to']=='A2')['length_m']=100
        errors = self.errors([command(task_id='T001'), command(at=200, action='wait', duration=10)], world=world, config={'initial_soc': 40, 'safety_soc': 30})
        self.assertEqual(errors[0]['code'], 'BATTERY_UNSAFE')
        self.assertEqual(errors[0]['time'], 115)
        self.assertTrue(any(e['code']=='DEPENDENT_COMMAND' for e in errors))

    def test_release_unknown_duplicate_and_wrong_loop(self):
        self.assertEqual(self.errors([command(task_id='T001')], tasks=[task(release_time=10)])[0]['code'], 'TASK_NOT_RELEASED')
        self.assertEqual(self.errors([command(task_id='missing')])[0]['code'], 'UNKNOWN_TASK')
        self.assertEqual(self.errors([command('R3', task_id='T001')])[0]['code'], 'WRONG_LOOP')
        errors = self.errors([command(task_id='T001'), command('R2', 100, task_id='T001')])
        self.assertEqual(errors[0]['code'], 'DUPLICATE_TASK')

    def test_missing_tasks_not_automatically_assigned(self):
        result = execute([command(task_id='T001')], tasks=[task(), task('T002')])
        self.assertEqual(result['summary']['completed'], 1)
        self.assertEqual(result['summary']['total_tasks'], 2)
        self.assertEqual(result['unexecuted_tasks'], ['T002'])

    def test_wait_consumption_and_chronological_execution(self):
        commands = [command(at=30, action='wait', duration=60), command(at=0, action='move_to', node='A2')]
        before = copy.deepcopy(commands)
        result = execute(commands, tasks=[])
        self.assertEqual(result['summary']['completed'], 0)
        self.assertAlmostEqual(result['summary']['energy'], 2.2)
        self.assertEqual([c['command_index'] for c in result['commands']], [2, 1])
        self.assertEqual(commands, before)
        self.assertEqual(result['summary']['makespan'], 90)

    def test_intersection_cannot_be_parked_in(self):
        self.assertTrue(any(e['code']=='INTERSECTION_BLOCKED' for e in self.errors([command(action='move_to', node='X')])))
        result = execute([command(action='move_to', node='X'), command(at=86, action='move_to', node='H')])
        self.assertEqual(validate_result(result), [])

    def test_map_edit_can_make_command_busy(self):
        world = default_map()
        next(e for e in world['edges'] if e['from']=='A1' and e['to']=='A2')['length_m']=40
        errors = self.errors([command(action='move_to', node='A2'), command(at=25, action='wait', duration=1)], world=world)
        self.assertEqual(errors[0]['code'], 'ROBOT_BUSY')
        self.assertIn('40.000', errors[0]['message'])

    def test_multiple_independent_errors(self):
        errors = self.errors([command('R1', task_id='nope'), command('R3', task_id='T001')])
        self.assertEqual({e['code'] for e in errors}, {'UNKNOWN_TASK','WRONG_LOOP'})

    def test_schema_is_data_only_and_rejects_old_weight_rules(self):
        with self.assertRaises(InputError):
            parse_command_plan(DEFAULT_RULES)
        for c in [command(action='eval'), command(at=-1, action='wait', duration=1), command(action='move_to', node='B1'), command(action='wait', duration=0), {**command(action='wait', duration=1), 'queue':True}]:
            with self.assertRaises(CommandPlanError):
                parse_command_plan({'name':'bad','commands':[c]})
        self.assertEqual(parse_command_plan(DEFAULT_COMMAND_PLAN), DEFAULT_COMMAND_PLAN)


if __name__ == '__main__':
    unittest.main()
