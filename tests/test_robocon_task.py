import copy
from concurrent.futures import Future
import contextlib
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest

import robocon_task as task


class FakeClient:
    def __init__(self):
        self.calls = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.calls.append((request.id, future))
        return future


class FakeReader:
    def __init__(self):
        self.positions = [2048]*5
        self.sample_started = 0
        self.updated = 0
        self.ok = True

    def fresh(self, now):
        return self.ok


class TaskTests(unittest.TestCase):
    def setUp(self):
        self.cfg = task.demo_config()

    def test_rule_order_and_existing_before_new(self):
        jobs = task.jobs_for(self.cfg)
        self.assertEqual([j['marker_id'] for j in jobs[:3]], [1, 3, 5])
        self.assertEqual([j['marker_id'] for j in jobs[3:6]], [2, 4, 6])
        self.assertTrue(all(j['slot'] == 0 for j in jobs[3:6]))
        self.assertTrue(all(j['slot'] == 1 for j in jobs[6:]))
        task.validate_config(self.cfg, execute=True)

    def test_shipped_map_matches_user_and_leaves_measurements_unset(self):
        cfg = json.loads(Path(task.__file__).with_name('robocon_task_config.json').read_text(encoding='utf-8'))
        self.assertEqual(cfg['marker_shapes'], {'1': 'square', '2': 'circle', '3': 'square',
                                               '4': 'square', '5': 'square', '6': 'circle'})
        self.assertTrue(cfg['marker_map_confirmed'])
        self.assertFalse(cfg['calibration_confirmed'])
        self.assertIsNone(cfg['discard_pose'])
        jobs = task.jobs_for(cfg)
        self.assertEqual(len(jobs), 8)
        self.assertEqual([j['marker_id'] for j in jobs if j['kind'] == 'discard'], [1, 3, 5])
        circle = [j for j in jobs if j['shape'] == 'circle']
        self.assertEqual([j['slot'] for j in circle], [0, 1, 2])
        self.assertFalse(any(j['shape'] == 'triangle' for j in jobs))

    def test_actual_ids_not_hardcoded_to_pdf(self):
        self.cfg['marker_shapes'] = {'0': 'square', '2': 'square', '3': 'circle',
                                     '6': 'circle', '8': 'triangle', '10': 'triangle'}
        jobs = task.jobs_for(self.cfg)
        self.assertEqual(jobs[0]['marker_id'], 3)
        self.assertEqual(jobs[0]['kind'], 'discard')
        new = {j['shape']: j['slot'] for j in jobs if j['kind'] == 'new'}
        self.assertEqual(new, {'triangle': 2, 'square': 2, 'circle': 1})

    def test_marker_local_offset_rotates_and_z_is_absolute(self):
        self.cfg['shapes']['triangle']['local_offset_xy'] = [0.01, 0]
        marker = {'xyz': (0.18, -0.01, 5.0),
                  'quaternion': (0, 0, math.sqrt(0.5), math.sqrt(0.5))}
        pose = task.pick_pose(self.cfg, 1, marker)
        self.assertAlmostEqual(pose[0], 0.15)
        self.assertAlmostEqual(pose[1], 0.0)
        self.assertAlmostEqual(pose[2], 0.03)
        # A marker flipped around X reverses its local Y axis.
        self.assertEqual(task.quaternion_xy((1, 0, 0, 0), 0, 0.01), (0, -0.01))

    def test_selected_id_profile_is_used(self):
        self.cfg['marker_overrides']['3'] = {'grasp_z': 0.04}
        marker = {'xyz': (0.18, 0.02, 1.0), 'quaternion': (0, 0, 0, 1)}
        self.assertEqual(task.pick_pose(self.cfg, 3, marker)[2], 0.04)
        self.assertEqual(task.pick_pose(self.cfg, 5, marker)[2], 0.03)

    def test_all_demo_transfers_preflight_and_finish_without_ros(self):
        with contextlib.redirect_stdout(io.StringIO()):
            plans = task.offline_demo()
        self.assertEqual(len(plans), 9)
        task.initial_actions(self.cfg)
        reader, sent, events = FakeReader(), [], []
        def send(commands):
            sent.append(commands)
            for motor_id, ticks, _, _ in commands:
                reader.positions[motor_id-11] = ticks
        executor = task.Executor(self.cfg, send, reader,
                                 lambda kind, **kw: events.append((kind, kw)))
        executor.start(plans[0]['actions'])
        for now in range(1000):
            reader.sample_started = now-0.01
            reader.updated = now
            if executor.tick(now):
                break
        else:
            self.fail('Executor never finished the transfer')
        self.assertIsNone(executor.error)
        self.assertEqual(len(sent), len(plans[0]['actions']))
        grips = [commands[0][1] for commands in sent if commands[0][0] == 15]
        self.assertEqual(grips, [int(2048-50/0.088), int(2048+10/0.088), int(2048-50/0.088)])

    def test_bad_route_fails_before_any_action_can_be_executed(self):
        self.cfg['clearance_z'] = 0.5
        with self.assertRaises(ValueError):
            task.plan_job(self.cfg, task.jobs_for(self.cfg)[0], (0.15, 0, 0.03, 90))

    def test_feedback_failure_stops_and_does_not_issue_next_action(self):
        sent, reader = [], FakeReader()
        reader.ok = False
        executor = task.Executor(self.cfg, sent.append, reader, lambda *a, **k: None)
        actions = [{'kind': 'move', 'ticks': [2048]*4, 'label': 'first'},
                   {'kind': 'grip', 'ticks': [2100], 'label': 'must not close'}]
        executor.start(actions)
        executor.tick(0)
        executor.tick(self.cfg['move_timeout_s']+1)
        self.assertEqual(len(sent), 1)
        self.assertIsNotNone(executor.error)
        self.assertEqual(len(executor.queue), 0)

    def test_old_feedback_cannot_complete_a_new_motion(self):
        reader, sent = FakeReader(), []
        executor = task.Executor(self.cfg, sent.append, reader, lambda *a, **k: None)
        executor.start([{'kind': 'move', 'ticks': [2048]*4, 'label': 'first'}])
        executor.tick(10)
        self.assertFalse(executor.tick(11))
        reader.sample_started = 10.1
        self.assertTrue(executor.tick(11.1))

    def test_new_product_without_existing_front_is_rejected(self):
        self.cfg['marker_shapes'].pop('2')
        with self.assertRaisesRegex(ValueError, 'retained existing'):
            task.jobs_for(self.cfg)

    def test_unconfigured_coordinates_and_unconfirmed_flags_are_rejected(self):
        self.cfg['rows']['circle']['slots'][0] = None
        self.cfg['calibration_confirmed'] = False
        with self.assertRaises(ValueError) as error:
            task.validate_config(self.cfg, execute=True)
        self.assertIn('calibration_confirmed', str(error.exception))
        self.assertIn('rows.circle.slots.0', str(error.exception))

    def test_teaching_and_shape_grasp_calibration_do_not_move(self):
        task.teach_value(self.cfg, 'rows.square.slots.0', (0.15, 0, 0.035, 90))
        self.assertEqual(self.cfg['rows']['square']['slots'][0], [0.15, 0, 0.035, 90])
        task.teach_value(self.cfg, 'shapes.square.grasp_z', (0.15, 0, 0.035, 90))
        self.assertEqual(self.cfg['shapes']['square']['grasp_z'], 0.035)
        marker = {'xyz': (0.18, -0.01, 0.03),
                  'quaternion': (0, 0, math.sqrt(0.5), math.sqrt(0.5))}
        shape, offset = task.calibrate_marker(self.cfg, 1, marker, (0.15, 0, 0.03, 90))
        self.assertEqual(shape, 'triangle')
        self.assertAlmostEqual(offset[0], 0.01)
        self.assertAlmostEqual(offset[1], 0)
        pose = task.pick_pose(self.cfg, 1, marker)
        self.assertAlmostEqual(pose[0], 0.15)
        self.assertAlmostEqual(pose[1], 0)
        with self.assertRaises(ValueError):
            task.teach_value(self.cfg, '__class__.__dict__', (0.15, 0, 0.03, 90))


class SensorTests(unittest.TestCase):
    def test_marker_stability_is_per_id_fresh_and_requires_independent_frames(self):
        cache = task.MarkerCache(task.demo_config())
        for i in range(3):
            cache.add(4, (0.17, 0.02, 0.03), (0, 0, 0, 1), 10+i*0.1)
            cache.add(6, (0.12, -0.03, 0.03), (0, 0, 0, 1), 10+i*0.1)
        self.assertEqual(cache.stable(4, 10.3)['xyz'][0], 0.17)
        self.assertEqual(cache.stable(6, 10.3)['xyz'][0], 0.12)
        self.assertIsNone(cache.stable(4, 20))
        for _ in range(3):
            cache.add(8, (0.17, 0.02, 0.03), (0, 0, 0, 1), 10)
        self.assertIsNone(cache.stable(8, 10.3))
        cache.add(9, (math.nan, 0, 0), (0, 0, 0, 1), 10)
        self.assertNotIn(9, cache.history)

    def test_jitter_is_not_treated_as_a_stable_detection(self):
        cache = task.MarkerCache(task.demo_config())
        for i, x in enumerate((0.1, 0.2, 0.1)):
            cache.add(4, (x, 0, 0.03), (0, 0, 0, 1), 10+i*0.1)
        self.assertIsNone(cache.stable(4, 10.3))

    def test_controller_reads_five_ids_and_invalidates_failed_samples(self):
        client = FakeClient()
        reader = task.JointReader(client, types.SimpleNamespace)
        reader.tick(10)
        for index in range(5):
            self.assertEqual(client.calls[-1][0], index+11)
            client.calls[-1][1].set_result(types.SimpleNamespace(position=2048+index))
            reader.tick(10.1+index*0.1)
        self.assertEqual(reader.positions, [2048, 2049, 2050, 2051, 2052])
        self.assertEqual(reader.sample_started, 10)
        self.assertTrue(reader.fresh(10.6))
        reader.tick(11)
        reader.tick(12.6)
        self.assertFalse(reader.fresh(12.6))
        self.assertIn('timeout', reader.error)

    def test_two_file_deployment_runs_demo_without_local_modules(self):
        base = Path(task.__file__).parent
        with tempfile.TemporaryDirectory() as folder:
            for name in ('robocon_task.py', 'robocon_task_config.json'):
                Path(folder, name).write_bytes(Path(base, name).read_bytes())
            result = subprocess.run([sys.executable, str(Path(folder, 'robocon_task.py')), '--demo'],
                                    cwd=folder, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('9 transfers planned', result.stdout)


class RuntimeTests(unittest.TestCase):
    def simulate(self, missing_id=None):
        from unittest.mock import patch
        cfg = task.demo_config()
        cfg['marker_shapes'] = {'1': 'square', '2': 'circle', '3': 'square',
                                '4': 'square', '5': 'square', '6': 'circle'}
        cfg['supplies'] = [i for i in cfg['supplies'] if i['shape'] != 'triangle']
        cfg['rows']['circle']['slots'] = [[0.11, 0.05, 0.03, 90],
                                         [0.15, 0.05, 0.03, 90], [0.19, 0.05, 0.03, 90]]
        task.validate_config(cfg, execute=True)
        clock = [100.0]
        node = types.SimpleNamespace(markers=task.MarkerCache(cfg), reader=FakeReader(),
                                     publisher=types.SimpleNamespace(get_subscription_count=lambda: 1),
                                     destroy_node=lambda: None)
        node.reader.positions[4] = int(2048+10/0.088)
        node.reader.tick = lambda now: None
        commands = []
        def send(items):
            commands.append(items)
            for motor_id, tick, _, _ in items:
                node.reader.positions[motor_id-11] = tick
        node.send = send
        def spin_once(*args, **kw):
            clock[0] += 0.05
            node.reader.sample_started = clock[0]
            node.reader.updated = clock[0]
            for marker_id in range(1, 7):
                if marker_id != missing_id:
                    node.markers.add(marker_id, (0.18, (marker_id-3)*0.015, 0.03),
                                     (0, 0, 0, 1), clock[0])
        ros = types.SimpleNamespace(init=lambda **k: None, ok=lambda: True,
                                    shutdown=lambda: None, spin_once=spin_once)
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as folder:
            args = types.SimpleNamespace(execute=True, observe=False, teach=None,
                                         calibrate_marker=None, prepare=False, ros_args=[],
                                         log=str(Path(folder, 'run.jsonl')))
            with patch.dict(sys.modules, {'rclpy': ros}), patch.object(task, 'make_node', return_value=node), \
                    patch.object(task.time, 'monotonic', side_effect=lambda: clock[0]), \
                    contextlib.redirect_stdout(output):
                if missing_id is None:
                    task.run_ros(args, cfg)
                else:
                    with self.assertRaisesRegex(RuntimeError, 'skipped'):
                        task.run_ros(args, cfg)
        return [json.loads(line) for line in output.getvalue().splitlines()], commands

    def test_confirmed_real_mapping_runs_to_automatic_completion(self):
        events, commands = self.simulate()
        result = [e for e in events if e['event'] == 'finished'][-1]
        self.assertEqual(result['skipped'], [])
        self.assertEqual(len(result['sequences_finished']), 8)
        self.assertEqual(result['sequences_finished'][:6],
                         ['discard_1', 'discard_3', 'discard_5', 'display_2', 'display_4', 'display_6'])
        self.assertGreater(len(commands), 100)

    def test_missing_existing_square_skips_its_new_stock_automatically(self):
        events, commands = self.simulate(missing_id=4)
        result = [e for e in events if e['event'] == 'finished'][-1]
        self.assertEqual(set(result['skipped']), {'display_4', 'new_square'})
        self.assertIn('new_circle', result['sequences_finished'])


if __name__ == '__main__':
    unittest.main()
