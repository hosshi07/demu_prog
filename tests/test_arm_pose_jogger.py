import math
import unittest
from unittest.mock import patch

from arm_kinematics import forward_kinematics, inverse_kinematics
from arm_pose_jogger import JogSession, PositionReader, make_jog_node


class JogTests(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.clock = patch('arm_pose_jogger.time.monotonic', return_value=10.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.session = JogSession(self.sent.append)

    def start(self):
        self.session.move(self.session.start)
        self.now.return_value += 2.1

    def test_no_startup_motion_and_explicit_start_matches_ik(self):
        self.assertEqual(self.sent, [])
        self.assertIsNone(self.session.pose)
        with self.assertRaises(ValueError):
            self.session.jog(0, 0.005)
        self.start()
        self.assertEqual([cmd[0] for cmd in self.sent[0]], [11, 12, 13, 14])
        xyz, pitch, _ = forward_kinematics(self.session.angles)
        for value, expected in zip(xyz, self.session.start[:3]):
            self.assertAlmostEqual(value, expected, places=10)
        self.assertAlmostEqual(math.degrees(pitch), 90)
        self.assertTrue(all(cmd[2:] == (25, 5) for cmd in self.sent[0]))

    def test_increment_copy_and_no_queued_clicks(self):
        self.start()
        self.session.jog(0, 0.005)
        self.assertEqual(self.session.pose_text(), '[0.1050, 0.0000, 0.1000, 90.00]')
        self.assertEqual(self.session.code_text(),
                         'open_manipulator_x.move([0.1050, 0.0000, 0.1000, 90.00], 25, 5)')
        with self.assertRaises(ValueError):
            self.session.jog(0, 0.005)
        self.assertEqual(len(self.sent), 2)
        self.now.return_value += 2.1
        self.assertEqual(len(self.sent), 2)  # Rejected click is never queued.

    def test_invalid_pose_keeps_last_target_and_sends_nothing(self):
        self.start()
        before = self.session.pose
        for pose in ((10, 0, 0.1, 90), (0, 0, 0.1, 90),
                     (0.1, 0, -0.1, 90), (math.nan, 0, 0.1, 90),
                     (0.1, 0, 0.1, 1000)):
            with self.assertRaises(ValueError):
                self.session.move(pose)
            self.assertEqual(self.session.pose, before)
            self.assertEqual(len(self.sent), 1)

    def test_publish_failure_does_not_commit_target(self):
        self.start()
        before = self.session.pose
        def disconnected(commands):
            raise ValueError('No controller')
        self.session.send = disconnected
        with self.assertRaises(ValueError):
            self.session.jog(1, 0.005)
        self.assertEqual(self.session.pose, before)
        self.assertFalse(self.session.busy())

    def test_gripper_does_not_change_xyz(self):
        self.start()
        before = self.session.pose
        self.session.gripper(True)
        self.assertEqual(self.sent[-1], [(15, int(2048-50/0.088), 25, 5)])
        self.assertEqual(self.session.pose, before)

    def test_adopt_current_pose_sends_no_motor_commands(self):
        angles, _ = inverse_kinematics((0.15, 0.0, 0.1, 90.0))
        self.session.adopt(angles)
        self.assertEqual(self.sent, [])
        self.assertAlmostEqual(self.session.pose[0], 0.15)
        self.assertAlmostEqual(self.session.pose[3], 90.0)

    def test_adopt_rejects_different_ik_branch(self):
        with self.assertRaises(ValueError):
            self.session.adopt([0, 0, -2.0, 2.0])
        self.assertIsNone(self.session.pose)
        self.assertEqual(self.sent, [])

    def test_each_axis_and_pitch_increment(self):
        self.start()
        for axis, delta in ((0, -0.005), (1, 0.005), (2, 0.005), (3, 3.0)):
            before = self.session.pose
            self.session.jog(axis, delta)
            for index in range(4):
                self.assertAlmostEqual(self.session.pose[index],
                                       before[index] + (delta if axis == index else 0))
            self.now.return_value += 2.1


class PublisherTests(unittest.TestCase):
    def test_disconnect_sends_nothing_and_connection_sends_fields(self):
        import sys
        import types
        sent = []
        count = [0]
        publisher = types.SimpleNamespace(get_subscription_count=lambda: count[0],
                                          publish=sent.append)
        class FakeNode:
            def __init__(self, name):
                pass
            def create_publisher(self, *args):
                return publisher
            def create_client(self, *args):
                return object()
        modules = {'rclpy.node': types.SimpleNamespace(Node=FakeNode),
                   'dynamixel_sdk_custom_interfaces.msg':
                       types.SimpleNamespace(SetPosition=types.SimpleNamespace),
                   'dynamixel_sdk_custom_interfaces.srv': types.SimpleNamespace(
                       GetPosition=types.SimpleNamespace(Request=types.SimpleNamespace))}
        with patch.dict(sys.modules, modules):
            node = make_jog_node('/set_position')
            with self.assertRaises(ValueError):
                node.send([(11, 2048, 25, 5)])
            self.assertEqual(sent, [])
            count[0] = 1
            node.send([(11, 2048, 25, 5), (12, 2000, 25, 5)])
            self.assertEqual([(m.id, m.position, m.velocity, m.acceleration)
                              for m in sent], [(11, 2048, 25, 5), (12, 2000, 25, 5)])


class ReaderTests(unittest.TestCase):
    def setUp(self):
        from test_arm_pose_viewer import FakeClient
        import types
        self.client = FakeClient()
        self.reader = PositionReader(self.client, types.SimpleNamespace)
        self.clock = patch('arm_pose_jogger.time.monotonic', return_value=10.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)

    def complete(self, value):
        import types
        self.client.calls[-1][1].set_result(types.SimpleNamespace(position=value))
        self.reader.tick()

    def test_reads_four_real_encoder_values_and_marks_stale(self):
        _, ticks = inverse_kinematics((0.15, 0.0, 0.1, 90.0))
        self.reader.tick()
        for index, value in enumerate(ticks):
            self.assertEqual(self.client.calls[-1][0], index+11)
            self.assertIsNone(self.reader.sample)
            self.complete(value)
        self.assertTrue(self.reader.fresh())
        for actual, expected in zip(self.reader.sample[0], (0.15, 0.0, 0.1)):
            self.assertAlmostEqual(actual, expected, delta=0.001)
        self.now.return_value = 12.1
        self.assertFalse(self.reader.fresh())

    def test_timeout_and_invalid_data_discard_partial_sweep(self):
        self.reader.tick()
        self.complete(2048)
        self.now.return_value = 11.1
        self.reader.tick()
        self.assertFalse(self.reader.fresh())
        self.assertEqual(self.reader.values, [])
        self.now.return_value = 12.2
        self.reader.tick()
        self.assertEqual(self.client.calls[-1][0], 11)
        self.complete(-1)
        self.assertIsNone(self.reader.sample)
        self.assertIn('outside', self.reader.status)

    def test_unavailable_service_never_claims_current_position(self):
        self.client.ready = False
        self.reader.tick()
        self.assertEqual(self.client.calls, [])
        self.assertFalse(self.reader.fresh())


class StandaloneTests(unittest.TestCase):
    def test_help_works_with_only_one_python_file(self):
        import os
        from pathlib import Path
        import subprocess
        import sys
        import tempfile
        with tempfile.TemporaryDirectory() as folder:
            script = Path(folder) / 'arm_pose_jogger.py'
            script.write_bytes(Path(__file__).resolve().parents[1].joinpath(
                'arm_pose_jogger.py').read_bytes())
            env = dict(os.environ, PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1')
            result = subprocess.run([sys.executable, str(script), '--help'],
                                    cwd=folder, env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--service', result.stdout)


if __name__ == '__main__':
    unittest.main()
