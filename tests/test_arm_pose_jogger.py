import math
import unittest
from unittest.mock import patch

from arm_kinematics import forward_kinematics, inverse_kinematics
from arm_pose_jogger import JogSession, make_jog_node


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
        modules = {'rclpy.node': types.SimpleNamespace(Node=FakeNode),
                   'dynamixel_sdk_custom_interfaces.msg':
                       types.SimpleNamespace(SetPosition=types.SimpleNamespace)}
        with patch.dict(sys.modules, modules):
            node = make_jog_node('/set_position')
            with self.assertRaises(ValueError):
                node.send([(11, 2048, 25, 5)])
            self.assertEqual(sent, [])
            count[0] = 1
            node.send([(11, 2048, 25, 5), (12, 2000, 25, 5)])
            self.assertEqual([(m.id, m.position, m.velocity, m.acceleration)
                              for m in sent], [(11, 2048, 25, 5), (12, 2000, 25, 5)])


if __name__ == '__main__':
    unittest.main()
