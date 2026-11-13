import math
import sys
import types
import unittest
from concurrent.futures import Future
from unittest.mock import patch

from arm_kinematics import forward_kinematics, ticks_to_radians, PSI
from arm_pose_viewer import make_node


class KinematicsTests(unittest.TestCase):
    def test_round_trip_supplied_inverse_kinematics(self):
        # Independent inverse calculation from the original programs, including
        # their dimensions and downward-positive tool pitch.
        for x, y, z, deg in [(0.1, -0.2, 0.05, 90), (0.1, 0.1, 0.1, 90),
                              (0.2, 0.03, 0.15, 30), (0.1, 0, 0.1, 90)]:
            pitch = math.radians(deg)
            h = z - 0.077 + 0.126 * math.sin(pitch)
            k = math.hypot(x, y) - 0.126 * math.cos(pitch)
            v = math.hypot(h, k)
            alpha = math.acos((0.13**2 + v*v - 0.124**2) / (2*0.13*v))
            beta = math.acos((0.13**2 + 0.124**2 - v*v) / (2*0.13*0.124))
            q2 = math.pi/2 - alpha - math.atan2(h, k) - PSI
            q3 = math.pi/2 - beta + PSI
            angles = [math.atan(y/x), q2, q3, pitch-q2-q3]
            xyz, actual_pitch, points = forward_kinematics(angles)
            for actual, expected in zip(xyz, (x, y, z)):
                self.assertAlmostEqual(actual, expected, places=10)
            self.assertAlmostEqual(actual_pitch, pitch)
            self.assertEqual(points[0], (0, 0, 0))

    def test_ticks_and_invalid_angles(self):
        self.assertEqual(ticks_to_radians(2048), 0)
        self.assertAlmostEqual(ticks_to_radians(3048), math.radians(88))
        with self.assertRaises(ValueError):
            forward_kinematics([0, 0, math.nan, 0])


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


class FakeNode:
    def __init__(self, name):
        self.messages = []
        self.fake_client = FakeClient()

    def create_client(self, *args):
        return self.fake_client

    def create_publisher(self, *args):
        return types.SimpleNamespace(publish=self.messages.append)

    def create_subscription(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 123))


class FakePoint:
    def __init__(self):
        self.header = types.SimpleNamespace()
        self.point = types.SimpleNamespace()


class MonitorTests(unittest.TestCase):
    def setUp(self):
        modules = {
            'rclpy.node': types.SimpleNamespace(Node=FakeNode),
            'geometry_msgs.msg': types.SimpleNamespace(PointStamped=FakePoint),
            'dynamixel_sdk_custom_interfaces.srv': types.SimpleNamespace(
                GetPosition=types.SimpleNamespace(Request=types.SimpleNamespace)),
            'dynamixel_sdk_custom_interfaces.msg': types.SimpleNamespace(SetPosition=object),
        }
        self.modules = patch.dict(sys.modules, modules)
        self.modules.start()
        self.clock = patch('arm_pose_viewer.time.monotonic', return_value=10.0)
        self.now = self.clock.start()
        self.addCleanup(self.modules.stop)
        self.addCleanup(self.clock.stop)

    def node(self, source='service'):
        return make_node(types.SimpleNamespace(source=source, service='/get_position',
                                              topic='/set_position', frame='arm_base'))

    def test_service_reads_four_ids_before_publish(self):
        node = self.node()
        node.tick()
        for i in range(4):
            motor_id, future = node.fake_client.calls[-1]
            self.assertEqual(motor_id, 11+i)
            self.assertEqual(len(node.messages), 0)
            future.set_result(types.SimpleNamespace(position=2048))
            node.tick()
        self.assertEqual(len(node.messages), 1)
        self.assertEqual(node.messages[0].header.frame_id, 'arm_base')
        self.assertEqual(node.sample[0], forward_kinematics([0]*4)[0])

    def test_timeout_discards_partial_sample_and_recovers(self):
        node = self.node()
        node.tick()
        node.fake_client.calls[-1][1].set_result(types.SimpleNamespace(position=2048))
        node.tick()
        self.now.return_value = 11.1
        node.tick()
        self.assertEqual(node.sweep, [])
        self.assertIsNone(node.sample)
        self.assertEqual(node.messages, [])
        self.now.return_value = 12.2
        node.tick()
        self.assertEqual(node.fake_client.calls[-1][0], 11)

    def test_invalid_encoder_never_published(self):
        node = self.node()
        node.tick()
        node.fake_client.calls[-1][1].set_result(types.SimpleNamespace(position=9999))
        node.tick()
        self.assertEqual(node.messages, [])
        self.assertIsNone(node.sample)

    def test_late_completed_response_is_discarded(self):
        node = self.node()
        node.tick()
        for _ in range(3):
            node.fake_client.calls[-1][1].set_result(types.SimpleNamespace(position=2048))
            node.tick()
        node.fake_client.calls[-1][1].set_result(types.SimpleNamespace(position=2048))
        self.now.return_value = 11.2
        node.tick()
        self.assertEqual(node.messages, [])
        self.assertIsNone(node.sample)

    def test_command_waits_for_all_four_joints_and_ignores_gripper(self):
        node = self.node('command')
        for motor_id in (11, 12, 13, 15):
            node.command(types.SimpleNamespace(id=motor_id, position=2048))
        self.now.return_value = 10.2
        node.tick()
        self.assertEqual(node.messages, [])
        node.command(types.SimpleNamespace(id=14, position=2048))
        self.now.return_value = 10.4
        node.tick()
        self.assertEqual(len(node.messages), 1)
        self.assertIn('LAST COMMAND', node.status)


if __name__ == '__main__':
    unittest.main()
