#!/usr/bin/env python3
"""Click to jog the arm and copy the target pose. See ARM_POSE_JOGGER.md."""
import argparse
import math
import time

from arm_kinematics import MOTOR_IDS, forward_kinematics, inverse_kinematics
from arm_pose_viewer import ArmDisplay


class JogSession:
    """Target state; sends only on explicit clicks, never on construction."""
    def __init__(self, send, start=(0.1, 0.0, 0.1, 90.0), wait=2.0):
        inverse_kinematics(start)
        if not math.isfinite(wait) or wait < 0.1:
            raise ValueError('Wait time must be at least 0.1 seconds')
        self.send = send
        self.start = tuple(start)
        self.pose = None
        self.angles = None
        self.busy_until = 0.0
        self.wait = wait
        self.velocity = 25
        self.acceleration = 5

    def busy(self):
        return time.monotonic() < self.busy_until

    def check_ready(self):
        if self.busy():
            raise ValueError('Please wait; the previous move may still be in progress')

    def settings(self):
        if not (isinstance(self.velocity, int) and 1 <= self.velocity <= 100
                and isinstance(self.acceleration, int) and 1 <= self.acceleration <= 20):
            raise ValueError('Velocity: 1..100; acceleration: 1..20 (integer SDK units)')

    def move(self, pose):
        self.check_ready()
        self.settings()
        angles, ticks = inverse_kinematics(pose)
        commands = [(motor_id, value, self.velocity, self.acceleration)
                    for motor_id, value in zip(MOTOR_IDS, ticks)]
        # The sender checks ROS subscriber availability before publishing.
        self.send(commands)
        self.pose = tuple(pose)
        self.angles = angles
        self.busy_until = time.monotonic() + self.wait

    def jog(self, axis, delta):
        if self.pose is None:
            raise ValueError('Click Move to start pose first')
        if axis not in range(4) or not math.isfinite(delta):
            raise ValueError('Invalid jog increment')
        candidate = list(self.pose)
        candidate[axis] = round(candidate[axis] + delta, 8)
        self.move(candidate)

    def gripper(self, opened):
        self.check_ready()
        self.settings()
        if self.pose is None:
            raise ValueError('Click Move to start pose first')
        degree = -50.0 if opened else 0.0  # keyboard.py defaults
        self.send([(15, int(2048 + degree / 0.088), self.velocity, self.acceleration)])
        self.busy_until = time.monotonic() + self.wait

    def pose_text(self):
        if self.pose is None:
            raise ValueError('No target has been sent yet')
        return '[{:.4f}, {:.4f}, {:.4f}, {:.2f}]'.format(*self.pose)

    def code_text(self):
        # Explicit literal values can be pasted into robocon.py or keyboard.py.
        return 'open_manipulator_x.move({}, {}, {})'.format(
            self.pose_text(), self.velocity, self.acceleration)


def make_jog_node(topic):
    from rclpy.node import Node
    from dynamixel_sdk_custom_interfaces.msg import SetPosition

    class ArmJogNode(Node):
        def __init__(self):
            super().__init__('arm_pose_jogger')
            self.publisher = self.create_publisher(SetPosition, topic, 10)

        def send(self, commands):
            if self.publisher.get_subscription_count() == 0:
                raise ValueError('No controller on {}. Start read_write_trapezoidal_node'.format(topic))
            messages = []
            for motor_id, position, velocity, acceleration in commands:
                msg = SetPosition()
                msg.id, msg.position = motor_id, position
                msg.velocity, msg.acceleration = velocity, acceleration
                messages.append(msg)
            for msg in messages:
                self.publisher.publish(msg)

    return ArmJogNode()


class JogWindow:
    def __init__(self, root, session, demo=False):
        import tkinter as tk
        from tkinter import ttk
        self.root, self.session, self.demo = root, session, demo
        self.display = ArmDisplay(root, 'jog_demo' if demo else 'jog', compact=True)
        root.title('OpenMANIPULATOR-X | Click to jog & copy XYZ')
        self.notice = 'Click Move to start pose to begin (no automatic motion on launch).'
        self.display.details.configure(text='Start pose (not sent): ' + str(list(session.start)))
        controls = ttk.Frame(root)
        controls.pack(before=self.display.charts, padx=20, pady=5)
        self.step = tk.StringVar(value='5')
        self.angle_step = tk.StringVar(value='3')
        self.velocity = tk.StringVar(value='25')
        self.acceleration = tk.StringVar(value='5')
        settings = ttk.Frame(controls)
        settings.pack()
        for label, variable, options in (
                ('XYZ step (mm)', self.step, ('1', '5', '10', '20')),
                ('Pitch step (deg)', self.angle_step, ('1', '3', '5')),
                ('Velocity', self.velocity, ('10', '25', '50', '100')),
                ('Acceleration', self.acceleration, ('1', '5', '10', '20'))):
            ttk.Label(settings, text=label).pack(side='left', padx=(8, 3))
            ttk.Combobox(settings, textvariable=variable, values=options,
                         state='readonly', width=5).pack(side='left')
        directions = ttk.Frame(controls)
        directions.pack(pady=8)
        self.jog_buttons = []
        for axis, name in enumerate(('X', 'Y', 'Z', 'Pitch')):
            box = ttk.Frame(directions)
            box.pack(side='left', padx=12)
            for sign, suffix in ((1, '+'), (-1, '-')):
                button = ttk.Button(box, text=name + ' ' + suffix, width=11,
                                    command=lambda a=axis, s=sign: self.jog(a, s))
                button.pack(side='left', padx=2)
                self.jog_buttons.append(button)
        actions = ttk.Frame(controls)
        actions.pack()
        self.start_button = ttk.Button(actions, text='Move to start pose',
                                       command=lambda: self.run(lambda: session.move(session.start)))
        self.start_button.pack(side='left', padx=5)
        for opened, label in ((True, 'Open gripper'), (False, 'Close gripper')):
            button = ttk.Button(actions, text=label,
                                command=lambda o=opened: self.run(lambda: session.gripper(o)))
            button.pack(side='left', padx=5)
            self.jog_buttons.append(button)
        self.display.copy_button.configure(command=lambda: self.copy(False))
        self.copy_code_button = ttk.Button(self.display.copy_button.master,
                                          text='Copy move(...) code', command=lambda: self.copy(True))
        self.copy_code_button.pack(pady=(5, 0))

    def run(self, action):
        try:
            self.session.velocity = int(self.velocity.get())
            self.session.acceleration = int(self.acceleration.get())
            action()
            self.notice = 'Target sent. Wait for the arm to settle, then copy the pose.'
        except (ValueError, RuntimeError) as exc:
            self.notice = 'Not sent: ' + str(exc)
        self.refresh()

    def jog(self, axis, sign):
        delta = float(self.angle_step.get()) if axis == 3 else float(self.step.get()) / 1000
        self.run(lambda: self.session.jog(axis, sign * delta))

    def copy(self, code):
        if self.session.busy() or self.session.pose is None:
            return
        text = self.session.code_text() if code else self.session.pose_text()
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.notice = 'Copied: ' + text
        self.refresh()

    def refresh(self):
        session = self.session
        busy = session.busy()
        ready = session.pose is not None
        sample = None
        if ready:
            _, pitch, points = forward_kinematics(session.angles)
            # Display and copy the exact commanded XYZ; diagram uses the same IK.
            sample = (session.pose[:3], pitch, points)
            self.display.yaw = session.angles[0]
        prefix = 'DEMO - NO MOTOR COMMANDS | ' if self.demo else 'TARGET (not measured) | '
        status = prefix + self.notice
        if busy:
            status += ' | wait {:.1f}s (timer, not motion feedback)'.format(
                session.busy_until - time.monotonic())
        self.display.update(sample, status, ready)
        if ready:
            self.display.details.configure(text='Pitch: {:.2f} deg | {}'.format(
                session.pose[3], 'MOVING / wait timer' if busy else 'Last commanded target'))
        self.start_button.configure(state='disabled' if busy else 'normal')
        for button in self.jog_buttons:
            button.configure(state='normal' if ready and not busy else 'disabled')
        for button in (self.display.copy_button, self.copy_code_button):
            button.configure(state='normal' if ready and not busy else 'disabled')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='Interactive GUI without ROS or motors')
    parser.add_argument('--topic', default='/set_position')
    parser.add_argument('--start', nargs=4, type=float, default=(0.1, 0.0, 0.1, 90.0),
                        metavar=('X', 'Y', 'Z', 'PITCH'))
    parser.add_argument('--wait', type=float, default=2.0, help='Seconds to wait between clicks')
    args, ros_args = parser.parse_known_args()
    node, ros = None, None
    try:
        # Validate before initializing ROS or creating a window.
        session = JogSession(lambda commands: None, args.start, args.wait)
        if not args.demo:
            import rclpy as ros
            ros.init(args=ros_args)
            node = make_jog_node(args.topic)
            session.send = node.send
        import tkinter as tk
        root = tk.Tk()
        window = JogWindow(root, session, args.demo)

        def tick():
            if node is not None:
                if not ros.ok():
                    root.destroy()
                    return
                ros.spin_once(node, timeout_sec=0.0)
            window.refresh()
            root.after(50, tick)

        tick()
        root.mainloop()
    except KeyboardInterrupt:
        pass
    except (ImportError, ValueError) as exc:
        raise SystemExit('{}\nSee ARM_POSE_JOGGER.md for setup.'.format(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()


if __name__ == '__main__':
    main()
