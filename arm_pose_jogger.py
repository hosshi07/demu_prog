#!/usr/bin/env python3
"""Click to jog the arm and copy the target pose. See ARM_POSE_JOGGER.md."""
import argparse
import math
import time

L1, L2, L3, L4 = 0.077, 0.13, 0.124, 0.126
PSI = math.atan2(0.024, 0.128)
MOTOR_IDS = (11, 12, 13, 14)


def inverse_kinematics(pose):
    """Validated version of the supplied IK; pose is [m, m, m, degrees].

    Stay in x > 0, the half-plane supported by the original atan(y/x).
    Checks geometric reach and tick range, not collisions or hardware limits.
    """
    if len(pose) != 4 or not all(math.isfinite(v) for v in pose):
        raise ValueError('XYZ and pitch must be finite numbers')
    x, y, z, pitch_degrees = pose
    if x <= 0 or z < 0:
        raise ValueError('Use X > 0 and Z >= 0 (original keyboard.py workspace)')
    pitch = math.radians(pitch_degrees)
    h = z - L1 + L4 * math.sin(pitch)
    k = math.hypot(x, y) - L4 * math.cos(pitch)
    v = math.hypot(h, k)
    if v < 1e-12 or not abs(L2 - L3) <= v <= L2 + L3:
        raise ValueError('Unreachable: wrist distance {:.3f} m (range {:.3f}..{:.3f} m). '
                         'Try a smaller step or change pitch.'.format(v, abs(L2-L3), L2+L3))

    def acos(value):
        return math.acos(max(-1.0, min(1.0, value)))

    alpha = acos((L2*L2 + v*v - L3*L3) / (2 * L2 * v))
    beta = acos((L2*L2 + L3*L3 - v*v) / (2 * L2 * L3))
    q2 = math.pi / 2 - alpha - math.atan2(h, k) - PSI
    q3 = math.pi / 2 - beta + PSI
    angles = [math.atan2(y, x), q2, q3, pitch - q2 - q3]
    ticks = [int(2048 + math.degrees(q) / 0.088) for q in angles]
    if any(not 0 <= value <= 4095 for value in ticks):
        raise ValueError('Target exceeds the 0..4095 motor position range')
    return angles, ticks


def ticks_to_radians(ticks):
    # Deliberately use the same rounded 0.088 deg/tick as the supplied code.
    return math.radians((ticks - 2048) * 0.088)


def forward_kinematics(angles):
    """Return ((x,y,z), pitch, joint_points) in the supplied code's frame.

    pitch = q2 + q3 + q4, positive towards the floor, as in move().
    joint_points include the base, shoulder, elbow, wrist and tool tip.
    """
    if len(angles) != 4 or not all(math.isfinite(a) for a in angles):
        raise ValueError('Four finite joint angles are required')
    q1, q2, q3, q4 = angles
    pitch = q2 + q3 + q4
    shoulder = math.pi / 2 - q2 - PSI
    forearm = -(q2 + q3)
    r2 = L2 * math.cos(shoulder)
    z2 = L1 + L2 * math.sin(shoulder)
    r3 = r2 + L3 * math.cos(forearm)
    z3 = z2 + L3 * math.sin(forearm)
    r4 = r3 + L4 * math.cos(pitch)
    z4 = z3 - L4 * math.sin(pitch)

    def point(radius, height):
        return (radius * math.cos(q1), radius * math.sin(q1), height)

    points = [(0.0, 0.0, 0.0), point(0, L1), point(r2, z2),
              point(r3, z3), point(r4, z4)]
    return points[-1], pitch, points


class ArmDisplay:
    def __init__(self, root, source, compact=False):
        import tkinter as tk
        from tkinter import ttk
        self.root = root
        self.sample = None
        root.title('OpenMANIPULATOR-X | XYZ monitor')
        root.configure(bg='#eef2f6')
        root.minsize(880, 620)
        title = {'service': 'MEASURED / encoder-based position',
                 'command': 'COMMAND / last target, NOT measured',
                 'demo': 'DEMO / simulated data, NOT connected',
                 'jog': 'JOG / target and encoder positions',
                 'jog_demo': 'JOG DEMO / no hardware connected'}[source]
        ttk.Label(root, text=title, font=('Sans', 18, 'bold')).pack(
            pady=(10 if compact else 18, 4))
        ttk.Label(root, text='Base frame | metres | pitch: positive downward').pack()
        row = ttk.Frame(root)
        row.pack(fill='x', padx=24, pady=10 if compact else 18)
        self.values = []
        for axis, colour in zip('XYZ', ('#c0392b', '#16834a', '#286ac0')):
            box = ttk.Frame(row)
            box.pack(side='left', expand=True, fill='x')
            ttk.Label(box, text=axis, foreground=colour,
                      font=('Sans', 18, 'bold')).pack()
            value = ttk.Label(box, text='--- m', font=('Monospace', 26, 'bold'))
            value.pack()
            self.values.append(value)
        self.details = ttk.Label(root, text='Waiting for all four joints...')
        self.details.pack()
        charts = ttk.Frame(root)
        self.charts = charts
        charts.pack(padx=16, pady=6 if compact else 12)
        self.canvases = []
        for name in ('TOP VIEW: X / Y', 'SIDE VIEW: radial distance / Z'):
            frame = ttk.Frame(charts)
            frame.pack(side='left', padx=5)
            ttk.Label(frame, text=name).pack()
            canvas = tk.Canvas(frame, width=410, height=180 if compact else 280, bg='white',
                               highlightthickness=0)
            canvas.pack()
            self.canvases.append(canvas)
        self.status = ttk.Label(root, text='Waiting...', wraplength=830)
        self.status.pack(pady=6)
        buttons = ttk.Frame(root)
        buttons.pack(pady=8)
        self.copy_button = ttk.Button(buttons, text='Copy [x, y, z, pitch]',
                                      command=self.copy, state='disabled')
        self.copy_button.pack()
        footer = ('Target coordinates only; actual arm position is not measured.'
                  if source in ('jog', 'jog_demo') else
                  'Display only: this program sends no motor commands.')
        self.footer = ttk.Label(root, text=footer)
        self.footer.pack()
        self.draw(None)

    def copy(self):
        if self.sample is not None:
            xyz, pitch, _ = self.sample
            text = '[{:.4f}, {:.4f}, {:.4f}, {:.2f}]'.format(
                *xyz, math.degrees(pitch))
            self.root.clipboard_clear()
            self.root.clipboard_append(text)

    def update(self, sample, status, valid):
        self.sample = sample if valid else None
        self.status.configure(text=status, foreground='#16834a' if valid else '#b35400')
        self.copy_button.configure(state='normal' if valid else 'disabled')
        for label, number in zip(self.values, sample[0] if sample else (None,) * 3):
            label.configure(text='{:+.4f} m'.format(number) if number is not None else '--- m',
                            foreground='#182c40' if valid else '#888888')
        if sample:
            self.details.configure(text='Pitch: {:.2f} deg | {}'.format(
                math.degrees(sample[1]), 'current sample' if valid else 'OLD SAMPLE'))
        self.draw(sample, valid)

    def draw(self, sample, valid=False):
        # Same scale on both axes: 400 pixels per metre.
        for index, canvas in enumerate(self.canvases):
            canvas.delete('all')
            height = int(canvas['height'])
            ox, oy, scale = ((205, height / 2, 320) if index == 0 else
                             (170, height - 35, 320)) if height < 280 else (
                                 (205, 140, 400) if index == 0 else (170, 235, 400))
            for step in range(-5, 6):
                pos = step * 0.1 * scale
                canvas.create_line(ox + pos, 0, ox + pos, height, fill='#edf0f3')
                canvas.create_line(0, oy - pos, 410, oy - pos, fill='#edf0f3')
            canvas.create_line(0, oy, 403, oy, arrow='last', fill='#7b8794')
            canvas.create_line(ox, height, ox, 7, arrow='last', fill='#7b8794')
            canvas.create_text(390, oy - 12, text='+X' if index == 0 else '+r')
            canvas.create_text(ox + 15, 12, text='+Y' if index == 0 else '+Z')
            canvas.create_text(60, height - 15, text='grid: 0.1 m', fill='#64748b')
            if sample is None:
                continue
            points = sample[2]
            # Signed radial projection in the plane of joint 1.
            yaw = getattr(self, 'yaw', 0.0)
            projected = [(p[0], p[1]) if index == 0 else
                         (p[0] * math.cos(yaw) + p[1] * math.sin(yaw), p[2])
                         for p in points]
            coords = [(ox + a * scale, oy - b * scale) for a, b in projected]
            colour = '#2476b8' if valid else '#9ca3af'
            for a, b in zip(coords, coords[1:]):
                canvas.create_line(*a, *b, fill=colour, width=6)
            for x, y in coords:
                canvas.create_oval(x - 4, y - 4, x + 4, y + 4, fill=colour, outline='')
            x, y = coords[-1]
            canvas.create_oval(x - 7, y - 7, x + 7, y + 7, fill='#ef8b23', outline='')
            canvas.create_text(x, y - 16, text='TIP', fill='#b65f0b')


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

    def adopt(self, angles):
        """Use fresh encoder angles as the jog reference without moving."""
        self.check_ready()
        xyz, pitch, _ = forward_kinematics(angles)
        pose = (*xyz, math.degrees(pitch))
        solved, _ = inverse_kinematics(pose)
        if any(abs(a - b) > math.radians(2) for a, b in zip(solved, angles)):
            raise ValueError('Current joint configuration differs from the keyboard.py IK branch')
        self.pose, self.angles = pose, list(angles)

    def pose_text(self):
        if self.pose is None:
            raise ValueError('No target has been sent yet')
        return '[{:.4f}, {:.4f}, {:.4f}, {:.2f}]'.format(*self.pose)

    def code_text(self):
        # Explicit literal values can be pasted into robocon.py or keyboard.py.
        return 'open_manipulator_x.move({}, {}, {})'.format(
            self.pose_text(), self.velocity, self.acceleration)


class PositionReader:
    """Nonblocking reads through the provided controller's GetPosition service."""
    def __init__(self, client, request_type):
        self.client, self.request_type = client, request_type
        self.pending = None
        self.values = []
        self.sample = None
        self.angles = None
        self.updated = None
        self.healthy = False
        self.next_read = 0.0
        self.status = 'Waiting for /get_position'

    def fresh(self):
        return (self.healthy and self.updated is not None
                and time.monotonic() - self.updated < 2.0)

    def fail(self, message):
        if self.pending is not None:
            self.pending.cancel()
        self.pending = None
        self.values = []
        self.healthy = False
        self.status = message
        self.next_read = time.monotonic() + 1.0

    def tick(self):
        now = time.monotonic()
        if self.pending is not None:
            if now - self.started > 1.0:
                self.fail('Encoder read timed out')
                return
            if not self.pending.done():
                return
            future, self.pending = self.pending, None
            try:
                value = int(future.result().position)
                if not 0 <= value <= 4095:
                    raise ValueError('encoder position outside 0..4095')
            except Exception as exc:
                self.fail('Encoder read failed: {}'.format(exc))
                return
            self.values.append(value)
            if len(self.values) == 4:
                self.angles = [ticks_to_radians(v) for v in self.values]
                self.sample = forward_kinematics(self.angles)
                self.updated = now
                self.healthy = True
                self.status = 'Encoder-derived position'
                self.values = []
                self.next_read = now + 0.5
        if now < self.next_read:
            return
        if not self.client.service_is_ready():
            self.fail('Position service unavailable (target controls still work)')
            return
        if not self.values:
            self.started = now
        request = self.request_type()
        request.id = MOTOR_IDS[len(self.values)]
        self.pending = self.client.call_async(request)


def make_jog_node(topic, service='/get_position'):
    from rclpy.node import Node
    from dynamixel_sdk_custom_interfaces.msg import SetPosition
    from dynamixel_sdk_custom_interfaces.srv import GetPosition

    class ArmJogNode(Node):
        def __init__(self):
            super().__init__('arm_pose_jogger')
            self.publisher = self.create_publisher(SetPosition, topic, 10)
            self.reader = PositionReader(self.create_client(GetPosition, service), GetPosition.Request)

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
    def __init__(self, root, session, demo=False, reader=None):
        import tkinter as tk
        from tkinter import ttk
        self.root, self.session, self.demo = root, session, demo
        self.reader = reader
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
        self.mode = tk.StringVar(value='target')
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
        self.adopt_button = ttk.Button(actions, text='Use current pose', command=self.adopt_current)
        self.adopt_button.pack(side='left', padx=5)
        modes = ttk.Frame(controls)
        modes.pack(pady=(5, 0))
        ttk.Label(modes, text='Display & copy:').pack(side='left', padx=5)
        for label, value in (('Target', 'target'), ('Encoder', 'encoder')):
            ttk.Radiobutton(modes, text=label, value=value, variable=self.mode,
                            command=self.refresh).pack(side='left', padx=5)
        self.encoder_status = ttk.Label(modes, text='Waiting for encoder data')
        self.encoder_status.pack(side='left', padx=10)
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

    def adopt_current(self):
        try:
            if self.reader is None or not self.reader.fresh():
                raise ValueError('Fresh encoder data is not available')
            self.session.adopt(self.reader.angles)
            self.mode.set('target')
            self.notice = 'Current pose loaded as jog reference. No motor command sent.'
        except ValueError as exc:
            self.notice = 'Not loaded: ' + str(exc)
        self.refresh()

    def jog(self, axis, sign):
        delta = float(self.angle_step.get()) if axis == 3 else float(self.step.get()) / 1000
        self.run(lambda: self.session.jog(axis, sign * delta))

    def copy(self, code):
        if self.session.busy():
            return
        if self.mode.get() == 'encoder':
            if self.reader is None or not self.reader.fresh():
                return
            xyz, pitch, _ = self.reader.sample
            pose = '[{:.4f}, {:.4f}, {:.4f}, {:.2f}]'.format(*xyz, math.degrees(pitch))
            text = ('open_manipulator_x.move({}, {}, {})'.format(
                pose, self.session.velocity, self.session.acceleration) if code else pose)
        else:
            if self.session.pose is None:
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
        measured = self.mode.get() == 'encoder'
        encoder_fresh = self.reader is not None and self.reader.fresh()
        sample = None
        if ready:
            _, pitch, points = forward_kinematics(session.angles)
            # Display and copy the exact commanded XYZ; diagram uses the same IK.
            sample = (session.pose[:3], pitch, points)
            self.display.yaw = session.angles[0]
        if measured:
            sample = self.reader.sample if self.reader is not None else None
            if self.reader is not None and self.reader.angles:
                self.display.yaw = self.reader.angles[0]
        prefix = 'DEMO - NO MOTOR COMMANDS | ' if self.demo else 'TARGET (not measured) | '
        status = prefix + self.notice
        if busy:
            status += ' | wait {:.1f}s (timer, not motion feedback)'.format(
                session.busy_until - time.monotonic())
        if measured:
            status = 'ENCODER-DERIVED | ' + (self.reader.status if self.reader else 'No encoder in demo')
            if not encoder_fresh:
                status += ' | WAITING / STALE'
        self.display.update(sample, status, encoder_fresh if measured else ready)
        self.display.footer.configure(text=(
            'Encoder-derived position; verify the controller log for communication errors.' if measured else
            'Target coordinates only; actual arm position may differ.'))
        if ready and not measured:
            self.display.details.configure(text='Pitch: {:.2f} deg | {}'.format(
                session.pose[3], 'MOVING / wait timer' if busy else 'Last commanded target'))
        self.encoder_status.configure(text=('Encoder: available' if encoder_fresh else
                                            'Encoder: demo' if self.demo else 'Encoder: waiting / stale'))
        self.adopt_button.configure(state='normal' if encoder_fresh and not busy else 'disabled')
        self.start_button.configure(state='disabled' if busy else 'normal')
        for button in self.jog_buttons:
            button.configure(state='normal' if ready and not busy else 'disabled')
        for button in (self.display.copy_button, self.copy_code_button):
            available = encoder_fresh if measured else ready
            button.configure(state='normal' if available and not busy else 'disabled')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='Interactive GUI without ROS or motors')
    parser.add_argument('--topic', default='/set_position')
    parser.add_argument('--service', default='/get_position')
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
            node = make_jog_node(args.topic, args.service)
            session.send = node.send
        import tkinter as tk
        root = tk.Tk()
        window = JogWindow(root, session, args.demo, node.reader if node else None)

        def tick():
            if node is not None:
                if not ros.ok():
                    root.destroy()
                    return
                ros.spin_once(node, timeout_sec=0.0)
                node.reader.tick()
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
