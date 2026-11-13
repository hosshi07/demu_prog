#!/usr/bin/env python3
"""ROS2 Foxy read-only arm monitor. See ARM_POSE_VIEWER.md for usage."""
import argparse
import math
import time

from arm_kinematics import MOTOR_IDS, forward_kinematics, ticks_to_radians


class ArmDisplay:
    def __init__(self, root, source):
        import tkinter as tk
        from tkinter import ttk
        self.root = root
        self.sample = None
        root.title('OpenMANIPULATOR-X | XYZ monitor')
        root.configure(bg='#eef2f6')
        root.minsize(880, 620)
        title = {'service': 'MEASURED / encoder-based position',
                 'command': 'COMMAND / last target, NOT measured',
                 'demo': 'DEMO / simulated data, NOT connected'}[source]
        ttk.Label(root, text=title, font=('Sans', 18, 'bold')).pack(pady=(18, 4))
        ttk.Label(root, text='Base frame | metres | pitch: positive downward').pack()
        row = ttk.Frame(root)
        row.pack(fill='x', padx=24, pady=18)
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
        charts.pack(padx=16, pady=12)
        self.canvases = []
        for name in ('TOP VIEW: X / Y', 'SIDE VIEW: radial distance / Z'):
            frame = ttk.Frame(charts)
            frame.pack(side='left', padx=5)
            ttk.Label(frame, text=name).pack()
            canvas = tk.Canvas(frame, width=410, height=280, bg='white',
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
        ttk.Label(root, text='Display only: this program sends no motor commands.').pack()
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
            ox, oy, scale = (205, 140, 400) if index == 0 else (170, 235, 400)
            for step in range(-5, 6):
                pos = step * 0.1 * scale
                canvas.create_line(ox + pos, 0, ox + pos, 280, fill='#edf0f3')
                canvas.create_line(0, oy - pos, 410, oy - pos, fill='#edf0f3')
            canvas.create_line(0, oy, 403, oy, arrow='last', fill='#7b8794')
            canvas.create_line(ox, 280, ox, 7, arrow='last', fill='#7b8794')
            canvas.create_text(390, oy - 12, text='+X' if index == 0 else '+r')
            canvas.create_text(ox + 15, 12, text='+Y' if index == 0 else '+Z')
            canvas.create_text(60, 265, text='grid: 0.1 m', fill='#64748b')
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


def make_node(args):
    # Lazy imports allow --demo and kinematics tests without ROS installed.
    from rclpy.node import Node
    from geometry_msgs.msg import PointStamped

    class ArmPoseMonitor(Node):
        def __init__(self):
            super().__init__('arm_pose_monitor')
            self.sample = None
            self.angles = None
            self.updated = None
            self.status = 'Waiting for data'
            self.pending = None
            self.next_poll = 0.0
            self.sweep = []
            self.ticks = {}
            self.dirty_at = None
            topic = ('/arm/end_effector_position' if args.source == 'service'
                     else '/arm/commanded_end_effector_position')
            self.publisher = self.create_publisher(PointStamped, topic, 10)
            if args.source == 'service':
                from dynamixel_sdk_custom_interfaces.srv import GetPosition
                self.request_type = GetPosition.Request
                self.client = self.create_client(GetPosition, args.service)
            else:
                from dynamixel_sdk_custom_interfaces.msg import SetPosition
                self.create_subscription(SetPosition, args.topic, self.command, 10)
            self.create_timer(0.05, self.tick)

        def accept(self, ticks):
            self.angles = [ticks_to_radians(t) for t in ticks]
            self.sample = forward_kinematics(self.angles)
            self.updated = time.monotonic()
            self.status = ('Encoder-based position' if args.source == 'service' else
                           'LAST COMMAND ONLY: actual arm position may differ')
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = args.frame
            msg.point.x, msg.point.y, msg.point.z = self.sample[0]
            self.publisher.publish(msg)

        def command(self, msg):
            if msg.id in MOTOR_IDS:
                if not 0 <= msg.position <= 4095:
                    self.status = 'Invalid command ticks (expected 0..4095)'
                    return
                self.ticks[msg.id] = msg.position
                self.dirty_at = time.monotonic()

        def fail(self, message):
            self.status = message
            self.sweep = []
            self.pending = None
            self.next_poll = time.monotonic() + 1.0

        def tick(self):
            now = time.monotonic()
            if args.source == 'command':
                if (self.dirty_at is not None and now - self.dirty_at >= 0.1
                        and all(i in self.ticks for i in MOTOR_IDS)):
                    self.accept([self.ticks[i] for i in MOTOR_IDS])
                    self.dirty_at = None
                elif self.sample is None:
                    missing = [str(i) for i in MOTOR_IDS if i not in self.ticks]
                    self.status = 'Waiting for commands for motor IDs: ' + ', '.join(missing)
                return
            if self.pending is not None:
                if now - self.sweep_started > 1.0:
                    self.pending.cancel()
                    self.fail('Four-joint read timed out; sample discarded')
                    return
                if self.pending.done():
                    future, self.pending = self.pending, None
                    try:
                        response = future.result()
                        value = int(response.position)
                        if not 0 <= value <= 4095:
                            raise ValueError('position outside 0..4095')
                    except Exception as exc:
                        self.fail('Read failed: {}'.format(exc))
                        return
                    self.sweep.append(value)
                    if len(self.sweep) == 4:
                        self.accept(self.sweep)
                        self.sweep = []
                        self.next_poll = now + 0.2
                elif now - self.started > 1.0:
                    self.pending.cancel()
                    self.fail('Read timed out; check the controller and motor connection')
                    return
                else:
                    return
            if now < self.next_poll:
                return
            if not self.client.service_is_ready():
                self.fail('Waiting for {} (GetPosition). See ARM_POSE_VIEWER.md'.format(args.service))
                return
            if self.sweep and now - self.sweep_started > 1.0:
                self.fail('Four-joint read took too long; sample discarded')
                return
            if not self.sweep:
                self.sweep_started = now
            request = self.request_type()
            request.id = MOTOR_IDS[len(self.sweep)]
            self.pending = self.client.call_async(request)
            self.started = now

    return ArmPoseMonitor()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', choices=('service', 'command'), default='service')
    parser.add_argument('--service', default='/get_position')
    parser.add_argument('--topic', default='/set_position')
    parser.add_argument('--frame', default='arm_base', help='Label for the code-defined base frame')
    parser.add_argument('--demo', action='store_true', help='GUI demo; no ROS or hardware access')
    parser.add_argument('--no-gui', action='store_true', help='Publish and print XYZ in the terminal')
    args, ros_args = parser.parse_known_args()
    if args.demo and args.no_gui:
        parser.error('--demo requires the GUI')
    root, display, node = None, None, None
    rclpy = None
    try:
        if not args.no_gui:
            import tkinter as tk
            root = tk.Tk()
            display = ArmDisplay(root, 'demo' if args.demo else args.source)
        if not args.demo:
            import rclpy
            rclpy.init(args=ros_args)
            node = make_node(args)
        if args.no_gui:
            last_print = 0.0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() - last_print > 1.0:
                    valid = node.updated is not None and (
                        args.source == 'command' or time.monotonic() - node.updated < 2.0)
                    print('{} | {} | {}'.format(
                        'VALID' if valid else 'WAITING/STALE', node.status,
                        node.sample[0] if node.sample else 'no position'), flush=True)
                    last_print = time.monotonic()
            return

        def refresh():
            if args.demo:
                angles = [0.4 * math.sin(time.monotonic() * 0.4), 0.1, 0.3, 1.17]
                display.yaw = angles[0]
                display.update(forward_kinematics(angles), 'DEMO: no hardware connected', True)
            else:
                if not rclpy.ok():
                    root.destroy()
                    return
                rclpy.spin_once(node, timeout_sec=0.0)
                valid = node.updated is not None and (
                    args.source == 'command' or time.monotonic() - node.updated < 2.0)
                if node.angles:
                    display.yaw = node.angles[0]
                age = (' | last sample {:.1f}s ago'.format(time.monotonic() - node.updated)
                       if node.updated is not None else '')
                display.update(node.sample, node.status + age, valid)
            root.after(30, refresh)

        refresh()
        root.mainloop()
    except KeyboardInterrupt:
        pass
    except ImportError as exc:
        raise SystemExit('Missing dependency: {}. See ARM_POSE_VIEWER.md'.format(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
