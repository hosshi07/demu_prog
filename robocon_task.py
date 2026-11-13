#!/usr/bin/env python3
"""KIT display/disposal task. One Python file + JSON configuration; see ROBOCON_TASK.md."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time


# Geometry is embedded below; no arm_kinematics/NumPy/GUI dependency.
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


SHAPES = ('triangle', 'square', 'circle')


def pose_text(pose):
    return '[{:.4f}, {:.4f}, {:.4f}, {:.2f}]'.format(*pose)


def quaternion_xy(q, dx, dy):
    """Project a marker-local XY offset into the supplied world/base frame."""
    x, y, z, w = q
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-8 or not math.isfinite(norm):
        raise ValueError('Invalid marker quaternion')
    x, y, z, w = (v / norm for v in q)
    return ((1-2*(y*y+z*z))*dx + 2*(x*y-z*w)*dy,
            2*(x*y+z*w)*dx + (1-2*(x*x+z*z))*dy)


class MarkerCache:
    def __init__(self, cfg):
        self.cfg = cfg
        self.history = {}

    def add(self, marker_id, xyz, quaternion, now):
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            return
        if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
            return
        try:
            quaternion_xy(quaternion, 0, 0)
        except ValueError:
            return
        history = self.history.setdefault(int(marker_id), deque(maxlen=20))
        if history and history[-1][0] == now:
            history.pop()  # Duplicate IDs in one message are not independent samples.
        history.append((now, tuple(xyz), tuple(quaternion)))

    def stable(self, marker_id, now):
        records = [r for r in self.history.get(marker_id, ())
                   if now-r[0] < self.cfg['marker_max_age_s']]
        records = records[-self.cfg['marker_samples']:]
        if len(records) < self.cfg['marker_samples']:
            return None
        average = tuple(sum(r[1][i] for r in records)/len(records) for i in range(3))
        if any(math.dist(r[1], average) > self.cfg['marker_jitter_m'] for r in records):
            return None
        # Check orientation too; q and -q represent the same rotation.
        latest_q = records[-1][2]
        def unit(q):
            norm = math.sqrt(sum(v*v for v in q))
            return tuple(v/norm for v in q)
        latest_unit = unit(latest_q)
        for _, _, q in records:
            cosine = min(1.0, abs(sum(a*b for a, b in zip(unit(q), latest_unit))))
            if 2*math.degrees(math.acos(cosine)) > self.cfg['marker_jitter_deg']:
                return None
        return {'xyz': average, 'quaternion': latest_q, 'received': records[-1][0]}


def profile_for(cfg, marker_id):
    shape = cfg['marker_shapes'][str(marker_id)]
    profile = dict(cfg['shapes'][shape])
    profile.update(cfg.get('marker_overrides', {}).get(str(marker_id), {}))
    return shape, profile


def pick_pose(cfg, marker_id, marker):
    _, profile = profile_for(cfg, marker_id)
    dx, dy = quaternion_xy(marker['quaternion'], *profile['local_offset_xy'])
    wx, wy = profile['world_offset_xy']
    x, y, _ = marker['xyz']
    # Absolute per-shape grasp Z avoids the unreliable AR Z reported by the user.
    pose = (x+wx+dx, y+wy+dy, profile['grasp_z'], profile['pitch_deg'])
    inverse_kinematics(pose)
    return pose


def jobs_for(cfg):
    mapping = {int(k): v for k, v in cfg['marker_shapes'].items()}
    discard = (set(cfg['discard_ids']) if cfg.get('discard_ids') is not None else
               {i for i in mapping if i % 2})
    if not discard.issubset(mapping):
        raise ValueError('discard_ids contains an ID absent from marker_shapes')
    jobs = []
    counts = {shape: 0 for shape in SHAPES}
    for marker_id in sorted(discard):
        jobs.append({'name': 'discard_{}'.format(marker_id), 'kind': 'discard',
                     'marker_id': marker_id, 'shape': mapping[marker_id]})
    for marker_id in sorted(set(mapping)-discard):
        shape = mapping[marker_id]
        slot = counts[shape]
        counts[shape] += 1
        jobs.append({'name': 'display_{}'.format(marker_id), 'kind': 'existing',
                     'marker_id': marker_id, 'shape': shape, 'slot': slot})
    for index, item in enumerate(cfg['supplies']):
        shape = item['shape']
        if shape not in SHAPES:
            raise ValueError('Unknown supply shape: '+str(shape))
        if not any(j['kind'] == 'existing' and j['shape'] == shape for j in jobs):
            raise ValueError('New {} products require a retained existing product in that row'.format(shape))
        jobs.append({'name': item['name'], 'kind': 'new', 'supply_index': index,
                     'shape': shape, 'slot': counts[shape]})
        counts[shape] += 1
    return jobs


def validate_config(cfg, execute=False):
    errors = []
    if execute:
        for flag in ('calibration_confirmed', 'marker_map_confirmed', 'same_coordinate_frame_confirmed'):
            if cfg.get(flag) is not True:
                errors.append('{} must be true after calibration'.format(flag))
    def pose(value, label):
        if value is None:
            errors.append('{}: not taught (null); use --teach'.format(label))
            return
        try:
            inverse_kinematics(value)
        except (ValueError, TypeError) as exc:
            errors.append('{}: {}'.format(label, exc))
    if not cfg['marker_shapes']:
        errors.append('marker_shapes must contain the actual marker IDs')
    for key, shape in cfg['marker_shapes'].items():
        try:
            if str(int(key)) != key or not 0 <= int(key) <= 65535:
                raise ValueError()
        except ValueError:
            errors.append('Invalid marker ID: ' + key)
        if shape not in SHAPES:
            errors.append('Unknown shape: ' + str(shape))
    if errors and any('shape' in e or 'marker ID' in e for e in errors):
        raise ValueError('\n'.join(errors))
    motion = cfg['motion']
    for key, low, high in (('velocity', 1, 100), ('acceleration', 1, 20)):
        if not isinstance(motion[key], int) or not low <= motion[key] <= high:
            errors.append('motion.{} outside {}..{}'.format(key, low, high))
    for key in ('move_min_s', 'move_timeout_s', 'grip_wait_s', 'marker_wait_s',
                'marker_max_age_s', 'joint_max_age_s', 'joint_tolerance_deg',
                'cartesian_step_m', 'pitch_step_deg', 'marker_jitter_m', 'marker_jitter_deg'):
        if not isinstance(cfg[key], (int, float)) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            errors.append('{} must be positive and finite'.format(key))
    if not isinstance(cfg['marker_samples'], int) or not 1 <= cfg['marker_samples'] <= 20:
        errors.append('marker_samples must be 1..20')
    if not isinstance(cfg['clearance_z'], (int, float)) or not math.isfinite(cfg['clearance_z']):
        errors.append('clearance_z must be finite')
    pose(cfg['observation_pose'], 'observation_pose')
    pose(cfg['discard_pose'], 'discard_pose (teach a point outside the board)')
    used_shapes = set(cfg['marker_shapes'].values()) | {item['shape'] for item in cfg['supplies']}
    for shape in SHAPES:
        p = cfg['shapes'][shape]
        if p['grasp_z'] is None and shape in used_shapes:
            errors.append('shapes.{}.grasp_z must be taught'.format(shape))
        elif p['grasp_z'] is not None:
            try:
                pose((0.1, 0, p['grasp_z'], p['pitch_deg']), 'shapes.'+shape)
            except TypeError as exc:
                errors.append(str(exc))
        for key in ('open_deg', 'close_deg'):
            value = p[key]
            if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= int(2048+value/0.088) <= 4095:
                errors.append('Invalid gripper setting {}.{}'.format(shape, key))
        for key in ('world_offset_xy', 'local_offset_xy'):
            if len(p[key]) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in p[key]):
                errors.append('Invalid {}.{}'.format(shape, key))
    for job in jobs_for(cfg):
        if job['kind'] != 'discard':
            slots = cfg['rows'][job['shape']]['slots']
            if job['slot'] >= len(slots):
                errors.append('rows.{}.slots needs index {}'.format(job['shape'], job['slot']))
            else:
                pose(slots[job['slot']], 'rows.{}.slots.{}'.format(job['shape'], job['slot']))
        if job['kind'] == 'new':
            pose(cfg['supplies'][job['supply_index']]['pick_pose'], job['name']+'.pick_pose')
    if len({j['name'] for j in jobs_for(cfg)}) != len(jobs_for(cfg)):
        errors.append('Job names must be unique')
    if errors:
        raise ValueError('\n'.join(errors))


def plan_job(cfg, job, source_pose):
    """Build AND validate every waypoint before sending any command for this job."""
    shape = job['shape']
    profile = (profile_for(cfg, job['marker_id'])[1] if 'marker_id' in job else cfg['shapes'][shape])
    destination = (cfg['discard_pose'] if job['kind'] == 'discard' else
                   cfg['rows'][shape]['slots'][job['slot']])
    actions = []
    current = list(cfg['observation_pose'])
    def move(target, label):
        nonlocal current
        inverse_kinematics(target)
        distance = math.dist(current[:3], target[:3])
        steps = max(1, math.ceil(distance/cfg['cartesian_step_m']),
                    math.ceil(abs(target[3]-current[3])/cfg['pitch_step_deg']))
        for n in range(1, steps+1):
            pose = [a+(b-a)*n/steps for a, b in zip(current, target)]
            _, ticks = inverse_kinematics(pose)
            actions.append({'kind': 'move', 'pose': pose, 'ticks': ticks, 'label': label})
        current = list(target)
    def grip(degree, label):
        tick = int(2048+degree/0.088)
        if not 0 <= tick <= 4095:
            raise ValueError('Gripper setting outside 0..4095')
        actions.append({'kind': 'grip', 'ticks': [tick], 'label': label})
    def above(pose):
        if cfg['clearance_z'] <= pose[2]:
            raise ValueError('clearance_z must exceed both pick and place heights')
        return [pose[0], pose[1], cfg['clearance_z'], pose[3]]
    move(above(source_pose), 'above pick')
    grip(profile['open_deg'], 'open before pick')
    move(source_pose, 'approach pick')
    grip(profile['close_deg'], 'close on object')
    move(above(source_pose), 'lift object')
    move(cfg['observation_pose'], 'carry via taught clearance pose')
    move(above(destination), 'above destination')
    move(destination, 'lower object')
    grip(profile['open_deg'], 'release object')
    move(above(destination), 'retreat from object')
    if job['kind'] != 'discard':
        # Optional experimentally taught pushes/squaring, relative to slot pose.
        for item in cfg['rows'][shape].get('after_place', []):
            if item['kind'] == 'move':
                move([a+b for a, b in zip(destination, item['offset'])], 'taught alignment')
            elif item['kind'] == 'grip':
                grip(item['degree'], 'alignment gripper')
            else:
                raise ValueError('Unknown after_place action')
    move(cfg['observation_pose'], 'return to observation')
    return actions


class JointReader:
    """Read the C++ node, all five IDs. No serial port is opened here."""
    def __init__(self, client, request_type, max_age=2.0):
        self.client, self.request_type, self.max_age = client, request_type, max_age
        self.pending = None
        self.values = []
        self.positions = None
        self.updated = None
        self.sample_started = None
        self.next_read = 0.0
        self.error = None

    def fresh(self, now):
        return (self.error is None and self.updated is not None
                and now-self.updated < self.max_age)

    def tick(self, now):
        def fail(message):
            if self.pending is not None:
                self.pending.cancel()
            self.pending, self.values = None, []
            self.error, self.next_read = message, now+1.0
        if self.pending is not None:
            if now-self.started > 1.5:
                fail('GetPosition timeout')
                return
            if not self.pending.done():
                return
            future, self.pending = self.pending, None
            try:
                value = int(future.result().position)
                if not 0 <= value <= 4095:
                    raise ValueError('position outside 0..4095')
            except Exception as exc:
                fail(str(exc))
                return
            self.values.append(value)
            if len(self.values) == 5:
                self.positions = self.values
                self.values = []
                self.updated, self.sample_started = now, self.started
                self.error = None
                self.next_read = now+0.1
        if now < self.next_read:
            return
        if not self.client.service_is_ready():
            fail('GetPosition service unavailable')
            return
        if not self.values:
            self.started = now
        request = self.request_type()
        request.id = 11+len(self.values)
        self.pending = self.client.call_async(request)


class Executor:
    def __init__(self, cfg, send, reader, event):
        self.cfg, self.send, self.reader, self.event = cfg, send, reader, event
        self.queue = deque()
        self.active = None
        self.error = None

    def start(self, actions):
        if self.active or self.queue:
            raise ValueError('Executor is already running')
        self.queue = deque(actions)

    def tick(self, now):
        if self.error:
            return False
        if self.active:
            elapsed = now-self.sent_at
            kind = self.active['kind']
            if kind == 'grip':
                # A grasped block may prevent reaching the requested close angle.
                finished = elapsed >= self.cfg['grip_wait_s']
            else:
                finished = (elapsed >= self.cfg['move_min_s'] and self.reader.fresh(now)
                            and self.reader.sample_started >= self.sent_at
                            and all(abs(a-b)*0.088 <= self.cfg['joint_tolerance_deg']
                                    for a, b in zip(self.reader.positions[:4], self.active['ticks'])))
                if elapsed > self.cfg['move_timeout_s']:
                    self.error = 'Motion feedback timeout: '+self.active['label']
                    self.queue.clear()
                    self.event('fault', reason=self.error)
                    return False
            if not finished:
                return False
            self.active = None
        if self.queue:
            action = self.queue.popleft()
            ids = (15,) if action['kind'] == 'grip' else MOTOR_IDS
            commands = [(i, t, self.cfg['motion']['velocity'], self.cfg['motion']['acceleration'])
                        for i, t in zip(ids, action['ticks'])]
            try:
                self.send(commands)
            except Exception as exc:
                self.error = 'Publish failed: '+str(exc)
                self.queue.clear()
                self.event('fault', reason=self.error)
                return False
            self.active, self.sent_at = action, now
            self.event('command', label=action['label'], pose=action.get('pose'), ids=list(ids))
            return False
        return True


def make_node(cfg):
    from rclpy.node import Node
    from aruco_interfaces.msg import ArucoMarkerArray
    from dynamixel_sdk_custom_interfaces.msg import SetPosition
    from dynamixel_sdk_custom_interfaces.srv import GetPosition
    class TaskNode(Node):
        def __init__(self):
            super().__init__('robocon_display_disposal')
            self.markers = MarkerCache(cfg)
            self.publisher = self.create_publisher(SetPosition, cfg['position_topic'], 10)
            self.create_subscription(ArucoMarkerArray, cfg['marker_topic'], self.on_markers, 10)
            self.reader = JointReader(self.create_client(GetPosition, cfg['position_service']),
                                      GetPosition.Request, cfg['joint_max_age_s'])
        def on_markers(self, msg):
            now = time.monotonic()
            for marker in msg.markers:
                p, q = marker.pose.position, marker.pose.orientation
                self.markers.add(marker.marker_id, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), now)
        def send(self, commands):
            if self.publisher.get_subscription_count() == 0:
                raise ValueError('No controller subscribing to '+cfg['position_topic'])
            messages = []
            for motor_id, position, velocity, acceleration in commands:
                msg = SetPosition()
                msg.id, msg.position = motor_id, position
                msg.velocity, msg.acceleration = velocity, acceleration
                messages.append(msg)
            for msg in messages:
                self.publisher.publish(msg)
    return TaskNode()


def initial_actions(cfg):
    start_xyz, pitch, _ = forward_kinematics([0.0]*4)
    # Reuse the same Cartesian interpolation validation as normal transfers.
    target = cfg['observation_pose']
    start = (*start_xyz, math.degrees(pitch))
    n = max(1, math.ceil(math.dist(start[:3], target[:3])/cfg['cartesian_step_m']),
            math.ceil(abs(start[3]-target[3])/cfg['pitch_step_deg']))
    result = []
    for i in range(1, n+1):
        p = [a+(b-a)*i/n for a, b in zip(start, target)]
        _, ticks = inverse_kinematics(p)
        result.append({'kind': 'move', 'pose': p, 'ticks': ticks, 'label': 'initial to observation'})
    return result


def teach_value(cfg, field, pose):
    """Accept explicit config paths; do not eval arbitrary user expressions."""
    parts = field.split('.')
    allowed = (field in ('discard_pose', 'observation_pose') or
               (len(parts) == 4 and parts[0] == 'rows' and parts[1] in SHAPES
                and parts[2] == 'slots' and parts[3].isdigit()) or
               (len(parts) == 3 and parts[0] == 'supplies' and parts[1].isdigit()
                and parts[2] == 'pick_pose') or
               (len(parts) == 3 and parts[0] == 'shapes' and parts[1] in SHAPES
                and parts[2] == 'grasp_z'))
    if not allowed:
        raise ValueError('Unsupported teach path: '+field)
    inverse_kinematics(pose)
    target = cfg
    for part in parts[:-1]:
        target = target[int(part)] if isinstance(target, list) else target[part]
    key = int(parts[-1]) if isinstance(target, list) else parts[-1]
    target[key] = pose[2] if parts[-1] == 'grasp_z' else list(pose)


def calibrate_marker(cfg, marker_id, marker, tool_pose):
    """Infer a shape-local grasp offset from one experimentally successful grip."""
    shape, profile = profile_for(cfg, marker_id)
    inverse_kinematics(tool_pose)
    q = marker['quaternion']
    a, c = quaternion_xy(q, 1, 0)
    b, d = quaternion_xy(q, 0, 1)
    determinant = a*d-b*c
    if abs(determinant) < 0.1:
        raise ValueError('Marker plane is too steep for an XY grasp calibration')
    wx, wy = profile['world_offset_xy']
    x = tool_pose[0]-marker['xyz'][0]-wx
    y = tool_pose[1]-marker['xyz'][1]-wy
    offset = [(d*x-b*y)/determinant, (-c*x+a*y)/determinant]
    # An ID override would hide a newly calibrated shape-wide offset.
    if str(marker_id) in cfg.get('marker_overrides', {}):
        raise ValueError('Remove the marker-specific override before calibrating its shape')
    cfg['shapes'][shape]['local_offset_xy'] = offset
    cfg['shapes'][shape]['grasp_z'] = tool_pose[2]
    cfg['shapes'][shape]['pitch_deg'] = tool_pose[3]
    return shape, offset


def demo_config():
    cfg = json.loads(Path(__file__).with_name('robocon_task_config.json').read_text(encoding='utf-8'))
    cfg['marker_shapes'] = {'1': 'triangle', '2': 'triangle', '3': 'square',
                            '4': 'square', '5': 'circle', '6': 'circle'}
    cfg['marker_map_confirmed'] = True
    cfg['calibration_confirmed'] = True
    cfg['same_coordinate_frame_confirmed'] = True
    cfg['discard_pose'] = [0.08, -0.14, 0.025, 90.0]
    cfg['supplies'] = []
    for shape, y in zip(SHAPES, (-0.06, 0.0, 0.06)):
        cfg['shapes'][shape]['grasp_z'] = 0.03
        cfg['rows'][shape]['slots'] = [[0.12, y, 0.03, 90.0], [0.16, y, 0.03, 90.0]]
        cfg['supplies'].append({'name': 'new_'+shape, 'shape': shape,
                                'pick_pose': [0.15, y+0.01, 0.03, 90.0]})
    return cfg


def offline_demo():
    cfg = demo_config()
    validate_config(cfg, execute=True)
    initial_actions(cfg)
    plans = []
    for job in jobs_for(cfg):
        if job['kind'] == 'new':
            source = cfg['supplies'][job['supply_index']]['pick_pose']
        else:
            shape = job['shape']
            source = (0.15, {'triangle': -0.06, 'square': 0, 'circle': 0.06}[shape], 0.03, 90)
        actions = plan_job(cfg, job, source)
        plans.append({'job': job, 'source': source, 'actions': actions})
        print('{} | {} | {} waypoints/actions'.format(job['name'], job['kind'], len(actions)))
    print('DEMO ONLY: {} transfers planned. No ROS or motor connection.'.format(len(plans)))
    return plans


def run_ros(args, cfg):
    import rclpy
    rclpy.init(args=args.ros_args)
    node = None
    log = None
    try:
        node = make_node(cfg)
        if args.execute:
            log = open(args.log, 'a', encoding='utf-8')
        def event(kind, **values):
            data = dict(event=kind, time=time.time(), **values)
            line = json.dumps(data, ensure_ascii=False)
            print(line, flush=True)
            if log:
                log.write(line+'\n')
                log.flush()
        def spin():
            if not rclpy.ok():
                raise RuntimeError('ROS shutdown')
            rclpy.spin_once(node, timeout_sec=0.02)
            node.reader.tick(time.monotonic())
        def wait_for(predicate, timeout, label):
            until = time.monotonic()+timeout
            while time.monotonic() < until:
                spin()
                value = predicate()
                if value:
                    return value
            raise ValueError('Timed out: '+label)
        def perform(actions):
            executor = Executor(cfg, node.send, node.reader, event)
            executor.start(actions)
            while True:
                spin()
                if executor.tick(time.monotonic()):
                    return
                if executor.error:
                    raise RuntimeError(executor.error+'; no further commands will be sent')
        if args.observe:
            until, next_print = time.monotonic()+args.observe_seconds, 0.0
            while time.monotonic() < until:
                spin()
                now = time.monotonic()
                if now >= next_print:
                    visible = {i: node.markers.stable(i, now) for i in sorted(node.markers.history)}
                    visible = {i: v for i, v in visible.items() if v is not None}
                    event('observation', markers=visible,
                          joint_positions=node.reader.positions if node.reader.fresh(now) else None)
                    next_print = now+1.0
            return
        wait_for(lambda: node.reader.fresh(time.monotonic()), 10, 'current joint positions')
        if args.teach or args.calibrate_marker is not None:
            xyz, pitch, _ = forward_kinematics([ticks_to_radians(t) for t in node.reader.positions[:4]])
            pose = (*xyz, math.degrees(pitch))
            if args.teach:
                teach_value(cfg, args.teach, pose)
            else:
                marker_id = args.calibrate_marker
                marker = wait_for(lambda: node.markers.stable(marker_id, time.monotonic()),
                                  cfg['marker_wait_s'], 'visible marker for grasp calibration')
                shape, offset = calibrate_marker(cfg, marker_id, marker, pose)
                event('grasp_calibrated', shape=shape, local_offset_xy=offset)
            path = Path(args.config)
            temporary = path.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(cfg, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            temporary.replace(path)
            event('taught', field=args.teach or 'marker '+str(args.calibrate_marker), pose=pose)
            return
        if args.prepare:
            wait_for(lambda: node.publisher.get_subscription_count() > 0, 10, 'controller subscriber')
            perform([{'kind': 'move', 'ticks': [2048]*4, 'label': 'prepare initial joints'},
                     {'kind': 'grip', 'ticks': [int(2048+cfg['initial_gripper_deg']/0.088)],
                      'label': 'prepare closed hand'}])
            event('prepared', note='Initial joint zero pose and closed hand commanded')
            return
        jobs = jobs_for(cfg)
        failed_shapes = set()
        if args.execute:
            if any(abs(t-2048)*0.088 > cfg['initial_tolerance_deg'] for t in node.reader.positions[:4]):
                raise ValueError('Start joints are not zero. Run --prepare before setting up the competition.')
            grip_angle = (node.reader.positions[4]-2048)*0.088
            if abs(grip_angle-cfg['initial_gripper_deg']) > cfg['initial_tolerance_deg']:
                raise ValueError('Initial hand is not closed. Run --prepare before competition setup.')
            perform(initial_actions(cfg))
        completed, skipped, preview = [], [], []
        for job in jobs:
            if job['kind'] != 'discard' and job['shape'] in failed_shapes:
                skipped.append(job['name'])
                event('skip', job=job['name'], reason='existing row product was not placed')
                continue
            try:
                if 'marker_id' in job:
                    marker = wait_for(lambda: node.markers.stable(job['marker_id'], time.monotonic()),
                                      cfg['marker_wait_s'], 'marker ID '+str(job['marker_id']))
                    source = pick_pose(cfg, job['marker_id'], marker)
                else:
                    source = cfg['supplies'][job['supply_index']]['pick_pose']
                actions = plan_job(cfg, job, source)
            except ValueError as exc:
                skipped.append(job['name'])
                if job['kind'] != 'discard':
                    failed_shapes.add(job['shape'])
                event('skip', job=job['name'], reason=str(exc))
                continue
            event('job_begin', job=job['name'], source=source, actions=len(actions))
            preview.append({'job': job, 'source': source, 'actions': actions})
            if args.execute:
                perform(actions)
            completed.append(job['name'])
            event('job_sequence_finished', job=job['name'], physical_grasp_verified=False)
        if not args.execute:
            Path(args.plan_output).write_text(json.dumps(preview, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        event('finished', mode='execute' if args.execute else 'plan_only',
              sequences_finished=completed, skipped=skipped,
              note='Command completion does not prove grasp success or competition score')
        if skipped:
            raise RuntimeError('Some jobs were skipped; see the reasons above')
    finally:
        if log:
            log.close()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).with_name('robocon_task_config.json')))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='Send physical motion commands')
    mode.add_argument('--observe', action='store_true', help='Print markers and joints; no motion')
    mode.add_argument('--teach', metavar='FIELD', help='Save current tool pose to a configuration field; no motion')
    mode.add_argument('--calibrate-marker', type=int, metavar='ID',
                      help='Save shape grasp Z/local offset from current tool and visible marker; no motion')
    mode.add_argument('--prepare', action='store_true', help='Move to joint-zero pose and close hand before setup')
    mode.add_argument('--demo', action='store_true', help='Offline example plan; no ROS required')
    parser.add_argument('--observe-seconds', type=float, default=20.0)
    parser.add_argument('--log', default='robocon_task_run.jsonl')
    parser.add_argument('--plan-output', default='robocon_task_plan.json')
    args, args.ros_args = parser.parse_known_args()
    try:
        if args.demo:
            offline_demo()
            return
        cfg = json.loads(Path(args.config).read_text(encoding='utf-8'))
        if not (args.observe or args.teach or args.prepare or args.calibrate_marker is not None):
            validate_config(cfg, args.execute)
        run_ros(args, cfg)
    except KeyboardInterrupt:
        print('Interrupted: further commands stopped; already issued motion is NOT cancelled.')
    except (ValueError, RuntimeError, ImportError, KeyError, IndexError, OSError) as exc:
        raise SystemExit('ERROR: {}\nSee ROBOCON_TASK.md'.format(exc))


if __name__ == '__main__':
    main()
