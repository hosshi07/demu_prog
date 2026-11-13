"""Forward kinematics matching inverseKinematics() in keyboard.py/robocon.py.

Lengths are metres; joint angles are radians. No ROS or NumPy required.
"""
import math

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
        raise ValueError('Target is outside the geometric reach of the arm')

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
