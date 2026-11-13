#!/usr/bin/env python3
"""arm_demo.txt の運動学と DYNAMIXEL 指令を使い、20 mm の積み木を2段積む。"""

import argparse
import math
import time
from dataclasses import dataclass
from typing import Optional


BLOCK_HEIGHT = 0.020
CLEARANCE = 0.020
WAYPOINT_SPACING = 0.020
ARM_IDS = (11, 12, 13, 14)
GRIPPER_ID = 15
POSITION_TOLERANCE = 15  # 約1.3度
START_TOLERANCE = 340    # 約30度。初回の移動を近くの待機姿勢から始める
JOINT_LIMITS = (
    (-math.pi, math.pi),
    (-2.05, math.pi / 2),
    (-math.pi / 2, 1.53),
    (-1.8, 2.0),
)


@dataclass
class Step:
    label: str
    poses: tuple = ()
    gripper_degree: Optional[float] = None


def position_ticks(degrees):
    ticks = int(2048 + degrees / 0.088)
    if not 0 <= ticks <= 4095:
        raise ValueError(f"モータ位置指令が範囲外です: {ticks}")
    return ticks


def arm_targets(pose):
    # arm_demo.txt と同じリンク寸法、逆運動学、下向きの pitch=90°。
    x, y, z = pose
    l1, l2, l3, l4, l5, l6 = 0.077, 0.130, 0.124, 0.126, 0.024, 0.128
    pitch = math.pi / 2
    h = z - l1 + l4 * math.sin(pitch)
    k = math.hypot(x, y) - l4 * math.cos(pitch)
    distance = math.hypot(h, k)
    if not abs(l2 - l3) < distance < l2 + l3:
        raise ValueError(f"アームが届かない位置です: {pose}")
    alpha = math.acos((l2**2 + distance**2 - l3**2) / (2 * l2 * distance))
    beta = math.acos((l2**2 + l3**2 - distance**2) / (2 * l2 * l3))
    shoulder = pitch - alpha - math.atan2(h, k) - math.atan2(l5, l6)
    elbow = pitch - beta + math.atan2(l5, l6)
    angles = (math.atan2(y, x), shoulder, elbow, pitch - shoulder - elbow)
    for motor_id, angle, (minimum, maximum) in zip(ARM_IDS, angles, JOINT_LIMITS):
        if not minimum <= angle <= maximum:
            raise ValueError(f"モータID {motor_id} の関節範囲外です: {pose}")
    return dict(zip(ARM_IDS, (position_ticks(math.degrees(angle)) for angle in angles)))


def movement(label, start, end):
    distance = math.dist(start, end)
    if distance > 0.8:
        raise ValueError(f"{label}: 移動距離が作業範囲を超えています")
    count = max(1, math.ceil(distance / WAYPOINT_SPACING))
    intermediate = tuple(
        tuple(a + (b - a) * index / count for a, b in zip(start, end))
        for index in range(1, count)
    )
    return Step(label, intermediate + (end,))


def build_plan(args):
    locations = tuple(map(tuple, (args.pick1, args.pick2, args.stack)))
    values = (
        *args.pick1, *args.pick2, *args.stack, args.table_z, args.tcp_offset,
        args.travel_z, args.open_deg, args.close_deg,
    )
    if not all(map(math.isfinite, values)):
        raise ValueError("座標と設定には有限の数値を指定してください")
    if len(set(locations)) != 3:
        raise ValueError("取り位置2か所と積む場所は別々に指定してください")
    if any(x <= 0 for x, _ in locations):
        raise ValueError("取り位置と積む場所はアーム前方（x > 0）に指定してください")
    if not -50 <= args.open_deg < args.close_deg <= 0:
        raise ValueError("グリッパ角は -50 ≤ 開く角度 < 閉じる角度 ≤ 0 としてください")

    pick_z = args.table_z + BLOCK_HEIGHT / 2 + args.tcp_offset
    minimum_travel_z = pick_z + 2 * BLOCK_HEIGHT + CLEARANCE
    if args.travel_z < minimum_travel_z - 1e-9:
        raise ValueError(f"運搬高さは {minimum_travel_z:.3f} m 以上にしてください")

    stack_above = (*args.stack, args.travel_z)
    first_above = (*args.pick1, args.travel_z)
    plan = [
        Step("1個目の真上へ移動", (first_above,)),
        Step("グリッパを開く", gripper_degree=args.open_deg),
    ]
    for index, pick_xy in enumerate(locations[:2]):
        label = f"{index + 1}個目"
        pick_above = (*pick_xy, args.travel_z)
        pick = (*pick_xy, pick_z)
        place = (*args.stack, pick_z + index * BLOCK_HEIGHT)
        if index:
            plan.append(movement(f"{label}の真上へ移動", stack_above, pick_above))
        plan.extend((
            movement(f"{label}を掴む高さまで下ろす", pick_above, pick),
            Step(f"{label}を掴む", gripper_degree=args.close_deg),
            movement(f"{label}を持ち上げる", pick, pick_above),
            movement(f"{label}を積む位置の上へ運ぶ", pick_above, stack_above),
            movement(f"{index + 1}段目に下ろす", stack_above, place),
            Step(f"{label}を離す", gripper_degree=args.open_deg),
            movement(f"{label}を離した位置から上へ退避", place, stack_above),
        ))
    return plan


def compile_commands(step):
    if step.poses:
        return [arm_targets(pose) for pose in step.poses]
    return [{GRIPPER_ID: position_ticks(step.gripper_degree)}]


class ArmController:
    def __init__(self, node):
        from dynamixel_sdk_custom_interfaces.msg import SetPosition
        from dynamixel_sdk_custom_interfaces.srv import GetPosition

        self.node = node
        self.command_type = SetPosition
        self.position_type = GetPosition
        self.publisher = node.create_publisher(SetPosition, "/set_position", 10)
        self.position_client = node.create_client(GetPosition, "/get_position")

    def ready(self):
        if not self.position_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/get_position が見つかりません")
        deadline = time.monotonic() + 5.0
        while self.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.1)
        if self.publisher.get_subscription_count() != 1:
            raise RuntimeError("/set_position の受信ノードが1つか確認してください")

    def read_position(self, motor_id):
        import rclpy

        request = self.position_type.Request()
        request.id = motor_id
        future = self.position_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        response = future.result() if future.done() else None
        if response is None:
            raise RuntimeError(f"モータID {motor_id} の位置を取得できません")
        ticks = response.position
        if not 0 <= ticks <= 4095:
            raise RuntimeError(f"モータID {motor_id} の現在位置が範囲外です: {ticks}")
        return ticks

    def check_start(self, targets):
        for motor_id, target in targets.items():
            actual = self.read_position(motor_id)
            if abs(actual - target) > START_TOLERANCE:
                raise RuntimeError(
                    f"開始姿勢が1個目の真上から離れています "
                    f"（ID {motor_id}、現在 {actual}、目標 {target}）"
                )

    def send(self, targets, check_arrival=True):
        import rclpy

        if not rclpy.ok() or self.publisher.get_subscription_count() != 1:
            raise RuntimeError("アーム制御ノードとの接続を確認してください")
        for motor_id, ticks in targets.items():
            msg = self.command_type()
            msg.id = motor_id
            msg.position = ticks
            msg.velocity = 25
            msg.acceleration = 5
            self.publisher.publish(msg)

        # 把持時は積み木に当たるので、閉じる目標値への到達は要求しない。
        if not check_arrival:
            time.sleep(2.0)
            return
        time.sleep(0.25)
        deadline = time.monotonic() + 8.0
        positions = {}
        while time.monotonic() < deadline:
            positions = {motor_id: self.read_position(motor_id) for motor_id in targets}
            if all(abs(positions[motor_id] - ticks) <= POSITION_TOLERANCE
                   for motor_id, ticks in targets.items()):
                return
            time.sleep(0.1)
        raise RuntimeError(f"目標位置への到達を確認できません。目標: {targets} 現在: {positions}")


def execute(plan, commands, args):
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=[])
    node = None
    try:
        node = Node("stack_two_blocks")
        arm = ArmController(node)
        arm.ready()
        arm.check_start(commands[0][0])
        for step, targets in zip(plan, commands):
            print(step.label, flush=True)
            if args.step:
                input("周囲と積み木の状態を確認し、実行するには Enter: ")
            for target in targets:
                arm.send(target, check_arrival=step.gripper_degree != args.close_deg)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="平置きした高さ20 mmの積み木を2個積む")
    for name in ("pick1", "pick2", "stack"):
        parser.add_argument(f"--{name}", type=float, nargs=2, metavar=("X", "Y"), required=True)
    parser.add_argument("--table-z", type=float, required=True, help="机面の高さ [m]")
    parser.add_argument("--tcp-offset", type=float, required=True, help="把持中心から制御点までのZ補正 [m]")
    parser.add_argument("--travel-z", type=float, required=True, help="運搬時の制御点の高さ [m]")
    parser.add_argument("--open-deg", type=float, default=-50.0, help="開くモータ角度 [度]")
    parser.add_argument("--close-deg", type=float, required=True, help="閉じるモータ角度 [度]")
    parser.add_argument("--execute", action="store_true", help="実機へ指令する")
    parser.add_argument("--step", action="store_true", help="実機実行時に工程ごとにEnterを待つ")
    args = parser.parse_args()
    try:
        plan = build_plan(args)
        commands = [compile_commands(step) for step in plan]
        for step in plan:
            if step.poses:
                x, y, z = step.poses[-1]
                print(f"{step.label}: x={x:.3f} y={y:.3f} z={z:.3f} m ({len(step.poses)}点)")
            else:
                print(f"{step.label}: {step.gripper_degree:g} 度")
        print(f"開始姿勢の確認用・1個目の真上のモータ目標値: {commands[0][0]}")
        if args.execute:
            execute(plan, commands, args)
            print("2段分の動作指令が完了しました。積み木の状態を確認してください。")
        else:
            print("予定表示のみ。ROSへの送信はありません。")
        return 0
    except KeyboardInterrupt:
        print("以後の送信を中止しました。送信済みの指令は有効な場合があります。")
        return 130
    except (ValueError, RuntimeError, ImportError, EOFError) as error:
        print(f"中止: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
