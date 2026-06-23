import argparse
import copy
import math
import sys
import time
from types import SimpleNamespace

import rclpy

from .client import EasyArmCli
from .terminal import RawTerminal


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run EasyArm task1: move to pose1, then ServoL a 100mm square corner path.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--named-state", default="pose1", help="SRDF group_state used as task origin.")
    parser.add_argument("--segment-duration", type=float, default=3.0, help="ServoL publish duration per segment.")
    parser.add_argument("--rate", type=float, default=50.0, help="ServoL target publish rate in Hz.")
    parser.add_argument("--velocity-scale", type=float, default=1.0, help="MoveNamedState velocity scale.")
    parser.add_argument("--acceleration-scale", type=float, default=1.0, help="MoveNamedState acceleration scale.")
    parser.add_argument("--settle-time", type=float, default=0.5, help="Wait time after reaching named state.")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--frame-id", default="base_link")
    return parser


def _pose_to_target(pose, dx: float, dy: float, dz: float) -> dict:
    return {
        "position": [
            pose.pose.position.x + dx,
            pose.pose.position.y + dy,
            pose.pose.position.z + dz,
        ],
        "orientation": [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ],
    }


def _quat_from_rpy(rx: float, ry: float, rz: float) -> list[float]:
    cr = math.cos(rx * 0.5)
    sr = math.sin(rx * 0.5)
    cp = math.cos(ry * 0.5)
    sp = math.sin(ry * 0.5)
    cy = math.cos(rz * 0.5)
    sy = math.sin(rz * 0.5)

    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def _quat_multiply(q1: list[float], q2: list[float]) -> list[float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def _quat_normalize(q: list[float]) -> list[float]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in q]


def _quat_slerp(q1: list[float], q2: list[float], ratio: float) -> list[float]:
    q1 = _quat_normalize(q1)
    q2 = _quat_normalize(q2)
    dot = sum(a * b for a, b in zip(q1, q2))
    if dot < 0.0:
        q2 = [-value for value in q2]
        dot = -dot

    if dot > 0.9995:
        return _quat_normalize([
            q1[index] + (q2[index] - q1[index]) * ratio
            for index in range(4)
        ])

    dot = max(-1.0, min(1.0, dot))
    theta_0 = math.acos(dot)
    theta = theta_0 * ratio
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    scale_1 = math.cos(theta) - dot * sin_theta / sin_theta_0
    scale_2 = sin_theta / sin_theta_0
    return _quat_normalize([
        scale_1 * q1[index] + scale_2 * q2[index]
        for index in range(4)
    ])


def _pose_rotate(pose, rx: float, ry: float, rz: float):
    rotated = copy.deepcopy(pose)
    q_base = [
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ]
    q_delta = _quat_from_rpy(rx, ry, rz)
    # Right multiplication means the RPY delta is applied in the local tool frame.
    q_rotated = _quat_normalize(_quat_multiply(q_base, q_delta))
    rotated.pose.orientation.x = q_rotated[0]
    rotated.pose.orientation.y = q_rotated[1]
    rotated.pose.orientation.z = q_rotated[2]
    rotated.pose.orientation.w = q_rotated[3]
    return rotated


def _smoothstep5(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return 10.0 * t ** 3 - 15.0 * t ** 4 + 6.0 * t ** 5


def _interpolate_target(start: dict, end: dict, ratio: float) -> dict:
    s = _smoothstep5(ratio)
    position = [
        start["position"][index] + (end["position"][index] - start["position"][index]) * s
        for index in range(3)
    ]
    return {
        "position": position,
        "orientation": _quat_slerp(start["orientation"], end["orientation"], s),
    }


def _publish_servol_segment(
    node: EasyArmCli,
    label: str,
    start: dict,
    target: dict,
    frame_id: str,
    duration: float,
    rate: float,
) -> int:
    interval = 1.0 / rate
    samples = max(2, int(round(duration * rate)))
    node.get_logger().info(
        f"{label}: target cmd: "
        f"{target['position'][0]:.6f} {target['position'][1]:.6f} {target['position'][2]:.6f} "
        f"{target['orientation'][0]:.6f} {target['orientation'][1]:.6f} "
        f"{target['orientation'][2]:.6f} {target['orientation'][3]:.6f}"
    )
    next_time = time.monotonic()
    count = 0
    for index in range(samples):
        ratio = index / (samples - 1)
        node.servol_pub.publish(
            node._make_servol_pose(_interpolate_target(start, target, ratio), frame_id)
        )
        count += 1
        next_time += interval
        sleep_time = next_time - time.monotonic()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_time = time.monotonic()
    node.get_logger().info(f"{label}: published ServoL targets: {count}")
    return count


def _wait_for_space(node: EasyArmCli) -> bool:
    node.get_logger().info("MoveJ finished. Press Space to start ServoL path, or Esc/q to abort.")
    with RawTerminal() as terminal:
        while rclpy.ok():
            key = terminal.read_key()
            if key == " ":
                print()
                return True
            if key in ("\x1b", "\x03", "q", "Q"):
                print()
                node.get_logger().warning("easyarm_task1 aborted before ServoL path")
                return False
            time.sleep(0.02)
    return False


def run_task(args) -> int:
    if args.rate <= 0.0:
        print("--rate must be greater than 0", file=sys.stderr)
        return 1
    if args.segment_duration <= 0.0:
        print("--segment-duration must be greater than 0", file=sys.stderr)
        return 1

    rclpy.init()
    node = EasyArmCli()
    try:
        if not node._require_position_mode(args.timeout):
            return 1

        node.get_logger().info(f"Moving to named state '{args.named_state}'")
        move_args = SimpleNamespace(
            name=args.named_state,
            velocity_scale=0.1,
            acceleration_scale=0.1,
            execute=True,
            timeout=args.timeout,
        )
        if node.move_named_state(move_args) != 0:
            return 1

        if args.settle_time > 0.0:
            node.get_logger().info(f"Settling for {args.settle_time:.2f}s before ServoL path")
            time.sleep(args.settle_time)

        if not _wait_for_space(node):
            return 1

        origin_pose = node.get_current_pose(args.timeout)
        if origin_pose is None:
            return 1

        origin = origin_pose.pose.position
        orientation = origin_pose.pose.orientation
        origin_target = _pose_to_target(origin_pose, 0.0, 0.0, 0.0)
        node.get_logger().info(
            "pose1 origin cmd: "
            f"{origin.x:.6f} {origin.y:.6f} {origin.z:.6f} "
            f"{orientation.x:.6f} {orientation.y:.6f} {orientation.z:.6f} {orientation.w:.6f}"
        )

        if not node._wait_for_subscribers(node.servol_pub, "/easyarm/servol_cmd", args.timeout):
            return 1

        segments = [
            ("1", _pose_to_target(origin_pose, 0.0, 0.1, 0.0), 1.0),
            ("2", _pose_to_target(origin_pose, 0.1, 0.1, 0.0), 1.0),
            ("3", _pose_to_target(origin_pose, 0.1, 0.0, 0.0), 1.0),
            ("4", _pose_to_target(origin_pose, 0.0, 0.0, 0.0), 1.0),
            ("5", _pose_to_target(origin_pose, 0.0, 0.0, 0.1), 1.0),
            ("6", _pose_to_target(origin_pose, 0.0, 0.0, 0.0), 1.0),
            ("rotate_x90", _pose_to_target(_pose_rotate(origin_pose, 0.0, math.pi / 2.0, 0.0), 0.0, 0.0, 0.0), 3.0),
            ("rotate_back", _pose_to_target(origin_pose, 0.0, 0.0, 0.0), 3.0),
        ]

        current_target = origin_target
        for label, target, segment_duration in segments:
            _publish_servol_segment(
                node,
                label,
                current_target,
                target,
                args.frame_id,
                segment_duration,
                args.rate,
            )
            current_target = target

        node.get_logger().info("easyarm_task1 completed")
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main() -> None:
    sys.exit(run_task(build_parser().parse_args()))


if __name__ == "__main__":
    main()
