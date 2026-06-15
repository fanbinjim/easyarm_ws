#!/usr/bin/env python3
"""Publish recorded EasyArm JSON as a MoveIt DisplayTrajectory."""

import json
import sys
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def print_missing_record_file(json_path: Path) -> None:
    resolved_path = json_path if json_path.is_absolute() else Path.cwd() / json_path
    print(f"Record file not found: {json_path}", file=sys.stderr)
    print(f"Resolved path: {resolved_path}", file=sys.stderr)
    print(
        "Please run `ros2 run easyarm_move_task easyarm_record` first, "
        "or pass an existing record JSON path.",
        file=sys.stderr,
    )

    record_root = Path.cwd() / "data" / "path_record"
    if record_root.is_dir():
        candidates = sorted(
            record_root.rglob("*.json"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        if candidates:
            print("Recent records:", file=sys.stderr)
            for candidate in candidates[:5]:
                print(f"  {candidate}", file=sys.stderr)


class ReplayRvizNode(Node):
    def __init__(self, json_path: Path) -> None:
        super().__init__("easyarm_replay_rviz")
        self.json_path = json_path
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", qos)

    def publish_file(self) -> None:
        display_trajectory = self._load_display_trajectory()
        self.publisher.publish(display_trajectory)
        point_count = len(display_trajectory.trajectory[0].joint_trajectory.points)
        self.get_logger().info(
            f"Published {point_count} points from {self.json_path} to /display_planned_path")

    def _load_display_trajectory(self) -> DisplayTrajectory:
        with self.json_path.open("r", encoding="utf-8") as file:
            data = json.load(file)

        joint_names = data.get("joint_names")
        samples = data.get("samples")
        if not isinstance(joint_names, list) or not joint_names:
            raise ValueError("JSON field 'joint_names' must be a non-empty list")
        if not isinstance(samples, list) or not samples:
            raise ValueError("JSON field 'samples' must be a non-empty list")

        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = [str(name) for name in joint_names]

        for index, sample in enumerate(samples):
            point = self._sample_to_point(sample, index, len(joint_names))
            trajectory.joint_trajectory.points.append(point)

        start_state = RobotState()
        start_state.joint_state = JointState()
        start_state.joint_state.name = trajectory.joint_trajectory.joint_names
        start_state.joint_state.position = list(
            trajectory.joint_trajectory.points[0].positions)

        display_trajectory = DisplayTrajectory()
        display_trajectory.model_id = "EasyARM-A1"
        display_trajectory.trajectory_start = start_state
        display_trajectory.trajectory.append(trajectory)
        return display_trajectory

    def _sample_to_point(
        self, sample: object, index: int, joint_count: int
    ) -> JointTrajectoryPoint:
        if not isinstance(sample, dict):
            raise ValueError(f"samples[{index}] must be an object")

        if "t" not in sample:
            raise ValueError(f"samples[{index}] missing field 't'")
        if "joints" not in sample:
            raise ValueError(f"samples[{index}] missing field 'joints'")

        t = float(sample["t"])
        joints = sample["joints"]
        if not isinstance(joints, list) or len(joints) != joint_count:
            raise ValueError(
                f"samples[{index}].joints must contain {joint_count} values")
        if t < 0.0:
            raise ValueError(f"samples[{index}].t must be non-negative")

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in joints]
        point.time_from_start = seconds_to_duration(t)
        return point


def seconds_to_duration(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    duration = Duration()
    duration.sec = sec
    duration.nanosec = nanosec
    return duration


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv if argv is None else argv
    rclpy.init(args=argv)

    args = rclpy.utilities.remove_ros_args(args=argv)[1:]
    if len(args) != 1:
        print("Usage: ros2 run easyarm_move_task easyarm_replay_rviz <record.json>")
        rclpy.shutdown()
        return 1

    json_path = Path(args[0]).expanduser()
    if not json_path.is_file():
        print_missing_record_file(json_path)
        rclpy.shutdown()
        return 1

    node = ReplayRvizNode(json_path)
    try:
        node.publish_file()
        rclpy.spin_once(node, timeout_sec=0.5)
    except Exception as ex:  # noqa: BLE001 - ROS CLI should report validation errors cleanly.
        node.get_logger().error(str(ex))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
