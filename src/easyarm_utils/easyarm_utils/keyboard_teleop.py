#!/usr/bin/env python3
"""Keyboard teleoperation for EasyArm end-effector pose."""

import math
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]


@dataclass
class TeleopCommand:
    name: str
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_axis: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_sign: float = 0.0


class RawTerminal:
    def __init__(self) -> None:
        self._enabled = sys.stdin.isatty()
        self._original = None
        if self._enabled:
            self._original = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())

    def close(self) -> None:
        if self._enabled and self._original is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._original)
            self._enabled = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def enabled(self) -> bool:
        return self._enabled

    def read_key(self) -> str | None:
        if not self._enabled:
            return None

        if not select.select([sys.stdin], [], [], 0.0)[0]:
            return None

        char = sys.stdin.read(1)
        if char != "\x1b":
            return char

        if not select.select([sys.stdin], [], [], 0.12)[0]:
            return "esc"
        second = sys.stdin.read(1)
        if second not in ("[", "O"):
            return "esc"
        if not select.select([sys.stdin], [], [], 0.12)[0]:
            return None
        third = sys.stdin.read(1)
        return {
            "A": "up",
            "B": "down",
            "C": "right",
            "D": "left",
        }.get(third)


class JointStateCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}

    def update(self, msg: JointState) -> None:
        with self._lock:
            for index, name in enumerate(msg.name):
                if name in JOINT_NAMES and index < len(msg.position):
                    self._positions[name] = msg.position[index]

    def positions(self) -> list[float] | None:
        with self._lock:
            if any(name not in self._positions for name in JOINT_NAMES):
                return None
            return [self._positions[name] for name in JOINT_NAMES]

    def joint_state_msg(self) -> JointState | None:
        positions = self.positions()
        if positions is None:
            return None
        msg = JointState()
        msg.name = list(JOINT_NAMES)
        msg.position = positions
        return msg


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("easyarm_keyboard_teleop")

        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.ee_frame = self.declare_parameter("ee_frame", "Link6").value
        self.group_name = self.declare_parameter("group_name", "arm").value
        self.linear_step = float(self.declare_parameter("linear_step", 0.005).value)
        self.angular_step_deg = float(
            self.declare_parameter("angular_step_deg", 2.0).value)
        self.trajectory_duration = float(
            self.declare_parameter("trajectory_duration", 0.25).value)
        self.stream_duration = float(
            self.declare_parameter("stream_duration", 0.12).value)
        self.ik_timeout = float(self.declare_parameter("ik_timeout", 0.1).value)
        self.max_joint_delta = float(
            self.declare_parameter("max_joint_delta_per_step", 0.25).value)
        self.require_confirm = bool(
            self.declare_parameter("require_confirm", True).value)

        self.status = "Initializing"
        self.joint_cache = JointStateCache()
        self.create_subscription(
            JointState,
            "joint_states",
            self.joint_cache.update,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.param_client = self.create_client(
            SetParameters, "/easyarm_hardware_control_mode/set_parameters")
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory")
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            "arm_controller/joint_trajectory",
            10,
        )
        self.active_goal = None
        self.target_pose: PoseStamped | None = None
        self.target_joints: list[float] | None = None

    def wait_until_ready(self) -> bool:
        self.status = "Waiting for joint_states"
        if not self._wait_for_joint_states(3.0):
            self.get_logger().error("Timeout waiting for Joint1-Joint6 states")
            return False

        self.status = "Waiting for TF"
        if not self._wait_for_tf(3.0):
            self.get_logger().error(
                f"Timeout waiting for TF {self.base_frame} -> {self.ee_frame}")
            return False

        self.status = "Waiting for /compute_ik"
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("/compute_ik service is not available")
            return False

        self.status = "Waiting for arm_controller"
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                "arm_controller/follow_joint_trajectory is not available")
            return False

        return True

    def initialize_target_pose(self) -> bool:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=RclpyDuration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().error(f"Failed to initialize target pose: {exc}")
            return False

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation.x = transform.transform.rotation.x
        pose.pose.orientation.y = transform.transform.rotation.y
        pose.pose.orientation.z = transform.transform.rotation.z
        pose.pose.orientation.w = transform.transform.rotation.w
        self.target_pose = pose
        self.target_joints = self.joint_cache.positions()
        self.status = "Target pose initialized"
        return True

    def enter_position_mode(self) -> bool:
        self.status = "Holding current position"
        current = self.joint_cache.positions()
        if current is None or not self.send_joint_goal(current):
            self.get_logger().error("Failed to send initial hold trajectory")
            return False
        self.target_joints = list(current)

        self.status = "Switching to POSITION"
        if not self.param_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                "/easyarm_hardware_control_mode/set_parameters is not available")
            return False

        request = SetParameters.Request()
        param = Parameter()
        param.name = "controller_mode"
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value="POSITION",
        )
        request.parameters = [param]
        future = self.param_client.call_async(request)
        if not self._wait_future(future, 3.0):
            self.get_logger().error("Timeout setting controller_mode to POSITION")
            return False
        if not all(result.successful for result in future.result().results):
            self.get_logger().error("Failed to set controller_mode to POSITION")
            return False

        time.sleep(0.2)
        self.status = "Ready"
        return True

    def execute_command(self, command: TeleopCommand) -> None:
        current_joints = self.joint_cache.positions()
        if current_joints is None:
            self.status = "No complete joint state"
            return

        target_pose = self._candidate_target_pose(command)
        if target_pose is None:
            self.status = "Target pose is not initialized"
            return

        reference_joints = self.target_joints or current_joints
        target_joints = self._compute_ik(target_pose, reference_joints)
        if target_joints is None:
            self.status = f"IK failed: {command.name}"
            return

        max_delta = max(
            abs(target - current)
            for target, current in zip(target_joints, reference_joints))
        if max_delta > self.max_joint_delta:
            self.status = (
                f"Rejected IK jump {max_delta:.3f} rad > "
                f"{self.max_joint_delta:.3f} rad")
            return

        self.publish_joint_target(target_joints, reference_joints)
        self.target_pose = target_pose
        self.target_joints = list(target_joints)
        self.status = f"Streaming: {command.name}"

    def send_joint_goal(self, positions: list[float]) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start = seconds_to_duration(self.trajectory_duration)
        goal.trajectory.points.append(point)

        send_future = self.action_client.send_goal_async(goal)
        if not self._wait_future(send_future, 2.0):
            return False
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False

        self.active_goal = goal_handle
        result_future = goal_handle.get_result_async()
        timeout = max(2.0, self.trajectory_duration + 1.0)
        if not self._wait_future(result_future, timeout):
            self.cancel_active_goal()
            return False
        self.active_goal = None
        return result_future.result().status == 4

    def publish_joint_target(
        self,
        positions: list[float],
        reference_positions: list[float] | None = None,
    ) -> None:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(JOINT_NAMES)

        duration = max(self.stream_duration, 0.02)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        if reference_positions is not None:
            point.velocities = [
                float((target - reference) / duration)
                for target, reference in zip(positions, reference_positions)
            ]
        point.time_from_start = seconds_to_duration(duration)
        trajectory.points.append(point)

        self.trajectory_publisher.publish(trajectory)

    def cancel_active_goal(self) -> None:
        if self.active_goal is not None:
            self.active_goal.cancel_goal_async()
            self.active_goal = None

    def hold_current_position(self) -> None:
        positions = self.joint_cache.positions()
        if positions is not None:
            self.send_joint_goal(positions)
            self.target_joints = list(positions)

    def _candidate_target_pose(self, command: TeleopCommand) -> PoseStamped | None:
        if self.target_pose is None:
            return None

        current_q = normalize_quaternion((
            self.target_pose.pose.orientation.x,
            self.target_pose.pose.orientation.y,
            self.target_pose.pose.orientation.z,
            self.target_pose.pose.orientation.w,
        ))
        local_translation = tuple(
            value * self.linear_step for value in command.translation)
        base_translation = rotate_vector(current_q, local_translation)

        delta_q = axis_angle_quaternion(
            command.rotation_axis,
            math.radians(self.angular_step_deg) * command.rotation_sign,
        )
        target_q = normalize_quaternion(quaternion_multiply(current_q, delta_q))

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.target_pose.pose.position.x + base_translation[0]
        pose.pose.position.y = self.target_pose.pose.position.y + base_translation[1]
        pose.pose.position.z = self.target_pose.pose.position.z + base_translation[2]
        pose.pose.orientation.x = target_q[0]
        pose.pose.orientation.y = target_q[1]
        pose.pose.orientation.z = target_q[2]
        pose.pose.orientation.w = target_q[3]
        return pose

    def _compute_ik(
        self,
        pose: PoseStamped,
        seed_positions: list[float] | None = None,
    ) -> list[float] | None:
        if seed_positions is None:
            seed_positions = self.joint_cache.positions()
        if seed_positions is None:
            return None

        seed = JointState()
        seed.name = list(JOINT_NAMES)
        seed.position = [float(value) for value in seed_positions]

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ee_frame
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = seed
        request.ik_request.timeout = seconds_to_duration(self.ik_timeout)
        request.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(request)
        if not self._wait_future(future, self.ik_timeout + 1.0):
            return None
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        solution = response.solution.joint_state
        joints: list[float] = []
        for name in JOINT_NAMES:
            if name not in solution.name:
                return None
            index = solution.name.index(name)
            joints.append(float(solution.position[index]))
        return joints

    def _wait_for_joint_states(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.joint_cache.positions() is not None:
                return True
            time.sleep(0.02)
        return False

    def _wait_for_tf(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_frame,
                    rclpy.time.Time(),
                    timeout=RclpyDuration(seconds=0.1),
                )
                return True
            except TransformException:
                time.sleep(0.05)
        return False

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return future.done()


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


def normalize_quaternion(
    q: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def quaternion_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_conjugate(
    q: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def rotate_vector(
    q: tuple[float, float, float, float],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    vector_q = (v[0], v[1], v[2], 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(q, vector_q),
        quaternion_conjugate(q),
    )
    return (rotated[0], rotated[1], rotated[2])


def axis_angle_quaternion(
    axis: tuple[float, float, float],
    angle: float,
) -> tuple[float, float, float, float]:
    if angle == 0.0 or axis == (0.0, 0.0, 0.0):
        return (0.0, 0.0, 0.0, 1.0)
    norm = math.sqrt(sum(value * value for value in axis))
    half = angle * 0.5
    scale = math.sin(half) / norm
    return (axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(half))


def command_from_key(key: str | None) -> TeleopCommand | None:
    commands = {
        "i": TeleopCommand("forward -Z", translation=(0.0, 0.0, -1.0)),
        "I": TeleopCommand("forward -Z", translation=(0.0, 0.0, -1.0)),
        "k": TeleopCommand("backward +Z", translation=(0.0, 0.0, 1.0)),
        "K": TeleopCommand("backward +Z", translation=(0.0, 0.0, 1.0)),
        "j": TeleopCommand("left -Y", translation=(0.0, -1.0, 0.0)),
        "J": TeleopCommand("left -Y", translation=(0.0, -1.0, 0.0)),
        "l": TeleopCommand("right +Y", translation=(0.0, 1.0, 0.0)),
        "L": TeleopCommand("right +Y", translation=(0.0, 1.0, 0.0)),
        " ": TeleopCommand("up -X", translation=(-1.0, 0.0, 0.0)),
        "c": TeleopCommand("down +X", translation=(1.0, 0.0, 0.0)),
        "C": TeleopCommand("down +X", translation=(1.0, 0.0, 0.0)),
        "w": TeleopCommand("pitch +", rotation_axis=(0.0, 1.0, 0.0), rotation_sign=1.0),
        "s": TeleopCommand("pitch -", rotation_axis=(0.0, 1.0, 0.0), rotation_sign=-1.0),
        "a": TeleopCommand("yaw +", rotation_axis=(0.0, 0.0, 1.0), rotation_sign=1.0),
        "d": TeleopCommand("yaw -", rotation_axis=(0.0, 0.0, 1.0), rotation_sign=-1.0),
        "q": TeleopCommand("roll -", rotation_axis=(1.0, 0.0, 0.0), rotation_sign=-1.0),
        "e": TeleopCommand("roll +", rotation_axis=(1.0, 0.0, 0.0), rotation_sign=1.0),
    }
    return commands.get(key)


def render_menu(node: KeyboardTeleop) -> None:
    print("\033[2J\033[H", end="")
    print()
    print("EasyArm Keyboard Teleop")
    print(f"Frame: {node.ee_frame} local frame, expressed through {node.base_frame}")
    print(f"Linear step: {node.linear_step:.4f} m")
    print(f"Angular step: {node.angular_step_deg:.2f} deg")
    print(f"Trajectory duration: {node.trajectory_duration:.2f} s")
    print("Target mode: maintained from startup pose")
    print(f"Status: {node.status}")
    print()
    print("Controls:")
    print("  I/K         forward/backward along Link6 Z (-Z/+Z)")
    print("  J/L         left/right along Link6 Y (-Y/+Y)")
    print("  Space/C     up/down along Link6 X")
    print("  W/S         pitch +/-")
    print("  A/D         yaw +/-")
    print("  Q/E         roll +/-")
    print("  Esc         hold current position and exit")
    sys.stdout.flush()


def confirm() -> bool:
    print("This tool will move the real robot through arm_controller.")
    answer = input("Type 'yes' to continue: ")
    return answer == "yes"


def main(args=None) -> int:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        if node.require_confirm and not confirm():
            node.get_logger().warn("Keyboard teleop cancelled by user")
            return 1

        if not node.wait_until_ready():
            return 1
        if not node.initialize_target_pose():
            return 1
        if not node.enter_position_mode():
            return 1

        with RawTerminal() as terminal:
            if not terminal.enabled:
                node.get_logger().error("stdin is not an interactive terminal")
                return 1

            render_menu(node)
            while rclpy.ok():
                key = terminal.read_key()
                if key == "esc":
                    node.status = "Exiting"
                    render_menu(node)
                    node.cancel_active_goal()
                    node.hold_current_position()
                    break

                command = command_from_key(key)
                if command is not None:
                    node.status = f"Moving: {command.name}"
                    render_menu(node)
                    node.execute_command(command)
                    render_menu(node)
                else:
                    time.sleep(0.02)
        return 0
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    sys.exit(main())
