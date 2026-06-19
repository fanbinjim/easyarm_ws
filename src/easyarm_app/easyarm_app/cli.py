import argparse
import atexit
import os
import readline
import select
import shlex
import subprocess
import sys
import termios
import time
import tty

import rclpy
from control_msgs.msg import JointJog
from easyarm_interfaces.action import MoveJ, MoveL
from easyarm_interfaces.srv import GetJoints, GetPose, GetState, SetMode, Stop
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger


class EasyArmCli(Node):
    def __init__(self):
        super().__init__("easyarm_app_cli")
        self.movej_client = ActionClient(self, MoveJ, "/easyarm/movej")
        self.movel_client = ActionClient(self, MoveL, "/easyarm/movel")
        self.set_mode_client = self.create_client(SetMode, "/easyarm/set_mode")
        self.stop_client = self.create_client(Stop, "/easyarm/stop")
        self.get_state_client = self.create_client(GetState, "/easyarm/get_state")
        self.get_joints_client = self.create_client(GetJoints, "/easyarm/get_joints")
        self.get_pose_client = self.create_client(GetPose, "/easyarm/get_pose")
        self.start_servo_client = self.create_client(Trigger, "/servo_node/start_servo")
        self.speedj_pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.speedl_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    def movej(self, args) -> int:
        if not self.movej_client.wait_for_server(timeout_sec=args.timeout):
            self.get_logger().error("/easyarm/movej action server not available")
            return 1

        goal = MoveJ.Goal()
        goal.joints = [float(value) for value in args.joints]
        goal.velocity_scale = float(args.velocity_scale)
        goal.acceleration_scale = float(args.acceleration_scale)
        goal.execute = bool(args.execute)
        return self._send_action_goal(self.movej_client, goal, args.timeout)

    def movel(self, args) -> int:
        if not self.movel_client.wait_for_server(timeout_sec=args.timeout):
            self.get_logger().error("/easyarm/movel action server not available")
            return 1

        goal = MoveL.Goal()
        pose = PoseStamped()
        pose.header.frame_id = args.frame_id
        pose.pose.position.x = float(args.x)
        pose.pose.position.y = float(args.y)
        pose.pose.position.z = float(args.z)
        pose.pose.orientation.x = float(args.qx)
        pose.pose.orientation.y = float(args.qy)
        pose.pose.orientation.z = float(args.qz)
        pose.pose.orientation.w = float(args.qw)
        goal.target_pose = pose
        goal.velocity_scale = float(args.velocity_scale)
        goal.acceleration_scale = float(args.acceleration_scale)
        goal.execute = bool(args.execute)
        return self._send_action_goal(self.movel_client, goal, args.timeout)

    def set_mode(self, args) -> int:
        if not self.set_mode_client.wait_for_service(timeout_sec=args.timeout):
            self.get_logger().error("/easyarm/set_mode service not available")
            return 1
        request = SetMode.Request()
        request.mode = args.mode
        return self._call_service(self.set_mode_client, request, args.timeout)

    def stop(self, args) -> int:
        if not self.stop_client.wait_for_service(timeout_sec=args.timeout):
            self.get_logger().error("/easyarm/stop service not available")
            return 1
        return self._call_service(self.stop_client, Stop.Request(), args.timeout)

    def get_state(self, args) -> int:
        if not self.get_state_client.wait_for_service(timeout_sec=args.timeout):
            self.get_logger().error("/easyarm/get_state service not available")
            return 1
        future = self.get_state_client.call_async(GetState.Request())
        if not _spin_until_complete(self, future, args.timeout):
            self.get_logger().error("Timeout calling /easyarm/get_state")
            return 1
        response = future.result()
        if response is None:
            self.get_logger().error("No response from /easyarm/get_state")
            return 1
        self._log_response(response.success, response.message)
        self._log_info(f"mode: {response.mode}")
        self._log_info(f"busy: {response.busy}")
        self._log_info(f"active_task: {response.active_task}")
        return 0 if response.success else 1

    def get_joints(self, args) -> int:
        if not self.get_joints_client.wait_for_service(timeout_sec=args.timeout):
            self.get_logger().error("/easyarm/get_joints service not available")
            return 1
        future = self.get_joints_client.call_async(GetJoints.Request())
        if not _spin_until_complete(self, future, args.timeout):
            self.get_logger().error("Timeout calling /easyarm/get_joints")
            return 1
        response = future.result()
        if response is None:
            self.get_logger().error("No response from /easyarm/get_joints")
            return 1
        self._log_response(response.success, response.message)
        for index, name in enumerate(response.names):
            position = _value_or_nan(response.positions, index)
            velocity = _value_or_nan(response.velocities, index)
            effort = _value_or_nan(response.efforts, index)
            self._log_info(
                f"{name}: position={position:.6f} velocity={velocity:.6f} effort={effort:.6f}")
        joint_positions = {
            name: _value_or_nan(response.positions, index)
            for index, name in enumerate(response.names)
        }
        ordered_positions = [
            joint_positions.get(f"Joint{index}", float("nan"))
            for index in range(1, 7)
        ]
        self._log_info(
            "cmd: " + " ".join(f"{position:.6f}" for position in ordered_positions))
        return 0 if response.success else 1

    def get_pose(self, args) -> int:
        if not self.get_pose_client.wait_for_service(timeout_sec=args.timeout):
            self.get_logger().error("/easyarm/get_pose service not available")
            return 1
        request = GetPose.Request()
        request.target_frame = args.target_frame
        request.source_frame = args.source_frame
        future = self.get_pose_client.call_async(request)
        if not _spin_until_complete(self, future, args.timeout):
            self.get_logger().error("Timeout calling /easyarm/get_pose")
            return 1
        response = future.result()
        if response is None:
            self.get_logger().error("No response from /easyarm/get_pose")
            return 1
        self._log_response(response.success, response.message)
        pose = response.pose
        self._log_info(f"frame_id: {pose.header.frame_id}")
        self._log_info(
            "position: "
            f"x={pose.pose.position.x:.6f} "
            f"y={pose.pose.position.y:.6f} "
            f"z={pose.pose.position.z:.6f}"
        )
        self._log_info(
            "orientation: "
            f"x={pose.pose.orientation.x:.6f} "
            f"y={pose.pose.orientation.y:.6f} "
            f"z={pose.pose.orientation.z:.6f} "
            f"w={pose.pose.orientation.w:.6f}"
        )
        self._log_info(
            "cmd: "
            f"{pose.pose.position.x:.6f} "
            f"{pose.pose.position.y:.6f} "
            f"{pose.pose.position.z:.6f} "
            f"{pose.pose.orientation.x:.6f} "
            f"{pose.pose.orientation.y:.6f} "
            f"{pose.pose.orientation.z:.6f} "
            f"{pose.pose.orientation.w:.6f}"
        )
        return 0 if response.success else 1

    def speedj(self, args) -> int:
        if not self._require_position_mode(args.timeout):
            return 1
        if not self._start_moveit_servo(args.timeout):
            return 1
        velocities = [float(value) for value in args.velocities]
        if not self._validate_stream_args(args.rate, args.duration):
            return 1
        if not self._wait_for_subscribers(self.speedj_pub, "/servo_node/delta_joint_cmds", args.timeout):
            return 1

        interval = 1.0 / float(args.rate)
        count = self._publish_for_duration(
            interval,
            float(args.duration),
            lambda: self._make_joint_jog(velocities),
            self.speedj_pub.publish,
        )
        self._publish_halt_joint_jog(args.halt_count, interval)
        self._log_info(f"published speedj commands: {count}")
        return 0

    def speedl(self, args) -> int:
        if not self._require_position_mode(args.timeout):
            return 1
        if not self._start_moveit_servo(args.timeout):
            return 1
        if not self._validate_stream_args(args.rate, args.duration):
            return 1
        twist = [
            float(args.vx),
            float(args.vy),
            float(args.vz),
            float(args.wx),
            float(args.wy),
            float(args.wz),
        ]
        if not self._wait_for_subscribers(self.speedl_pub, "/servo_node/delta_twist_cmds", args.timeout):
            return 1

        interval = 1.0 / float(args.rate)
        count = self._publish_for_duration(
            interval,
            float(args.duration),
            lambda: self._make_twist(twist, args.frame_id),
            self.speedl_pub.publish,
        )
        self._publish_halt_twist(args.halt_count, interval, args.frame_id)
        self._log_info(f"published speedl commands: {count}")
        return 0

    def _send_action_goal(self, client, goal, timeout: float) -> int:
        goal_future = client.send_goal_async(goal)
        try:
            if not _spin_until_complete(self, goal_future, timeout):
                self.get_logger().error("Timeout waiting for goal response")
                return 1
        except KeyboardInterrupt:
            self.get_logger().warning("Command interrupted before goal response")
            raise
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return 1

        result_future = goal_handle.get_result_async()
        try:
            if not _spin_until_complete(self, result_future, None):
                self.get_logger().error("Failed while waiting for result")
                return 1
        except KeyboardInterrupt:
            self.get_logger().warning("Command interrupted; canceling action goal")
            cancel_future = goal_handle.cancel_goal_async()
            _spin_until_complete(self, cancel_future, timeout)
            raise
        wrapped_result = result_future.result()
        result = wrapped_result.result
        self._log_response(result.success, result.message)
        return 0 if result.success else 1

    def _call_service(self, client, request, timeout: float) -> int:
        future = client.call_async(request)
        if not _spin_until_complete(self, future, timeout):
            self.get_logger().error("Timeout waiting for service response")
            return 1
        response = future.result()
        if response is None:
            self.get_logger().error("No service response")
            return 1
        self._log_response(response.success, response.message)
        return 0 if response.success else 1

    def _log_response(self, success: bool, message: str) -> None:
        logger = self.get_logger()
        if success:
            logger.info(f"success: {success}")
            logger.info(f"message: {message}")
        else:
            logger.error(f"success: {success}")
            logger.error(f"message: {message}")

    def _log_info(self, message: str) -> None:
        self.get_logger().info(message)

    def _require_position_mode(self, timeout: float) -> bool:
        if not self.get_state_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/easyarm/get_state service not available")
            return False
        future = self.get_state_client.call_async(GetState.Request())
        if not _spin_until_complete(self, future, timeout):
            self.get_logger().error("Timeout calling /easyarm/get_state")
            return False
        response = future.result()
        if response is None:
            self.get_logger().error("No response from /easyarm/get_state")
            return False
        if response.mode != "POSITION":
            self.get_logger().error(
                f"Speed commands require POSITION mode; current mode is {response.mode}")
            return False
        return True

    def _validate_stream_args(self, rate: float, duration: float) -> bool:
        if rate <= 0.0:
            self.get_logger().error("--rate must be greater than 0")
            return False
        if duration < 0.0:
            self.get_logger().error("--duration must be greater than or equal to 0")
            return False
        return True

    def _start_moveit_servo(self, timeout: float) -> bool:
        if not self.start_servo_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/servo_node/start_servo service not available")
            return False
        future = self.start_servo_client.call_async(Trigger.Request())
        if not _spin_until_complete(self, future, timeout):
            self.get_logger().error("Timeout calling /servo_node/start_servo")
            return False
        response = future.result()
        if response is None:
            self.get_logger().error("No response from /servo_node/start_servo")
            return False
        if not response.success:
            self.get_logger().error(f"/servo_node/start_servo failed: {response.message}")
            return False
        if response.message:
            self._log_info(f"MoveIt Servo start: {response.message}")
        return True

    def _wait_for_subscribers(self, publisher, topic_name: str, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().error(f"{topic_name} has no subscribers")
        return False

    def _publish_for_duration(self, interval: float, duration: float, make_message, publish) -> int:
        end_time = time.monotonic() + duration
        next_time = time.monotonic()
        count = 0
        while rclpy.ok() and time.monotonic() < end_time:
            publish(make_message())
            count += 1
            next_time += interval
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_time = time.monotonic()
        return count

    def _make_joint_jog(self, velocities) -> JointJog:
        message = JointJog()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.joint_names = [f"Joint{index}" for index in range(1, 7)]
        message.velocities = list(velocities)
        return message

    def _make_twist(self, values, frame_id: str) -> TwistStamped:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.twist.linear.x = values[0]
        message.twist.linear.y = values[1]
        message.twist.linear.z = values[2]
        message.twist.angular.x = values[3]
        message.twist.angular.y = values[4]
        message.twist.angular.z = values[5]
        return message

    def _publish_halt_joint_jog(self, count: int, interval: float) -> None:
        zeros = [0.0] * 6
        for _ in range(max(0, int(count))):
            self.speedj_pub.publish(self._make_joint_jog(zeros))
            time.sleep(interval)

    def _publish_halt_twist(self, count: int, interval: float, frame_id: str) -> None:
        zeros = [0.0] * 6
        for _ in range(max(0, int(count))):
            self.speedl_pub.publish(self._make_twist(zeros, frame_id))
            time.sleep(interval)

    def run_speedj_teleop(self) -> int:
        if not self._require_position_mode(5.0):
            return 1
        if not self._start_moveit_servo(5.0):
            return 1
        if not self._wait_for_subscribers(self.speedj_pub, "/servo_node/delta_joint_cmds", 5.0):
            return 1

        controller = SpeedJTeleopController(self)
        return controller.run()

    def run_speedl_teleop(self) -> int:
        if not self._require_position_mode(5.0):
            return 1
        if not self._start_moveit_servo(5.0):
            return 1
        if not self._wait_for_subscribers(self.speedl_pub, "/servo_node/delta_twist_cmds", 5.0):
            return 1

        controller = SpeedLTeleopController(self)
        return controller.run()


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None


class SpeedJTeleopController:
    POSITIVE_KEYS = "123456"
    NEGATIVE_KEYS = "qwerty"

    def __init__(self, node: EasyArmCli):
        self.node = node
        self.rate_hz = 50.0
        self.dt = 1.0 / self.rate_hz
        self.max_speed = 10.0
        self.accel = 10.0
        self.decel = 20.0
        self.key_timeout = 0.12
        self.velocities = [0.0] * 6
        self.active_until = [0.0] * 6
        self.target_signs = [0] * 6

    def run(self) -> int:
        self.node.get_logger().info(
            "SpeedJ teleop mode: 1-6 positive, qwerty negative, Esc exits.")
        self.node.get_logger().info(
            f"Hold a key to ramp joint speed up to {self.max_speed:.1f} rad/s; release ramps quickly to zero.")

        try:
            with RawTerminal() as terminal:
                while rclpy.ok():
                    start = time.monotonic()
                    if self._handle_key(terminal.read_key(), start):
                        break
                    self._update_velocities(start)
                    self.node.speedj_pub.publish(self.node._make_joint_jog(self.velocities))
                    sleep_time = self.dt - (time.monotonic() - start)
                    if sleep_time > 0.0:
                        time.sleep(sleep_time)
        except KeyboardInterrupt:
            print()
        finally:
            self._halt()
            print()
        return 0

    def _handle_key(self, key, now: float) -> bool:
        if key is None:
            return False
        if key == "\x1b":
            return True
        if key in self.POSITIVE_KEYS:
            index = self.POSITIVE_KEYS.index(key)
            self.target_signs[index] = 1
            self.active_until[index] = now + self.key_timeout
        elif key in self.NEGATIVE_KEYS:
            index = self.NEGATIVE_KEYS.index(key)
            self.target_signs[index] = -1
            self.active_until[index] = now + self.key_timeout
        return False

    def _update_velocities(self, now: float) -> None:
        for index in range(6):
            if now <= self.active_until[index]:
                target = self.target_signs[index] * self.max_speed
                step = self.accel * self.dt
            else:
                target = 0.0
                self.target_signs[index] = 0
                step = self.decel * self.dt
            self.velocities[index] = _approach(self.velocities[index], target, step)

    def _halt(self) -> None:
        interval = self.dt
        while any(abs(value) > 1e-3 for value in self.velocities):
            self.velocities = [_approach(value, 0.0, self.decel * interval) for value in self.velocities]
            self.node.speedj_pub.publish(self.node._make_joint_jog(self.velocities))
            time.sleep(interval)
        for _ in range(4):
            self.node.speedj_pub.publish(self.node._make_joint_jog([0.0] * 6))
            time.sleep(interval)


class SpeedLTeleopController:
    KEY_BINDINGS = {
        "w": (1, 1),       # y+
        "s": (1, -1),      # y-
        "a": (0, -1),      # x-
        "d": (0, 1),       # x+
        " ": (2, 1),       # z+
        "c": (2, -1),      # z-
        "q": (4, -1),      # -wy
        "e": (4, 1),       # +wy
        "i": (3, -1),      # +x clockwise pitch up.
        "k": (3, 1),       # +x counterclockwise pitch down.
        "j": (5, -1),      # +z clockwise yaw left.
        "l": (5, 1),       # +z counterclockwise yaw right.
    }

    def __init__(self, node: EasyArmCli):
        self.node = node
        self.rate_hz = 50.0
        self.dt = 1.0 / self.rate_hz
        self.max_linear_speed = 0.2
        self.max_angular_speed = 0.3
        self.linear_accel = 0.30
        self.linear_decel = 0.40
        self.angular_accel = 0.80
        self.angular_decel = 1.50
        self.key_timeout = 0.12
        self.twist = [0.0] * 6
        self.active_until = [0.0] * 6
        self.target_signs = [0] * 6

    def run(self) -> int:
        self.node.get_logger().info(
            "SpeedL teleop mode: wasd/space/c translate, q/e/ikjl rotate, Esc exits.")
        self.node.get_logger().info(
            "w y+, s y-, a x-, d x+, space z+, c z-.")
        self.node.get_logger().info(
            "q -wy, e +wy, i/k pitch around x, j/l yaw around z.")

        try:
            with RawTerminal() as terminal:
                while rclpy.ok():
                    start = time.monotonic()
                    if self._handle_key(terminal.read_key(), start):
                        break
                    self._update_twist(start)
                    self.node.speedl_pub.publish(self.node._make_twist(self.twist, "base_link"))
                    sleep_time = self.dt - (time.monotonic() - start)
                    if sleep_time > 0.0:
                        time.sleep(sleep_time)
        except KeyboardInterrupt:
            print()
        finally:
            self._halt()
            print()
        return 0

    def _handle_key(self, key, now: float) -> bool:
        if key is None:
            return False
        if key in ("\x1b", "\x03"):
            return True
        binding = self.KEY_BINDINGS.get(key)
        if binding is None:
            return False
        index, sign = binding
        self.target_signs[index] = sign
        self.active_until[index] = now + self.key_timeout
        return False

    def _update_twist(self, now: float) -> None:
        for index in range(6):
            if index < 3:
                max_speed = self.max_linear_speed
                accel = self.linear_accel
                decel = self.linear_decel
            else:
                max_speed = self.max_angular_speed
                accel = self.angular_accel
                decel = self.angular_decel

            if now <= self.active_until[index]:
                target = self.target_signs[index] * max_speed
                step = accel * self.dt
            else:
                target = 0.0
                self.target_signs[index] = 0
                step = decel * self.dt
            self.twist[index] = _approach(self.twist[index], target, step)

    def _halt(self) -> None:
        interval = self.dt
        while any(abs(value) > 1e-4 for value in self.twist):
            next_twist = []
            for index, value in enumerate(self.twist):
                decel = self.linear_decel if index < 3 else self.angular_decel
                next_twist.append(_approach(value, 0.0, decel * interval))
            self.twist = next_twist
            self.node.speedl_pub.publish(self.node._make_twist(self.twist, "base_link"))
            time.sleep(interval)
        for _ in range(4):
            self.node.speedl_pub.publish(self.node._make_twist([0.0] * 6, "base_link"))
            time.sleep(interval)


def _spin_until_complete(node: Node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.done()


def _value_or_nan(values, index: int) -> float:
    if index >= len(values):
        return float("nan")
    return float(values[index])


def _approach(value: float, target: float, step: float) -> float:
    if value < target:
        return min(value + step, target)
    if value > target:
        return max(value - step, target)
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="EasyArm app CLI",
        epilog=(
            "examples:\n"
            "  movej 0.0025 0.25 2 0.1 -1.57 0.0\n"
            "  speedj_teleop    # in easyarm_shell: keyboard JointJog mode\n"
            "  movel 0.25 0.0 0.25 0.0 0.0 0.0 1.0\n"
            "  speedj 0 0.05 0 0 0 0 --duration 1.0 --rate 50\n"
            "  speedl 0.01 0 0 0 0 0 --duration 1.0 --rate 50\n"
            "  set-mode DRAG\n"
            "  set-mode POSITION\n"
            "  speedl_teleop    # in easyarm_shell: keyboard Cartesian teleop mode"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    subparsers = parser.add_subparsers(dest="command", required=True)

    movej = subparsers.add_parser(
        "movej",
        help="Call /easyarm/movej",
        epilog=(
            "example:\n"
            "  movej 0.0025 0.25 2 0.1 -1.57 0.0"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    movej.add_argument("joints", nargs=6, type=float, metavar="J")
    movej.add_argument("--velocity-scale", type=float, default=0.2)
    movej.add_argument("--acceleration-scale", type=float, default=0.2)
    movej.add_argument("--plan-only", dest="execute", action="store_false")
    movej.set_defaults(execute=True)

    movel = subparsers.add_parser(
        "movel",
        help="Call /easyarm/movel",
        epilog="example:\n  movel 0.25 0.0 0.25 0.0 0.0 0.0 1.0",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    movel.add_argument("x", type=float)
    movel.add_argument("y", type=float)
    movel.add_argument("z", type=float)
    movel.add_argument("qx", type=float)
    movel.add_argument("qy", type=float)
    movel.add_argument("qz", type=float)
    movel.add_argument("qw", type=float)
    movel.add_argument("--frame-id", default="base_link")
    movel.add_argument("--velocity-scale", type=float, default=0.1)
    movel.add_argument("--acceleration-scale", type=float, default=0.1)
    movel.add_argument("--plan-only", dest="execute", action="store_false")
    movel.set_defaults(execute=True)

    set_mode = subparsers.add_parser(
        "set-mode",
        help="Call /easyarm/set_mode",
        epilog="examples:\n  set-mode DRAG\n  set-mode POSITION",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    set_mode.add_argument("mode", choices=["POSITION", "IDLE", "DRAG", "position", "idle", "drag"])

    subparsers.add_parser("stop", help="Call /easyarm/stop")
    subparsers.add_parser("get-state", help="Call /easyarm/get_state")
    subparsers.add_parser("get-joints", help="Call /easyarm/get_joints")

    get_pose = subparsers.add_parser("get-pose", help="Call /easyarm/get_pose")
    get_pose.add_argument("--target-frame", default="base_link")
    get_pose.add_argument("--source-frame", default="Link6")

    speedj = subparsers.add_parser(
        "speedj",
        help="Publish MoveIt Servo JointJog commands",
        epilog="example:\n  speedj 0 0.05 0 0 0 0 --duration 1.0 --rate 50",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    speedj.add_argument("velocities", nargs=6, type=float, metavar="V")
    speedj.add_argument("--duration", type=float, default=1.0)
    speedj.add_argument("--rate", type=float, default=50.0)
    speedj.add_argument("--halt-count", type=int, default=4)

    speedl = subparsers.add_parser(
        "speedl",
        help="Publish MoveIt Servo TwistStamped commands",
        epilog="example:\n  speedl 0.01 0 0 0 0 0 --duration 1.0 --rate 50",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    speedl.add_argument("vx", type=float)
    speedl.add_argument("vy", type=float)
    speedl.add_argument("vz", type=float)
    speedl.add_argument("wx", type=float)
    speedl.add_argument("wy", type=float)
    speedl.add_argument("wz", type=float)
    speedl.add_argument("--frame-id", default="base_link")
    speedl.add_argument("--duration", type=float, default=1.0)
    speedl.add_argument("--rate", type=float, default=50.0)
    speedl.add_argument("--halt-count", type=int, default=4)

    subparsers.add_parser(
        "speedj_teleop",
        help="Run keyboard SpeedJ teleoperation in easyarm_shell",
    )

    subparsers.add_parser(
        "speedl_teleop",
        help="Run keyboard SpeedL teleoperation in easyarm_shell",
    )

    safe_shutdown = subparsers.add_parser("safe_shutdown", help="Run safe shutdown and exit shell")
    safe_shutdown.add_argument("args", nargs=argparse.REMAINDER, help="Arguments passed to safe_shutdown.sh")

    return parser


def run_command(node: EasyArmCli, args) -> int:
    if args.command == "safe_shutdown":
        node.get_logger().error("safe_shutdown is only supported by easyarm_shell")
        return 1
    if args.command == "movej":
        return node.movej(args)
    if args.command == "movel":
        return node.movel(args)
    if args.command == "set-mode":
        return node.set_mode(args)
    if args.command == "stop":
        return node.stop(args)
    if args.command == "get-state":
        return node.get_state(args)
    if args.command == "get-joints":
        return node.get_joints(args)
    if args.command == "get-pose":
        return node.get_pose(args)
    if args.command == "speedj":
        return node.speedj(args)
    if args.command == "speedl":
        return node.speedl(args)
    if args.command == "speedj_teleop":
        node.get_logger().error("speedj_teleop is only supported by easyarm_shell")
        return 1
    if args.command == "speedl_teleop":
        node.get_logger().error("speedl_teleop is only supported by easyarm_shell")
        return 1
    raise RuntimeError(f"unknown command {args.command}")


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    rclpy.init()
    node = EasyArmCli()
    try:
        return run_command(node, args)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def console_main():
    sys.exit(main())


def configure_readline_history(node: EasyArmCli) -> None:
    executable_dir = os.path.dirname(os.path.realpath(sys.argv[0]))
    history_path = os.path.join(executable_dir, ".easyarm_shell_history")
    try:
        if os.path.exists(history_path):
            readline.read_history_file(history_path)
        readline.set_history_length(1000)
        atexit.register(readline.write_history_file, history_path)
    except OSError as exception:
        node.get_logger().warning(f"Command history disabled: {exception}")


def run_safe_shutdown_command(node: EasyArmCli, extra_args) -> int:
    command = ["ros2", "run", "easyarm_a1_bringup", "safe_shutdown.sh", *extra_args]
    node.get_logger().info("Running safe shutdown")
    node.get_logger().info("cmd: " + " ".join(shlex.quote(value) for value in command))
    try:
        return subprocess.call(command)
    except FileNotFoundError:
        node.get_logger().error("ros2 command not found")
        return 1


def shell_main() -> None:
    parser = build_parser()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = EasyArmCli()
    configure_readline_history(node)
    node.get_logger().info("EasyArm shell ready. Type 'help' for commands, 'exit' to quit.")
    try:
        while rclpy.ok():
            try:
                line = input("easyarm> ").strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                print()
                continue

            if not line:
                continue
            if line in ("exit", "quit"):
                break
            if line in ("help", "?"):
                parser.print_help()
                continue
            if line == "speedj_teleop":
                node.run_speedj_teleop()
                continue
            if line == "speedl_teleop":
                node.run_speedl_teleop()
                continue
            if line == "safe_shutdown" or line.startswith("safe_shutdown "):
                try:
                    extra_args = shlex.split(line)[1:]
                except ValueError as exception:
                    node.get_logger().error(str(exception))
                    continue

                return_code = run_safe_shutdown_command(node, extra_args)
                if return_code == 0:
                    node.get_logger().info("Safe shutdown completed. Exiting shell.")
                    break
                node.get_logger().error(f"Safe shutdown failed with exit code {return_code}")
                continue

            try:
                args = parser.parse_args(shlex.split(line))
            except SystemExit:
                continue

            try:
                run_command(node, args)
            except KeyboardInterrupt:
                print()
                if rclpy.ok():
                    node.get_logger().warning("Command interrupted")
            except Exception as exception:  # noqa: BLE001 - keep shell alive after command errors.
                node.get_logger().error(str(exception))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
