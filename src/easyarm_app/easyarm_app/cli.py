import argparse
import atexit
import os
import readline
import shlex
import subprocess
import sys

import rclpy
from easyarm_interfaces.action import MoveJ, MoveL
from easyarm_interfaces.srv import GetJoints, GetPose, GetState, SetMode, Stop
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


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


def _spin_until_complete(node: Node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.done()


def _value_or_nan(values, index: int) -> float:
    if index >= len(values):
        return float("nan")
    return float(values[index])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="EasyArm app CLI",
        epilog=(
            "examples:\n"
            "  movej 0.0025 0.25 2 0.1 -1.57 0.0\n"
            "  movel 0.25 0.0 0.25 0.0 0.0 0.0 1.0\n"
            "  set-mode DRAG\n"
            "  set-mode POSITION"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    subparsers = parser.add_subparsers(dest="command", required=True)

    movej = subparsers.add_parser(
        "movej",
        help="Call /easyarm/movej",
        epilog="example:\n  movej 0.0025 0.25 2 0.1 -1.57 0.0",
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
