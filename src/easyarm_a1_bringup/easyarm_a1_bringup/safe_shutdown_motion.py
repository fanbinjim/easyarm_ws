import argparse
import sys

import rclpy
from easyarm_interfaces.action import MoveJ
from easyarm_interfaces.srv import SetMode, Stop
from rclpy.action import ActionClient
from rclpy.node import Node


class SafeShutdownMotion(Node):
    def __init__(self):
        super().__init__("easyarm_safe_shutdown_motion")
        self.stop_client = self.create_client(Stop, "/easyarm/stop")
        self.set_mode_client = self.create_client(SetMode, "/easyarm/set_mode")
        self.movej_client = ActionClient(self, MoveJ, "/easyarm/movej")

    def run(self, args) -> int:
        if not args.skip_stop and not self.call_stop(args.timeout):
            return 1
        if not args.skip_set_position and not self.call_set_position(args.timeout):
            return 1
        if not args.skip_move_ready and not self.call_move_ready(args):
            return 1
        return 0

    def call_stop(self, timeout: float) -> bool:
        self.get_logger().info("Calling /easyarm/stop")
        if not self.stop_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/easyarm/stop service not available")
            return False

        response = self.call_service(self.stop_client, Stop.Request(), timeout)
        return self.log_response(response)

    def call_set_position(self, timeout: float) -> bool:
        self.get_logger().info("Calling /easyarm/set_mode POSITION")
        if not self.set_mode_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/easyarm/set_mode service not available")
            return False

        request = SetMode.Request()
        request.mode = "POSITION"
        response = self.call_service(self.set_mode_client, request, timeout)
        return self.log_response(response)

    def call_move_ready(self, args) -> bool:
        if len(args.ready_joints) != 6:
            self.get_logger().error(
                f"--ready-joints must contain 6 joint values, got {len(args.ready_joints)}")
            return False

        self.get_logger().info(
            "Calling /easyarm/movej ready: "
            + " ".join(f"{joint:.5f}" for joint in args.ready_joints)
        )
        if not self.movej_client.wait_for_server(timeout_sec=args.timeout):
            self.get_logger().error("/easyarm/movej action server not available")
            return False

        goal = MoveJ.Goal()
        goal.joints = [float(value) for value in args.ready_joints]
        goal.velocity_scale = float(args.velocity_scale)
        goal.acceleration_scale = float(args.acceleration_scale)
        goal.execute = True

        goal_future = self.movej_client.send_goal_async(goal)
        if not self.spin_until_complete(goal_future, args.timeout):
            self.get_logger().error("Timeout waiting for MoveJ goal response")
            return False

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveJ goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        if not self.spin_until_complete(result_future, None):
            self.get_logger().error("Failed while waiting for MoveJ result")
            return False

        wrapped_result = result_future.result()
        return self.log_response(wrapped_result.result)

    def call_service(self, client, request, timeout: float):
        future = client.call_async(request)
        if not self.spin_until_complete(future, timeout):
            self.get_logger().error("Timeout waiting for service response")
            return None
        return future.result()

    def spin_until_complete(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.done()

    def log_response(self, response) -> bool:
        if response is None:
            self.get_logger().error("No response received")
            return False

        if response.success:
            self.get_logger().info(f"success: {response.success}")
            self.get_logger().info(f"message: {response.message}")
        else:
            self.get_logger().error(f"success: {response.success}")
            self.get_logger().error(f"message: {response.message}")
        return bool(response.success)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the motion part of EasyArm safe shutdown with one ROS node.")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--skip-stop", action="store_true")
    parser.add_argument("--skip-set-position", action="store_true")
    parser.add_argument("--skip-move-ready", action="store_true")
    parser.add_argument(
        "--ready-joints",
        nargs=6,
        type=float,
        default=[0.0, 1.85005, 2.68781, 0.9599, 1.57, 0.0],
        metavar="J",
    )
    parser.add_argument("--velocity-scale", type=float, default=0.2)
    parser.add_argument("--acceleration-scale", type=float, default=0.2)
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    rclpy.init()
    node = SafeShutdownMotion()
    try:
        return node.run(args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def console_main():
    sys.exit(main())
