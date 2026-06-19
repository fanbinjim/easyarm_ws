import time

import rclpy
from control_msgs.msg import JointJog
from easyarm_interfaces.action import MoveJ, MoveL
from easyarm_interfaces.srv import GetJoints, GetPose, GetState, SetMode, Stop
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from .teleop import SpeedJTeleopController, SpeedLTeleopController
from .utils import _spin_until_complete, _value_or_nan


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
