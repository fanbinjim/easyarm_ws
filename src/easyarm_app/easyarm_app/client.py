import time

import rclpy
from control_msgs.msg import JointJog
from easyarm_interfaces.action import MoveJ, MoveL, MoveNamedState
from easyarm_interfaces.srv import GetJoints, GetPose, GetState, ListNamedState, SetMode, Stop
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .teleop import ServoJTeleopController, ServoLTeleopController, SpeedJTeleopController, SpeedLTeleopController
from .utils import _spin_until_complete, _value_or_nan


class EasyArmCli(Node):
    def __init__(self):
        super().__init__("easyarm_app_cli")
        self.movej_client = ActionClient(self, MoveJ, "/easyarm/movej")
        self.movel_client = ActionClient(self, MoveL, "/easyarm/movel")
        self.move_named_state_client = ActionClient(self, MoveNamedState, "/easyarm/move_named_state")
        self.set_mode_client = self.create_client(SetMode, "/easyarm/set_mode")
        self.stop_client = self.create_client(Stop, "/easyarm/stop")
        self.get_state_client = self.create_client(GetState, "/easyarm/get_state")
        self.get_joints_client = self.create_client(GetJoints, "/easyarm/get_joints")
        self.get_pose_client = self.create_client(GetPose, "/easyarm/get_pose")
        self.list_named_state_client = self.create_client(ListNamedState, "/easyarm/list_named_state")
        self.speedj_pub = self.create_publisher(JointJog, "/easyarm/speedj_cmd", 10)
        self.speedl_pub = self.create_publisher(TwistStamped, "/easyarm/speedl_cmd", 10)
        self.servoj_pub = self.create_publisher(JointTrajectory, "/easyarm/servoj_cmd", 10)
        self.servol_pub = self.create_publisher(PoseStamped, "/easyarm/servol_cmd", 10)

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

    def move_named_state(self, args) -> int:
        if not self.move_named_state_client.wait_for_server(timeout_sec=args.timeout):
            self.get_logger().error("/easyarm/move_named_state action server not available")
            return 1

        goal = MoveNamedState.Goal()
        goal.name = args.name
        goal.velocity_scale = float(args.velocity_scale)
        goal.acceleration_scale = float(args.acceleration_scale)
        goal.execute = bool(args.execute)
        return self._send_action_goal(self.move_named_state_client, goal, args.timeout)

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

    def list_named_state(self, args) -> int:
        if not self.list_named_state_client.wait_for_service(timeout_sec=args.timeout):
            self.get_logger().error("/easyarm/list_named_state service not available")
            return 1
        future = self.list_named_state_client.call_async(ListNamedState.Request())
        if not _spin_until_complete(self, future, args.timeout):
            self.get_logger().error("Timeout calling /easyarm/list_named_state")
            return 1
        response = future.result()
        if response is None:
            self.get_logger().error("No response from /easyarm/list_named_state")
            return 1
        self._log_response(response.success, response.message)
        if response.names:
            self._log_info("names: " + " ".join(response.names))
            width = len(response.joint_names)
            for index, name in enumerate(response.names):
                start = index * width
                values = list(response.positions[start:start + width])
                if len(values) != width:
                    continue
                self._log_info(
                    f"{name}: " + " ".join(
                        f"{joint}={value:.6f}" for joint, value in zip(response.joint_names, values)
                    )
                )
                self._log_info(
                    f"{name} cmd: " + " ".join(f"{value:.6f}" for value in values)
                )
        else:
            self._log_info("names:")
        return 0 if response.success else 1

    def speedj(self, args) -> int:
        if not self._require_position_mode(args.timeout):
            return 1
        velocities = [float(value) for value in args.velocities]
        if not self._validate_stream_args(args.rate, args.duration):
            return 1
        if not self._wait_for_subscribers(self.speedj_pub, "/easyarm/speedj_cmd", args.timeout):
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
        if not self._wait_for_subscribers(self.speedl_pub, "/easyarm/speedl_cmd", args.timeout):
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

    def servoj(self, args) -> int:
        if not self._require_position_mode(args.timeout):
            return 1
        joints = [float(value) for value in args.joints]
        if not self._validate_stream_args(args.rate, args.duration):
            return 1
        if not self._wait_for_subscribers(self.servoj_pub, "/easyarm/servoj_cmd", args.timeout):
            return 1

        interval = 1.0 / float(args.rate)
        count = self._publish_for_duration(
            interval,
            float(args.duration),
            lambda: self._make_servoj_target(joints),
            self.servoj_pub.publish,
        )
        self._log_info(f"published servoj targets: {count}")
        return 0

    def servol(self, args) -> int:
        if not self._require_position_mode(args.timeout):
            return 1
        if not self._validate_stream_args(args.rate, args.duration):
            return 1
        if not self._wait_for_subscribers(self.servol_pub, "/easyarm/servol_cmd", args.timeout):
            return 1

        interval = 1.0 / float(args.rate)
        count = self._publish_for_duration(
            interval,
            float(args.duration),
            lambda: self._make_servol_target(args),
            self.servol_pub.publish,
        )
        self._log_info(f"published servol targets: {count}")
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

    def _make_servoj_target(self, joints) -> JointTrajectory:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.joint_names = [f"Joint{index}" for index in range(1, 7)]
        point = JointTrajectoryPoint()
        point.positions = list(joints)
        message.points = [point]
        return message

    def _make_servol_target(self, args) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = args.frame_id
        message.pose.position.x = float(args.x)
        message.pose.position.y = float(args.y)
        message.pose.position.z = float(args.z)
        message.pose.orientation.x = float(args.qx)
        message.pose.orientation.y = float(args.qy)
        message.pose.orientation.z = float(args.qz)
        message.pose.orientation.w = float(args.qw)
        return message

    def _make_servol_pose(self, target_pose, frame_id: str) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.pose.position.x = float(target_pose["position"][0])
        message.pose.position.y = float(target_pose["position"][1])
        message.pose.position.z = float(target_pose["position"][2])
        message.pose.orientation.x = float(target_pose["orientation"][0])
        message.pose.orientation.y = float(target_pose["orientation"][1])
        message.pose.orientation.z = float(target_pose["orientation"][2])
        message.pose.orientation.w = float(target_pose["orientation"][3])
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
        if not self._wait_for_subscribers(self.speedj_pub, "/easyarm/speedj_cmd", 5.0):
            return 1

        controller = SpeedJTeleopController(self)
        return controller.run()

    def run_speedl_teleop(self) -> int:
        if not self._require_position_mode(5.0):
            return 1
        if not self._wait_for_subscribers(self.speedl_pub, "/easyarm/speedl_cmd", 5.0):
            return 1

        controller = SpeedLTeleopController(self)
        return controller.run()

    def run_servoj_teleop(self) -> int:
        if not self._require_position_mode(5.0):
            return 1
        if not self._wait_for_subscribers(self.servoj_pub, "/easyarm/servoj_cmd", 5.0):
            return 1

        controller = ServoJTeleopController(self)
        return controller.run()

    def run_servol_teleop(self) -> int:
        if not self._require_position_mode(5.0):
            return 1
        if not self._wait_for_subscribers(self.servol_pub, "/easyarm/servol_cmd", 5.0):
            return 1

        controller = ServoLTeleopController(self)
        return controller.run()

    def get_current_joint_positions(self, timeout: float):
        if not self.get_joints_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/easyarm/get_joints service not available")
            return None
        future = self.get_joints_client.call_async(GetJoints.Request())
        if not _spin_until_complete(self, future, timeout):
            self.get_logger().error("Timeout calling /easyarm/get_joints")
            return None
        response = future.result()
        if response is None or not response.success:
            message = "" if response is None else response.message
            self.get_logger().error(f"Failed to get current joints: {message}")
            return None
        joint_positions = {
            name: _value_or_nan(response.positions, index)
            for index, name in enumerate(response.names)
        }
        return [
            joint_positions.get(f"Joint{index}", float("nan"))
            for index in range(1, 7)
        ]

    def get_current_pose(self, timeout: float):
        if not self.get_pose_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/easyarm/get_pose service not available")
            return None
        request = GetPose.Request()
        request.target_frame = "base_link"
        request.source_frame = "Link6"
        future = self.get_pose_client.call_async(request)
        if not _spin_until_complete(self, future, timeout):
            self.get_logger().error("Timeout calling /easyarm/get_pose")
            return None
        response = future.result()
        if response is None or not response.success:
            message = "" if response is None else response.message
            self.get_logger().error(f"Failed to get current pose: {message}")
            return None
        return response.pose
