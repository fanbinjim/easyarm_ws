# Copyright 2026 linx
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Motion helpers for the EasyArm ball balance task."""

from dataclasses import dataclass
import math
import threading
import time

from easyarm_interfaces.action import MoveL
from easyarm_interfaces.action import MoveNamedState
from easyarm_interfaces.srv import GetPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass(frozen=True)
class MotionConfig:
    """Configuration for ball-balance motion commands."""

    named_state: str
    frame_id: str
    source_frame: str
    angle_limit_deg: float
    angle_gain_deg: float
    angle_derivative_gain_deg: float
    camera_to_plate_yaw_deg: float
    filter_alpha: float
    max_measurement_age_sec: float
    velocity_scale: float
    acceleration_scale: float
    servol_rate_hz: float


class BallBalanceMotionController:
    """Send pose1, one-shot MoveL, and continuous ServoL commands."""

    def __init__(self, node: Node, config: MotionConfig) -> None:
        """Create action and service clients."""
        self.node = node
        self.config = config
        self.move_named_state_client = ActionClient(
            node,
            MoveNamedState,
            "/easyarm/move_named_state",
        )
        self.movel_client = ActionClient(node, MoveL, "/easyarm/movel")
        self.get_pose_client = node.create_client(GetPose, "/easyarm/get_pose")
        self.servol_pub = node.create_publisher(
            PoseStamped,
            "/easyarm/servol_cmd",
            10,
        )
        self.origin_pose = None
        self.busy = False
        self.active_command = ""
        self.servol_active = False
        self.servol_return_samples = 0
        self.state_lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.filtered_offset: tuple[float, float] | None = None
        self.filtered_velocity: tuple[float, float] = (0.0, 0.0)
        self.last_measurement_time: float | None = None
        self.status = "Space: pose1, B: MoveL step, O/P: ServoL run/stop"
        self.last_tilt_x_deg = 0.0
        self.last_tilt_y_deg = 0.0
        self.last_command_sent = False
        self.last_control_source = ""
        self.last_measurement_stale = False
        self.last_target_pose = None
        self.servol_thread = threading.Thread(
            target=self._servol_loop,
            name="easyarm_ball_balance_servol",
            daemon=True,
        )
        self.servol_thread.start()

    def status_lines(self) -> list[str]:
        """Return short status lines for the camera preview."""
        with self.state_lock:
            origin = "set" if self.origin_pose is not None else "not set"
            servol_active = self.servol_active
            last_command_sent = self.last_command_sent
            tilt_x_deg = self.last_tilt_x_deg
            tilt_y_deg = self.last_tilt_y_deg
            filtered_offset = self.filtered_offset
            filtered_velocity = self.filtered_velocity
            last_measurement_time = self.last_measurement_time
        lines = [
            f"origin: {origin}",
            f"motion: {self.status}",
        ]
        if last_command_sent:
            lines.append(
                "tilt cmd: "
                f"x={tilt_x_deg:+.2f}deg "
                f"y={tilt_y_deg:+.2f}deg"
            )
        if filtered_offset is not None and last_measurement_time is not None:
            age = max(0.0, time.monotonic() - last_measurement_time)
            lines.append(
                "pd state: "
                f"x={filtered_offset[0]:+.3f} "
                f"y={filtered_offset[1]:+.3f} "
                f"vx={filtered_velocity[0]:+.2f} "
                f"vy={filtered_velocity[1]:+.2f}"
            )
            lines.append(f"measurement age: {age:.2f}s")
        state = "active" if servol_active else "stopped"
        lines.append(f"servol: {state} {self.config.servol_rate_hz:.0f}Hz")
        return lines

    def snapshot(self) -> dict:
        """Return a thread-safe motion state snapshot for logging."""
        with self.state_lock:
            measurement_age = None
            if self.last_measurement_time is not None:
                measurement_age = max(
                    0.0,
                    time.monotonic() - self.last_measurement_time,
                )
            return {
                "origin_pose": self.origin_pose,
                "busy": self.busy,
                "active_command": self.active_command,
                "servol_active": self.servol_active,
                "status": self.status,
                "filtered_offset": self.filtered_offset,
                "filtered_velocity": self.filtered_velocity,
                "measurement_age_sec": measurement_age,
                "last_tilt_x_deg": self.last_tilt_x_deg,
                "last_tilt_y_deg": self.last_tilt_y_deg,
                "last_command_sent": self.last_command_sent,
                "last_control_source": self.last_control_source,
                "last_measurement_stale": self.last_measurement_stale,
                "last_target_pose": self.last_target_pose,
            }

    def shutdown(self) -> None:
        """Stop the ServoL worker thread."""
        with self.state_lock:
            self.servol_active = False
            self.servol_return_samples = 0
        self.shutdown_event.set()
        if self.servol_thread.is_alive():
            self.servol_thread.join(timeout=1.0)

    def move_to_named_state(self) -> None:
        """Send the configured named-state action."""
        self._disable_servol()
        if self.busy:
            self._set_status(f"busy: {self.active_command}")
            return
        if not self.move_named_state_client.server_is_ready():
            self._set_status("/easyarm/move_named_state not ready")
            return

        goal = MoveNamedState.Goal()
        goal.name = self.config.named_state
        goal.velocity_scale = self.config.velocity_scale
        goal.acceleration_scale = self.config.acceleration_scale
        goal.execute = True
        self.busy = True
        self.active_command = "MoveNamedState"
        self._set_status(f"sending {self.config.named_state}")
        future = self.move_named_state_client.send_goal_async(
            goal,
            feedback_callback=self._on_action_feedback,
        )
        future.add_done_callback(self._on_named_state_goal_response)

    def send_balance_step(self, offset: tuple[float, float] | None) -> None:
        """Send one MoveL command using the latest normalized ball offset."""
        if self._is_servol_active():
            self._set_status("stop ServoL before one-shot MoveL")
            return
        if self.busy:
            self._set_status(f"busy: {self.active_command}")
            return
        with self.state_lock:
            origin_pose = self.origin_pose
        if origin_pose is None:
            self._set_status("press Space to set pose1 origin first")
            return
        if offset is None:
            self._set_status("no ball offset for MoveL")
            return
        if not self.movel_client.server_is_ready():
            self._set_status("/easyarm/movel not ready")
            return

        compensated_offset = compensate_offset(
            offset,
            self.config.camera_to_plate_yaw_deg,
        )
        tilt_x_deg, tilt_y_deg = balance_tilt_from_offset(
            compensated_offset,
            self.config,
        )
        with self.state_lock:
            origin_pose = self.origin_pose
        if origin_pose is None:
            self._set_status("origin pose lost")
            return
        target_pose = make_balance_target_pose(
            origin_pose,
            self.config.frame_id,
            tilt_x_deg,
            tilt_y_deg,
        )
        goal = MoveL.Goal()
        goal.target_pose = target_pose
        goal.velocity_scale = self.config.velocity_scale
        goal.acceleration_scale = self.config.acceleration_scale
        goal.execute = True

        with self.state_lock:
            self.last_tilt_x_deg = tilt_x_deg
            self.last_tilt_y_deg = tilt_y_deg
            self.last_command_sent = True
            self.last_control_source = "movel_step"
            self.last_measurement_stale = False
            self.last_target_pose = target_pose
        self.busy = True
        self.active_command = "MoveL"
        self._set_status(
            f"sending MoveL x={tilt_x_deg:+.2f}deg y={tilt_y_deg:+.2f}deg"
        )
        future = self.movel_client.send_goal_async(
            goal,
            feedback_callback=self._on_action_feedback,
        )
        future.add_done_callback(self._on_movel_goal_response)

    def start_servol(self) -> None:
        """Start continuous ServoL balance control."""
        if self.busy:
            self._set_status(f"busy: {self.active_command}")
            return
        with self.state_lock:
            origin_pose = self.origin_pose
        if origin_pose is None:
            self._set_status("press Space to set pose1 origin first")
            return
        with self.state_lock:
            has_measurement = self.filtered_offset is not None
        if not has_measurement:
            self._set_status("no visual offset for ServoL")
            return
        if self.servol_pub.get_subscription_count() <= 0:
            self._set_status("/easyarm/servol_cmd has no subscriber")
            return
        with self.state_lock:
            self.servol_active = True
            self.servol_return_samples = 0
        self._set_status("ServoL balance running")

    def stop_servol(self) -> None:
        """Stop ServoL and publish origin orientation briefly."""
        with self.state_lock:
            origin_pose = self.origin_pose
        if origin_pose is None:
            with self.state_lock:
                self.servol_active = False
            self._set_status("ServoL stopped")
            return
        return_samples = max(1, int(round(self.config.servol_rate_hz * 0.1)))
        with self.state_lock:
            self.servol_active = False
            self.servol_return_samples = return_samples
            self.last_tilt_x_deg = 0.0
            self.last_tilt_y_deg = 0.0
            self.last_command_sent = True
            self.last_control_source = "servol_stop"
        self._set_status("ServoL stopping, returning origin attitude")

    def compensated_offset(
        self,
        offset: tuple[float, float],
    ) -> tuple[float, float]:
        """Rotate an image-space offset into the plate/TCP frame."""
        return compensate_offset(offset, self.config.camera_to_plate_yaw_deg)

    def update_measurement(self, offset: tuple[float, float] | None) -> None:
        """Low-pass filter ball offset and estimate normalized velocity."""
        if offset is None:
            return
        offset_x, offset_y = self.compensated_offset(offset)
        now = time.monotonic()
        alpha = clamp(self.config.filter_alpha, 0.0, 1.0)
        with self.state_lock:
            if (
                self.filtered_offset is None
                or self.last_measurement_time is None
            ):
                self.filtered_offset = (offset_x, offset_y)
                self.filtered_velocity = (0.0, 0.0)
                self.last_measurement_time = now
                return

            dt = now - self.last_measurement_time
            if dt <= 1.0e-4 or dt > 0.5:
                self.filtered_offset = (offset_x, offset_y)
                self.filtered_velocity = (0.0, 0.0)
                self.last_measurement_time = now
                return

            previous_offset = self.filtered_offset
            filtered_offset = (
                previous_offset[0] + alpha * (offset_x - previous_offset[0]),
                previous_offset[1] + alpha * (offset_y - previous_offset[1]),
            )
            measured_velocity = (
                (filtered_offset[0] - previous_offset[0]) / dt,
                (filtered_offset[1] - previous_offset[1]) / dt,
            )
            previous_velocity = self.filtered_velocity
            self.filtered_velocity = (
                previous_velocity[0]
                + alpha * (measured_velocity[0] - previous_velocity[0]),
                previous_velocity[1]
                + alpha * (measured_velocity[1] - previous_velocity[1]),
            )
            self.filtered_offset = filtered_offset
            self.last_measurement_time = now

    def request_origin_pose(self) -> None:
        """Read the current TCP pose and store it as the balance origin."""
        if not self.get_pose_client.service_is_ready():
            self._set_status("/easyarm/get_pose not ready")
            return

        request = GetPose.Request()
        request.target_frame = self.config.frame_id
        request.source_frame = self.config.source_frame
        self.busy = True
        self.active_command = "GetPose"
        self._disable_servol()
        self._set_status("reading origin pose")
        future = self.get_pose_client.call_async(request)
        future.add_done_callback(self._on_origin_pose_response)

    def _on_named_state_goal_response(self, future) -> None:
        """Handle MoveNamedState goal acceptance."""
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish_with_error(f"pose1 goal error: {exc}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self._finish_with_error("pose1 goal rejected")
            return
        self._set_status("pose1 executing")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_named_state_result)

    def _on_named_state_result(self, future) -> None:
        """Handle MoveNamedState completion."""
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self._finish_with_error(f"pose1 result error: {exc}")
            return
        if not result.success:
            self._finish_with_error(f"pose1 failed: {result.message}")
            return
        self.busy = False
        self.active_command = ""
        self._set_status("pose1 reached")
        self.request_origin_pose()

    def _on_movel_goal_response(self, future) -> None:
        """Handle MoveL goal acceptance."""
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish_with_error(f"MoveL goal error: {exc}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self._finish_with_error("MoveL goal rejected")
            return
        self._set_status("MoveL executing")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_movel_result)

    def _on_movel_result(self, future) -> None:
        """Handle MoveL completion."""
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self._finish_with_error(f"MoveL result error: {exc}")
            return
        if result.success:
            self._finish_ok(f"MoveL done: {result.message}")
        else:
            self._finish_with_error(f"MoveL failed: {result.message}")

    def _on_origin_pose_response(self, future) -> None:
        """Handle current-pose service response."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish_with_error(f"get pose error: {exc}")
            return
        if response is None or not response.success:
            message = "" if response is None else response.message
            self._finish_with_error(f"get pose failed: {message}")
            return
        with self.state_lock:
            self.origin_pose = response.pose
        pose = response.pose.pose
        position = pose.position
        self._finish_ok(
            "origin set: "
            f"{position.x:.3f} {position.y:.3f} {position.z:.3f}"
        )

    def _on_action_feedback(self, feedback) -> None:
        """Display compact action feedback in the preview."""
        state = getattr(feedback.feedback, "state", "")
        if state:
            self._set_status(f"{self.active_command}: {state}")

    def _finish_ok(self, status: str) -> None:
        """Mark the active motion command as finished successfully."""
        self.busy = False
        self.active_command = ""
        self._set_status(status)

    def _finish_with_error(self, status: str) -> None:
        """Mark the active motion command as failed."""
        self.busy = False
        self.active_command = ""
        self._set_status(status, error=True)

    def _set_status(self, status: str, error: bool = False) -> None:
        """Update UI status and ROS logs."""
        self.status = status
        if error:
            self.node.get_logger().error(status)
        else:
            self.node.get_logger().info(status)

    def _servol_loop(self) -> None:
        """Publish ServoL targets at the configured fixed rate."""
        rate = max(self.config.servol_rate_hz, 1.0)
        interval = 1.0 / rate
        next_tick = time.monotonic()
        while not self.shutdown_event.is_set():
            command = self._build_servol_command()
            if command is not None:
                self.servol_pub.publish(command)

            next_tick += interval
            delay = next_tick - time.monotonic()
            if delay <= 0.0:
                next_tick = time.monotonic()
                delay = interval
            self.shutdown_event.wait(delay)

    def _build_servol_command(self) -> PoseStamped | None:
        """Build one ServoL pose from the latest filtered PD state."""
        now = time.monotonic()
        with self.state_lock:
            origin_pose = self.origin_pose
            if origin_pose is None:
                return None

            if self.servol_active:
                offset = self.filtered_offset
                velocity = self.filtered_velocity
                measurement_time = self.last_measurement_time
                measurement_stale = (
                    offset is None
                    or measurement_time is None
                    or now - measurement_time
                    > self.config.max_measurement_age_sec
                )
                if measurement_stale:
                    tilt_x_deg = 0.0
                    tilt_y_deg = 0.0
                    control_source = "servol_stale_origin"
                else:
                    tilt_x_deg, tilt_y_deg = balance_tilt_from_pd(
                        offset,
                        velocity,
                        self.config,
                    )
                    control_source = "servol_pd"
            elif self.servol_return_samples > 0:
                tilt_x_deg = 0.0
                tilt_y_deg = 0.0
                self.servol_return_samples -= 1
                measurement_stale = False
                control_source = "servol_return_origin"
            else:
                return None

            self.last_tilt_x_deg = tilt_x_deg
            self.last_tilt_y_deg = tilt_y_deg
            self.last_command_sent = True
            self.last_control_source = control_source
            self.last_measurement_stale = measurement_stale
            frame_id = self.config.frame_id

        target_pose = make_balance_target_pose(
            origin_pose,
            frame_id,
            tilt_x_deg,
            tilt_y_deg,
        )
        target_pose.header.stamp = self.node.get_clock().now().to_msg()
        with self.state_lock:
            self.last_target_pose = target_pose
        return target_pose

    def _disable_servol(self) -> None:
        """Disable continuous ServoL publishing without return samples."""
        with self.state_lock:
            self.servol_active = False
            self.servol_return_samples = 0

    def _is_servol_active(self) -> bool:
        """Return whether ServoL continuous control is active."""
        with self.state_lock:
            return self.servol_active


def balance_tilt_from_offset(
    offset: tuple[float, float],
    config: MotionConfig,
) -> tuple[float, float]:
    """Map normalized image offset to bounded x/y tilt angles in degrees."""
    offset_x, offset_y = offset
    limit = abs(config.angle_limit_deg)
    gain = config.angle_gain_deg
    tilt_x = clamp(offset_y * gain, -limit, limit)
    tilt_y = clamp(offset_x * gain, -limit, limit)
    return tilt_x, tilt_y


def compensate_offset(
    offset: tuple[float, float],
    yaw_deg: float,
) -> tuple[float, float]:
    """Rotate image-space normalized offset into the plate/TCP frame."""
    offset_x, offset_y = offset
    yaw = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * offset_x - sin_yaw * offset_y,
        sin_yaw * offset_x + cos_yaw * offset_y,
    )


def balance_tilt_from_pd(
    offset: tuple[float, float],
    velocity: tuple[float, float],
    config: MotionConfig,
) -> tuple[float, float]:
    """Map filtered offset and velocity to bounded x/y tilt angles."""
    offset_x, offset_y = offset
    velocity_x, velocity_y = velocity
    limit = abs(config.angle_limit_deg)
    proportional_gain = config.angle_gain_deg
    derivative_gain = config.angle_derivative_gain_deg
    tilt_x = clamp(
        offset_y * proportional_gain + velocity_y * derivative_gain,
        -limit,
        limit,
    )
    tilt_y = clamp(
        offset_x * proportional_gain + velocity_x * derivative_gain,
        -limit,
        limit,
    )
    return tilt_x, tilt_y


def make_balance_target_pose(
    origin: PoseStamped,
    frame_id: str,
    tilt_x_deg: float,
    tilt_y_deg: float,
) -> PoseStamped:
    """Create a MoveL target that keeps xyz and applies local x/y tilt."""
    target = PoseStamped()
    target.header.frame_id = frame_id
    target.pose.position.x = origin.pose.position.x
    target.pose.position.y = origin.pose.position.y
    target.pose.position.z = origin.pose.position.z

    base_q = [
        origin.pose.orientation.x,
        origin.pose.orientation.y,
        origin.pose.orientation.z,
        origin.pose.orientation.w,
    ]
    delta_q = quat_from_rpy(
        math.radians(tilt_x_deg),
        math.radians(tilt_y_deg),
        0.0,
    )
    target_q = quat_normalize(quat_multiply(base_q, delta_q))
    target.pose.orientation.x = target_q[0]
    target.pose.orientation.y = target_q[1]
    target.pose.orientation.z = target_q[2]
    target.pose.orientation.w = target_q[3]
    return target


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> list[float]:
    """Convert roll/pitch/yaw radians to an xyzw quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def quat_multiply(q1: list[float], q2: list[float]) -> list[float]:
    """Multiply two xyzw quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quat_normalize(q: list[float]) -> list[float]:
    """Normalize an xyzw quaternion."""
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in q]


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a value into a closed interval."""
    return max(lower, min(upper, value))
