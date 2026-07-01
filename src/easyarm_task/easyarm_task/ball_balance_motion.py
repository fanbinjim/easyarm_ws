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
from dataclasses import replace
from collections import deque
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
    tangent_damping_deg: float
    integral_gain_deg: float
    integral_limit_deg: float
    integral_radius: float
    integral_deadband_radius: float
    integral_speed: float
    initial_trim_x_deg: float
    initial_trim_y_deg: float
    velocity_filter_alpha: float
    delay_compensation_sec: float
    target_offset_x: float
    target_offset_y: float
    camera_to_plate_yaw_deg: float
    filter_alpha: float
    max_measurement_age_sec: float
    velocity_window_size: int
    center_radius: float
    center_exit_radius: float
    center_speed_enter_velocity: float
    center_speed_exit_velocity: float
    center_angle_limit_deg: float
    center_position_scale: float
    center_radial_damping_deg: float
    center_tangent_damping_deg: float
    recovery_radius: float
    recovery_exit_radius: float
    recovery_tangent_enter_velocity: float
    recovery_tangent_exit_velocity: float
    recovery_radial_exit_velocity: float
    recovery_speed_exit_velocity: float
    recovery_radial_gain_deg: float
    recovery_radial_damping_deg: float
    recovery_tangent_damping_deg: float
    recovery_angle_limit_deg: float
    tilt_rate_limit_deg_s: float
    velocity_scale: float
    acceleration_scale: float
    servol_rate_hz: float


@dataclass(frozen=True)
class BalanceControlState:
    """Debug state for the latest balance controller update."""

    mode: str = "idle"
    radius: float = 0.0
    radial_velocity: float = 0.0
    tangent_velocity: float = 0.0
    speed: float = 0.0
    radial_command: float = 0.0
    tangent_command: float = 0.0
    radial_scale: float = 1.0
    effective_limit_deg: float = 0.0


class BallBalanceMotionController:
    """Send pose1, one-shot MoveL, and continuous ServoL commands."""

    def __init__(self, node: Node, config: MotionConfig) -> None:
        """Create action and service clients."""
        self.node = node
        self.config = normalize_motion_config(config)
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
        self.measurement_history: deque[tuple[float, float, float]] = deque(
            maxlen=max(2, self.config.velocity_window_size),
        )
        self.last_measurement_time: float | None = None
        self.status = "Space: pose1, B: MoveL step, O/P: ServoL run/stop"
        self.last_tilt_x_deg = 0.0
        self.last_tilt_y_deg = 0.0
        self.last_tilt_time: float | None = None
        self.last_command_sent = False
        self.last_control_source = ""
        self.last_measurement_stale = False
        self.last_target_pose = None
        self.last_control_offset: tuple[float, float] | None = None
        self.last_control_state = BalanceControlState()
        self.integral_trim_x_deg = self.config.initial_trim_x_deg
        self.integral_trim_y_deg = self.config.initial_trim_y_deg
        self.integral_active = False
        self.last_integral_time: float | None = None
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
            control_state = self.last_control_state
            last_measurement_time = self.last_measurement_time
            integral_trim_x_deg = self.integral_trim_x_deg
            integral_trim_y_deg = self.integral_trim_y_deg
            integral_active = self.integral_active
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
            lines.append(
                "balance: "
                f"{control_state.mode} "
                f"r={control_state.radius:.3f} "
                f"vr={control_state.radial_velocity:+.2f} "
                f"vt={control_state.tangent_velocity:+.2f} "
                f"lim={control_state.effective_limit_deg:.1f}"
            )
            lines.append(
                "i trim: "
                f"x={integral_trim_x_deg:+.2f}deg "
                f"y={integral_trim_y_deg:+.2f}deg "
                f"{'on' if integral_active else 'hold'}"
            )
        state = "active" if servol_active else "stopped"
        lines.append(f"servol: {state} {self.config.servol_rate_hz:.0f}Hz")
        return lines

    def snapshot(self) -> dict:
        """Return a thread-safe motion state snapshot for logging."""
        with self.state_lock:
            config = self.config
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
                "last_control_offset": self.last_control_offset,
                "last_control_state": self.last_control_state,
                "integral_trim_x_deg": self.integral_trim_x_deg,
                "integral_trim_y_deg": self.integral_trim_y_deg,
                "integral_active": self.integral_active,
                "config": config,
            }

    def update_config(self, **changes) -> None:
        """Replace motion configuration fields at runtime."""
        with self.state_lock:
            self.config = normalize_motion_config(replace(
                self.config,
                **changes,
            ))
            self._clamp_integral_trim_locked()
            self.measurement_history = deque(
                self.measurement_history,
                maxlen=max(2, self.config.velocity_window_size),
            )

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

        compensated_offset = self.control_error_from_offset(offset)
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
            self.last_control_offset = compensated_offset
            self.last_control_state = BalanceControlState(mode="movel_step")
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
            self.last_tilt_time = None
            self.last_integral_time = None
            self.measurement_history.clear()
            if (
                self.filtered_offset is not None
                and self.last_measurement_time is not None
            ):
                self.measurement_history.append((
                    self.last_measurement_time,
                    self.filtered_offset[0],
                    self.filtered_offset[1],
                ))
                self.filtered_velocity = (0.0, 0.0)
            self.last_control_state = BalanceControlState(mode="pd")
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
            self.last_tilt_time = None
            self.last_integral_time = None
            self.integral_active = False
            self.last_command_sent = True
            self.last_control_source = "servol_stop"
            self.last_control_offset = (0.0, 0.0)
            self.last_control_state = BalanceControlState(mode="stop")
        self._set_status("ServoL stopping, returning origin attitude")

    def compensated_offset(
        self,
        offset: tuple[float, float],
    ) -> tuple[float, float]:
        """Rotate an image-space offset into the plate/TCP frame."""
        with self.state_lock:
            yaw_deg = self.config.camera_to_plate_yaw_deg
        return compensate_offset(offset, yaw_deg)

    def control_error_from_offset(
        self,
        offset: tuple[float, float],
    ) -> tuple[float, float]:
        """Return target-compensated control error from image offset."""
        with self.state_lock:
            yaw_deg = self.config.camera_to_plate_yaw_deg
            target_x = self.config.target_offset_x
            target_y = self.config.target_offset_y
        offset_x, offset_y = compensate_offset(offset, yaw_deg)
        return offset_x - target_x, offset_y - target_y

    def reset_integral_trim(self) -> None:
        """Reset learned attitude trim to the configured startup value."""
        with self.state_lock:
            self.integral_trim_x_deg = self.config.initial_trim_x_deg
            self.integral_trim_y_deg = self.config.initial_trim_y_deg
            self._clamp_integral_trim_locked()
            self.integral_active = False
            self.last_integral_time = None

    def update_measurement(self, offset: tuple[float, float] | None) -> None:
        """Store the latest visual offset and fit normalized velocity."""
        if offset is None:
            return
        offset_x, offset_y = self.control_error_from_offset(offset)
        now = time.monotonic()
        with self.state_lock:
            if (
                self.filtered_offset is None
                or self.last_measurement_time is None
            ):
                self.filtered_offset = (offset_x, offset_y)
                self.filtered_velocity = (0.0, 0.0)
                self.measurement_history.clear()
                self.measurement_history.append((now, offset_x, offset_y))
                self.last_measurement_time = now
                return

            dt = now - self.last_measurement_time
            if dt <= 1.0e-4 or dt > 0.5:
                self.filtered_offset = (offset_x, offset_y)
                self.filtered_velocity = (0.0, 0.0)
                self.measurement_history.clear()
                self.measurement_history.append((now, offset_x, offset_y))
                self.last_measurement_time = now
                return

            latest_offset = (offset_x, offset_y)
            self.measurement_history.append((now, offset_x, offset_y))
            fitted_velocity = fit_velocity(self.measurement_history)
            velocity_alpha = self.config.velocity_filter_alpha
            self.filtered_velocity = (
                self.filtered_velocity[0]
                + velocity_alpha * (
                    fitted_velocity[0] - self.filtered_velocity[0]
                ),
                self.filtered_velocity[1]
                + velocity_alpha * (
                    fitted_velocity[1] - self.filtered_velocity[1]
                ),
            )
            self.filtered_offset = latest_offset
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
            self.integral_trim_x_deg = self.config.initial_trim_x_deg
            self.integral_trim_y_deg = self.config.initial_trim_y_deg
            self._clamp_integral_trim_locked()
            self.integral_active = False
            self.last_integral_time = None
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
        next_tick = time.monotonic()
        while not self.shutdown_event.is_set():
            with self.state_lock:
                rate = max(self.config.servol_rate_hz, 1.0)
            interval = 1.0 / rate
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
        """Build one ServoL pose from the latest visual control state."""
        now = time.monotonic()
        with self.state_lock:
            config = self.config
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
                    > config.max_measurement_age_sec
                )
                if measurement_stale:
                    tilt_x_deg = self.integral_trim_x_deg
                    tilt_y_deg = self.integral_trim_y_deg
                    control_offset = (0.0, 0.0)
                    control_state = BalanceControlState(
                        mode="stale",
                    )
                    self.integral_active = False
                    self.last_integral_time = None
                    control_source = "servol_stale_trim"
                else:
                    control_offset = predict_offset(
                        offset,
                        velocity,
                        config.delay_compensation_sec,
                    )
                    (
                        tilt_x_deg,
                        tilt_y_deg,
                        control_state,
                    ) = balance_tilt_from_pd(
                        offset,
                        velocity,
                        config,
                    )
                    trim_x_deg, trim_y_deg = (
                        self._update_integral_trim_locked(
                            offset,
                            velocity,
                            now,
                        )
                    )
                    tilt_x_deg += trim_x_deg
                    tilt_y_deg += trim_y_deg
                    final_limit = max(
                        0.0,
                        min(
                            abs(config.angle_limit_deg),
                            control_state.effective_limit_deg
                            + abs(config.integral_limit_deg),
                        ),
                    )
                    tilt_x_deg = clamp(
                        tilt_x_deg,
                        -final_limit,
                        final_limit,
                    )
                    tilt_y_deg = clamp(
                        tilt_y_deg,
                        -final_limit,
                        final_limit,
                    )
                    control_source = "servol_balance"
            elif self.servol_return_samples > 0:
                tilt_x_deg = 0.0
                tilt_y_deg = 0.0
                control_offset = (0.0, 0.0)
                control_state = BalanceControlState(mode="return")
                self.servol_return_samples -= 1
                measurement_stale = False
                control_source = "servol_return_origin"
            else:
                return None

            dt = 0.0
            if self.last_tilt_time is not None:
                dt = max(0.0, now - self.last_tilt_time)
            tilt_x_deg, tilt_y_deg = rate_limit_tilt(
                self.last_tilt_x_deg,
                self.last_tilt_y_deg,
                tilt_x_deg,
                tilt_y_deg,
                config.tilt_rate_limit_deg_s,
                dt,
            )
            self.last_tilt_time = now
            self.last_tilt_x_deg = tilt_x_deg
            self.last_tilt_y_deg = tilt_y_deg
            self.last_command_sent = True
            self.last_control_source = control_source
            self.last_measurement_stale = measurement_stale
            self.last_control_offset = control_offset
            self.last_control_state = control_state
            frame_id = config.frame_id

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
            self.last_tilt_time = None
            self.last_integral_time = None
            self.integral_active = False

    def _is_servol_active(self) -> bool:
        """Return whether ServoL continuous control is active."""
        with self.state_lock:
            return self.servol_active

    def _update_integral_trim_locked(
        self,
        offset: tuple[float, float],
        velocity: tuple[float, float],
        now: float,
    ) -> tuple[float, float]:
        """Update the slow attitude trim from near-center steady error."""
        config = self.config
        if self.last_integral_time is None:
            self.last_integral_time = now
            self.integral_active = False
            return self.integral_trim_x_deg, self.integral_trim_y_deg

        dt = max(0.0, min(now - self.last_integral_time, 0.1))
        self.last_integral_time = now
        radius = math.hypot(offset[0], offset[1])
        speed = math.hypot(velocity[0], velocity[1])
        self.integral_active = (
            config.integral_gain_deg > 0.0
            and config.integral_limit_deg > 0.0
            and radius >= config.integral_deadband_radius
            and radius <= config.integral_radius
            and speed <= config.integral_speed
        )
        if not self.integral_active or dt <= 0.0:
            return self.integral_trim_x_deg, self.integral_trim_y_deg

        self.integral_trim_x_deg += (
            config.integral_gain_deg * offset[1] * dt
        )
        self.integral_trim_y_deg += (
            config.integral_gain_deg * offset[0] * dt
        )
        self._clamp_integral_trim_locked()
        return self.integral_trim_x_deg, self.integral_trim_y_deg

    def _clamp_integral_trim_locked(self) -> None:
        """Limit the learned trim vector magnitude."""
        limit = max(0.0, abs(self.config.integral_limit_deg))
        if limit <= 0.0:
            self.integral_trim_x_deg = 0.0
            self.integral_trim_y_deg = 0.0
            return
        magnitude = math.hypot(
            self.integral_trim_x_deg,
            self.integral_trim_y_deg,
        )
        if magnitude <= limit or magnitude <= 1.0e-9:
            return
        scale = limit / magnitude
        self.integral_trim_x_deg *= scale
        self.integral_trim_y_deg *= scale


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
) -> tuple[float, float, BalanceControlState]:
    """Map visual offset and velocity to a continuous balance tilt."""
    velocity_x, velocity_y = velocity
    control_x, control_y = predict_offset(
        offset,
        velocity,
        config.delay_compensation_sec,
    )
    limit = abs(config.angle_limit_deg)
    proportional_gain = config.angle_gain_deg
    damping_gain = config.angle_derivative_gain_deg
    radius, radial_velocity, tangent_velocity = radial_tangent_state(
        control_x,
        control_y,
        velocity_x,
        velocity_y,
    )
    speed = math.hypot(velocity_x, velocity_y)
    tangent_speed = abs(tangent_velocity)

    center_weight = 1.0 - smoothstep(
        config.center_radius,
        config.center_exit_radius,
        radius,
    )
    edge_weight = smoothstep(
        config.recovery_exit_radius,
        config.recovery_radius,
        radius,
    )

    position_gain = proportional_gain
    center_position_scale = lerp(1.0, config.center_position_scale,
                                 center_weight)
    radial_scale = lerp(1.0, edge_radial_scale(tangent_speed, config),
                        edge_weight)
    position_gain *= center_position_scale * radial_scale
    position_gain = lerp(
        position_gain,
        min(position_gain, config.recovery_radial_gain_deg),
        edge_weight,
    )
    radial_damping_gain = (
        damping_gain
        + center_weight * config.center_radial_damping_deg
        + edge_weight * config.recovery_radial_damping_deg
    )
    tangent_damping_gain = (
        damping_gain
        + config.tangent_damping_deg
        + center_weight * config.center_tangent_damping_deg
        + edge_weight * config.recovery_tangent_damping_deg
    )
    center_quiet_weight = 1.0 - smoothstep(
        config.center_radius * 0.45,
        config.center_radius,
        radius,
    )
    velocity_deadband = 0.08 * center_quiet_weight
    if abs(radial_velocity) < velocity_deadband:
        radial_velocity = 0.0
    if abs(tangent_velocity) < velocity_deadband:
        tangent_velocity = 0.0
    tangent_damping_gain *= 1.0 - 0.45 * center_quiet_weight
    radial_damping_gain *= 1.0 - 0.25 * center_quiet_weight
    effective_limit = continuous_effective_limit(config, radius)

    radial_command = -position_gain * radius - radial_damping_gain * (
        radial_velocity
    )
    tangent_command = -tangent_damping_gain * tangent_velocity
    if radial_command > 0.0:
        radial_command *= 1.0 - edge_weight
    edge_pull = edge_min_radial_pull(config.recovery_angle_limit_deg, radius)
    radial_command = min(radial_command, -edge_weight * edge_pull)
    radial_command, tangent_command = clamp_radial_tangent_commands(
        radial_command,
        tangent_command,
        effective_limit,
        radius,
        radial_velocity,
        tangent_velocity,
        edge_weight,
    )
    acceleration_x, acceleration_y = accel_from_radial_tangent_command(
        control_x,
        control_y,
        radial_command,
        tangent_command,
    )

    tilt_y = -acceleration_x
    tilt_x = -acceleration_y

    tilt_x = clamp(tilt_x, -effective_limit, effective_limit)
    tilt_y = clamp(tilt_y, -effective_limit, effective_limit)
    if radius < config.center_radius * 0.32 and speed < 0.18:
        tilt_x *= 0.65
        tilt_y *= 0.65
    state = BalanceControlState(
        mode="balance",
        radius=radius,
        radial_velocity=radial_velocity,
        tangent_velocity=tangent_velocity,
        speed=speed,
        radial_command=radial_command,
        tangent_command=tangent_command,
        radial_scale=radial_scale,
        effective_limit_deg=effective_limit,
    )
    return tilt_x, tilt_y, state


def tangent_priority_radial_scale(
    tangent_speed: float,
    exit_velocity: float,
    enter_velocity: float,
) -> float:
    """Fade radial pull-in while tangential velocity is still high."""
    low = max(0.0, exit_velocity)
    high = max(low + 0.1, enter_velocity)
    if tangent_speed <= low:
        return 1.0
    if tangent_speed >= high:
        return 0.0
    return (high - tangent_speed) / (high - low)


def rate_limit_tilt(
    previous_x_deg: float,
    previous_y_deg: float,
    target_x_deg: float,
    target_y_deg: float,
    rate_limit_deg_s: float,
    dt: float,
) -> tuple[float, float]:
    """Limit per-cycle tilt changes to reduce control excitation."""
    limit = max(0.0, rate_limit_deg_s)
    if dt <= 0.0 or limit <= 0.0:
        return target_x_deg, target_y_deg
    step = limit * dt
    return (
        previous_x_deg + clamp(target_x_deg - previous_x_deg, -step, step),
        previous_y_deg + clamp(target_y_deg - previous_y_deg, -step, step),
    )


def clamp_radial_tangent_commands(
    radial_command: float,
    tangent_command: float,
    effective_limit_deg: float,
    radius: float,
    radial_velocity: float,
    tangent_velocity: float,
    edge_weight: float,
) -> tuple[float, float]:
    """Clamp polar commands before x/y projection to preserve authority."""
    limit = max(0.0, effective_limit_deg)
    if limit <= 0.0:
        return 0.0, 0.0
    tangent_limit = lerp(
        limit,
        edge_tangent_limit(limit, radius, radial_velocity),
        edge_weight,
    )
    reserve = edge_weight * edge_tangent_reserve(
        limit,
        radius,
        radial_velocity,
        tangent_velocity,
    )
    radial_limit = math.sqrt(max(0.0, limit * limit - reserve ** 2))
    radial_command = clamp(radial_command, -radial_limit, radial_limit)
    remaining = math.sqrt(max(0.0, limit * limit - radial_command ** 2))
    tangent_limit = min(tangent_limit, remaining)
    return (
        radial_command,
        clamp(tangent_command, -tangent_limit, tangent_limit),
    )


def continuous_effective_limit(
    config: MotionConfig,
    radius: float,
) -> float:
    """Blend center, normal, and edge tilt limits without mode switches."""
    limit = abs(config.angle_limit_deg)
    center_weight = 1.0 - smoothstep(
        config.center_radius,
        config.center_exit_radius,
        radius,
    )
    edge_weight = smoothstep(
        config.recovery_exit_radius,
        config.recovery_radius,
        radius,
    )
    center_limit = min(limit, config.center_angle_limit_deg)
    edge_limit = min(limit, config.recovery_angle_limit_deg)
    blended = lerp(limit, center_limit, center_weight)
    return lerp(blended, edge_limit, edge_weight)


def edge_radial_scale(tangent_speed: float, config: MotionConfig) -> float:
    """Fade radial pull-in while tangential velocity is still high."""
    return tangent_priority_radial_scale(
        tangent_speed,
        config.recovery_tangent_exit_velocity,
        config.recovery_tangent_enter_velocity,
    )


def edge_tangent_limit(
    edge_limit_deg: float,
    radius: float,
    radial_velocity: float,
) -> float:
    """Limit tangent damping when the ball is still escaping outward."""
    outward_speed = max(0.0, radial_velocity)
    velocity_scale = 1.0 - clamp((outward_speed - 0.8) / 2.4, 0.0, 1.0)
    radius_scale = 1.0 - 0.5 * clamp((radius - 0.78) / 0.14, 0.0, 1.0)
    scale = max(0.28, min(velocity_scale, radius_scale))
    return 0.45 * edge_limit_deg * scale


def edge_tangent_reserve(
    edge_limit_deg: float,
    radius: float,
    radial_velocity: float,
    tangent_velocity: float,
) -> float:
    """Reserve some edge authority for stopping wall-following motion."""
    tangent_speed = abs(tangent_velocity)
    if radius < 0.78 or tangent_speed < 2.0:
        return 0.0
    wall_scale = clamp((radius - 0.78) / 0.10, 0.0, 1.0)
    tangent_scale = clamp((tangent_speed - 2.0) / 3.0, 0.0, 1.0)
    outward_scale = clamp(radial_velocity / 3.0, 0.0, 1.0)
    scale = max(tangent_scale, 0.6 * outward_scale)
    return edge_limit_deg * 0.30 * wall_scale * scale


def edge_min_radial_pull(edge_limit_deg: float, radius: float) -> float:
    """Return a minimum inward pull for balls close to the wall."""
    if radius <= 0.6:
        return 0.0
    limit = max(0.0, edge_limit_deg)
    scale = clamp((radius - 0.6) / 0.3, 0.0, 1.0)
    return 0.45 * limit * scale


def smoothstep(edge0: float, edge1: float, value: float) -> float:
    """Return a smooth 0..1 transition between two thresholds."""
    if edge1 <= edge0:
        return 1.0 if value >= edge1 else 0.0
    t = clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def lerp(start: float, end: float, weight: float) -> float:
    """Linearly blend two values."""
    blend = clamp(weight, 0.0, 1.0)
    return start + (end - start) * blend


def radial_tangent_state(
    offset_x: float,
    offset_y: float,
    velocity_x: float,
    velocity_y: float,
) -> tuple[float, float, float]:
    """Return radius, radial velocity, and tangent velocity."""
    radius = math.hypot(offset_x, offset_y)
    if radius <= 1.0e-6:
        return 0.0, 0.0, 0.0
    radial_x = offset_x / radius
    radial_y = offset_y / radius
    tangent_x = -radial_y
    tangent_y = radial_x
    radial_velocity = velocity_x * radial_x + velocity_y * radial_y
    tangent_velocity = velocity_x * tangent_x + velocity_y * tangent_y
    return radius, radial_velocity, tangent_velocity


def accel_from_radial_tangent_command(
    offset_x: float,
    offset_y: float,
    radial_command: float,
    tangent_command: float,
) -> tuple[float, float]:
    """Convert radial/tangent acceleration proxy into x/y acceleration."""
    radius = math.hypot(offset_x, offset_y)
    if radius <= 1.0e-6:
        return 0.0, 0.0
    radial_x = offset_x / radius
    radial_y = offset_y / radius
    tangent_x = -radial_y
    tangent_y = radial_x
    accel_x = radial_command * radial_x + tangent_command * tangent_x
    accel_y = radial_command * radial_y + tangent_command * tangent_y
    return accel_x, accel_y


def fit_velocity(
    history: deque[tuple[float, float, float]],
) -> tuple[float, float]:
    """Fit x/y velocity from a short time window using least squares."""
    if len(history) < 2:
        return 0.0, 0.0
    times = [sample[0] for sample in history]
    mean_time = sum(times) / len(times)
    centered = [value - mean_time for value in times]
    denominator = sum(value * value for value in centered)
    if denominator <= 1.0e-9:
        return 0.0, 0.0
    mean_x = sum(sample[1] for sample in history) / len(history)
    mean_y = sum(sample[2] for sample in history) / len(history)
    velocity_x = sum(
        centered[index] * (sample[1] - mean_x)
        for index, sample in enumerate(history)
    ) / denominator
    velocity_y = sum(
        centered[index] * (sample[2] - mean_y)
        for index, sample in enumerate(history)
    ) / denominator
    return velocity_x, velocity_y


def predict_offset(
    offset: tuple[float, float],
    velocity: tuple[float, float],
    lead_time_sec: float,
) -> tuple[float, float]:
    """Predict ball offset forward to compensate camera/control latency."""
    lead = clamp(lead_time_sec, 0.0, 0.5)
    return (
        offset[0] + velocity[0] * lead,
        offset[1] + velocity[1] * lead,
    )


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


def normalize_motion_config(config: MotionConfig) -> MotionConfig:
    """Clamp runtime-tuned motion parameters to valid ranges."""
    recovery_radius = clamp(config.recovery_radius, 0.05, 1.2)
    recovery_exit_radius = clamp(config.recovery_exit_radius, 0.0, 1.1)
    if recovery_exit_radius >= recovery_radius:
        recovery_exit_radius = max(0.0, recovery_radius - 0.05)
    center_radius = clamp(config.center_radius, 0.02, 1.0)
    center_exit_radius = clamp(
        max(config.center_exit_radius, center_radius),
        0.02,
        1.1,
    )
    center_speed_exit_velocity = max(0.0, config.center_speed_exit_velocity)
    center_speed_enter_velocity = max(
        center_speed_exit_velocity,
        config.center_speed_enter_velocity,
    )
    return replace(
        config,
        angle_limit_deg=max(0.1, abs(config.angle_limit_deg)),
        tangent_damping_deg=max(0.0, config.tangent_damping_deg),
        integral_gain_deg=max(0.0, config.integral_gain_deg),
        integral_limit_deg=clamp(abs(config.integral_limit_deg), 0.0, 10.0),
        integral_radius=clamp(config.integral_radius, 0.02, 1.0),
        integral_deadband_radius=clamp(
            min(config.integral_deadband_radius, config.integral_radius),
            0.0,
            0.5,
        ),
        integral_speed=max(0.0, config.integral_speed),
        initial_trim_x_deg=clamp(config.initial_trim_x_deg, -10.0, 10.0),
        initial_trim_y_deg=clamp(config.initial_trim_y_deg, -10.0, 10.0),
        velocity_filter_alpha=clamp(config.velocity_filter_alpha, 0.01, 1.0),
        delay_compensation_sec=clamp(config.delay_compensation_sec, 0.0, 0.5),
        target_offset_x=clamp(config.target_offset_x, -1.0, 1.0),
        target_offset_y=clamp(config.target_offset_y, -1.0, 1.0),
        filter_alpha=clamp(config.filter_alpha, 0.01, 1.0),
        max_measurement_age_sec=max(0.02, config.max_measurement_age_sec),
        velocity_window_size=max(2, int(config.velocity_window_size)),
        center_radius=center_radius,
        center_exit_radius=center_exit_radius,
        center_speed_enter_velocity=center_speed_enter_velocity,
        center_speed_exit_velocity=center_speed_exit_velocity,
        center_angle_limit_deg=clamp(
            abs(config.center_angle_limit_deg),
            0.1,
            abs(config.angle_limit_deg),
        ),
        center_position_scale=clamp(config.center_position_scale, 0.0, 1.0),
        center_radial_damping_deg=max(
            0.0,
            config.center_radial_damping_deg,
        ),
        center_tangent_damping_deg=max(
            0.0,
            config.center_tangent_damping_deg,
        ),
        recovery_radius=recovery_radius,
        recovery_exit_radius=recovery_exit_radius,
        recovery_tangent_enter_velocity=max(
            0.0,
            config.recovery_tangent_enter_velocity,
        ),
        recovery_tangent_exit_velocity=max(
            0.0,
            config.recovery_tangent_exit_velocity,
        ),
        recovery_radial_exit_velocity=max(
            0.0,
            config.recovery_radial_exit_velocity,
        ),
        recovery_speed_exit_velocity=max(
            0.0,
            config.recovery_speed_exit_velocity,
        ),
        recovery_radial_gain_deg=max(0.0, config.recovery_radial_gain_deg),
        recovery_radial_damping_deg=max(
            0.0,
            config.recovery_radial_damping_deg,
        ),
        recovery_tangent_damping_deg=max(
            0.0,
            config.recovery_tangent_damping_deg,
        ),
        recovery_angle_limit_deg=clamp(
            abs(config.recovery_angle_limit_deg),
            0.1,
            abs(config.angle_limit_deg),
        ),
        tilt_rate_limit_deg_s=max(0.0, config.tilt_rate_limit_deg_s),
        velocity_scale=clamp(config.velocity_scale, 0.0, 1.0),
        acceleration_scale=clamp(config.acceleration_scale, 0.0, 1.0),
        servol_rate_hz=max(1.0, config.servol_rate_hz),
    )
