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

"""Detect the plate and red ball for the EasyArm ball balance task."""

import argparse
from collections import deque
import csv
from dataclasses import dataclass
from dataclasses import replace
import math
from pathlib import Path
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

from .ball_balance_detector import BallBalanceDetection
from .ball_balance_detector import CircleDetection
from .ball_balance_detector import DetectionConfig
from .ball_balance_detector import clamp
from .ball_balance_detector import detect_objects
from .ball_balance_detector import PlateCandidateDebug
from .ball_balance_motion import BallBalanceMotionController
from .ball_balance_motion import compensate_offset
from .ball_balance_motion import MotionConfig


DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_DISPLAY_WIDTH = 640
DEFAULT_DISPLAY_HEIGHT = 480
DEBUG_DISPLAY_WIDTH = 320
DEBUG_DISPLAY_HEIGHT = 240
WINDOW_NAME = "EasyArm Ball Balance"
TUNING_WINDOW_NAME = "Ball Balance Tuning"


@dataclass(frozen=True)
class TrackbarSpec:
    """One OpenCV trackbar mapped to a runtime parameter."""

    name: str
    group: str
    field: str
    minimum: float
    maximum: float
    scale: float
    is_int: bool = False

    @property
    def max_position(self) -> int:
        """Return the integer trackbar range."""
        return int(round((self.maximum - self.minimum) * self.scale))

    def value_to_position(self, value: float) -> int:
        """Convert a real parameter value to a trackbar position."""
        clamped = clamp(float(value), self.minimum, self.maximum)
        return int(round((clamped - self.minimum) * self.scale))

    def position_to_value(self, position: int) -> float | int:
        """Convert a trackbar position to a real parameter value."""
        value = self.minimum + float(position) / self.scale
        value = clamp(value, self.minimum, self.maximum)
        if self.is_int:
            return int(round(value))
        return value


TRACKBAR_SPECS = [
    TrackbarSpec("kp x0.1", "motion", "angle_gain_deg", 0.0, 40.0, 10.0),
    TrackbarSpec(
        "damp x0.1",
        "motion",
        "angle_derivative_gain_deg",
        0.0,
        20.0,
        10.0,
    ),
    TrackbarSpec(
        "tangent damp x0.1",
        "motion",
        "tangent_damping_deg",
        0.0,
        30.0,
        10.0,
    ),
    TrackbarSpec(
        "i gain x0.01",
        "motion",
        "integral_gain_deg",
        0.0,
        2.0,
        100.0,
    ),
    TrackbarSpec(
        "i limit deg",
        "motion",
        "integral_limit_deg",
        0.0,
        10.0,
        10.0,
    ),
    TrackbarSpec(
        "i radius x0.01",
        "motion",
        "integral_radius",
        0.05,
        1.0,
        100.0,
    ),
    TrackbarSpec(
        "i speed x0.1",
        "motion",
        "integral_speed",
        0.0,
        3.0,
        10.0,
    ),
    TrackbarSpec(
        "lead ms",
        "motion",
        "delay_compensation_sec",
        0.0,
        0.5,
        1000.0,
    ),
    TrackbarSpec(
        "target x x0.01",
        "motion",
        "target_offset_x",
        -1.0,
        1.0,
        100.0,
    ),
    TrackbarSpec(
        "target y x0.01",
        "motion",
        "target_offset_y",
        -1.0,
        1.0,
        100.0,
    ),
    TrackbarSpec(
        "limit deg",
        "motion",
        "angle_limit_deg",
        1.0,
        30.0,
        1.0,
    ),
    TrackbarSpec(
        "yaw deg +180",
        "motion",
        "camera_to_plate_yaw_deg",
        -180.0,
        180.0,
        1.0,
    ),
    TrackbarSpec(
        "filter x0.01",
        "motion",
        "filter_alpha",
        0.01,
        1.0,
        100.0,
    ),
    TrackbarSpec(
        "max age x0.001s",
        "motion",
        "max_measurement_age_sec",
        0.02,
        1.0,
        1000.0,
    ),
    TrackbarSpec(
        "vel window",
        "motion",
        "velocity_window_size",
        2.0,
        12.0,
        1.0,
        is_int=True,
    ),
    TrackbarSpec(
        "center in x0.01",
        "motion",
        "center_radius",
        0.05,
        1.00,
        100.0,
    ),
    TrackbarSpec(
        "center out x0.01",
        "motion",
        "center_exit_radius",
        0.05,
        1.10,
        100.0,
    ),
    TrackbarSpec(
        "center limit deg",
        "motion",
        "center_angle_limit_deg",
        1.0,
        20.0,
        1.0,
    ),
    TrackbarSpec(
        "center kp scale",
        "motion",
        "center_position_scale",
        0.0,
        1.0,
        100.0,
    ),
    TrackbarSpec(
        "center radial kd",
        "motion",
        "center_radial_damping_deg",
        0.0,
        30.0,
        10.0,
    ),
    TrackbarSpec(
        "center tangent kd",
        "motion",
        "center_tangent_damping_deg",
        0.0,
        30.0,
        10.0,
    ),
    TrackbarSpec(
        "edge in x0.01",
        "motion",
        "recovery_radius",
        0.10,
        1.20,
        100.0,
    ),
    TrackbarSpec(
        "edge out x0.01",
        "motion",
        "recovery_exit_radius",
        0.05,
        1.10,
        100.0,
    ),
    TrackbarSpec(
        "edge vt in x0.1",
        "motion",
        "recovery_tangent_enter_velocity",
        0.0,
        10.0,
        10.0,
    ),
    TrackbarSpec(
        "edge vt out x0.1",
        "motion",
        "recovery_tangent_exit_velocity",
        0.0,
        10.0,
        10.0,
    ),
    TrackbarSpec(
        "edge radial kp",
        "motion",
        "recovery_radial_gain_deg",
        0.0,
        30.0,
        10.0,
    ),
    TrackbarSpec(
        "edge radial kd",
        "motion",
        "recovery_radial_damping_deg",
        0.0,
        30.0,
        10.0,
    ),
    TrackbarSpec(
        "edge tangent kd",
        "motion",
        "recovery_tangent_damping_deg",
        0.0,
        30.0,
        10.0,
    ),
    TrackbarSpec(
        "edge limit deg",
        "motion",
        "recovery_angle_limit_deg",
        1.0,
        30.0,
        1.0,
    ),
    TrackbarSpec(
        "tilt rate deg/s",
        "motion",
        "tilt_rate_limit_deg_s",
        0.0,
        300.0,
        1.0,
    ),
    TrackbarSpec(
        "plate r min",
        "detection",
        "plate_min_radius",
        20.0,
        180.0,
        1.0,
    ),
    TrackbarSpec(
        "plate r max",
        "detection",
        "plate_max_radius",
        20.0,
        220.0,
        1.0,
    ),
    TrackbarSpec(
        "plate expect r",
        "detection",
        "plate_expected_radius",
        20.0,
        220.0,
        1.0,
    ),
    TrackbarSpec(
        "roi x min x0.001",
        "detection",
        "plate_roi_x_min",
        0.0,
        1.0,
        1000.0,
    ),
    TrackbarSpec(
        "roi x max x0.001",
        "detection",
        "plate_roi_x_max",
        0.0,
        1.0,
        1000.0,
    ),
    TrackbarSpec(
        "roi y min x0.001",
        "detection",
        "plate_roi_y_min",
        0.0,
        1.0,
        1000.0,
    ),
    TrackbarSpec(
        "roi y max x0.001",
        "detection",
        "plate_roi_y_max",
        0.0,
        1.0,
        1000.0,
    ),
    TrackbarSpec(
        "expect x x0.001",
        "detection",
        "plate_expected_x",
        0.0,
        1.0,
        1000.0,
    ),
    TrackbarSpec(
        "expect y x0.001",
        "detection",
        "plate_expected_y",
        0.0,
        1.0,
        1000.0,
    ),
    TrackbarSpec(
        "ball r min",
        "detection",
        "ball_min_radius",
        1.0,
        60.0,
        1.0,
    ),
    TrackbarSpec(
        "ball r max",
        "detection",
        "ball_max_radius",
        1.0,
        80.0,
        1.0,
    ),
    TrackbarSpec(
        "ball area min",
        "detection",
        "ball_min_area",
        1.0,
        3000.0,
        1.0,
    ),
    TrackbarSpec(
        "ball area max",
        "detection",
        "ball_max_area",
        10.0,
        8000.0,
        1.0,
    ),
    TrackbarSpec(
        "ball circ x0.01",
        "detection",
        "ball_min_circularity",
        0.01,
        1.0,
        100.0,
    ),
    TrackbarSpec(
        "inner scale x0.01",
        "detection",
        "ball_plate_inner_scale",
        0.10,
        1.20,
        100.0,
    ),
]


def default_debug_capture_dir() -> Path:
    """Return the package debug directory used for image capture."""
    workspace_package = Path.cwd() / "src" / "easyarm_task"
    if (workspace_package / "package.xml").exists():
        return workspace_package / "debug"
    return Path(__file__).resolve().parents[1] / "debug"


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser for the task."""
    parser = argparse.ArgumentParser(
        description="Preview and detect the plate/ball for EasyArm balance.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--image-topic",
        default=DEFAULT_IMAGE_TOPIC,
        help="ROS image topic to subscribe.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=DEFAULT_DISPLAY_WIDTH,
        help="Preview window image width.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=DEFAULT_DISPLAY_HEIGHT,
        help="Preview window image height.",
    )
    parser.add_argument(
        "--window-name",
        default=WINDOW_NAME,
        help="OpenCV preview window title.",
    )
    parser.add_argument(
        "--debug-mask",
        action="store_true",
        help="Show intermediate plate and ball binary masks.",
    )
    parser.add_argument(
        "--control-log",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Record visual/control/robot state rows when plate and ball exist."
        ),
    )
    parser.add_argument(
        "--named-state",
        default="pose1",
        help="Named state reached when Space is pressed.",
    )
    parser.add_argument(
        "--motion-frame-id",
        default="base_link",
        help="Target frame for GetPose and MoveL commands.",
    )
    parser.add_argument(
        "--motion-source-frame",
        default="",
        help="Source frame for GetPose. Empty uses motion server ee_link.",
    )
    parser.add_argument(
        "--balance-angle-limit-deg",
        type=float,
        default=15.0,
        help="Maximum absolute x/y tilt angle sent by one MoveL command.",
    )
    parser.add_argument(
        "--balance-angle-gain-deg",
        type=float,
        default=6.5,
        help="Tilt degrees per normalized ball offset.",
    )
    parser.add_argument(
        "--balance-angle-d-gain-deg",
        type=float,
        default=1.5,
        help=(
            "Damping tilt degrees per normalized ball offset "
            "velocity."
        ),
    )
    parser.add_argument(
        "--balance-tangent-d-gain-deg",
        type=float,
        default=3.0,
        help=(
            "Extra tangent-velocity damping applied at every radius."
        ),
    )
    parser.add_argument(
        "--balance-integral-gain-deg",
        type=float,
        default=0.25,
        help=(
            "Slow integral trim gain in tilt degrees per normalized "
            "offset-second."
        ),
    )
    parser.add_argument(
        "--balance-integral-limit-deg",
        type=float,
        default=4.0,
        help="Maximum learned integral trim magnitude in degrees.",
    )
    parser.add_argument(
        "--balance-integral-radius",
        type=float,
        default=0.65,
        help="Only integrate visual error inside this normalized radius.",
    )
    parser.add_argument(
        "--balance-integral-speed",
        type=float,
        default=0.8,
        help="Only integrate visual error below this normalized xy speed.",
    )
    parser.add_argument(
        "--balance-delay-compensation-sec",
        type=float,
        default=0.0,
        help=(
            "Forward prediction time for visual offset latency "
            "compensation."
        ),
    )
    parser.add_argument(
        "--balance-target-offset-x",
        type=float,
        default=0.0,
        help="Target normalized x offset after yaw compensation.",
    )
    parser.add_argument(
        "--balance-target-offset-y",
        type=float,
        default=0.0,
        help="Target normalized y offset after yaw compensation.",
    )
    parser.add_argument(
        "--camera-to-plate-yaw-deg",
        type=float,
        default=-8.0,
        help=(
            "Yaw rotation from image offset frame to plate/TCP frame. "
            "Positive values rotate image offset counterclockwise."
        ),
    )
    parser.add_argument(
        "--balance-filter-alpha",
        type=float,
        default=0.35,
        help=(
            "Deprecated compatibility value; visual position is no longer "
            "low-pass filtered."
        ),
    )
    parser.add_argument(
        "--balance-max-measurement-age-sec",
        type=float,
        default=0.2,
        help="Maximum visual measurement age before ServoL commands origin.",
    )
    parser.add_argument(
        "--balance-velocity-window-size",
        type=int,
        default=5,
        help="Recent visual samples used for least-squares velocity fitting.",
    )
    parser.add_argument(
        "--balance-center-radius",
        type=float,
        default=0.35,
        help="Radius where continuous center damping is fully applied.",
    )
    parser.add_argument(
        "--balance-center-exit-radius",
        type=float,
        default=0.62,
        help="Radius where continuous center damping fades out.",
    )
    parser.add_argument(
        "--balance-center-speed-enter-velocity",
        type=float,
        default=1.2,
        help=(
            "Normalized xy speed that enters center damping before the "
            "ball reaches the center radius."
        ),
    )
    parser.add_argument(
        "--balance-center-speed-exit-velocity",
        type=float,
        default=0.6,
        help="Normalized xy speed below which center damping can exit.",
    )
    parser.add_argument(
        "--balance-center-angle-limit-deg",
        type=float,
        default=6.0,
        help="Tilt limit blended in near the center region.",
    )
    parser.add_argument(
        "--balance-center-position-scale",
        type=float,
        default=0.15,
        help="Position gain scale blended in near the center region.",
    )
    parser.add_argument(
        "--balance-center-radial-d-gain-deg",
        type=float,
        default=4.0,
        help="Extra radial damping gain while center damping is active.",
    )
    parser.add_argument(
        "--balance-center-tangent-d-gain-deg",
        type=float,
        default=8.0,
        help="Extra tangent damping gain while center damping is active.",
    )
    parser.add_argument(
        "--balance-recovery-radius",
        type=float,
        default=0.72,
        help="Radius where continuous edge recovery is fully applied.",
    )
    parser.add_argument(
        "--balance-recovery-exit-radius",
        type=float,
        default=0.62,
        help="Radius where continuous edge recovery starts to fade in.",
    )
    parser.add_argument(
        "--balance-recovery-tangent-enter-velocity",
        type=float,
        default=1.0,
        help=(
            "Normalized tangent velocity that can enter edge recovery "
            "before the ball reaches the hard radius threshold."
        ),
    )
    parser.add_argument(
        "--balance-recovery-tangent-exit-velocity",
        type=float,
        default=0.6,
        help=(
            "Tangent velocity where edge recovery allows full radial pull."
        ),
    )
    parser.add_argument(
        "--balance-recovery-radial-exit-velocity",
        type=float,
        default=0.8,
        help=(
            "Maximum normalized radial velocity allowed before edge "
            "recovery can switch back to PD."
        ),
    )
    parser.add_argument(
        "--balance-recovery-speed-exit-velocity",
        type=float,
        default=1.0,
        help=(
            "Maximum normalized xy speed allowed before edge recovery "
            "can switch back to PD."
        ),
    )
    parser.add_argument(
        "--balance-recovery-radial-gain-deg",
        type=float,
        default=8.0,
        help="Edge recovery radial pull-in gain in tilt degrees.",
    )
    parser.add_argument(
        "--balance-recovery-radial-d-gain-deg",
        type=float,
        default=2.0,
        help="Edge recovery radial damping gain in tilt degrees.",
    )
    parser.add_argument(
        "--balance-recovery-tangent-d-gain-deg",
        type=float,
        default=8.0,
        help="Edge recovery tangent damping gain in tilt degrees.",
    )
    parser.add_argument(
        "--balance-recovery-angle-limit-deg",
        type=float,
        default=8.0,
        help="Tilt limit blended in near the edge region.",
    )
    parser.add_argument(
        "--balance-tilt-rate-limit-deg-s",
        type=float,
        default=80.0,
        help="Maximum tilt command slew rate in degrees per second.",
    )
    parser.add_argument(
        "--motion-velocity-scale",
        type=float,
        default=0.1,
        help="Velocity scale for pose1 and MoveL commands.",
    )
    parser.add_argument(
        "--motion-acceleration-scale",
        type=float,
        default=0.1,
        help="Acceleration scale for pose1 and MoveL commands.",
    )
    parser.add_argument(
        "--servol-rate-hz",
        type=float,
        default=100.0,
        help=(
            "ServoL target publish rate when O key starts "
            "continuous control."
        ),
    )
    parser.add_argument(
        "--plate-min-radius",
        type=float,
        default=100.0,
        help="Minimum plate radius in preview pixels.",
    )
    parser.add_argument(
        "--plate-max-radius",
        type=float,
        default=150.0,
        help="Maximum plate radius in preview pixels.",
    )
    parser.add_argument(
        "--plate-roi-x-min",
        type=float,
        default=0.22,
        help="Normalized left bound of the plate search ROI.",
    )
    parser.add_argument(
        "--plate-roi-x-max",
        type=float,
        default=0.86,
        help="Normalized right bound of the plate search ROI.",
    )
    parser.add_argument(
        "--plate-roi-y-min",
        type=float,
        default=0.22,
        help="Normalized top bound of the plate search ROI.",
    )
    parser.add_argument(
        "--plate-roi-y-max",
        type=float,
        default=0.95,
        help="Normalized bottom bound of the plate search ROI.",
    )
    parser.add_argument(
        "--plate-expected-x",
        type=float,
        default=0.543,
        help="Expected normalized plate center x used as a soft prior.",
    )
    parser.add_argument(
        "--plate-expected-y",
        type=float,
        default=0.603,
        help="Expected normalized plate center y used as a soft prior.",
    )
    parser.add_argument(
        "--plate-expected-radius",
        type=float,
        default=124.0,
        help="Expected plate radius in preview pixels.",
    )
    parser.add_argument(
        "--ball-min-area",
        type=float,
        default=45.0,
        help="Minimum contour area for the red ball.",
    )
    parser.add_argument(
        "--ball-max-area",
        type=float,
        default=1200.0,
        help="Maximum contour area for the red ball.",
    )
    parser.add_argument(
        "--ball-min-radius",
        type=float,
        default=4.0,
        help="Minimum ball radius in preview pixels.",
    )
    parser.add_argument(
        "--ball-max-radius",
        type=float,
        default=18.0,
        help="Maximum ball radius in preview pixels.",
    )
    parser.add_argument(
        "--ball-max-value",
        type=int,
        default=65,
        help="Deprecated compatibility value; red ball uses hue/saturation.",
    )
    parser.add_argument(
        "--ball-min-circularity",
        type=float,
        default=0.25,
        help="Minimum circularity for a ball contour.",
    )
    parser.add_argument(
        "--ball-plate-inner-scale",
        type=float,
        default=0.92,
        help="Plate radius ratio used as the valid ball search area.",
    )
    return parser


def detection_config_from_args(args: argparse.Namespace) -> DetectionConfig:
    """Create a detector configuration from command-line arguments."""
    return normalize_detection_config(DetectionConfig(
        plate_min_radius=args.plate_min_radius,
        plate_max_radius=args.plate_max_radius,
        plate_roi_x_min=args.plate_roi_x_min,
        plate_roi_x_max=args.plate_roi_x_max,
        plate_roi_y_min=args.plate_roi_y_min,
        plate_roi_y_max=args.plate_roi_y_max,
        plate_expected_x=args.plate_expected_x,
        plate_expected_y=args.plate_expected_y,
        plate_expected_radius=args.plate_expected_radius,
        ball_min_area=args.ball_min_area,
        ball_max_area=args.ball_max_area,
        ball_min_radius=args.ball_min_radius,
        ball_max_radius=args.ball_max_radius,
        ball_max_value=args.ball_max_value,
        ball_min_circularity=args.ball_min_circularity,
        ball_plate_inner_scale=args.ball_plate_inner_scale,
    ))


def normalize_detection_config(config: DetectionConfig) -> DetectionConfig:
    """Clamp runtime-tuned detection parameters to valid ranges."""
    plate_min_radius = max(1.0, config.plate_min_radius)
    plate_max_radius = max(plate_min_radius + 1.0, config.plate_max_radius)
    ball_min_radius = max(0.5, config.ball_min_radius)
    ball_max_radius = max(ball_min_radius + 0.5, config.ball_max_radius)
    ball_min_area = max(0.5, config.ball_min_area)
    ball_max_area = max(ball_min_area + 1.0, config.ball_max_area)
    roi_x_min = clamp(config.plate_roi_x_min, 0.0, 1.0)
    roi_x_max = clamp(config.plate_roi_x_max, 0.0, 1.0)
    roi_y_min = clamp(config.plate_roi_y_min, 0.0, 1.0)
    roi_y_max = clamp(config.plate_roi_y_max, 0.0, 1.0)
    if roi_x_max <= roi_x_min:
        roi_x_min = min(0.99, roi_x_min)
        roi_x_max = min(1.0, roi_x_min + 0.01)
    if roi_y_max <= roi_y_min:
        roi_y_min = min(0.99, roi_y_min)
        roi_y_max = min(1.0, roi_y_min + 0.01)
    return replace(
        config,
        plate_min_radius=plate_min_radius,
        plate_max_radius=plate_max_radius,
        plate_roi_x_min=roi_x_min,
        plate_roi_x_max=roi_x_max,
        plate_roi_y_min=roi_y_min,
        plate_roi_y_max=roi_y_max,
        plate_expected_x=clamp(config.plate_expected_x, 0.0, 1.0),
        plate_expected_y=clamp(config.plate_expected_y, 0.0, 1.0),
        plate_expected_radius=max(1.0, config.plate_expected_radius),
        ball_min_area=ball_min_area,
        ball_max_area=ball_max_area,
        ball_min_radius=ball_min_radius,
        ball_max_radius=ball_max_radius,
        ball_max_value=int(clamp(config.ball_max_value, 1, 255)),
        ball_min_circularity=clamp(config.ball_min_circularity, 0.01, 1.0),
        ball_plate_inner_scale=clamp(config.ball_plate_inner_scale, 0.1, 1.2),
    )


def motion_config_from_args(args: argparse.Namespace) -> MotionConfig:
    """Create a motion-control configuration from command-line arguments."""
    return MotionConfig(
        named_state=args.named_state,
        frame_id=args.motion_frame_id,
        source_frame=args.motion_source_frame,
        angle_limit_deg=args.balance_angle_limit_deg,
        angle_gain_deg=args.balance_angle_gain_deg,
        angle_derivative_gain_deg=args.balance_angle_d_gain_deg,
        tangent_damping_deg=args.balance_tangent_d_gain_deg,
        integral_gain_deg=args.balance_integral_gain_deg,
        integral_limit_deg=args.balance_integral_limit_deg,
        integral_radius=args.balance_integral_radius,
        integral_speed=args.balance_integral_speed,
        delay_compensation_sec=args.balance_delay_compensation_sec,
        target_offset_x=args.balance_target_offset_x,
        target_offset_y=args.balance_target_offset_y,
        camera_to_plate_yaw_deg=args.camera_to_plate_yaw_deg,
        filter_alpha=args.balance_filter_alpha,
        max_measurement_age_sec=args.balance_max_measurement_age_sec,
        velocity_window_size=args.balance_velocity_window_size,
        center_radius=args.balance_center_radius,
        center_exit_radius=args.balance_center_exit_radius,
        center_speed_enter_velocity=args.balance_center_speed_enter_velocity,
        center_speed_exit_velocity=args.balance_center_speed_exit_velocity,
        center_angle_limit_deg=args.balance_center_angle_limit_deg,
        center_position_scale=args.balance_center_position_scale,
        center_radial_damping_deg=args.balance_center_radial_d_gain_deg,
        center_tangent_damping_deg=args.balance_center_tangent_d_gain_deg,
        recovery_radius=args.balance_recovery_radius,
        recovery_exit_radius=args.balance_recovery_exit_radius,
        recovery_tangent_enter_velocity=(
            args.balance_recovery_tangent_enter_velocity
        ),
        recovery_tangent_exit_velocity=(
            args.balance_recovery_tangent_exit_velocity
        ),
        recovery_radial_exit_velocity=(
            args.balance_recovery_radial_exit_velocity
        ),
        recovery_speed_exit_velocity=(
            args.balance_recovery_speed_exit_velocity
        ),
        recovery_radial_gain_deg=args.balance_recovery_radial_gain_deg,
        recovery_radial_damping_deg=(
            args.balance_recovery_radial_d_gain_deg
        ),
        recovery_tangent_damping_deg=(
            args.balance_recovery_tangent_d_gain_deg
        ),
        recovery_angle_limit_deg=args.balance_recovery_angle_limit_deg,
        tilt_rate_limit_deg_s=args.balance_tilt_rate_limit_deg_s,
        velocity_scale=args.motion_velocity_scale,
        acceleration_scale=args.motion_acceleration_scale,
        servol_rate_hz=args.servol_rate_hz,
    )


class BallBalanceNode(Node):
    """Subscribe to a ROS image topic and keep the newest OpenCV frame."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Create subscriptions and preview state."""
        super().__init__("easyarm_ball_balance")
        self.image_topic = args.image_topic
        self.display_width = args.width
        self.display_height = args.height
        self.window_name = args.window_name
        self.debug_mask = args.debug_mask
        self.capture = FrameCapture(default_debug_capture_dir())
        self.control_logger = ControlLogger(
            self.capture.session_dir,
            enabled=args.control_log,
        )
        self.detector_config = detection_config_from_args(args)
        self.motion = BallBalanceMotionController(
            self,
            motion_config_from_args(args),
        )
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = None
        self.latest_joint_state = None
        self.config_lock = threading.Lock()
        self.latest_fps = 0.0
        self.frame_count = 0
        self.frame_times = deque(maxlen=90)
        self.last_log_time = time.monotonic()

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            1,
        )
        self.joint_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            10,
        )
        self.get_logger().info(f"Subscribing image topic: {self.image_topic}")
        if args.control_log:
            self.get_logger().info(
                f"Control log: {self.control_logger.path}"
            )

    def on_image(self, msg: Image) -> None:
        """Convert incoming ROS images to BGR OpenCV frames."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Failed to convert image: {exc}")
            return

        now = time.monotonic()
        with self.frame_lock:
            self.latest_frame = frame
            self.latest_stamp = msg.header.stamp
            self.frame_count += 1
            self.frame_times.append(now)
            self.latest_fps = estimate_fps(self.frame_times)
            frame_count = self.frame_count
            fps = self.latest_fps

        if now - self.last_log_time >= 2.0:
            self.get_logger().info(
                f"image {frame.shape[1]}x{frame.shape[0]}, "
                f"preview {self.display_width}x{self.display_height}, "
                f"{fps:.1f} Hz, frames {frame_count}"
            )
            self.last_log_time = now

    def on_joint_state(self, msg: JointState) -> None:
        """Keep the newest joint state for control logging."""
        with self.frame_lock:
            self.latest_joint_state = msg

    def get_frame(self):
        """Return a copy of the newest frame plus metadata."""
        with self.frame_lock:
            if self.latest_frame is None:
                return None, None, self.frame_count, self.latest_fps
            return (
                self.latest_frame.copy(),
                self.latest_stamp,
                self.frame_count,
                self.latest_fps,
            )

    def get_joint_state(self) -> JointState | None:
        """Return the latest cached joint state."""
        with self.frame_lock:
            return self.latest_joint_state

    def get_detection_config(self) -> DetectionConfig:
        """Return the latest detector configuration."""
        with self.config_lock:
            return self.detector_config

    def update_detection_config(self, **changes) -> None:
        """Replace detector configuration fields at runtime."""
        with self.config_lock:
            config = replace(self.detector_config, **changes)
            config = normalize_detection_config(config)
            self.detector_config = config


class FrameCapture:
    """Save raw image frames while capture is active."""

    def __init__(self, root_dir: Path) -> None:
        """Create a capture controller."""
        self.root_dir = root_dir
        self.active = False
        self.session_name = time.strftime("%Y%m%d%H%M")
        self.session_dir = self.root_dir / self.session_name
        self.saved_count = 0

    def start(self) -> Path:
        """Start a new capture session."""
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.active = True
        return self.session_dir

    def stop(self) -> int:
        """Stop the current capture session and return saved frame count."""
        saved_count = self.saved_count
        self.active = False
        return saved_count

    def save(self, frame: np.ndarray, frame_count: int) -> None:
        """Save one raw frame as PNG when capture is active."""
        if not self.active or self.session_dir is None:
            return
        filename = f"frame_{frame_count:06d}_{time.time_ns()}.png"
        path = self.session_dir / filename
        if cv2.imwrite(str(path), frame):
            self.saved_count += 1


class TuningPanel:
    """OpenCV trackbar panel for runtime detector and controller tuning."""

    width = 1260
    height = 760
    margin = 18
    column_gap = 22
    row_height = 38
    header_height = 52

    def __init__(
        self,
        window_name: str,
        detection_config: DetectionConfig,
        motion_config: MotionConfig,
    ) -> None:
        """Create the tuning window and initialize all sliders."""
        self.window_name = window_name
        self.specs = TRACKBAR_SPECS
        self.last_positions: dict[str, int] = {}
        self.positions: dict[str, int] = {}
        self.slider_boxes: dict[str, tuple[int, int, int, int]] = {}
        self.dragging_spec: TrackbarSpec | None = None
        self.columns = [
            ("Motion", [spec for spec in self.specs
                        if spec.group == "motion"
                        and not spec.name.startswith(("center", "edge"))]),
            ("Center / Edge", [spec for spec in self.specs
                               if spec.group == "motion"
                               and spec.name.startswith(("center", "edge"))]),
            ("Detection", [spec for spec in self.specs
                           if spec.group == "detection"]),
        ]
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.width, self.height)
        cv2.setMouseCallback(self.window_name, self._on_mouse)
        for spec in self.specs:
            value = self._read_config_value(
                spec,
                detection_config,
                motion_config,
            )
            position = spec.value_to_position(value)
            self.positions[spec.name] = position
            self.last_positions[spec.name] = position

    def sync_to_node(self, node: BallBalanceNode) -> None:
        """Apply changed trackbar values to runtime configs."""
        detection_changes = {}
        motion_changes = {}
        for spec in self.specs:
            position = self.positions.get(spec.name)
            if position is None:
                continue
            if position == self.last_positions.get(spec.name):
                continue
            self.last_positions[spec.name] = position
            value = spec.position_to_value(position)
            if spec.group == "detection":
                detection_changes[spec.field] = value
            elif spec.group == "motion":
                motion_changes[spec.field] = value
        if detection_changes:
            node.update_detection_config(**detection_changes)
            self._sync_positions(node.get_detection_config(),
                                 node.motion.snapshot()["config"])
        if motion_changes:
            node.motion.update_config(**motion_changes)
            self._sync_positions(node.get_detection_config(),
                                 node.motion.snapshot()["config"])

    def draw_values(
        self,
        detection_config: DetectionConfig,
        motion_config: MotionConfig,
    ) -> None:
        """Show a compact multi-column slider panel in the tuning window."""
        image = np.full(
            (self.height, self.width, 3),
            (245, 248, 246),
            dtype=np.uint8,
        )
        self.slider_boxes = {}
        cv2.putText(
            image,
            "Ball Balance Tuning",
            (self.margin, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (18, 112, 82),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "drag sliders; values apply live",
            (self.margin, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (90, 112, 104),
            1,
            cv2.LINE_AA,
        )
        column_width = (
            self.width
            - 2 * self.margin
            - self.column_gap * (len(self.columns) - 1)
        ) // len(self.columns)
        for column_index, (title, specs) in enumerate(self.columns):
            x = self.margin + column_index * (
                column_width + self.column_gap
            )
            self._draw_column(
                image,
                title,
                specs,
                x,
                self.header_height,
                column_width,
            )
        cv2.imshow(self.window_name, image)

    def _sync_positions(
        self,
        detection_config: DetectionConfig,
        motion_config: MotionConfig,
    ) -> None:
        """Move trackbars back to normalized config values when clamped."""
        for spec in self.specs:
            value = self._read_config_value(
                spec,
                detection_config,
                motion_config,
            )
            position = spec.value_to_position(value)
            if position == self.last_positions.get(spec.name):
                continue
            self.positions[spec.name] = position
            self.last_positions[spec.name] = position

    def _read_config_value(
        self,
        spec: TrackbarSpec,
        detection_config: DetectionConfig,
        motion_config: MotionConfig,
    ) -> float:
        """Read one parameter value from the matching config object."""
        if spec.group == "detection":
            return float(getattr(detection_config, spec.field))
        return float(getattr(motion_config, spec.field))

    def _draw_column(
        self,
        image: np.ndarray,
        title: str,
        specs: list[TrackbarSpec],
        x: int,
        y: int,
        width: int,
    ) -> None:
        """Draw one column of custom sliders."""
        cv2.rectangle(
            image,
            (x, y - 8),
            (x + width, self.height - self.margin),
            (230, 238, 234),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            title,
            (x + 10, y + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (36, 54, 48),
            2,
            cv2.LINE_AA,
        )
        row_y = y + 46
        for spec in specs:
            self._draw_slider(image, spec, x + 10, row_y, width - 20)
            row_y += self.row_height

    def _draw_slider(
        self,
        image: np.ndarray,
        spec: TrackbarSpec,
        x: int,
        y: int,
        width: int,
    ) -> None:
        """Draw one slider row and remember its hit box."""
        dark = (36, 54, 48)
        muted = (90, 112, 104)
        accent = (18, 112, 82)
        active = spec == self.dragging_spec
        position = self.positions.get(spec.name, 0)
        value = spec.position_to_value(position)
        cv2.putText(
            image,
            spec.name,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            accent if active else dark,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            self._format_slider_value(spec, value),
            (x + width - 76, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            dark,
            1,
            cv2.LINE_AA,
        )
        slider_y = y + 12
        slider_x0 = x
        slider_x1 = x + width
        fraction = 0.0
        if spec.max_position > 0:
            fraction = clamp(position / spec.max_position, 0.0, 1.0)
        knob_x = int(round(slider_x0 + fraction * (slider_x1 - slider_x0)))
        cv2.line(
            image,
            (slider_x0, slider_y),
            (slider_x1, slider_y),
            (196, 209, 203),
            4,
            cv2.LINE_AA,
        )
        cv2.line(
            image,
            (slider_x0, slider_y),
            (knob_x, slider_y),
            accent,
            4,
            cv2.LINE_AA,
        )
        cv2.circle(
            image,
            (knob_x, slider_y),
            7 if active else 6,
            accent,
            -1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"{spec.minimum:g}",
            (slider_x0, slider_y + 19),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.30,
            muted,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"{spec.maximum:g}",
            (slider_x1 - 36, slider_y + 19),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.30,
            muted,
            1,
            cv2.LINE_AA,
        )
        self.slider_boxes[spec.name] = (
            slider_x0,
            slider_y - 12,
            slider_x1,
            slider_y + 16,
        )

    def _format_slider_value(
        self,
        spec: TrackbarSpec,
        value: float | int,
    ) -> str:
        """Format a slider value compactly."""
        if spec.is_int:
            return str(int(value))
        if abs(float(value)) >= 100.0:
            return f"{float(value):.0f}"
        if abs(float(value)) >= 10.0:
            return f"{float(value):.1f}"
        return f"{float(value):.2f}"

    def _on_mouse(
        self,
        event: int,
        x: int,
        y: int,
        flags: int,
        _userdata,
    ) -> None:
        """Handle mouse dragging for the custom slider panel."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging_spec = self._spec_at(x, y)
            if self.dragging_spec is not None:
                self._set_slider_from_x(self.dragging_spec, x)
        elif (
            event == cv2.EVENT_MOUSEMOVE
            and self.dragging_spec is not None
            and flags & cv2.EVENT_FLAG_LBUTTON
        ):
            self._set_slider_from_x(self.dragging_spec, x)
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging_spec = None

    def _spec_at(self, x: int, y: int) -> TrackbarSpec | None:
        """Return the slider spec under the mouse cursor."""
        for spec in self.specs:
            box = self.slider_boxes.get(spec.name)
            if box is None:
                continue
            x0, y0, x1, y1 = box
            if x0 <= x <= x1 and y0 <= y <= y1:
                return spec
        return None

    def _set_slider_from_x(self, spec: TrackbarSpec, x: int) -> None:
        """Update one slider position from the mouse x coordinate."""
        box = self.slider_boxes.get(spec.name)
        if box is None:
            return
        x0, _y0, x1, _y1 = box
        fraction = clamp((x - x0) / max(1, x1 - x0), 0.0, 1.0)
        self.positions[spec.name] = int(round(fraction * spec.max_position))


class ControlLogger:
    """Write visual, control, and robot state rows to a CSV file."""

    def __init__(self, session_dir: Path, enabled: bool) -> None:
        """Create a CSV logger in the task debug session directory."""
        self.enabled = enabled
        self.path = session_dir / "control_log.csv"
        self.file = None
        self.writer = None
        self.last_flush_time = time.monotonic()
        self.flush_interval_sec = 0.5
        if not self.enabled:
            return
        session_dir.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", newline="")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=control_log_fields(),
        )
        self.writer.writeheader()
        self.file.flush()

    def close(self) -> None:
        """Close the CSV file."""
        if self.file is None:
            return
        self.file.flush()
        self.file.close()
        self.file = None
        self.writer = None

    def write(
        self,
        frame_count: int,
        fps: float,
        stamp,
        detection: BallBalanceDetection,
        motion: BallBalanceMotionController,
        joint_state: JointState | None,
    ) -> None:
        """Write one row when both plate and ball are detected."""
        if (
            not self.enabled
            or self.writer is None
            or detection.plate is None
            or detection.ball is None
            or detection.offset is None
        ):
            return
        snapshot = motion.snapshot()
        config = snapshot.get("config", motion.config)
        row = build_control_log_row(
            frame_count,
            fps,
            stamp,
            detection,
            config,
            snapshot,
            joint_state,
        )
        self.writer.writerow(row)
        now = time.monotonic()
        if (
            self.file is not None
            and now - self.last_flush_time >= self.flush_interval_sec
        ):
            self.file.flush()
            self.last_flush_time = now


def control_log_fields() -> list[str]:
    """Return the stable CSV schema for control tuning logs."""
    fields = [
        "wall_time_sec",
        "frame_count",
        "image_stamp_sec",
        "image_stamp_nanosec",
        "fps",
        "plate_found",
        "ball_found",
        "plate_x_px",
        "plate_y_px",
        "plate_radius_px",
        "ball_x_px",
        "ball_y_px",
        "ball_radius_px",
        "raw_offset_x",
        "raw_offset_y",
        "target_offset_x",
        "target_offset_y",
        "plate_offset_x",
        "plate_offset_y",
        "filtered_offset_x",
        "filtered_offset_y",
        "filtered_velocity_x",
        "filtered_velocity_y",
        "control_offset_x",
        "control_offset_y",
        "measurement_age_sec",
        "kp_deg",
        "kd_deg",
        "tangent_damping_deg",
        "integral_gain_deg",
        "integral_limit_deg",
        "integral_radius",
        "integral_speed",
        "integral_trim_x_deg",
        "integral_trim_y_deg",
        "integral_active",
        "angle_limit_deg",
        "lead_time_sec",
        "filter_alpha",
        "velocity_window_size",
        "center_radius",
        "center_exit_radius",
        "center_speed_enter_velocity",
        "center_speed_exit_velocity",
        "center_angle_limit_deg",
        "center_position_scale",
        "center_radial_damping_deg",
        "center_tangent_damping_deg",
        "recovery_radius",
        "recovery_exit_radius",
        "recovery_tangent_enter_velocity",
        "recovery_tangent_exit_velocity",
        "recovery_radial_exit_velocity",
        "recovery_speed_exit_velocity",
        "recovery_radial_gain_deg",
        "recovery_radial_damping_deg",
        "recovery_tangent_damping_deg",
        "recovery_angle_limit_deg",
        "tilt_rate_limit_deg_s",
        "camera_to_plate_yaw_deg",
        "servol_rate_hz",
        "servol_active",
        "control_source",
        "control_mode",
        "control_stale",
        "control_radius",
        "control_radial_velocity",
        "control_tangent_velocity",
        "control_speed",
        "control_radial_command",
        "control_tangent_command",
        "control_radial_scale",
        "control_effective_limit_deg",
        "tilt_x_deg",
        "tilt_y_deg",
        "motion_busy",
        "active_command",
        "motion_status",
    ]
    fields.extend(pose_fields("target_pose"))
    fields.extend(pose_fields("origin_pose"))
    fields.extend(joint_array_fields("joint_position"))
    fields.extend(joint_array_fields("joint_velocity"))
    fields.extend(joint_array_fields("joint_effort"))
    fields.extend(["joint_names", "joint_stamp_sec", "joint_stamp_nanosec"])
    return fields


def pose_fields(prefix: str) -> list[str]:
    """Return CSV fields for one PoseStamped value."""
    return [
        f"{prefix}_frame_id",
        f"{prefix}_x",
        f"{prefix}_y",
        f"{prefix}_z",
        f"{prefix}_qx",
        f"{prefix}_qy",
        f"{prefix}_qz",
        f"{prefix}_qw",
    ]


def joint_array_fields(prefix: str) -> list[str]:
    """Return CSV fields for a six-joint array."""
    return [f"{prefix}_{index}" for index in range(1, 7)]


def build_control_log_row(
    frame_count: int,
    fps: float,
    stamp,
    detection: BallBalanceDetection,
    config: MotionConfig,
    motion_snapshot: dict,
    joint_state: JointState | None,
) -> dict:
    """Build one CSV row from visual detection and motion state."""
    raw_offset = detection.offset or (None, None)
    plate_offset = (
        compensate_offset(detection.offset, config.camera_to_plate_yaw_deg)
        if detection.offset is not None else (None, None)
    )
    filtered_offset = motion_snapshot.get("filtered_offset") or (None, None)
    filtered_velocity = motion_snapshot.get("filtered_velocity") or (
        None,
        None,
    )
    control_offset = motion_snapshot.get("last_control_offset") or (
        None,
        None,
    )
    control_state = motion_snapshot.get("last_control_state")
    row = {
        "wall_time_sec": time.time(),
        "frame_count": frame_count,
        "image_stamp_sec": "" if stamp is None else stamp.sec,
        "image_stamp_nanosec": "" if stamp is None else stamp.nanosec,
        "fps": fps,
        "plate_found": detection.plate is not None,
        "ball_found": detection.ball is not None,
        "plate_x_px": detection.plate.center[0],
        "plate_y_px": detection.plate.center[1],
        "plate_radius_px": detection.plate.radius,
        "ball_x_px": detection.ball.center[0],
        "ball_y_px": detection.ball.center[1],
        "ball_radius_px": detection.ball.radius,
        "raw_offset_x": raw_offset[0],
        "raw_offset_y": raw_offset[1],
        "target_offset_x": config.target_offset_x,
        "target_offset_y": config.target_offset_y,
        "plate_offset_x": plate_offset[0],
        "plate_offset_y": plate_offset[1],
        "filtered_offset_x": filtered_offset[0],
        "filtered_offset_y": filtered_offset[1],
        "filtered_velocity_x": filtered_velocity[0],
        "filtered_velocity_y": filtered_velocity[1],
        "control_offset_x": control_offset[0],
        "control_offset_y": control_offset[1],
        "measurement_age_sec": motion_snapshot.get("measurement_age_sec"),
        "kp_deg": config.angle_gain_deg,
        "kd_deg": config.angle_derivative_gain_deg,
        "tangent_damping_deg": config.tangent_damping_deg,
        "integral_gain_deg": config.integral_gain_deg,
        "integral_limit_deg": config.integral_limit_deg,
        "integral_radius": config.integral_radius,
        "integral_speed": config.integral_speed,
        "integral_trim_x_deg": motion_snapshot.get("integral_trim_x_deg"),
        "integral_trim_y_deg": motion_snapshot.get("integral_trim_y_deg"),
        "integral_active": motion_snapshot.get("integral_active"),
        "angle_limit_deg": config.angle_limit_deg,
        "lead_time_sec": config.delay_compensation_sec,
        "filter_alpha": config.filter_alpha,
        "velocity_window_size": config.velocity_window_size,
        "center_radius": config.center_radius,
        "center_exit_radius": config.center_exit_radius,
        "center_speed_enter_velocity": config.center_speed_enter_velocity,
        "center_speed_exit_velocity": config.center_speed_exit_velocity,
        "center_angle_limit_deg": config.center_angle_limit_deg,
        "center_position_scale": config.center_position_scale,
        "center_radial_damping_deg": config.center_radial_damping_deg,
        "center_tangent_damping_deg": config.center_tangent_damping_deg,
        "recovery_radius": config.recovery_radius,
        "recovery_exit_radius": config.recovery_exit_radius,
        "recovery_tangent_enter_velocity": (
            config.recovery_tangent_enter_velocity
        ),
        "recovery_tangent_exit_velocity": (
            config.recovery_tangent_exit_velocity
        ),
        "recovery_radial_exit_velocity": (
            config.recovery_radial_exit_velocity
        ),
        "recovery_speed_exit_velocity": (
            config.recovery_speed_exit_velocity
        ),
        "recovery_radial_gain_deg": config.recovery_radial_gain_deg,
        "recovery_radial_damping_deg": (
            config.recovery_radial_damping_deg
        ),
        "recovery_tangent_damping_deg": (
            config.recovery_tangent_damping_deg
        ),
        "recovery_angle_limit_deg": config.recovery_angle_limit_deg,
        "tilt_rate_limit_deg_s": config.tilt_rate_limit_deg_s,
        "camera_to_plate_yaw_deg": config.camera_to_plate_yaw_deg,
        "servol_rate_hz": config.servol_rate_hz,
        "servol_active": motion_snapshot.get("servol_active"),
        "control_source": motion_snapshot.get("last_control_source"),
        "control_mode": getattr(control_state, "mode", ""),
        "control_stale": motion_snapshot.get("last_measurement_stale"),
        "control_radius": getattr(control_state, "radius", ""),
        "control_radial_velocity": getattr(
            control_state,
            "radial_velocity",
            "",
        ),
        "control_tangent_velocity": getattr(
            control_state,
            "tangent_velocity",
            "",
        ),
        "control_speed": getattr(control_state, "speed", ""),
        "control_radial_command": getattr(
            control_state,
            "radial_command",
            "",
        ),
        "control_tangent_command": getattr(
            control_state,
            "tangent_command",
            "",
        ),
        "control_radial_scale": getattr(control_state, "radial_scale", ""),
        "control_effective_limit_deg": getattr(
            control_state,
            "effective_limit_deg",
            "",
        ),
        "tilt_x_deg": motion_snapshot.get("last_tilt_x_deg"),
        "tilt_y_deg": motion_snapshot.get("last_tilt_y_deg"),
        "motion_busy": motion_snapshot.get("busy"),
        "active_command": motion_snapshot.get("active_command"),
        "motion_status": motion_snapshot.get("status"),
    }
    row.update(flatten_pose("target_pose",
                            motion_snapshot.get("last_target_pose")))
    row.update(flatten_pose("origin_pose", motion_snapshot.get("origin_pose")))
    row.update(flatten_joint_state(joint_state))
    return row


def flatten_pose(prefix: str, pose_stamped) -> dict:
    """Flatten a PoseStamped into CSV columns."""
    row = {field: "" for field in pose_fields(prefix)}
    if pose_stamped is None:
        return row
    pose = pose_stamped.pose
    row.update({
        f"{prefix}_frame_id": pose_stamped.header.frame_id,
        f"{prefix}_x": pose.position.x,
        f"{prefix}_y": pose.position.y,
        f"{prefix}_z": pose.position.z,
        f"{prefix}_qx": pose.orientation.x,
        f"{prefix}_qy": pose.orientation.y,
        f"{prefix}_qz": pose.orientation.z,
        f"{prefix}_qw": pose.orientation.w,
    })
    return row


def flatten_joint_state(joint_state: JointState | None) -> dict:
    """Flatten latest JointState into CSV columns."""
    row = {}
    for prefix in ("joint_position", "joint_velocity", "joint_effort"):
        row.update({field: "" for field in joint_array_fields(prefix)})
    row.update({
        "joint_names": "",
        "joint_stamp_sec": "",
        "joint_stamp_nanosec": "",
    })
    if joint_state is None:
        return row
    row["joint_names"] = " ".join(joint_state.name)
    row["joint_stamp_sec"] = joint_state.header.stamp.sec
    row["joint_stamp_nanosec"] = joint_state.header.stamp.nanosec
    for prefix, values in (
        ("joint_position", joint_state.position),
        ("joint_velocity", joint_state.velocity),
        ("joint_effort", joint_state.effort),
    ):
        for index, value in enumerate(values[:6], start=1):
            row[f"{prefix}_{index}"] = value
    return row


def estimate_fps(frame_times: deque[float]) -> float:
    """Estimate input image FPS from recent callback timestamps."""
    if len(frame_times) < 2:
        return 0.0
    duration = frame_times[-1] - frame_times[0]
    if duration <= 0.0:
        return 0.0
    return (len(frame_times) - 1) / duration


def draw_text_lines(
    frame: np.ndarray,
    lines: list[str],
    origin: tuple[int, int],
) -> None:
    """Draw readable status text with a dark shadow."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_color = (120, 255, 180)
    shadow_color = (0, 0, 0)
    x, y = origin
    for index, text in enumerate(lines):
        text_y = y + index * 28
        cv2.putText(
            frame,
            text,
            (x, text_y),
            font,
            0.65,
            shadow_color,
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            text,
            (x, text_y),
            font,
            0.65,
            text_color,
            1,
            cv2.LINE_AA,
        )


def draw_detection(
    frame: np.ndarray,
    detection: BallBalanceDetection,
    config: DetectionConfig,
    motion_config: MotionConfig,
) -> None:
    """Draw plate and ball detections on the frame."""
    draw_plate_roi(frame, config)
    if detection.plate is not None:
        draw_circle_detection(frame, detection.plate, (0, 0, 255), "plate")
        draw_target_offset(frame, detection.plate, motion_config)
    if detection.ball is not None:
        draw_circle_detection(frame, detection.ball, (0, 220, 0), "ball")
    if detection.plate is not None and detection.ball is not None:
        plate_center = round_point(detection.plate.center)
        ball_center = round_point(detection.ball.center)
        cv2.line(frame, plate_center, ball_center, (255, 180, 40), 2)
        cv2.drawMarker(
            frame,
            plate_center,
            (255, 180, 40),
            markerType=cv2.MARKER_CROSS,
            markerSize=14,
            thickness=2,
        )
        cv2.drawMarker(
            frame,
            ball_center,
            (0, 220, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=12,
            thickness=2,
        )
        draw_yaw_compensation_overlay(
            frame,
            detection,
            motion_config.camera_to_plate_yaw_deg,
        )


def draw_target_offset(
    frame: np.ndarray,
    plate: CircleDetection,
    motion_config: MotionConfig,
) -> None:
    """Draw the configured balance target point on the plate."""
    target_point = offset_to_image_point(
        plate.center,
        plate.radius,
        compensate_offset(
            (motion_config.target_offset_x, motion_config.target_offset_y),
            -motion_config.camera_to_plate_yaw_deg,
        ),
    )
    point = round_point(target_point)
    cv2.drawMarker(
        frame,
        point,
        (255, 80, 255),
        markerType=cv2.MARKER_TILTED_CROSS,
        markerSize=18,
        thickness=2,
    )
    cv2.putText(
        frame,
        "target",
        (point[0] + 8, point[1] - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 80, 255),
        1,
        cv2.LINE_AA,
    )


def draw_yaw_compensation_overlay(
    frame: np.ndarray,
    detection: BallBalanceDetection,
    camera_to_plate_yaw_deg: float,
) -> None:
    """Draw raw and yaw-compensated offset vectors on the preview."""
    if (
        detection.plate is None
        or detection.ball is None
        or detection.offset is None
    ):
        return

    plate_center = detection.plate.center
    radius = detection.plate.radius
    compensated_offset = compensate_offset(
        detection.offset,
        camera_to_plate_yaw_deg,
    )
    raw_endpoint = offset_to_image_point(plate_center, radius,
                                         detection.offset)
    compensated_endpoint = offset_to_image_point(
        plate_center,
        radius,
        compensated_offset,
    )
    center = round_point(plate_center)
    raw_point = round_point(raw_endpoint)
    compensated_point = round_point(compensated_endpoint)

    draw_arrow(frame, center, raw_point, (0, 220, 220), "raw")
    draw_arrow(frame, center, compensated_point, (255, 220, 40), "plate")
    draw_plate_axes(frame, plate_center, radius, camera_to_plate_yaw_deg)


def draw_plate_axes(
    frame: np.ndarray,
    center: tuple[float, float],
    radius: float,
    camera_to_plate_yaw_deg: float,
) -> None:
    """Draw the compensated plate coordinate axes in image coordinates."""
    axis_length = max(35.0, radius * 0.55)
    yaw = math.radians(camera_to_plate_yaw_deg)
    x_axis = (math.cos(yaw), math.sin(yaw))
    y_axis = (-math.sin(yaw), math.cos(yaw))
    center_point = round_point(center)
    x_end = round_point((
        center[0] + x_axis[0] * axis_length,
        center[1] + x_axis[1] * axis_length,
    ))
    y_end = round_point((
        center[0] + y_axis[0] * axis_length,
        center[1] + y_axis[1] * axis_length,
    ))
    draw_arrow(frame, center_point, x_end, (255, 120, 40), "plate x")
    draw_arrow(frame, center_point, y_end, (40, 170, 255), "plate y")


def draw_arrow(
    frame: np.ndarray,
    start: tuple[int, int],
    end: tuple[int, int],
    color: tuple[int, int, int],
    label: str,
) -> None:
    """Draw one labelled arrow."""
    cv2.arrowedLine(frame, start, end, color, 2, cv2.LINE_AA, tipLength=0.18)
    cv2.putText(
        frame,
        label,
        (end[0] + 6, end[1] - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        color,
        1,
        cv2.LINE_AA,
    )


def offset_to_image_point(
    center: tuple[float, float],
    radius: float,
    offset: tuple[float, float],
) -> tuple[float, float]:
    """Convert a normalized plate offset to an image-space point."""
    return (
        center[0] + offset[0] * radius,
        center[1] + offset[1] * radius,
    )


def draw_circle_detection(
    frame: np.ndarray,
    detection: CircleDetection,
    color: tuple[int, int, int],
    label: str,
) -> None:
    """Draw a rectangle and circle for a circular detection."""
    x, y, width, height = detection.bbox
    cv2.rectangle(frame, (x, y), (x + width, y + height), color, 3)
    cv2.circle(frame, round_point(detection.center), int(detection.radius),
               color, 2)
    cv2.putText(
        frame,
        label,
        (x, max(18, y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )


def draw_plate_roi(frame: np.ndarray, config: DetectionConfig) -> None:
    """Draw the configured plate search ROI."""
    height, width = frame.shape[:2]
    x0 = int(round(clamp(config.plate_roi_x_min, 0.0, 1.0) * width))
    x1 = int(round(clamp(config.plate_roi_x_max, 0.0, 1.0) * width))
    y0 = int(round(clamp(config.plate_roi_y_min, 0.0, 1.0) * height))
    y1 = int(round(clamp(config.plate_roi_y_max, 0.0, 1.0) * height))
    cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 220, 220), 1)


def round_point(point: tuple[float, float]) -> tuple[int, int]:
    """Round a floating-point image coordinate to integer pixels."""
    return int(round(point[0])), int(round(point[1]))


def build_status_lines(
    frame_count: int,
    fps: float,
    source_size: tuple[int, int],
    stamp,
    detection: BallBalanceDetection,
    motion: BallBalanceMotionController,
    motion_config: MotionConfig,
) -> list[str]:
    """Build the text lines displayed in the preview."""
    stamp_text = (
        f"{stamp.sec}.{stamp.nanosec:09d}"
        if stamp is not None else "n/a"
    )
    lines = [
        f"fps: {fps:.1f}",
        f"source: {source_size[0]}x{source_size[1]}",
        f"frames: {frame_count}",
        f"stamp: {stamp_text}",
    ]
    if detection.plate is None:
        lines.append("plate: not found")
    else:
        x, y = detection.plate.center
        radius = detection.plate.radius
        lines.append(f"plate: ({x:.2f}, {y:.2f}), r={radius:.2f}")
    if detection.ball is None:
        lines.append("ball: not found")
    elif detection.offset is not None:
        dx, dy = detection.offset
        plate_dx, plate_dy = compensate_offset(
            detection.offset,
            motion_config.camera_to_plate_yaw_deg,
        )
        lines.append(f"raw offset: x={dx:+.3f}, y={dy:+.3f}")
        lines.append(
            f"plate offset: x={plate_dx:+.3f}, y={plate_dy:+.3f}, "
            f"yaw={motion_config.camera_to_plate_yaw_deg:+.1f}deg"
        )
        lines.append(
            "target/error: "
            f"tx={motion_config.target_offset_x:+.3f} "
            f"ty={motion_config.target_offset_y:+.3f} "
            f"ex={plate_dx - motion_config.target_offset_x:+.3f} "
            f"ey={plate_dy - motion_config.target_offset_y:+.3f}"
        )
    lines.extend(motion.status_lines())
    return lines


def resize_frame(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Resize a frame to the preview size when needed."""
    if frame.shape[1] == width and frame.shape[0] == height:
        return frame
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def show_debug_masks(detection: BallBalanceDetection) -> None:
    """Show all detector debug images in one compact panel."""
    panel = build_debug_panel(detection)
    if panel is not None:
        cv2.imshow("ball_balance_debug", panel)


def build_debug_panel(detection: BallBalanceDetection) -> np.ndarray | None:
    """Build a tiled debug panel from detector intermediate images."""
    debug = detection.debug
    if debug is None:
        return None
    tiles = [
        make_debug_tile("01 value", debug.gray),
        make_debug_tile("02 roi", debug.roi_mask),
        make_debug_tile("03 green raw", debug.edge_mask),
        make_debug_tile("04 green plate", debug.plate_mask),
    ]
    if debug.ball_mask is not None:
        tiles.append(make_debug_tile("05 red ball", debug.ball_mask))
    else:
        tiles.append(make_blank_debug_tile("05 red ball"))
    if len(tiles) % 2 != 0:
        tiles.append(make_blank_debug_tile(""))
    return tile_debug_images(tiles, columns=2)


last_preview_frame = np.zeros(
    (DEBUG_DISPLAY_HEIGHT, DEBUG_DISPLAY_WIDTH, 3),
    dtype=np.uint8,
)


def make_debug_tile(title: str, image: np.ndarray) -> np.ndarray:
    """Resize a debug image and draw a title label on it."""
    tile = resize_debug_image(image)
    if len(tile.shape) == 2:
        tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
    draw_tile_title(tile, title)
    return tile


def make_blank_debug_tile(title: str) -> np.ndarray:
    """Create an empty debug tile."""
    shape = (DEBUG_DISPLAY_HEIGHT, DEBUG_DISPLAY_WIDTH, 3)
    tile = np.zeros(shape, dtype=np.uint8)
    draw_tile_title(tile, title)
    return tile


def draw_tile_title(tile: np.ndarray, title: str) -> None:
    """Draw a small title in the top-left corner of a tile."""
    if not title:
        return
    cv2.rectangle(tile, (0, 0), (tile.shape[1], 24), (0, 0, 0), -1)
    cv2.putText(
        tile,
        title,
        (8, 17),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (120, 255, 180),
        1,
        cv2.LINE_AA,
    )


def tile_debug_images(tiles: list[np.ndarray], columns: int) -> np.ndarray:
    """Arrange debug tiles into a grid panel."""
    rows = []
    for start in range(0, len(tiles), columns):
        rows.append(np.hstack(tiles[start:start + columns]))
    return np.vstack(rows)


def resize_debug_image(image: np.ndarray) -> np.ndarray:
    """Resize a debug image to a compact fixed preview size."""
    return cv2.resize(
        image,
        (DEBUG_DISPLAY_WIDTH, DEBUG_DISPLAY_HEIGHT),
        interpolation=cv2.INTER_NEAREST,
    )


def build_debug_overlay(
    frame: np.ndarray,
    detection: BallBalanceDetection,
) -> np.ndarray:
    """Build a frame that shows all plate candidates and reject reasons."""
    overlay = frame.copy()
    debug = detection.debug
    if debug is None:
        return overlay
    for candidate in select_debug_candidates(debug.plate_candidates):
        draw_plate_candidate_debug(overlay, candidate)
    return overlay


def select_debug_candidates(
    candidates: list[PlateCandidateDebug],
) -> list[PlateCandidateDebug]:
    """Keep the candidate overlay readable."""
    accepted = []
    rejected = []
    for candidate in candidates:
        if candidate.accepted:
            accepted.append(candidate)
        elif candidate.reason == "roi":
            continue
        else:
            rejected.append(candidate)
    rejected.sort(key=lambda item: item.detection.area, reverse=True)
    return accepted[:8] + rejected[:8]


def draw_plate_candidate_debug(
    frame: np.ndarray,
    candidate: PlateCandidateDebug,
) -> None:
    """Draw one plate candidate with its acceptance state and metrics."""
    detection = candidate.detection
    color = (0, 220, 0) if candidate.accepted else (0, 120, 255)
    x, y, width, height = detection.bbox
    cv2.circle(frame, round_point(detection.center),
               int(round(detection.radius)), color, 2)
    cv2.rectangle(frame, (x, y), (x + width, y + height), color, 1)
    label = (
        f"{candidate.reason} "
        f"x={detection.center[0]:.2f} y={detection.center[1]:.2f} "
        f"r={detection.radius:.2f} "
        f"area={detection.area:.0f}"
    )
    if candidate.score is not None:
        label += f" s={candidate.score:.0f}"
    if candidate.edge_support is not None:
        label += f" e={candidate.edge_support:.2f}"
    if candidate.color_ratio is not None:
        label += f" c={candidate.color_ratio:.2f}"
    cv2.putText(
        frame,
        label,
        (x, max(16, y - 6)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        color,
        1,
        cv2.LINE_AA,
    )


def run_preview(node: BallBalanceNode) -> int:
    """Run the OpenCV preview loop while spinning the ROS node."""
    cv2.namedWindow(node.window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(node.window_name, node.display_width, node.display_height)
    tuning_panel = TuningPanel(
        TUNING_WINDOW_NAME,
        node.get_detection_config(),
        node.motion.snapshot()["config"],
    )
    node.get_logger().info("Press q or Esc in the preview window to exit.")
    spin_stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=spin_node_until_stopped,
        args=(node, spin_stop_event),
        name="easyarm_ball_balance_spin",
        daemon=True,
    )
    spin_thread.start()
    last_measurement_frame_count = 0
    last_processed_frame_count = 0
    last_detection = None
    last_tuning_draw_time = 0.0
    try:
        while rclpy.ok():
            tuning_panel.sync_to_node(node)
            now = time.monotonic()
            detection_config = node.get_detection_config()
            motion_config = node.motion.snapshot()["config"]
            if now - last_tuning_draw_time >= 0.1:
                tuning_panel.draw_values(detection_config, motion_config)
                last_tuning_draw_time = now

            frame, stamp, frame_count, fps = node.get_frame()
            if frame is None or frame_count == last_processed_frame_count:
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    return 0
                handle_key(node, key, last_detection)
                continue

            last_processed_frame_count = frame_count
            node.capture.save(frame, frame_count)
            source_size = (frame.shape[1], frame.shape[0])
            preview = resize_frame(
                frame,
                node.display_width,
                node.display_height,
            )
            detection = detect_objects(preview, detection_config)
            last_detection = detection
            draw_detection(
                preview,
                detection,
                detection_config,
                motion_config,
            )
            set_last_preview_frame(preview)
            if frame_count != last_measurement_frame_count:
                node.motion.update_measurement(detection.offset)
                last_measurement_frame_count = frame_count
                node.control_logger.write(
                    frame_count,
                    fps,
                    stamp,
                    detection,
                    node.motion,
                    node.get_joint_state(),
                )
            status_lines = build_status_lines(
                frame_count,
                fps,
                source_size,
                stamp,
                detection,
                node.motion,
                motion_config,
            )
            draw_text_lines(preview, status_lines, (14, 28))
            cv2.imshow(node.window_name, preview)
            if node.debug_mask:
                show_debug_masks(detection)
                show_candidate_debug(preview, detection)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                return 0
            handle_key(node, key, detection)
        return 0
    finally:
        spin_stop_event.set()
        spin_thread.join(timeout=1.0)


def spin_node_until_stopped(
    node: BallBalanceNode,
    stop_event: threading.Event,
) -> None:
    """Spin ROS callbacks in the background so UI work cannot block them."""
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.02)


def handle_key(
    node: BallBalanceNode,
    key: int,
    detection: BallBalanceDetection | None,
) -> None:
    """Handle keyboard shortcuts from the OpenCV preview."""
    if key == ord(" "):
        node.motion.move_to_named_state()
    elif key in (ord("b"), ord("B")):
        offset = None if detection is None else detection.offset
        node.motion.send_balance_step(offset)
    elif key in (ord("o"), ord("O")):
        node.motion.start_servol()
    elif key in (ord("p"), ord("P")):
        node.motion.stop_servol()
    else:
        handle_capture_key(node, key)


def handle_capture_key(node: BallBalanceNode, key: int) -> None:
    """Handle capture keyboard shortcuts."""
    if key == ord("c"):
        if node.capture.active:
            return
        path = node.capture.start()
        node.get_logger().info(f"Started capture: {path}")
    elif key == ord("s"):
        if not node.capture.active:
            return
        saved_count = node.capture.stop()
        node.get_logger().info(f"Stopped capture, saved {saved_count} frames")


def set_last_preview_frame(frame: np.ndarray) -> None:
    """Store the latest preview frame for debug panel rendering."""
    global last_preview_frame
    last_preview_frame = frame.copy()


def show_candidate_debug(
    frame: np.ndarray,
    detection: BallBalanceDetection,
) -> None:
    """Show plate candidates in a full-size debug window."""
    candidate_overlay = build_debug_overlay(frame, detection)
    cv2.imshow("07_plate_candidates", candidate_overlay)


def main() -> int:
    """Start the ball balance image preview and detector."""
    args = build_parser().parse_args()
    if args.width <= 0 or args.height <= 0:
        raise SystemExit("--width and --height must be positive")
    if args.ball_plate_inner_scale <= 0.0:
        raise SystemExit("--ball-plate-inner-scale must be positive")
    if args.servol_rate_hz <= 0.0:
        raise SystemExit("--servol-rate-hz must be positive")
    if not 0.0 < args.balance_filter_alpha <= 1.0:
        raise SystemExit("--balance-filter-alpha must be in (0, 1]")
    if args.balance_max_measurement_age_sec <= 0.0:
        raise SystemExit("--balance-max-measurement-age-sec must be positive")
    if args.balance_delay_compensation_sec < 0.0:
        raise SystemExit(
            "--balance-delay-compensation-sec must be nonnegative"
        )
    if args.balance_integral_gain_deg < 0.0:
        raise SystemExit("--balance-integral-gain-deg must be nonnegative")
    if args.balance_integral_limit_deg < 0.0:
        raise SystemExit("--balance-integral-limit-deg must be nonnegative")
    if args.balance_integral_radius <= 0.0:
        raise SystemExit("--balance-integral-radius must be positive")
    if args.balance_integral_speed < 0.0:
        raise SystemExit("--balance-integral-speed must be nonnegative")
    if not -1.0 <= args.balance_target_offset_x <= 1.0:
        raise SystemExit("--balance-target-offset-x must be in [-1, 1]")
    if not -1.0 <= args.balance_target_offset_y <= 1.0:
        raise SystemExit("--balance-target-offset-y must be in [-1, 1]")
    if args.balance_velocity_window_size < 2:
        raise SystemExit("--balance-velocity-window-size must be >= 2")
    if args.balance_center_radius <= 0.0:
        raise SystemExit("--balance-center-radius must be positive")
    if args.balance_center_exit_radius < args.balance_center_radius:
        raise SystemExit(
            "--balance-center-exit-radius must be >= "
            "--balance-center-radius"
        )
    if args.balance_center_speed_exit_velocity < 0.0:
        raise SystemExit(
            "--balance-center-speed-exit-velocity must be nonnegative"
        )
    if (
        args.balance_center_speed_enter_velocity
        < args.balance_center_speed_exit_velocity
    ):
        raise SystemExit(
            "--balance-center-speed-enter-velocity must be >= "
            "--balance-center-speed-exit-velocity"
        )
    if args.balance_center_angle_limit_deg <= 0.0:
        raise SystemExit("--balance-center-angle-limit-deg must be positive")
    if not 0.0 <= args.balance_center_position_scale <= 1.0:
        raise SystemExit(
            "--balance-center-position-scale must be in [0, 1]"
        )
    if args.balance_center_radial_d_gain_deg < 0.0:
        raise SystemExit(
            "--balance-center-radial-d-gain-deg must be nonnegative"
        )
    if args.balance_center_tangent_d_gain_deg < 0.0:
        raise SystemExit(
            "--balance-center-tangent-d-gain-deg must be nonnegative"
        )
    if args.balance_recovery_radius <= args.balance_recovery_exit_radius:
        raise SystemExit(
            "--balance-recovery-radius must be greater than "
            "--balance-recovery-exit-radius"
        )
    if args.balance_recovery_tangent_enter_velocity < 0.0:
        raise SystemExit(
            "--balance-recovery-tangent-enter-velocity must be nonnegative"
        )
    if args.balance_recovery_tangent_exit_velocity < 0.0:
        raise SystemExit(
            "--balance-recovery-tangent-exit-velocity must be nonnegative"
        )
    if args.balance_recovery_radial_exit_velocity < 0.0:
        raise SystemExit(
            "--balance-recovery-radial-exit-velocity must be nonnegative"
        )
    if args.balance_recovery_speed_exit_velocity < 0.0:
        raise SystemExit(
            "--balance-recovery-speed-exit-velocity must be nonnegative"
        )
    if args.balance_tangent_d_gain_deg < 0.0:
        raise SystemExit("--balance-tangent-d-gain-deg must be nonnegative")
    if args.balance_recovery_angle_limit_deg <= 0.0:
        raise SystemExit(
            "--balance-recovery-angle-limit-deg must be positive"
        )
    if args.balance_tilt_rate_limit_deg_s < 0.0:
        raise SystemExit(
            "--balance-tilt-rate-limit-deg-s must be nonnegative"
        )

    rclpy.init()
    node = BallBalanceNode(args)
    try:
        return run_preview(node)
    except KeyboardInterrupt:
        return 0
    finally:
        node.motion.shutdown()
        node.control_logger.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
