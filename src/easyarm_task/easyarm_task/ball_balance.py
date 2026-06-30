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

"""Detect the plate and dark ball for the EasyArm ball balance task."""

import argparse
from collections import deque
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
        default=20.0,
        help="Tilt degrees per normalized ball offset.",
    )
    parser.add_argument(
        "--balance-angle-d-gain-deg",
        type=float,
        default=2.0,
        help="Tilt degrees per normalized ball offset velocity.",
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
        help="Low-pass alpha for ServoL visual offset filtering.",
    )
    parser.add_argument(
        "--balance-max-measurement-age-sec",
        type=float,
        default=0.2,
        help="Maximum visual measurement age before ServoL commands origin.",
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
        default=200.0,
        help=(
            "ServoL target publish rate when O key starts "
            "continuous control."
        ),
    )
    parser.add_argument(
        "--plate-min-radius",
        type=float,
        default=75.0,
        help="Minimum plate radius in preview pixels.",
    )
    parser.add_argument(
        "--plate-max-radius",
        type=float,
        default=115.0,
        help="Maximum plate radius in preview pixels.",
    )
    parser.add_argument(
        "--plate-roi-x-min",
        type=float,
        default=0.3,
        help="Normalized left bound of the plate search ROI.",
    )
    parser.add_argument(
        "--plate-roi-x-max",
        type=float,
        default=0.77,
        help="Normalized right bound of the plate search ROI.",
    )
    parser.add_argument(
        "--plate-roi-y-min",
        type=float,
        default=0.31,
        help="Normalized top bound of the plate search ROI.",
    )
    parser.add_argument(
        "--plate-roi-y-max",
        type=float,
        default=0.86,
        help="Normalized bottom bound of the plate search ROI.",
    )
    parser.add_argument(
        "--plate-expected-x",
        type=float,
        default=0.525,
        help="Expected normalized plate center x used as a soft prior.",
    )
    parser.add_argument(
        "--plate-expected-y",
        type=float,
        default=0.595,
        help="Expected normalized plate center y used as a soft prior.",
    )
    parser.add_argument(
        "--plate-expected-radius",
        type=float,
        default=90.5,
        help="Expected plate radius in preview pixels.",
    )
    parser.add_argument(
        "--ball-min-area",
        type=float,
        default=120.0,
        help="Minimum contour area for the dark ball.",
    )
    parser.add_argument(
        "--ball-max-area",
        type=float,
        default=4500.0,
        help="Maximum contour area for the dark ball.",
    )
    parser.add_argument(
        "--ball-min-radius",
        type=float,
        default=8.0,
        help="Minimum ball radius in preview pixels.",
    )
    parser.add_argument(
        "--ball-max-radius",
        type=float,
        default=45.0,
        help="Maximum ball radius in preview pixels.",
    )
    parser.add_argument(
        "--ball-max-value",
        type=int,
        default=95,
        help="Maximum HSV value used to segment the dark ball.",
    )
    parser.add_argument(
        "--ball-min-circularity",
        type=float,
        default=0.15,
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
    return DetectionConfig(
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
        camera_to_plate_yaw_deg=args.camera_to_plate_yaw_deg,
        filter_alpha=args.balance_filter_alpha,
        max_measurement_age_sec=args.balance_max_measurement_age_sec,
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
        self.detector_config = detection_config_from_args(args)
        self.motion = BallBalanceMotionController(
            self,
            motion_config_from_args(args),
        )
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = None
        self.latest_fps = 0.0
        self.frame_count = 0
        self.frame_times = deque(maxlen=90)
        self.last_log_time = time.monotonic()

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            10,
        )
        self.get_logger().info(f"Subscribing image topic: {self.image_topic}")

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
    camera_to_plate_yaw_deg: float,
) -> None:
    """Draw plate and ball detections on the frame."""
    draw_plate_roi(frame, config)
    if detection.plate is not None:
        draw_circle_detection(frame, detection.plate, (0, 0, 255), "plate")
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
            camera_to_plate_yaw_deg,
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
    camera_to_plate_yaw_deg: float,
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
            camera_to_plate_yaw_deg,
        )
        lines.append(f"raw offset: x={dx:+.3f}, y={dy:+.3f}")
        lines.append(
            f"plate offset: x={plate_dx:+.3f}, y={plate_dy:+.3f}, "
            f"yaw={camera_to_plate_yaw_deg:+.1f}deg"
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
        make_debug_tile("01 gray", debug.gray),
        make_debug_tile("02 roi", debug.roi_mask),
        make_debug_tile("03 edges", debug.edge_mask),
        make_debug_tile("04 plate", debug.plate_mask),
    ]
    if debug.ball_mask is not None:
        tiles.append(make_debug_tile("05 ball", debug.ball_mask))
    else:
        tiles.append(make_blank_debug_tile("05 ball"))
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
    node.get_logger().info("Press q or Esc in the preview window to exit.")
    last_measurement_frame_count = 0

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        frame, stamp, frame_count, fps = node.get_frame()
        if frame is None:
            key = cv2.waitKey(10) & 0xFF
            if key in (27, ord("q")):
                return 0
            handle_key(node, key, None)
            continue

        node.capture.save(frame, frame_count)
        source_size = (frame.shape[1], frame.shape[0])
        preview = resize_frame(
            frame,
            node.display_width,
            node.display_height,
        )
        detection = detect_objects(preview, node.detector_config)
        draw_detection(
            preview,
            detection,
            node.detector_config,
            node.motion.config.camera_to_plate_yaw_deg,
        )
        set_last_preview_frame(preview)
        if frame_count != last_measurement_frame_count:
            node.motion.update_measurement(detection.offset)
            last_measurement_frame_count = frame_count
        status_lines = build_status_lines(
            frame_count,
            fps,
            source_size,
            stamp,
            detection,
            node.motion,
            node.motion.config.camera_to_plate_yaw_deg,
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

    rclpy.init()
    node = BallBalanceNode(args)
    try:
        return run_preview(node)
    except KeyboardInterrupt:
        return 0
    finally:
        node.motion.shutdown()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
