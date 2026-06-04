"""Collect synchronized chessboard images and joint states for zero calibration."""

from pathlib import Path
import json
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from easyarm_calib.camera_preview import (
    ACQUIRE_TIMEOUT_MS,
    CAMERA_INDEX,
    MAX_PREVIEW_SIZE,
    TerminalKeyReader,
    _color_improvement_params,
    _configure_camera,
    _import_gxipy,
    _raw_image_to_bgr,
    _read_key,
    _resize_for_display,
)
from easyarm_calib.joint_zero_vision_common import (
    CAMERA_INTRINSICS,
    CAMERA_MODEL,
    CAMERA_SERIAL,
    CHESSBOARD_COLS,
    CHESSBOARD_ROWS,
    JOINT_NAMES,
    SQUARE_SIZE_M,
    find_chessboard_corners,
)


WINDOW_NAME = "Joint Zero Vision Collect"
OUTPUT_ROOT = Path("data/joint_zero_vision")
PREVIEW_DETECT_INTERVAL_SEC = 0.25


class JointStateCache(Node):
    """Store the latest complete Joint1-Joint6 state."""

    def __init__(self):
        super().__init__("joint_zero_vision_collector")
        self._lock = threading.Lock()
        self._joints = None
        self._stamp_sec = None
        self.create_subscription(JointState, "/joint_states", self._callback, 10)

    def _callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in JOINT_NAMES):
            return
        with self._lock:
            self._joints = {name: float(positions[name]) for name in JOINT_NAMES}
            self._stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def latest(self):
        with self._lock:
            if self._joints is None:
                return None, None
            return dict(self._joints), self._stamp_sec


def _create_output_dir():
    now = time.localtime()
    output_dir = OUTPUT_ROOT / time.strftime("%Y%m%d", now) / time.strftime("%H%M%S", now)
    (output_dir / "images").mkdir(parents=True, exist_ok=True)
    (output_dir / "corners_preview").mkdir(parents=True, exist_ok=True)
    return output_dir


def _draw_status(image, found, sample_count, has_joints):
    color = (0, 255, 0) if found and has_joints else (0, 0, 255)
    text = "corners: OK" if found else "corners: NOT FOUND"
    joint_text = "joints: OK" if has_joints else "joints: WAITING"
    cv2.putText(image, text, (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
    cv2.putText(image, joint_text, (30, 85), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
    cv2.putText(
        image,
        f"saved: {sample_count}  c/C=save  q/Q/Esc=quit",
        (30, 125),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return image


def _write_samples(output_dir, samples):
    data = {
        "camera_model": CAMERA_MODEL,
        "camera_serial": CAMERA_SERIAL,
        "camera_intrinsics": str(CAMERA_INTRINSICS),
        "board": {
            "type": "chessboard",
            "cols": CHESSBOARD_COLS,
            "rows": CHESSBOARD_ROWS,
            "square_size_m": SQUARE_SIZE_M,
        },
        "robot": {
            "base_frame": "base_link",
            "flange_frame": "Link6",
            "joints": JOINT_NAMES,
        },
        "samples": samples,
    }
    (output_dir / "samples.json").write_text(json.dumps(data, indent=2), encoding="utf-8")


def _scaled_corners_to_full_resolution(corners, scale):
    if corners is None or scale == 1.0:
        return corners
    return corners / scale


def _make_preview_frame(frame):
    height, width = frame.shape[:2]
    longest_side = max(width, height)
    if longest_side <= MAX_PREVIEW_SIZE:
        return frame.copy(), 1.0
    scale = MAX_PREVIEW_SIZE / longest_side
    preview = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return preview, scale


def main():
    """Run the image + joint-state collector."""
    rclpy.init()
    node = JointStateCache()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    gx = _import_gxipy()
    device_manager = gx.DeviceManager()
    dev_num, dev_info_list = device_manager.update_device_list()
    if dev_num == 0:
        print("No Daheng Galaxy camera found")
        rclpy.shutdown()
        return

    print(f"Found {dev_num} camera(s)")
    for index, info in enumerate(dev_info_list, start=1):
        print(f"[{index}] {info}")

    output_dir = _create_output_dir()
    print(f"Output directory: {output_dir}")
    print("Press c/C to save only when corners and joints are both OK.")
    print("Press q/Q or Esc to exit.")

    cam = device_manager.open_device_by_index(CAMERA_INDEX)
    terminal_reader = TerminalKeyReader()
    stream_on = False
    samples = []

    try:
        color_camera = cam.PixelColorFilter.is_implemented()
        _configure_camera(gx, cam)
        improvement_params = _color_improvement_params(gx, cam) if color_camera else None

        cam.stream_on()
        stream_on = True
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        last_preview_detect_time = 0.0
        preview_found = False
        preview_corners = None

        while True:
            raw_image = cam.data_stream[0].get_image(timeout=ACQUIRE_TIMEOUT_MS)
            if raw_image is None:
                print("Getting image failed")
                continue

            frame = _raw_image_to_bgr(raw_image, color_camera, improvement_params)
            if frame is None:
                print("Converting image failed")
                continue

            joints, joint_stamp = node.latest()
            preview, preview_scale = _make_preview_frame(frame)
            now = time.monotonic()
            if now - last_preview_detect_time >= PREVIEW_DETECT_INTERVAL_SEC:
                preview_found, preview_corners = find_chessboard_corners(preview)
                last_preview_detect_time = now

            if preview_found:
                cv2.drawChessboardCorners(
                    preview,
                    (CHESSBOARD_COLS, CHESSBOARD_ROWS),
                    preview_corners,
                    True,
                )
            preview = _draw_status(preview, preview_found, len(samples), joints is not None)
            cv2.imshow(WINDOW_NAME, preview)

            key = _read_key(terminal_reader)
            if key in ("\x1b", "q", "Q"):
                break
            if key in ("c", "C"):
                found, corners = find_chessboard_corners(frame)
                if not found and preview_found:
                    corners = _scaled_corners_to_full_resolution(preview_corners, preview_scale)
                    found = corners is not None
                if not found:
                    print("Skip capture: chessboard corners not found")
                    continue
                if joints is None:
                    print("Skip capture: no complete Joint1-Joint6 /joint_states yet")
                    continue

                index = len(samples) + 1
                image_name = f"IMG{index:04d}.png"
                image_rel = f"images/{image_name}"
                preview_rel = f"corners_preview/{image_name}"
                corner_image = frame.copy()
                cv2.drawChessboardCorners(corner_image, (CHESSBOARD_COLS, CHESSBOARD_ROWS), corners, True)
                cv2.imwrite(str(output_dir / image_rel), frame)
                cv2.imwrite(str(output_dir / preview_rel), corner_image)

                sample = {
                    "image": image_rel,
                    "corners_preview": preview_rel,
                    "frame_id": int(raw_image.get_frame_id()),
                    "joint_state_stamp_sec": joint_stamp,
                    "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "joints": joints,
                    "corners": corners.tolist(),
                }
                samples.append(sample)
                _write_samples(output_dir, samples)
                print(f"Saved sample {index}: {output_dir / image_rel}")

    except KeyboardInterrupt:
        print("Collection interrupted")
    finally:
        _write_samples(output_dir, samples)
        terminal_reader.close()
        if stream_on:
            cam.stream_off()
        cam.close_device()
        cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print(f"Saved {len(samples)} sample(s) to {output_dir}")


if __name__ == "__main__":
    main()
