#!/usr/bin/env python3
"""Analyze and smooth recorded EasyArm joint trajectories."""

from __future__ import annotations

import argparse
import copy
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_WINDOW_SEC = 0.15
DEFAULT_URDF_RELATIVE_PATH = Path(
    "src/easyarm_description/urdf/easyarm_a1_h0616.urdf")


@dataclass(frozen=True)
class RecordData:
    """Validated trajectory record data."""

    data: dict[str, Any]
    times: np.ndarray
    joints: np.ndarray
    joint_names: list[str]


@dataclass(frozen=True)
class Metrics:
    """Trajectory jitter and smoothing metrics."""

    jitter_index: float
    joint_jitter: list[float]
    raw_path_length: float
    smooth_path_length: float
    jerk_rms_raw: float
    jerk_rms_smooth: float
    jerk_rms_ratio: float
    residual_rms: float
    residual_max: float


def load_record(record_path: Path) -> RecordData:
    """Load and validate an EasyArm record JSON."""
    with record_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    if not isinstance(data, dict):
        raise ValueError("Record JSON root must be an object")

    joint_names = data.get("joint_names")
    if not isinstance(joint_names, list) or not joint_names:
        raise ValueError("JSON field 'joint_names' must be a non-empty list")
    joint_names = [str(name) for name in joint_names]

    samples = data.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("JSON field 'samples' must be a non-empty list")

    times: list[float] = []
    joints: list[list[float]] = []
    joint_count = len(joint_names)

    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise ValueError(f"samples[{index}] must be an object")
        if "t" not in sample:
            raise ValueError(f"samples[{index}] missing field 't'")
        if "joints" not in sample:
            raise ValueError(f"samples[{index}] missing field 'joints'")

        t = float(sample["t"])
        if not math.isfinite(t) or t < 0.0:
            raise ValueError(f"samples[{index}].t must be a finite non-negative value")

        sample_joints = sample["joints"]
        if not isinstance(sample_joints, list) or len(sample_joints) != joint_count:
            raise ValueError(
                f"samples[{index}].joints must contain {joint_count} values")

        parsed_joints = [float(value) for value in sample_joints]
        if not all(math.isfinite(value) for value in parsed_joints):
            raise ValueError(f"samples[{index}].joints contains a non-finite value")

        if times and t <= times[-1]:
            raise ValueError("Sample timestamps must be strictly increasing")

        times.append(t)
        joints.append(parsed_joints)

    return RecordData(
        data=data,
        times=np.asarray(times, dtype=float),
        joints=np.asarray(joints, dtype=float),
        joint_names=joint_names,
    )


def output_path_for(record_path: Path) -> Path:
    """Return the default sibling output path for a smoothed record."""
    suffix = record_path.suffix if record_path.suffix else ".json"
    stem = record_path.stem if record_path.suffix else record_path.name
    return record_path.with_name(f"{stem}_smooth{suffix}")


def odd_window_size(window_sec: float, sample_period: float) -> int:
    """Convert a smoothing window in seconds to an odd sample count."""
    if not math.isfinite(window_sec) or window_sec <= 0.0:
        raise ValueError("--window-sec must be positive")
    if not math.isfinite(sample_period) or sample_period <= 0.0:
        raise ValueError("Sample period must be positive")

    window_size = max(1, int(round(window_sec / sample_period)))
    if window_size % 2 == 0:
        window_size += 1
    return window_size


def reflect_index(index: int, size: int) -> int:
    """Reflect an array index around the boundaries."""
    if size <= 1:
        return 0
    while index < 0 or index >= size:
        if index < 0:
            index = -index
        if index >= size:
            index = 2 * size - 2 - index
    return index


def hann_weights(window_size: int) -> np.ndarray:
    """Return normalized Hann window weights."""
    if window_size <= 1:
        return np.ones(1, dtype=float)
    weights = np.hanning(window_size)
    total = float(np.sum(weights))
    if total <= 0.0:
        return np.ones(window_size, dtype=float) / float(window_size)
    return weights / total


def smooth_joints(joints: np.ndarray, window_size: int) -> np.ndarray:
    """Smooth joint positions with a centered Hann window."""
    if window_size <= 1 or len(joints) <= 2:
        return joints.copy()

    weights = hann_weights(window_size)
    half_window = window_size // 2
    smoothed = np.empty_like(joints)

    for index in range(len(joints)):
        smoothed[index, :] = 0.0
        for offset, weight in enumerate(weights):
            source_index = reflect_index(index + offset - half_window, len(joints))
            smoothed[index, :] += weight * joints[source_index, :]

    smoothed[0, :] = joints[0, :]
    smoothed[-1, :] = joints[-1, :]
    return smoothed


def path_length(joints: np.ndarray) -> float:
    """Return joint-space path length using L2 distance per segment."""
    if len(joints) < 2:
        return 0.0
    deltas = np.diff(joints, axis=0)
    return float(np.sum(np.linalg.norm(deltas, axis=1)))


def joint_path_lengths(joints: np.ndarray) -> np.ndarray:
    """Return per-joint absolute path lengths."""
    if len(joints) < 2:
        return np.zeros(joints.shape[1], dtype=float)
    return np.sum(np.abs(np.diff(joints, axis=0)), axis=0)


def finite_difference_jerk_rms(joints: np.ndarray, times: np.ndarray) -> float:
    """Return RMS norm of the third finite difference divided by dt^3."""
    if len(joints) < 4:
        return 0.0

    sample_period = float(np.median(np.diff(times)))
    if sample_period <= 0.0:
        return 0.0

    third_diff = (
        joints[3:, :]
        - 3.0 * joints[2:-1, :]
        + 3.0 * joints[1:-2, :]
        - joints[:-3, :]
    ) / (sample_period ** 3)
    jerk_norm = np.linalg.norm(third_diff, axis=1)
    return float(math.sqrt(float(np.mean(jerk_norm ** 2))))


def compute_metrics(raw_joints: np.ndarray, smooth: np.ndarray, times: np.ndarray) -> Metrics:
    """Compute jitter and smoothing metrics."""
    raw_length = path_length(raw_joints)
    smooth_length = path_length(smooth)
    jitter_index = (
        100.0 * max(0.0, raw_length - smooth_length) / raw_length
        if raw_length > 0.0 else 0.0
    )

    raw_joint_lengths = joint_path_lengths(raw_joints)
    smooth_joint_lengths = joint_path_lengths(smooth)
    joint_jitter = []
    for raw_length_axis, smooth_length_axis in zip(raw_joint_lengths, smooth_joint_lengths):
        if raw_length_axis > 0.0:
            value = 100.0 * max(0.0, raw_length_axis - smooth_length_axis) / raw_length_axis
        else:
            value = 0.0
        joint_jitter.append(float(value))

    residual = smooth - raw_joints
    residual_rms = float(math.sqrt(float(np.mean(residual ** 2))))
    residual_max = float(np.max(np.abs(residual)))

    jerk_raw = finite_difference_jerk_rms(raw_joints, times)
    jerk_smooth = finite_difference_jerk_rms(smooth, times)
    jerk_ratio = jerk_smooth / jerk_raw if jerk_raw > 0.0 else 0.0

    return Metrics(
        jitter_index=float(jitter_index),
        joint_jitter=joint_jitter,
        raw_path_length=raw_length,
        smooth_path_length=smooth_length,
        jerk_rms_raw=jerk_raw,
        jerk_rms_smooth=jerk_smooth,
        jerk_rms_ratio=float(jerk_ratio),
        residual_rms=residual_rms,
        residual_max=residual_max,
    )


def find_workspace_root(start: Path) -> Path | None:
    """Find the nearest colcon workspace root from a path."""
    current = start.resolve()
    if current.is_file():
        current = current.parent
    for candidate in (current, *current.parents):
        if (candidate / "src").is_dir():
            return candidate
    return None


def resolve_default_urdf(record_path: Path) -> Path:
    """Resolve the default EasyArm URDF path."""
    workspace_root = find_workspace_root(record_path)
    if workspace_root is not None:
        source_urdf = workspace_root / DEFAULT_URDF_RELATIVE_PATH
        if source_urdf.is_file():
            return source_urdf

    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory("easyarm_description"))
        installed_urdf = share_dir / "urdf/easyarm_a1_h0616.urdf"
        if installed_urdf.is_file():
            return installed_urdf
    except Exception:
        pass

    raise FileNotFoundError(
        "Could not find easyarm_a1_h0616.urdf; pass --urdf explicitly")


def recompute_ee_poses(
    data: dict[str, Any],
    joints: np.ndarray,
    joint_names: list[str],
    urdf_path: Path,
) -> None:
    """Recompute samples[].ee_pose from smoothed joint positions using FK."""
    import pinocchio as pin

    model = pin.buildModelFromUrdf(str(urdf_path))
    if model.nq != joints.shape[1]:
        raise ValueError(
            f"URDF model nq={model.nq} does not match joint count {joints.shape[1]}")

    for index, joint_name in enumerate(joint_names):
        model_joint_id = model.getJointId(joint_name)
        if model_joint_id == 0:
            raise ValueError(f"URDF does not contain joint '{joint_name}'")
        if model.names[model_joint_id] != joint_name:
            raise ValueError(
                f"Unexpected URDF joint order at {index}: {model.names[model_joint_id]}")

    ee_frame = str(data.get("ee_frame", "Link6"))
    frame_id = model.getFrameId(ee_frame)
    if frame_id >= len(model.frames):
        raise ValueError(f"URDF does not contain end-effector frame '{ee_frame}'")

    pin_data = model.createData()
    samples = data["samples"]
    for index, sample_joints in enumerate(joints):
        pin.forwardKinematics(model, pin_data, sample_joints)
        pin.updateFramePlacements(model, pin_data)
        placement = pin_data.oMf[frame_id]
        quaternion = pin.Quaternion(placement.rotation)
        quaternion.normalize()

        samples[index]["ee_pose"] = {
            "translation": [
                float(placement.translation[0]),
                float(placement.translation[1]),
                float(placement.translation[2]),
            ],
            "rotation": [
                float(quaternion.x),
                float(quaternion.y),
                float(quaternion.z),
                float(quaternion.w),
            ],
        }


def update_output_data(
    record: RecordData,
    smoothed_joints: np.ndarray,
    metrics: Metrics,
    window_sec: float,
    window_size: int,
    recomputed_ee_pose: bool,
    urdf_path: Path | None,
) -> dict[str, Any]:
    """Build output JSON data with smoothed joints and metadata."""
    output_data = copy.deepcopy(record.data)
    for sample, sample_joints in zip(output_data["samples"], smoothed_joints):
        sample["joints"] = [float(value) for value in sample_joints]

    smoothing_info = {
        "method": "centered_hann",
        "window_sec": float(window_sec),
        "window_samples": int(window_size),
        "preserve_endpoints": True,
        "source_joints": "samples[].joints",
        "ee_pose_recomputed": bool(recomputed_ee_pose),
        "jitter_index": metrics.jitter_index,
        "joint_jitter": {
            name: value for name, value in zip(record.joint_names, metrics.joint_jitter)
        },
        "raw_path_length_rad": metrics.raw_path_length,
        "smooth_path_length_rad": metrics.smooth_path_length,
        "jerk_rms_raw": metrics.jerk_rms_raw,
        "jerk_rms_smooth": metrics.jerk_rms_smooth,
        "jerk_rms_ratio": metrics.jerk_rms_ratio,
        "residual_rms_rad": metrics.residual_rms,
        "residual_max_rad": metrics.residual_max,
    }
    if urdf_path is not None:
        smoothing_info["urdf"] = str(urdf_path)
    if not recomputed_ee_pose:
        smoothing_info["ee_pose_note"] = "Original ee_pose values were preserved"

    output_data["smoothing"] = smoothing_info
    return output_data


def print_report(
    record_path: Path,
    output_path: Path,
    record: RecordData,
    metrics: Metrics,
    window_sec: float,
    window_size: int,
    recomputed_ee_pose: bool,
    urdf_path: Path | None,
) -> None:
    """Print a concise smoothing report."""
    duration = float(record.times[-1] - record.times[0]) if len(record.times) > 1 else 0.0
    sample_period = float(np.median(np.diff(record.times))) if len(record.times) > 1 else 0.0
    sample_rate = 1.0 / sample_period if sample_period > 0.0 else 0.0

    print(f"Input: {record_path}")
    print(f"Output: {output_path}")
    print(
        f"Samples: {len(record.times)}, duration: {duration:.3f} s, "
        f"sample_rate: {sample_rate:.3f} Hz")
    print(
        f"Smoothing: Hann window {window_sec:.3f} s "
        f"({window_size} samples), endpoints preserved")
    print(f"Jitter index: {metrics.jitter_index:.2f}%")
    print("Per-joint jitter:")
    for name, value in zip(record.joint_names, metrics.joint_jitter):
        print(f"  {name}: {value:.2f}%")
    print(
        "Path length: "
        f"raw {metrics.raw_path_length:.6f} rad, "
        f"smooth {metrics.smooth_path_length:.6f} rad")
    print(
        "Jerk RMS: "
        f"raw {metrics.jerk_rms_raw:.6g}, "
        f"smooth {metrics.jerk_rms_smooth:.6g}, "
        f"ratio {metrics.jerk_rms_ratio:.4f}")
    print(
        "Residual: "
        f"rms {metrics.residual_rms:.6g} rad "
        f"({math.degrees(metrics.residual_rms):.4f} deg), "
        f"max {metrics.residual_max:.6g} rad "
        f"({math.degrees(metrics.residual_max):.4f} deg)")
    if recomputed_ee_pose:
        print(f"ee_pose: recomputed from FK using {urdf_path}")
    else:
        print("ee_pose: preserved from input")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Analyze jitter and smooth samples[].joints in an EasyArm record JSON")
    parser.add_argument("record", type=Path, help="Path to EasyArm record JSON")
    parser.add_argument(
        "-o", "--output", type=Path,
        help="Output JSON path; defaults to '<input_stem>_smooth.json'")
    parser.add_argument(
        "--window-sec", type=float, default=DEFAULT_WINDOW_SEC,
        help=f"Hann smoothing window in seconds (default: {DEFAULT_WINDOW_SEC})")
    parser.add_argument(
        "--urdf", type=Path,
        help="URDF path for FK ee_pose recomputation")
    parser.add_argument(
        "--no-recompute-ee-pose", action="store_true",
        help="Preserve input ee_pose values instead of recomputing them from FK")
    parser.add_argument(
        "--indent", type=int, default=2,
        help="JSON indentation for output; use 0 for compact JSON")
    return parser.parse_args()


def main() -> None:
    """Analyze and smooth an EasyArm trajectory JSON."""
    args = parse_args()
    record_path = args.record.expanduser()
    if not record_path.is_file():
        raise FileNotFoundError(f"Record file not found: {record_path}")

    output_path = (
        args.output.expanduser()
        if args.output else output_path_for(record_path)
    )

    record = load_record(record_path)
    sample_period = float(np.median(np.diff(record.times))) if len(record.times) > 1 else 0.0
    window_size = odd_window_size(args.window_sec, sample_period)
    smoothed_joints = smooth_joints(record.joints, window_size)
    metrics = compute_metrics(record.joints, smoothed_joints, record.times)

    recompute_ee_pose = not args.no_recompute_ee_pose
    urdf_path = None
    if recompute_ee_pose:
        urdf_path = args.urdf.expanduser() if args.urdf else resolve_default_urdf(record_path)
        if not urdf_path.is_file():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    output_data = update_output_data(
        record,
        smoothed_joints,
        metrics,
        args.window_sec,
        window_size,
        recompute_ee_pose,
        urdf_path,
    )
    if recompute_ee_pose:
        recompute_ee_poses(output_data, smoothed_joints, record.joint_names, urdf_path)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    indent = None if args.indent == 0 else args.indent
    with output_path.open("w", encoding="utf-8") as file:
        json.dump(output_data, file, indent=indent)
        file.write("\n")

    print_report(
        record_path,
        output_path,
        record,
        metrics,
        args.window_sec,
        window_size,
        recompute_ee_pose,
        urdf_path,
    )


if __name__ == "__main__":
    main()
