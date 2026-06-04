"""Optimize selected joint zero offsets from chessboard reprojection error."""

from pathlib import Path
import json
import time

import cv2
import numpy as np
from scipy.optimize import least_squares

from easyarm_calib.joint_zero_vision_common import (
    BOARD_TO_FLANGE_DISTANCE_M,
    CAMERA_INTRINSICS,
    CAMERA_MODEL,
    CAMERA_SERIAL,
    JOINT_NAMES,
    OPTIMIZED_JOINTS,
    ROS2_CONTROL_XACRO,
    fk_base_link6,
    initial_camera_base_transform,
    initial_link6_board_transform,
    load_camera_intrinsics,
    make_chessboard_points,
    parse_urdf_chain,
    parse_xacro_joint_offsets,
    project_points,
    radians_to_degrees,
    rotvec_t_from_transform,
    transform_from_rotvec_t,
    transform_points,
)


SAMPLES_JSON = Path("data/joint_zero_vision/latest/samples.json")
OUTPUT_ROOT = Path("data/joint_zero_calibration")
MAX_PREVIEW_SIZE = 1200
REPROJECTION_PREVIEW_COUNT = 30
JOINT_ZERO_PRIOR_SIGMA_RAD = np.deg2rad(10.0)


def _find_samples_json():
    if SAMPLES_JSON.exists():
        return SAMPLES_JSON
    candidates = sorted(Path("data/joint_zero_vision").glob("*/*/samples.json"))
    if not candidates:
        raise RuntimeError(
            "No samples.json found. Run ros2 run easyarm_calib collect_joint_zero_vision first."
        )
    return candidates[-1]


def _create_output_dir():
    now = time.localtime()
    output_dir = OUTPUT_ROOT / time.strftime("%Y%m%d", now) / time.strftime("%H%M%S", now)
    (output_dir / "reprojection_preview").mkdir(parents=True, exist_ok=True)
    return output_dir


def _pack_initial_params():
    camera_rotvec, camera_t = rotvec_t_from_transform(initial_camera_base_transform())
    board_rotvec, board_t = rotvec_t_from_transform(initial_link6_board_transform())
    return np.r_[np.zeros(len(OPTIMIZED_JOINTS)), camera_rotvec, camera_t, board_rotvec, board_t]


def _unpack_params(params):
    joint_count = len(OPTIMIZED_JOINTS)
    camera_start = joint_count
    board_start = camera_start + 6
    joint_delta = dict(zip(OPTIMIZED_JOINTS, params[:joint_count]))
    camera_base = transform_from_rotvec_t(
        params[camera_start:camera_start + 3],
        params[camera_start + 3:camera_start + 6],
    )
    link6_board = transform_from_rotvec_t(
        params[board_start:board_start + 3],
        params[board_start + 3:board_start + 6],
    )
    return joint_delta, camera_base, link6_board


def _parameter_slices():
    joint_count = len(OPTIMIZED_JOINTS)
    camera_start = joint_count
    board_start = camera_start + 6
    return {
        "joints": slice(0, joint_count),
        "camera_translation": slice(camera_start + 3, camera_start + 6),
        "board_translation": slice(board_start + 3, board_start + 6),
    }


def _corrected_joints(measured_joints, joint_delta):
    corrected = {name: float(measured_joints[name]) for name in JOINT_NAMES}
    for name, delta in joint_delta.items():
        corrected[name] += float(delta)
    return corrected


def _sample_residuals(sample, board_points, chain, camera_matrix, distortion, joint_delta, camera_base, link6_board):
    joints = _corrected_joints(sample["joints"], joint_delta)
    base_link6 = fk_base_link6(joints, chain)
    camera_board = camera_base @ base_link6 @ link6_board
    points_camera = transform_points(camera_board, board_points)
    projected = project_points(points_camera, camera_matrix, distortion)
    observed = np.asarray(sample["corners"], dtype=np.float64)
    return (projected - observed).reshape(-1)


def _residuals(params, samples, board_points, chain, camera_matrix, distortion):
    joint_delta, camera_base, link6_board = _unpack_params(params)
    residuals = []
    for sample in samples:
        residuals.append(_sample_residuals(
            sample,
            board_points,
            chain,
            camera_matrix,
            distortion,
            joint_delta,
            camera_base,
            link6_board,
        ))

    prior = np.asarray([joint_delta[name] / JOINT_ZERO_PRIOR_SIGMA_RAD for name in OPTIMIZED_JOINTS])
    residuals.append(prior)
    return np.concatenate(residuals)


def _rms_pixel_error(params, samples, board_points, chain, camera_matrix, distortion):
    joint_delta, camera_base, link6_board = _unpack_params(params)
    squared = 0.0
    count = 0
    per_sample = []
    for sample in samples:
        residual = _sample_residuals(
            sample,
            board_points,
            chain,
            camera_matrix,
            distortion,
            joint_delta,
            camera_base,
            link6_board,
        ).reshape(-1, 2)
        error = np.linalg.norm(residual, axis=1)
        squared += float(np.sum(error ** 2))
        count += len(error)
        per_sample.append({
            "image": sample["image"],
            "mean_error_px": float(np.mean(error)),
            "max_error_px": float(np.max(error)),
            "rms_error_px": float(np.sqrt(np.mean(error ** 2))),
        })
    return float(np.sqrt(squared / max(count, 1))), per_sample


def _resize_for_preview(image):
    height, width = image.shape[:2]
    longest = max(width, height)
    if longest <= MAX_PREVIEW_SIZE:
        return image
    scale = MAX_PREVIEW_SIZE / longest
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


def _save_reprojection_previews(output_dir, dataset_dir, samples, params, board_points, chain, camera_matrix, distortion):
    joint_delta, camera_base, link6_board = _unpack_params(params)
    preview_dir = output_dir / "reprojection_preview"
    for sample in samples[:REPROJECTION_PREVIEW_COUNT]:
        image = cv2.imread(str(dataset_dir / sample["image"]), cv2.IMREAD_COLOR)
        if image is None:
            continue
        joints = _corrected_joints(sample["joints"], joint_delta)
        camera_board = camera_base @ fk_base_link6(joints, chain) @ link6_board
        projected = project_points(transform_points(camera_board, board_points), camera_matrix, distortion)
        observed = np.asarray(sample["corners"], dtype=np.float64)
        for point in observed:
            cv2.circle(image, tuple(np.round(point).astype(int)), 4, (0, 255, 0), -1)
        for point in projected:
            cv2.drawMarker(
                image,
                tuple(np.round(point).astype(int)),
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=12,
                thickness=2,
            )
        name = Path(sample["image"]).name
        cv2.imwrite(str(preview_dir / name), image)


def _write_yaml(path, data):
    with path.open("w", encoding="utf-8") as file:
        for key, value in data.items():
            file.write(f"{key}:\n")
            for sub_key, sub_value in value.items():
                file.write(f"  {sub_key}: {sub_value}\n")


def main():
    """Run offline optimization and save suggested position_offset values."""
    samples_path = _find_samples_json()
    dataset_dir = samples_path.parent
    data = json.loads(samples_path.read_text(encoding="utf-8"))
    samples = data.get("samples", [])
    if len(samples) < 8:
        raise RuntimeError(f"Need at least 8 samples, got {len(samples)}")

    camera_matrix, distortion, image_size = load_camera_intrinsics(CAMERA_INTRINSICS)
    chain = parse_urdf_chain()
    board_points = make_chessboard_points()
    old_offsets = parse_xacro_joint_offsets(ROS2_CONTROL_XACRO)

    initial_params = _pack_initial_params()
    initial_rms, _ = _rms_pixel_error(initial_params, samples, board_points, chain, camera_matrix, distortion)

    print(f"Samples: {samples_path}")
    print(f"Sample count: {len(samples)}")
    print(f"Camera intrinsics: {CAMERA_INTRINSICS}")
    print(f"Initial RMS pixel error: {initial_rms:.4f}px")
    print(f"Optimizing joints: {', '.join(OPTIMIZED_JOINTS)}")

    slices = _parameter_slices()
    lower = np.full(len(initial_params), -np.inf)
    upper = np.full(len(initial_params), np.inf)
    lower[slices["joints"]] = np.deg2rad(-20.0)
    upper[slices["joints"]] = np.deg2rad(20.0)
    lower[slices["camera_translation"]] = initial_params[slices["camera_translation"]] - np.array([0.8, 0.8, 0.8])
    upper[slices["camera_translation"]] = initial_params[slices["camera_translation"]] + np.array([0.8, 0.8, 0.8])
    lower[slices["board_translation"]] = np.array([-0.08, -0.08, BOARD_TO_FLANGE_DISTANCE_M - 0.08])
    upper[slices["board_translation"]] = np.array([0.08, 0.08, BOARD_TO_FLANGE_DISTANCE_M + 0.08])

    result = least_squares(
        _residuals,
        initial_params,
        args=(samples, board_points, chain, camera_matrix, distortion),
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=2.0,
        max_nfev=300,
        verbose=1,
    )

    final_rms, per_sample_errors = _rms_pixel_error(
        result.x,
        samples,
        board_points,
        chain,
        camera_matrix,
        distortion,
    )
    joint_delta, camera_base, link6_board = _unpack_params(result.x)

    suggested_offsets = {}
    for name in JOINT_NAMES:
        old = old_offsets[name]["position_offset"]
        direction = old_offsets[name]["direction"]
        delta = joint_delta.get(name, 0.0)
        suggested_offsets[name] = old - delta * direction

    output_dir = _create_output_dir()
    _save_reprojection_previews(output_dir, dataset_dir, samples, result.x, board_points, chain, camera_matrix, distortion)

    result_data = {
        "camera_model": CAMERA_MODEL,
        "camera_serial": CAMERA_SERIAL,
        "samples_json": str(samples_path),
        "camera_intrinsics": str(CAMERA_INTRINSICS),
        "image_size": {"width": image_size[0], "height": image_size[1]},
        "optimized_joints": OPTIMIZED_JOINTS,
        "initial_rms_error_px": initial_rms,
        "final_rms_error_px": final_rms,
        "joint_delta_rad": joint_delta,
        "joint_delta_deg": radians_to_degrees(joint_delta),
        "old_position_offsets": {name: old_offsets[name]["position_offset"] for name in JOINT_NAMES},
        "directions": {name: old_offsets[name]["direction"] for name in JOINT_NAMES},
        "suggested_position_offsets": suggested_offsets,
        "camera_base_transform": camera_base.tolist(),
        "link6_board_transform": link6_board.tolist(),
        "per_sample_errors": per_sample_errors,
        "least_squares": {
            "success": bool(result.success),
            "message": result.message,
            "cost": float(result.cost),
            "optimality": float(result.optimality),
            "nfev": int(result.nfev),
        },
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    (output_dir / "result.json").write_text(json.dumps(result_data, indent=2), encoding="utf-8")

    _write_yaml(output_dir / "joint_zero_offsets.yaml", {
        "joint_delta_rad": joint_delta,
        "joint_delta_deg": radians_to_degrees(joint_delta),
        "suggested_position_offsets": suggested_offsets,
    })
    _write_yaml(output_dir / "extrinsics.yaml", {
        "camera_base": {"matrix": camera_base.tolist()},
        "link6_board": {"matrix": link6_board.tolist()},
    })

    print("Optimization finished")
    print(f"Initial RMS pixel error: {initial_rms:.4f}px")
    print(f"Final RMS pixel error: {final_rms:.4f}px")
    print("Joint deltas:")
    for name in OPTIMIZED_JOINTS:
        print(f"  {name}: {joint_delta[name]: .8f} rad ({np.rad2deg(joint_delta[name]): .4f} deg)")
    print("Suggested position_offset values:")
    for name in JOINT_NAMES:
        print(f"  {name}: {suggested_offsets[name]: .8f}")
    print(f"Saved results to {output_dir}")
    print("Green dots are detected corners; red crosses are optimized reprojections.")


if __name__ == "__main__":
    main()
