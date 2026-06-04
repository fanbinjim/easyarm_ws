"""Calibrate the Daheng camera from a fixed chessboard image set."""

from pathlib import Path
import json
import time

import cv2
import numpy as np


CAMERA_MODEL = "MER2-301-125U3M"
CAMERA_SERIAL = "FCZ21070977"
IMAGE_DIR = Path("data/camera_capture/20260526/171222")
OUTPUT_DIR = Path("data/camera_calibration") / f"{CAMERA_MODEL}_{CAMERA_SERIAL}" / "20260526_171222"

CHESSBOARD_COLS = 11
CHESSBOARD_ROWS = 8
SQUARE_SIZE_M = 0.01

MAX_VISUALIZATION_SIZE = 1000
VISUALIZATION_DELAY_MS = 350
UNDISTORT_PREVIEW_COUNT = 10


def _make_object_points():
    object_points = np.zeros((CHESSBOARD_ROWS * CHESSBOARD_COLS, 3), np.float32)
    grid = np.mgrid[0:CHESSBOARD_COLS, 0:CHESSBOARD_ROWS].T.reshape(-1, 2)
    object_points[:, :2] = grid * SQUARE_SIZE_M
    return object_points


def _resize_for_visualization(image):
    height, width = image.shape[:2]
    longest_side = max(width, height)
    if longest_side <= MAX_VISUALIZATION_SIZE:
        return image
    scale = MAX_VISUALIZATION_SIZE / longest_side
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


def _write_yaml(path, image_size, camera_matrix, distortion_coefficients, rms_error):
    width, height = image_size
    with path.open("w", encoding="utf-8") as file:
        file.write("%YAML:1.0\n")
        file.write("---\n")
        file.write(f"camera_name: {CAMERA_MODEL}_{CAMERA_SERIAL}\n")
        file.write(f"image_width: {width}\n")
        file.write(f"image_height: {height}\n")
        file.write("camera_matrix:\n")
        file.write("  rows: 3\n")
        file.write("  cols: 3\n")
        file.write("  data: [%s]\n" % ", ".join(f"{v:.12g}" for v in camera_matrix.reshape(-1)))
        file.write("distortion_model: plumb_bob\n")
        file.write("distortion_coefficients:\n")
        file.write("  rows: 1\n")
        file.write(f"  cols: {distortion_coefficients.size}\n")
        file.write("  data: [%s]\n" % ", ".join(f"{v:.12g}" for v in distortion_coefficients.reshape(-1)))
        file.write("rectification_matrix:\n")
        file.write("  rows: 3\n")
        file.write("  cols: 3\n")
        file.write("  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n")
        file.write("projection_matrix:\n")
        file.write("  rows: 3\n")
        file.write("  cols: 4\n")
        file.write(
            "  data: [%s, 0, %s, 0, 0, %s, %s, 0, 0, 0, 1, 0]\n"
            % (
                f"{camera_matrix[0, 0]:.12g}",
                f"{camera_matrix[0, 2]:.12g}",
                f"{camera_matrix[1, 1]:.12g}",
                f"{camera_matrix[1, 2]:.12g}",
            )
        )
        file.write(f"rms_reprojection_error: {rms_error:.12g}\n")


def _write_json(path, image_size, camera_matrix, distortion_coefficients, rms_error, per_image_errors,
                successful_images, failed_images):
    width, height = image_size
    result = {
        "camera_model": CAMERA_MODEL,
        "camera_serial": CAMERA_SERIAL,
        "image_dir": str(IMAGE_DIR),
        "image_width": width,
        "image_height": height,
        "chessboard_inner_corners": {
            "cols": CHESSBOARD_COLS,
            "rows": CHESSBOARD_ROWS,
        },
        "square_size_m": SQUARE_SIZE_M,
        "rms_reprojection_error": float(rms_error),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_model": "plumb_bob",
        "distortion_coefficients": distortion_coefficients.reshape(-1).tolist(),
        "successful_images": successful_images,
        "failed_images": failed_images,
        "per_image_reprojection_error": per_image_errors,
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    path.write_text(json.dumps(result, indent=2), encoding="utf-8")


def _save_undistort_previews(successful_images, camera_matrix, distortion_coefficients):
    preview_dir = OUTPUT_DIR / "undistort_preview"
    preview_dir.mkdir(parents=True, exist_ok=True)

    for image_name in successful_images[:UNDISTORT_PREVIEW_COUNT]:
        image_path = IMAGE_DIR / image_name
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        undistorted = cv2.undistort(image, camera_matrix, distortion_coefficients)
        cv2.imwrite(str(preview_dir / image_name), undistorted)


def _detect_chessboard(image_path, object_template):
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        return None, None, None, "read_failed"

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_COLS, CHESSBOARD_ROWS), flags)
    if not found:
        return image, gray.shape[::-1], None, "corners_not_found"

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return image, gray.shape[::-1], (object_template.copy(), corners), None


def _compute_per_image_errors(object_points, image_points, rvecs, tvecs, camera_matrix, distortion_coefficients,
                              image_names):
    errors = []
    for index, image_name in enumerate(image_names):
        projected_points, _ = cv2.projectPoints(
            object_points[index],
            rvecs[index],
            tvecs[index],
            camera_matrix,
            distortion_coefficients,
        )
        error = cv2.norm(image_points[index], projected_points, cv2.NORM_L2) / len(projected_points)
        errors.append({"image": image_name, "error_px": float(error)})
    return errors


def _visualize_corners(visualization_images):
    if not visualization_images:
        return

    print("Showing chessboard corner visualization. Press q/Esc to close, any other key for next image.")
    cv2.namedWindow("Chessboard Corners", cv2.WINDOW_NORMAL)
    for image_name, image in visualization_images:
        shown = _resize_for_visualization(image)
        cv2.imshow("Chessboard Corners", shown)
        print(f"Visualizing {image_name}")
        key = cv2.waitKey(VISUALIZATION_DELAY_MS)
        if key in (27, ord("q"), ord("Q")):
            break
    cv2.destroyWindow("Chessboard Corners")


def main():
    """Run camera calibration and visualize detected chessboard corners."""
    image_paths = sorted(IMAGE_DIR.glob("IMG*.png"))
    if not image_paths:
        raise RuntimeError(f"No images found in {IMAGE_DIR}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    corners_dir = OUTPUT_DIR / "corners_preview"
    corners_dir.mkdir(parents=True, exist_ok=True)

    object_template = _make_object_points()
    object_points = []
    image_points = []
    successful_images = []
    failed_images = []
    visualization_images = []
    image_size = None

    print(f"Input images: {IMAGE_DIR}")
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Chessboard: {CHESSBOARD_COLS}x{CHESSBOARD_ROWS}, square={SQUARE_SIZE_M}m")

    for image_path in image_paths:
        image, detected_image_size, points, failure_reason = _detect_chessboard(image_path, object_template)
        if detected_image_size is not None:
            if image_size is None:
                image_size = detected_image_size
            elif image_size != detected_image_size:
                raise RuntimeError(f"Image size mismatch in {image_path}")

        if failure_reason is not None:
            failed_images.append({"image": image_path.name, "reason": failure_reason})
            print(f"[FAIL] {image_path.name}: {failure_reason}")
            continue

        object_point, corner = points
        object_points.append(object_point)
        image_points.append(corner)
        successful_images.append(image_path.name)

        corner_image = image.copy()
        cv2.drawChessboardCorners(corner_image, (CHESSBOARD_COLS, CHESSBOARD_ROWS), corner, True)
        cv2.imwrite(str(corners_dir / image_path.name), corner_image)
        visualization_images.append((image_path.name, corner_image))
        print(f"[ OK ] {image_path.name}")

    if len(successful_images) < 5:
        raise RuntimeError(f"Only {len(successful_images)} valid images found, need at least 5")

    rms_error, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    distortion_coefficients = distortion_coefficients.reshape(1, -1)

    per_image_errors = _compute_per_image_errors(
        object_points,
        image_points,
        rvecs,
        tvecs,
        camera_matrix,
        distortion_coefficients,
        successful_images,
    )

    _write_yaml(OUTPUT_DIR / "camera_calibration.yaml", image_size, camera_matrix, distortion_coefficients, rms_error)
    _write_json(
        OUTPUT_DIR / "camera_calibration.json",
        image_size,
        camera_matrix,
        distortion_coefficients,
        rms_error,
        per_image_errors,
        successful_images,
        failed_images,
    )
    _save_undistort_previews(successful_images, camera_matrix, distortion_coefficients)

    print("Calibration finished")
    print(f"Valid images: {len(successful_images)} / {len(image_paths)}")
    print(f"RMS reprojection error: {rms_error:.6f} px")
    print("Camera matrix:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(distortion_coefficients.reshape(-1))
    print(f"Saved: {OUTPUT_DIR / 'camera_calibration.yaml'}")
    print(f"Saved: {OUTPUT_DIR / 'camera_calibration.json'}")

    _visualize_corners(visualization_images)


if __name__ == "__main__":
    main()
