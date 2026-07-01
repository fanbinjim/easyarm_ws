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

"""Offline color-detector benchmark for EasyArm ball balance frames."""

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import statistics
import time

import cv2
import numpy as np

from .ball_balance_detector import BallBalanceDetection
from .ball_balance_detector import CircleDetection
from .ball_balance_detector import DetectionConfig
from .ball_balance_detector import detect_objects
from .ball_balance_detector import detect_objects_by_edges
from .ball_balance_detector import round_point


DEFAULT_DATASET = (
    Path(__file__).resolve().parents[1] / "debug" / "202606301845"
)


@dataclass(frozen=True)
class BenchmarkResult:
    """One benchmark method result."""

    name: str
    detections: list[BallBalanceDetection]
    times_ms: list[float]


def build_parser() -> argparse.ArgumentParser:
    """Build command line parser."""
    parser = argparse.ArgumentParser(
        description="Compare edge-based and color-based ball detectors.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "dataset",
        nargs="?",
        type=Path,
        default=DEFAULT_DATASET,
        help="Directory that contains captured PNG frames.",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=20,
        help="Timing repeats over the loaded frame set.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output directory for summary, CSV, and overlays.",
    )
    parser.add_argument(
        "--overlay-count",
        type=int,
        default=8,
        help="Number of comparison overlays to save.",
    )
    return parser


def main() -> int:
    """Run the offline color detector benchmark."""
    args = build_parser().parse_args()
    dataset = args.dataset
    output_dir = args.output or dataset / "analysis_color_benchmark"
    output_dir.mkdir(parents=True, exist_ok=True)

    frame_paths = sorted(dataset.glob("frame_*.png"))
    if not frame_paths:
        frame_paths = sorted(dataset.glob("*.png"))
    if not frame_paths:
        raise SystemExit(f"No PNG frames found in {dataset}")

    frames = load_frames(frame_paths)
    config = default_detection_config()
    edge_result = benchmark_method(
        "edge",
        frames,
        args.repeats,
        lambda frame: detect_objects_by_edges(frame, config),
    )
    color_result = benchmark_method(
        "color",
        frames,
        args.repeats,
        lambda frame: detect_objects(frame, config),
    )
    write_summary(output_dir / "summary.txt", edge_result, color_result)
    write_csv(output_dir / "detections.csv", frame_paths, edge_result,
              color_result)
    write_overlays(output_dir / "overlays", frame_paths, frames, edge_result,
                   color_result, args.overlay_count)
    print_summary(edge_result, color_result, output_dir)
    return 0


def load_frames(paths: list[Path]) -> list[np.ndarray]:
    """Load PNG frames into memory so timing excludes disk I/O."""
    frames = []
    for path in paths:
        frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if frame is None:
            raise SystemExit(f"Failed to read image: {path}")
        frames.append(frame)
    return frames


def default_detection_config() -> DetectionConfig:
    """Return the current ball-balance detector defaults."""
    return DetectionConfig(
        plate_min_radius=100.0,
        plate_max_radius=150.0,
        plate_roi_x_min=0.22,
        plate_roi_x_max=0.86,
        plate_roi_y_min=0.22,
        plate_roi_y_max=0.95,
        plate_expected_x=0.543,
        plate_expected_y=0.603,
        plate_expected_radius=124.0,
        ball_min_area=45.0,
        ball_max_area=1200.0,
        ball_min_radius=4.0,
        ball_max_radius=18.0,
        ball_max_value=65,
        ball_min_circularity=0.25,
        ball_plate_inner_scale=0.92,
    )


def benchmark_method(
    name: str,
    frames: list[np.ndarray],
    repeats: int,
    detector,
) -> BenchmarkResult:
    """Benchmark one detector over preloaded frames."""
    for frame in frames[: min(5, len(frames))]:
        detector(frame)

    detections = [detector(frame) for frame in frames]
    times_ms = []
    repeat_count = max(1, repeats)
    for _index in range(repeat_count):
        for frame in frames:
            start = time.perf_counter()
            detector(frame)
            times_ms.append((time.perf_counter() - start) * 1000.0)
    return BenchmarkResult(name, detections, times_ms)


def write_summary(
    path: Path,
    edge_result: BenchmarkResult,
    color_result: BenchmarkResult,
) -> None:
    """Write a plain-text benchmark summary."""
    lines = summary_lines(edge_result, color_result)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def print_summary(
    edge_result: BenchmarkResult,
    color_result: BenchmarkResult,
    output_dir: Path,
) -> None:
    """Print benchmark summary to stdout."""
    for line in summary_lines(edge_result, color_result):
        print(line)
    print(f"output: {output_dir}")


def summary_lines(
    edge_result: BenchmarkResult,
    color_result: BenchmarkResult,
) -> list[str]:
    """Return human-readable benchmark lines."""
    edge_stats = detection_stats(edge_result)
    color_stats = detection_stats(color_result)
    diff_stats = comparison_stats(edge_result, color_result)
    speedup = (
        percentile(edge_result.times_ms, 0.50)
        / max(percentile(color_result.times_ms, 0.50), 1.0e-9)
    )
    return [
        "EasyArm ball balance color benchmark",
        "",
        format_result(edge_result, edge_stats),
        format_result(color_result, color_stats),
        f"p50 speedup: {speedup:.2f}x",
        "",
        (
            "center diff color-vs-edge px "
            f"plate p50={diff_stats['plate_p50']:.2f} "
            f"p95={diff_stats['plate_p95']:.2f}; "
            f"ball p50={diff_stats['ball_p50']:.2f} "
            f"p95={diff_stats['ball_p95']:.2f}"
        ),
        (
            "offset diff color-vs-edge norm "
            f"p50={diff_stats['offset_p50']:.4f} "
            f"p95={diff_stats['offset_p95']:.4f}"
        ),
    ]


def format_result(
    result: BenchmarkResult,
    stats: dict[str, float],
) -> str:
    """Format one detector benchmark line."""
    return (
        f"{result.name}: found={stats['found']:.0f}/{stats['total']:.0f} "
        f"time_ms mean={statistics.mean(result.times_ms):.3f} "
        f"p50={percentile(result.times_ms, 0.50):.3f} "
        f"p90={percentile(result.times_ms, 0.90):.3f} "
        f"p95={percentile(result.times_ms, 0.95):.3f} "
        f"p99={percentile(result.times_ms, 0.99):.3f}"
    )


def detection_stats(result: BenchmarkResult) -> dict[str, float]:
    """Return detection count statistics."""
    total = len(result.detections)
    found = sum(
        1
        for detection in result.detections
        if detection.plate is not None and detection.ball is not None
    )
    return {"total": float(total), "found": float(found)}


def comparison_stats(
    edge_result: BenchmarkResult,
    color_result: BenchmarkResult,
) -> dict[str, float]:
    """Return color-vs-edge detection differences."""
    plate_diffs = []
    ball_diffs = []
    offset_diffs = []
    for edge, color in zip(edge_result.detections, color_result.detections):
        if edge.plate is not None and color.plate is not None:
            plate_diffs.append(math.dist(edge.plate.center,
                                         color.plate.center))
        if edge.ball is not None and color.ball is not None:
            ball_diffs.append(math.dist(edge.ball.center,
                                        color.ball.center))
        if edge.offset is not None and color.offset is not None:
            offset_diffs.append(math.dist(edge.offset, color.offset))
    return {
        "plate_p50": percentile(plate_diffs, 0.50),
        "plate_p95": percentile(plate_diffs, 0.95),
        "ball_p50": percentile(ball_diffs, 0.50),
        "ball_p95": percentile(ball_diffs, 0.95),
        "offset_p50": percentile(offset_diffs, 0.50),
        "offset_p95": percentile(offset_diffs, 0.95),
    }


def percentile(values: list[float], q: float) -> float:
    """Return a nearest-rank percentile."""
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = round((len(ordered) - 1) * q)
    index = max(0, min(len(ordered) - 1, index))
    return float(ordered[index])


def write_csv(
    path: Path,
    frame_paths: list[Path],
    edge_result: BenchmarkResult,
    color_result: BenchmarkResult,
) -> None:
    """Write per-frame detector comparison CSV."""
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "frame",
                "edge_found",
                "color_found",
                "edge_plate_x",
                "edge_plate_y",
                "edge_plate_r",
                "color_plate_x",
                "color_plate_y",
                "color_plate_r",
                "edge_ball_x",
                "edge_ball_y",
                "edge_ball_r",
                "color_ball_x",
                "color_ball_y",
                "color_ball_r",
                "edge_offset_x",
                "edge_offset_y",
                "color_offset_x",
                "color_offset_y",
            ],
        )
        writer.writeheader()
        for path_item, edge, color in zip(
            frame_paths,
            edge_result.detections,
            color_result.detections,
        ):
            writer.writerow(row_for_frame(path_item, edge, color))


def row_for_frame(
    path: Path,
    edge: BallBalanceDetection,
    color: BallBalanceDetection,
) -> dict[str, object]:
    """Build one CSV comparison row."""
    row = {
        "frame": path.name,
        "edge_found": edge.plate is not None and edge.ball is not None,
        "color_found": color.plate is not None and color.ball is not None,
    }
    row.update(circle_fields("edge_plate", edge.plate))
    row.update(circle_fields("color_plate", color.plate))
    row.update(circle_fields("edge_ball", edge.ball))
    row.update(circle_fields("color_ball", color.ball))
    row.update(offset_fields("edge_offset", edge.offset))
    row.update(offset_fields("color_offset", color.offset))
    return row


def circle_fields(
    prefix: str,
    circle: CircleDetection | None,
) -> dict[str, object]:
    """Return CSV fields for one circle."""
    if circle is None:
        return {f"{prefix}_x": "", f"{prefix}_y": "", f"{prefix}_r": ""}
    return {
        f"{prefix}_x": circle.center[0],
        f"{prefix}_y": circle.center[1],
        f"{prefix}_r": circle.radius,
    }


def offset_fields(
    prefix: str,
    offset: tuple[float, float] | None,
) -> dict[str, object]:
    """Return CSV fields for one offset."""
    if offset is None:
        return {f"{prefix}_x": "", f"{prefix}_y": ""}
    return {f"{prefix}_x": offset[0], f"{prefix}_y": offset[1]}


def write_overlays(
    output_dir: Path,
    frame_paths: list[Path],
    frames: list[np.ndarray],
    edge_result: BenchmarkResult,
    color_result: BenchmarkResult,
    count: int,
) -> None:
    """Write a few overlay images for visual inspection."""
    output_dir.mkdir(parents=True, exist_ok=True)
    for index, (path, frame, edge, color) in enumerate(
        zip(frame_paths, frames, edge_result.detections,
            color_result.detections)
    ):
        if index >= max(0, count):
            break
        overlay = frame.copy()
        draw_detection(overlay, edge, (255, 80, 40), "edge")
        draw_detection(overlay, color, (40, 255, 80), "color")
        cv2.imwrite(str(output_dir / path.name), overlay)


def draw_detection(
    image: np.ndarray,
    detection: BallBalanceDetection,
    color: tuple[int, int, int],
    label: str,
) -> None:
    """Draw one detector result."""
    if detection.plate is not None:
        cv2.circle(
            image,
            round_point(detection.plate.center),
            int(round(detection.plate.radius)),
            color,
            2,
        )
    if detection.ball is not None:
        cv2.circle(
            image,
            round_point(detection.ball.center),
            int(round(detection.ball.radius)),
            color,
            2,
        )
        cv2.putText(
            image,
            label,
            round_point(detection.ball.center),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )


if __name__ == "__main__":
    raise SystemExit(main())
