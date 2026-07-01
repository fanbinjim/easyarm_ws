#!/usr/bin/env python3
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

"""Analyze EasyArm ball-balance control logs and plot plate trajectories."""

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


DEFAULT_LOG = Path(__file__).resolve().parent / "latest" / "control_log.csv"


@dataclass(frozen=True)
class ControlLog:
    """Numeric arrays parsed from one control log."""

    path: Path
    time_s: list[float]
    offset_x: list[float]
    offset_y: list[float]
    radius: list[float]
    velocity_x: list[float]
    velocity_y: list[float]
    speed: list[float]
    tilt_x_deg: list[float]
    tilt_y_deg: list[float]
    tilt_mag_deg: list[float]
    trim_x_deg: list[float]
    trim_y_deg: list[float]
    integral_active: list[bool]
    stale: list[bool]


@dataclass(frozen=True)
class WindowStats:
    """Metrics for one sliding analysis window."""

    start_s: float
    end_s: float
    start_index: int
    end_index: int
    mean_radius: float
    p95_radius: float
    mean_speed: float
    std_xy: float
    score: float


def build_parser() -> argparse.ArgumentParser:
    """Build command line parser."""
    parser = argparse.ArgumentParser(
        description="Plot and summarize an EasyArm ball-balance control log.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "log",
        nargs="?",
        type=Path,
        default=None,
        help="Path to control_log.csv. Defaults to newest debug log.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for generated plots and summary.",
    )
    parser.add_argument(
        "--window-sec",
        type=float,
        default=10.0,
        help="Sliding window length for oscillation detection.",
    )
    parser.add_argument(
        "--step-sec",
        type=float,
        default=1.0,
        help="Sliding window step for oscillation detection.",
    )
    return parser


def main() -> int:
    """Run log analysis."""
    args = build_parser().parse_args()
    log_path = args.log or newest_control_log()
    log = read_control_log(log_path)
    output_dir = args.output_dir or log_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    oscillation = detect_oscillation_window(
        log,
        args.window_sec,
        args.step_sec,
    )
    prefix = log_path.parent.name
    series_path = output_dir / f"control_analysis_{prefix}.png"
    trajectory_path = output_dir / f"plate_trajectory_{prefix}.png"
    summary_path = output_dir / f"control_analysis_{prefix}.txt"

    plot_time_series(log, oscillation, series_path)
    plot_plate_trajectory(log, oscillation, trajectory_path)
    summary = build_summary(log, oscillation)
    summary_path.write_text(summary, encoding="utf-8")

    print(summary)
    print(f"time series: {series_path}")
    print(f"trajectory: {trajectory_path}")
    print(f"summary: {summary_path}")
    return 0


def newest_control_log() -> Path:
    """Return the newest control_log.csv under the debug directory."""
    debug_dir = Path(__file__).resolve().parent
    logs = sorted(
        debug_dir.glob("20*/control_log.csv"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not logs:
        raise SystemExit(f"No control_log.csv found under {debug_dir}")
    return logs[0]


def read_control_log(path: Path) -> ControlLog:
    """Read one CSV log into numeric arrays."""
    rows = list(csv.DictReader(path.open(newline="")))
    if not rows:
        raise SystemExit(f"No rows in {path}")

    wall_time = [read_float(row, "wall_time_sec") for row in rows]
    start_time = wall_time[0]
    time_s = [value - start_time for value in wall_time]
    offset_x = [read_float(row, "plate_offset_x") for row in rows]
    offset_y = [read_float(row, "plate_offset_y") for row in rows]
    velocity_x = [read_float(row, "filtered_velocity_x") for row in rows]
    velocity_y = [read_float(row, "filtered_velocity_y") for row in rows]
    tilt_x = [read_float(row, "tilt_x_deg") for row in rows]
    tilt_y = [read_float(row, "tilt_y_deg") for row in rows]
    trim_x = [read_float(row, "integral_trim_x_deg") for row in rows]
    trim_y = [read_float(row, "integral_trim_y_deg") for row in rows]
    return ControlLog(
        path=path,
        time_s=time_s,
        offset_x=offset_x,
        offset_y=offset_y,
        radius=[math.hypot(x, y) for x, y in zip(offset_x, offset_y)],
        velocity_x=velocity_x,
        velocity_y=velocity_y,
        speed=[math.hypot(x, y) for x, y in zip(velocity_x, velocity_y)],
        tilt_x_deg=tilt_x,
        tilt_y_deg=tilt_y,
        tilt_mag_deg=[math.hypot(x, y) for x, y in zip(tilt_x, tilt_y)],
        trim_x_deg=trim_x,
        trim_y_deg=trim_y,
        integral_active=[
            read_bool(row, "integral_active")
            for row in rows
        ],
        stale=[read_bool(row, "control_stale") for row in rows],
    )


def read_float(row: dict[str, str], key: str) -> float:
    """Read a floating-point CSV field."""
    value = row.get(key, "")
    if value == "":
        return float("nan")
    return float(value)


def read_bool(row: dict[str, str], key: str) -> bool:
    """Read a boolean CSV field."""
    return row.get(key, "").lower() in ("true", "1", "yes")


def detect_oscillation_window(
    log: ControlLog,
    window_sec: float,
    step_sec: float,
) -> WindowStats:
    """Find the window with the strongest oscillation-like behavior."""
    duration = log.time_s[-1] - log.time_s[0]
    best: WindowStats | None = None
    start = 0.0
    while start + window_sec <= duration + 1.0e-9:
        end = start + window_sec
        indices = [
            index for index, value in enumerate(log.time_s)
            if start <= value < end
        ]
        if len(indices) >= 3:
            stats = window_stats(log, indices, start, end)
            if best is None or stats.score > best.score:
                best = stats
        start += step_sec
    if best is None:
        indices = list(range(len(log.time_s)))
        best = window_stats(log, indices, log.time_s[0], log.time_s[-1])
    return best


def window_stats(
    log: ControlLog,
    indices: list[int],
    start_s: float,
    end_s: float,
) -> WindowStats:
    """Compute oscillation metrics for a set of indices."""
    radius = [log.radius[index] for index in indices]
    speed = [log.speed[index] for index in indices]
    xs = [log.offset_x[index] for index in indices]
    ys = [log.offset_y[index] for index in indices]
    std_xy = math.hypot(statistics.pstdev(xs), statistics.pstdev(ys))
    mean_radius = mean(radius)
    p95_radius = percentile(radius, 0.95)
    mean_speed = mean(speed)
    score = p95_radius * 2.0 + mean_speed + std_xy * 1.5 + mean_radius
    return WindowStats(
        start_s=start_s,
        end_s=end_s,
        start_index=indices[0],
        end_index=indices[-1],
        mean_radius=mean_radius,
        p95_radius=p95_radius,
        mean_speed=mean_speed,
        std_xy=std_xy,
        score=score,
    )


def plot_time_series(
    log: ControlLog,
    oscillation: WindowStats,
    output: Path,
) -> None:
    """Plot time-domain control metrics."""
    fig, axes = plt.subplots(6, 1, figsize=(14, 13), sharex=True)
    fig.suptitle(f"Ball Balance Control Log {log.path.parent.name}",
                 fontsize=14)

    mark_window(axes[0], oscillation)
    axes[0].plot(log.time_s, log.offset_x, label="plate_offset_x",
                 color="#1f77b4", lw=1.2)
    axes[0].plot(log.time_s, log.offset_y, label="plate_offset_y",
                 color="#ff7f0e", lw=1.2)
    axes[0].axhline(0.0, color="black", lw=0.8, alpha=0.5)
    axes[0].set_ylabel("offset")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)

    mark_window(axes[1], oscillation)
    axes[1].plot(log.time_s, log.radius, label="radius",
                 color="#2ca02c", lw=1.2)
    axes[1].axhline(0.15, color="#2ca02c", ls="--", lw=0.8,
                    alpha=0.5, label="r=0.15")
    axes[1].axhline(0.25, color="#888888", ls="--", lw=0.8,
                    alpha=0.5, label="r=0.25")
    axes[1].axhline(0.72, color="#d62728", ls="--", lw=0.8,
                    alpha=0.5, label="edge=0.72")
    axes[1].set_ylabel("radius")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)

    mark_window(axes[2], oscillation)
    axes[2].plot(log.time_s, log.velocity_x, label="vel_x",
                 color="#1f77b4", lw=1.0)
    axes[2].plot(log.time_s, log.velocity_y, label="vel_y",
                 color="#ff7f0e", lw=1.0)
    axes[2].plot(log.time_s, log.speed, label="speed",
                 color="#9467bd", lw=1.0, alpha=0.85)
    axes[2].axhline(0.0, color="black", lw=0.8, alpha=0.5)
    axes[2].set_ylabel("velocity")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.25)

    mark_window(axes[3], oscillation)
    axes[3].plot(log.time_s, log.tilt_x_deg, label="tilt_x_deg",
                 color="#1f77b4", lw=1.0)
    axes[3].plot(log.time_s, log.tilt_y_deg, label="tilt_y_deg",
                 color="#ff7f0e", lw=1.0)
    axes[3].plot(log.time_s, log.tilt_mag_deg, label="tilt_mag",
                 color="#d62728", lw=1.0, alpha=0.8)
    axes[3].axhline(0.0, color="black", lw=0.8, alpha=0.5)
    axes[3].set_ylabel("tilt deg")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.25)

    mark_window(axes[4], oscillation)
    axes[4].plot(log.time_s, log.trim_x_deg, label="integral_trim_x_deg",
                 color="#1f77b4", lw=1.2)
    axes[4].plot(log.time_s, log.trim_y_deg, label="integral_trim_y_deg",
                 color="#ff7f0e", lw=1.2)
    axes[4].fill_between(
        log.time_s,
        0,
        1,
        where=log.integral_active,
        color="#2ca02c",
        alpha=0.12,
        transform=axes[4].get_xaxis_transform(),
        label="integral active",
    )
    axes[4].axhline(0.0, color="black", lw=0.8, alpha=0.5)
    axes[4].set_ylabel("trim deg")
    axes[4].legend(loc="upper right")
    axes[4].grid(True, alpha=0.25)

    mark_window(axes[5], oscillation)
    axes[5].plot(log.time_s, [1.0 if value > 0.72 else 0.0
                              for value in log.radius],
                 label="r > 0.72", color="#d62728", lw=1.0)
    axes[5].plot(log.time_s, [1.0 if value < 0.15 else 0.0
                              for value in log.radius],
                 label="r < 0.15", color="#2ca02c", lw=1.0)
    axes[5].plot(log.time_s, [1.0 if value else 0.0 for value in log.stale],
                 label="stale", color="#9467bd", lw=1.0)
    axes[5].set_ylabel("flags")
    axes[5].set_xlabel("time (s)")
    axes[5].set_ylim(-0.05, 1.05)
    axes[5].legend(loc="upper right")
    axes[5].grid(True, alpha=0.25)

    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(output, dpi=150)
    plt.close(fig)


def mark_window(axis, oscillation: WindowStats) -> None:
    """Highlight the detected oscillation window."""
    axis.axvspan(
        oscillation.start_s,
        oscillation.end_s,
        color="#ffcc66",
        alpha=0.22,
        label="oscillation window",
    )


def plot_plate_trajectory(
    log: ControlLog,
    oscillation: WindowStats,
    output: Path,
) -> None:
    """Plot ball trajectory in normalized plate coordinates."""
    start = oscillation.start_index
    end = oscillation.end_index + 1
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle(f"Ball Trajectory On Plate {log.path.parent.name}",
                 fontsize=14)
    draw_plate_axes(
        axes[0],
        log.offset_x,
        log.offset_y,
        log.time_s,
        "full trajectory",
    )
    draw_plate_axes(
        axes[1],
        log.offset_x[start:end],
        log.offset_y[start:end],
        log.time_s[start:end],
        (
            "detected oscillation "
            f"{oscillation.start_s:.1f}-{oscillation.end_s:.1f}s"
        ),
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(output, dpi=150)
    plt.close(fig)


def draw_plate_axes(
    axis,
    xs: list[float],
    ys: list[float],
    time_s: list[float],
    title: str,
) -> None:
    """Draw one plate-coordinate trajectory subplot."""
    axis.set_aspect("equal", adjustable="box")
    axis.set_title(title)
    for radius, color, linestyle in (
        (1.0, "#bbbbbb", "-"),
        (0.72, "#d62728", "--"),
        (0.25, "#888888", "--"),
        (0.15, "#2ca02c", "--"),
    ):
        circle = plt.Circle(
            (0.0, 0.0),
            radius,
            fill=False,
            color=color,
            linestyle=linestyle,
            lw=1.0,
            alpha=0.75,
        )
        axis.add_patch(circle)
    scatter = axis.scatter(
        xs,
        ys,
        c=time_s,
        cmap="viridis",
        s=8,
        alpha=0.85,
    )
    if xs and ys:
        axis.scatter([xs[0]], [ys[0]], color="#1f77b4", s=70,
                     marker="o", label="start")
        axis.scatter([xs[-1]], [ys[-1]], color="#d62728", s=80,
                     marker="x", label="end")
    axis.axhline(0.0, color="black", lw=0.8, alpha=0.4)
    axis.axvline(0.0, color="black", lw=0.8, alpha=0.4)
    axis.set_xlim(-1.05, 1.05)
    axis.set_ylim(1.05, -1.05)
    axis.set_xlabel("plate offset x")
    axis.set_ylabel("plate offset y")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="upper right")
    plt.colorbar(scatter, ax=axis, fraction=0.046, pad=0.04,
                 label="time (s)")


def build_summary(log: ControlLog, oscillation: WindowStats) -> str:
    """Build a human-readable analysis summary."""
    duration = log.time_s[-1] - log.time_s[0]
    line_rate = len(log.time_s) / duration if duration > 0.0 else 0.0
    last_window = indices_after(log, max(0.0, duration - 10.0))
    lines = [
        f"log: {log.path}",
        f"rows: {len(log.time_s)}",
        f"duration_s: {duration:.3f}",
        f"row_hz: {line_rate:.2f}",
        "",
        "overall:",
        metric_line("x", log.offset_x),
        metric_line("y", log.offset_y),
        metric_line("radius", log.radius),
        metric_line("speed", log.speed),
        metric_line("tilt_mag_deg", log.tilt_mag_deg),
        metric_line("trim_x_deg", log.trim_x_deg),
        metric_line("trim_y_deg", log.trim_y_deg),
        (
            "fractions: "
            f"stale={fraction(log.stale):.4f}, "
            f"integral_active={fraction(log.integral_active):.4f}, "
            f"r<0.15={fraction([v < 0.15 for v in log.radius]):.4f}, "
            f"r<0.25={fraction([v < 0.25 for v in log.radius]):.4f}, "
            f"r>0.72={fraction([v > 0.72 for v in log.radius]):.4f}"
        ),
        "",
        (
            "detected_oscillation_window: "
            f"{oscillation.start_s:.2f}-{oscillation.end_s:.2f}s, "
            f"mean_radius={oscillation.mean_radius:.4f}, "
            f"p95_radius={oscillation.p95_radius:.4f}, "
            f"mean_speed={oscillation.mean_speed:.4f}, "
            f"std_xy={oscillation.std_xy:.4f}, "
            f"score={oscillation.score:.4f}"
        ),
        "",
        "last_10s:",
        window_metric_line(log, last_window),
        "",
        "10s windows:",
    ]
    lines.extend(build_window_lines(log, 10.0))
    return "\n".join(lines)


def indices_after(log: ControlLog, start_s: float) -> list[int]:
    """Return indices after a given timestamp."""
    return [index for index, value in enumerate(log.time_s)
            if value >= start_s]


def window_metric_line(log: ControlLog, indices: list[int]) -> str:
    """Summarize x/y/radius over selected indices."""
    xs = [log.offset_x[index] for index in indices]
    ys = [log.offset_y[index] for index in indices]
    radius = [log.radius[index] for index in indices]
    return (
        f"mean_x={mean(xs):+.4f}, mean_y={mean(ys):+.4f}, "
        f"mean_radius={mean(radius):.4f}, "
        f"p95_radius={percentile(radius, 0.95):.4f}"
    )


def build_window_lines(log: ControlLog, window_sec: float) -> list[str]:
    """Return fixed-window summary lines."""
    lines = []
    start = 0.0
    duration = log.time_s[-1]
    while start <= duration:
        end = start + window_sec
        indices = [
            index for index, value in enumerate(log.time_s)
            if start <= value < end
        ]
        if indices:
            lines.append(
                f"{start:06.1f}-{end:06.1f}s: "
                f"n={len(indices)}, "
                f"{window_metric_line(log, indices)}, "
                f"mean_speed={mean([log.speed[i] for i in indices]):.4f}, "
                f"mean_tilt={mean([log.tilt_mag_deg[i] for i in indices]):.4f}, "
                f"trim_end=({log.trim_x_deg[indices[-1]]:+.3f}, "
                f"{log.trim_y_deg[indices[-1]]:+.3f})"
            )
        start = end
    return lines


def metric_line(name: str, values: list[float]) -> str:
    """Return summary stats for one numeric array."""
    return (
        f"{name}: mean={mean(values):+.4f}, "
        f"p50={percentile(values, 0.50):+.4f}, "
        f"p95={percentile(values, 0.95):+.4f}, "
        f"p99={percentile(values, 0.99):+.4f}, "
        f"min={min(values):+.4f}, max={max(values):+.4f}"
    )


def mean(values: list[float]) -> float:
    """Return the arithmetic mean."""
    return statistics.fmean(values) if values else float("nan")


def percentile(values: list[float], ratio: float) -> float:
    """Return a simple nearest-rank percentile."""
    if not values:
        return float("nan")
    sorted_values = sorted(values)
    index = int((len(sorted_values) - 1) * ratio)
    return sorted_values[index]


def fraction(values: list[bool]) -> float:
    """Return the true fraction of boolean values."""
    return sum(1 for value in values if value) / len(values) if values else 0.0


if __name__ == "__main__":
    raise SystemExit(main())
