#!/usr/bin/env python3
"""Decode EasyArm hardware binary debug logs."""

from __future__ import annotations

import argparse
import csv
import re
import struct
from pathlib import Path


HEADER_FORMAT = "<8s4I2q32s"
SAMPLE_PREFIX_FORMAT = "<Qqd4B5I"
JOINT_FORMAT = "<13d8B"
MAGIC = b"EAHDBG1\0"

HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
SAMPLE_PREFIX_SIZE = struct.calcsize(SAMPLE_PREFIX_FORMAT)
JOINT_SIZE = struct.calcsize(JOINT_FORMAT)

DEFAULT_LOG_GLOB = "/dev/shm/easyarm_log_*.bin"
DEFAULT_OUTPUT_ROOT = Path("debug/plot")

JOINT_FIELDS = [
    "state_position",
    "state_velocity",
    "state_effort",
    "command_position",
    "command_velocity",
    "command_effort",
    "smoothed_position",
    "smoothed_velocity",
    "motor_position",
    "motor_velocity",
    "motor_torque",
    "kp",
    "kd",
]

PLOT_SPECS = [
    (
        "position",
        "rad",
        (
            ("command_position", "command"),
            ("state_position", "state"),
            ("smoothed_position", "smoothed"),
        ),
    ),
    (
        "position error",
        "rad",
        (
            ("position_error", "command - state"),
            ("smoothed_position_error", "smoothed - state"),
        ),
    ),
    (
        "velocity",
        "rad/s",
        (
            ("command_velocity", "command interface"),
            ("state_velocity", "state"),
            ("motor_velocity", "motor command"),
        ),
    ),
    (
        "velocity error",
        "rad/s",
        (
            ("velocity_error", "command interface - state"),
            ("motor_velocity_error", "motor command - state"),
        ),
    ),
]


def parse_bool(value: str) -> bool:
    normalized = value.lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Decode EasyArm hardware debug binary logs.")
    parser.add_argument(
        "log",
        type=Path,
        nargs="?",
        default=None,
        help=f"Input .bin log path. Default: newest {DEFAULT_LOG_GLOB}",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help=f"Output root directory. Default: {DEFAULT_OUTPUT_ROOT}",
    )
    parser.add_argument(
        "--plot",
        type=Path,
        default=None,
        help="Output PNG path for position/velocity/error plots. "
        "Default: <csv_stem>/<start|all>_<end|all>/all.png next to CSV.",
    )
    parser.add_argument(
        "--start",
        type=float,
        default=None,
        help="Plot start time in seconds from log start. Only affects --plot.",
    )
    parser.add_argument(
        "--end",
        type=float,
        default=None,
        help="Plot end time in seconds from log start. Only affects --plot.",
    )
    parser.add_argument(
        "--split",
        type=parse_bool,
        nargs="?",
        const="true",
        default=False,
        help="Also save each subplot as a separate PNG in "
        "<csv_stem>/<start|all>_<end|all>/split/ next to CSV. "
        "Accepts true/false. Default: false.",
    )
    return parser.parse_args()


def read_header(file_obj) -> dict:
    raw = file_obj.read(HEADER_SIZE)
    if len(raw) != HEADER_SIZE:
        raise ValueError("File is too short to contain a debug log header")

    (
        magic,
        version,
        header_size,
        sample_size,
        joint_count,
        start_steady_time_ns,
        start_system_time_ns,
        _,
    ) = struct.unpack(HEADER_FORMAT, raw)

    if magic != MAGIC:
        raise ValueError(f"Invalid magic {magic!r}; expected {MAGIC!r}")
    if version != 1:
        raise ValueError(f"Unsupported debug log version: {version}")
    if header_size != HEADER_SIZE:
        raise ValueError(f"Unexpected header size: {header_size}")
    if sample_size != SAMPLE_PREFIX_SIZE + joint_count * JOINT_SIZE:
        raise ValueError(f"Unexpected sample size: {sample_size}")

    return {
        "version": version,
        "sample_size": sample_size,
        "joint_count": joint_count,
        "start_steady_time_ns": start_steady_time_ns,
        "start_system_time_ns": start_system_time_ns,
    }


def latest_default_log_path() -> Path:
    candidates = sorted(Path("/dev/shm").glob("easyarm_log_*.bin"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(
            f"No debug log found at {DEFAULT_LOG_GLOB}. "
            "Start hardware with debug_enable=true first, or pass a .bin log path explicitly."
        )
    return candidates[-1]


def iter_samples(path: Path):
    with path.open("rb") as file_obj:
        header = read_header(file_obj)
        sample_size = header["sample_size"]
        joint_count = header["joint_count"]
        start_ns = header["start_steady_time_ns"]

        while True:
            raw = file_obj.read(sample_size)
            if not raw:
                break
            if len(raw) != sample_size:
                raise ValueError("Truncated sample at end of file")

            prefix = struct.unpack_from(SAMPLE_PREFIX_FORMAT, raw, 0)
            (
                seq,
                steady_time_ns,
                period_s,
                hardware_mode,
                motor_mode,
                skipped_from_joint,
                _reserved0,
                send_retry_count,
                send_fail_count,
                dropped_before,
                write_duration_us,
                _reserved1,
            ) = prefix

            offset = SAMPLE_PREFIX_SIZE
            for joint_index in range(joint_count):
                values = struct.unpack_from(JOINT_FORMAT, raw, offset)
                offset += JOINT_SIZE

                row = {
                    "seq": seq,
                    "time_s": (steady_time_ns - start_ns) * 1.0e-9,
                    "steady_time_ns": steady_time_ns,
                    "period_s": period_s,
                    "hardware_mode": hardware_mode,
                    "motor_mode": motor_mode,
                    "skipped_from_joint": skipped_from_joint,
                    "send_retry_count": send_retry_count,
                    "send_fail_count": send_fail_count,
                    "dropped_before": dropped_before,
                    "write_duration_us": write_duration_us,
                    "joint_index": joint_index,
                }
                for field, value in zip(JOINT_FIELDS, values[: len(JOINT_FIELDS)]):
                    row[field] = value
                row["motor_id"] = values[13]
                row["send_ok"] = values[14]
                row["position_error"] = row["command_position"] - row["state_position"]
                row["smoothed_position_error"] = (
                    row["smoothed_position"] - row["state_position"]
                )
                row["velocity_error"] = row["command_velocity"] - row["state_velocity"]
                row["motor_velocity_error"] = row["motor_velocity"] - row["state_velocity"]
                yield row


def write_csv(log_path: Path, output_path: Path) -> list[dict]:
    rows = list(iter_samples(log_path))
    if not rows:
        raise ValueError("Log contains no samples")

    fieldnames = list(rows[0].keys())
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def time_label(value: float | None) -> str:
    return "all" if value is None else str(value).replace(".", "_")


def time_window_dir_name(start_s: float | None, end_s: float | None) -> str:
    return f"{time_label(start_s)}_{time_label(end_s)}"


def output_base_dir(log_path: Path, output_root: Path) -> Path:
    return output_root / log_path.stem


def output_csv_path(log_path: Path, output_root: Path) -> Path:
    return output_base_dir(log_path, output_root) / f"{log_path.stem}.csv"


def default_plot_dir(csv_path: Path, start_s: float | None, end_s: float | None) -> Path:
    return csv_path.parent / time_window_dir_name(start_s, end_s)


def default_plot_path(csv_path: Path, start_s: float | None, end_s: float | None) -> Path:
    return default_plot_dir(csv_path, start_s, end_s) / "all.png"


def default_split_dir(csv_path: Path, start_s: float | None, end_s: float | None) -> Path:
    return default_plot_dir(csv_path, start_s, end_s) / "split"


def safe_plot_filename(title: str) -> str:
    return re.sub(r"_+", "_", re.sub(r"[^A-Za-z0-9_-]+", "_", title)).strip("_")


def plot_title(joint_id: int, spec_title: str) -> str:
    return f"Joint {joint_id + 1} {spec_title}"


def expanded_plot_specs():
    specs = []
    for spec_title, ylabel, series in PLOT_SPECS:
        specs.append((spec_title, ylabel, series))
        for field, label in series:
            specs.append((f"{spec_title} {label}", ylabel, ((field, label),)))
    return specs


def draw_spec(ax, time_s: list[float], rows: list[dict], title: str, ylabel: str, series):
    for field, label in series:
        ax.plot(time_s, [row[field] for row in rows], label=label)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend()


def filter_plot_rows(
    rows: list[dict],
    start_s: float | None,
    end_s: float | None,
) -> list[dict]:
    if start_s is not None and start_s < 0.0:
        raise ValueError("--start must be greater than or equal to 0")
    if end_s is not None and end_s < 0.0:
        raise ValueError("--end must be greater than or equal to 0")
    if start_s is not None and end_s is not None and end_s <= start_s:
        raise ValueError("--end must be greater than --start")

    filtered = [
        row
        for row in rows
        if (start_s is None or row["time_s"] >= start_s)
        and (end_s is None or row["time_s"] <= end_s)
    ]
    if not filtered:
        raise ValueError("No samples in selected plot time range")
    return filtered


def plot_rows(
    rows: list[dict],
    output_path: Path,
    start_s: float | None = None,
    end_s: float | None = None,
) -> None:
    import matplotlib.pyplot as plt

    rows = filter_plot_rows(rows, start_s, end_s)
    joint_ids = sorted({int(row["joint_index"]) for row in rows})
    min_time_s = min(row["time_s"] for row in rows)
    max_time_s = max(row["time_s"] for row in rows)
    xlim_start = start_s if start_s is not None else min_time_s
    xlim_end = end_s if end_s is not None else max_time_s
    plot_specs = expanded_plot_specs()
    fig, axes = plt.subplots(
        nrows=len(joint_ids),
        ncols=len(plot_specs),
        figsize=(4.5 * len(plot_specs), max(3, 2.5 * len(joint_ids))),
        squeeze=False,
    )

    for row_index, joint_id in enumerate(joint_ids):
        joint_rows = [row for row in rows if int(row["joint_index"]) == joint_id]
        time_s = [row["time_s"] for row in joint_rows]

        for column_index, (spec_title, ylabel, series) in enumerate(plot_specs):
            draw_spec(
                axes[row_index][column_index],
                time_s,
                joint_rows,
                plot_title(joint_id, spec_title),
                ylabel,
                series,
            )

    for ax in axes[-1]:
        ax.set_xlabel("time (s)")

    if xlim_end > xlim_start:
        for ax in axes.flat:
            ax.set_xlim(xlim_start, xlim_end)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_split_rows(
    rows: list[dict],
    output_dir: Path,
    start_s: float | None = None,
    end_s: float | None = None,
) -> int:
    import matplotlib.pyplot as plt

    rows = filter_plot_rows(rows, start_s, end_s)
    joint_ids = sorted({int(row["joint_index"]) for row in rows})
    min_time_s = min(row["time_s"] for row in rows)
    max_time_s = max(row["time_s"] for row in rows)
    xlim_start = start_s if start_s is not None else min_time_s
    xlim_end = end_s if end_s is not None else max_time_s
    plot_specs = expanded_plot_specs()

    output_dir.mkdir(parents=True, exist_ok=True)
    written_count = 0
    for joint_id in joint_ids:
        joint_rows = [row for row in rows if int(row["joint_index"]) == joint_id]
        time_s = [row["time_s"] for row in joint_rows]

        for spec_title, ylabel, series in plot_specs:
            title = plot_title(joint_id, spec_title)
            fig, ax = plt.subplots(figsize=(8, 4), squeeze=True)
            draw_spec(ax, time_s, joint_rows, title, ylabel, series)
            ax.set_xlabel("time (s)")
            if xlim_end > xlim_start:
                ax.set_xlim(xlim_start, xlim_end)
            fig.tight_layout()
            fig.savefig(output_dir / f"{safe_plot_filename(title)}.png", dpi=150)
            plt.close(fig)
            written_count += 1
    return written_count


def main() -> int:
    args = parse_args()
    log_path = args.log if args.log is not None else latest_default_log_path()
    csv_output = output_csv_path(log_path, args.output)
    rows = write_csv(log_path, csv_output)
    print(f"Wrote {len(rows)} CSV rows to {csv_output}")
    plot_output = args.plot if args.plot is not None else default_plot_path(
        csv_output,
        args.start,
        args.end,
    )
    plot_rows(rows, plot_output, args.start, args.end)
    print(f"Wrote plot to {plot_output}")
    if args.split:
        split_output_dir = default_split_dir(csv_output, args.start, args.end)
        split_count = plot_split_rows(rows, split_output_dir, args.start, args.end)
        print(f"Wrote {split_count} split plots to {split_output_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError) as error:
        print(f"Error: {error}")
        raise SystemExit(1)
