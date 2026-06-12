#!/usr/bin/env python3
"""Plot single_motor_csp_demo CSV logs."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt
import pandas as pd


DEFAULT_CSV = Path("/tmp/single_motor_csp_demo.csv")
REQUIRED_COLUMNS = {
    "timestamp",
    "motor_id",
    "loc_cmd",
    "loc_state",
    "speed_cmd",
    "speed_state",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot Robstride CSP demo CSV data.")
    parser.add_argument(
        "csv",
        nargs="?",
        type=Path,
        default=DEFAULT_CSV,
        help=f"CSV file path, default: {DEFAULT_CSV}",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output image path. Default: result_[start]_[end].png next to this script.",
    )
    parser.add_argument("--start", type=float, default=None, help="Start time in seconds.")
    parser.add_argument("--end", type=float, default=None, help="End time in seconds.")
    parser.add_argument("--show", action="store_true", help="Show the plot window after saving.")
    args = parser.parse_args()
    if args.start is not None and args.end is not None and args.start > args.end:
        parser.error("--start must be less than or equal to --end")
    return args


def format_range_token(value: float | None) -> str:
    if value is None:
        return "all"
    return f"{value:g}".replace("-", "neg").replace(".", "p")


def default_output_path(start: float | None, end: float | None) -> Path:
    script_dir = Path(__file__).resolve().parent
    return script_dir / f"result_{format_range_token(start)}_{format_range_token(end)}.png"


def load_csv(path: Path) -> pd.DataFrame:
    data = pd.read_csv(path)
    missing = REQUIRED_COLUMNS.difference(data.columns)
    if missing:
        raise ValueError(f"CSV missing required columns: {', '.join(sorted(missing))}")

    data = data.sort_values(["motor_id", "timestamp"]).copy()
    data["time_s"] = data.groupby("motor_id")["timestamp"].transform(lambda x: x - x.iloc[0])
    data["loc_error"] = data["loc_cmd"] - data["loc_state"]
    data["speed_error"] = data["speed_cmd"] - data["speed_state"]
    return data


def filter_time_range(data: pd.DataFrame, start: float | None, end: float | None) -> pd.DataFrame:
    filtered = data
    if start is not None:
        filtered = filtered[filtered["time_s"] >= start]
    if end is not None:
        filtered = filtered[filtered["time_s"] <= end]
    return filtered.copy()


def plot(data: pd.DataFrame, output: Path, show: bool) -> None:
    motor_ids = sorted(data["motor_id"].dropna().unique())
    if not motor_ids:
        raise ValueError("CSV contains no motor_id data")

    fig, axes = plt.subplots(
        nrows=len(motor_ids),
        ncols=4,
        figsize=(22, max(4, 3.5 * len(motor_ids))),
        squeeze=False,
    )

    for row, motor_id in enumerate(motor_ids):
        motor_data = data[data["motor_id"] == motor_id]
        time_s = motor_data["time_s"].to_numpy()
        title_prefix = f"Motor {int(motor_id)}"

        ax = axes[row][0]
        ax.plot(time_s, motor_data["loc_cmd"].to_numpy(), label="loc_cmd")
        ax.plot(time_s, motor_data["loc_state"].to_numpy(), label="loc_state")
        ax.set_title(f"{title_prefix} position")
        ax.set_ylabel("rad")

        ax = axes[row][1]
        ax.plot(time_s, motor_data["loc_error"].to_numpy(), color="tab:red", label="loc_error")
        ax.set_title(f"{title_prefix} position error")
        ax.set_ylabel("rad")

        ax = axes[row][2]
        ax.plot(time_s, motor_data["speed_cmd"].to_numpy(), label="speed_cmd")
        ax.plot(time_s, motor_data["speed_state"].to_numpy(), label="speed_state")
        ax.set_title(f"{title_prefix} velocity")
        ax.set_ylabel("rad/s")

        ax = axes[row][3]
        ax.plot(time_s, motor_data["speed_error"].to_numpy(), color="tab:red", label="speed_error")
        ax.set_title(f"{title_prefix} velocity error")
        ax.set_ylabel("rad/s")

        for ax in axes[row]:
            ax.set_xlabel("time (s)")
            ax.grid(True)
            ax.legend()

    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f"Saved plot to {output}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> int:
    args = parse_args()
    data = filter_time_range(load_csv(args.csv), args.start, args.end)
    output = args.output if args.output is not None else default_output_path(args.start, args.end)
    plot(data, output, args.show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
