#!/usr/bin/env python3
"""Plot recorded EasyArm end-effector trajectory in 3D."""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt


def load_ee_points(record_path: Path) -> tuple[list[float], list[float], list[float]]:
    with record_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    samples = data.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("JSON field 'samples' must be a non-empty list")

    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []

    for index, sample in enumerate(samples):
        try:
            translation = sample["ee_pose"]["translation"]
        except (KeyError, TypeError) as exc:
            raise ValueError(
                f"samples[{index}] missing ee_pose.translation") from exc

        if not isinstance(translation, list) or len(translation) != 3:
            raise ValueError(
                f"samples[{index}].ee_pose.translation must contain 3 values")

        xs.append(float(translation[0]))
        ys.append(float(translation[1]))
        zs.append(float(translation[2]))

    return xs, ys, zs


def set_axes_equal(ax) -> None:
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    radius = 0.5 * max(x_range, y_range, z_range)

    x_middle = sum(x_limits) * 0.5
    y_middle = sum(y_limits) * 0.5
    z_middle = sum(z_limits) * 0.5

    ax.set_xlim3d([x_middle - radius, x_middle + radius])
    ax.set_ylim3d([y_middle - radius, y_middle + radius])
    ax.set_zlim3d([z_middle - radius, z_middle + radius])


def plot_trajectory(
    record_path: Path,
    output_path: Path | None,
    title: str | None,
    show_points: bool,
) -> None:
    xs, ys, zs = load_ee_points(record_path)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(xs, ys, zs, linewidth=2.0, label="EE trajectory")

    if show_points:
        ax.scatter(xs, ys, zs, s=8, alpha=0.45)

    ax.scatter([xs[0]], [ys[0]], [zs[0]], s=60, marker="o", label="start")
    ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], s=60, marker="^", label="end")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(title if title else f"EE trajectory: {record_path.name}")
    ax.legend()
    ax.grid(True)
    set_axes_equal(ax)
    fig.tight_layout()

    if output_path:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150)
        print(f"Saved plot to {output_path}")
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot samples[].ee_pose.translation from an EasyArm record JSON")
    parser.add_argument("record", type=Path, help="Path to easyarm_record JSON")
    parser.add_argument(
        "-o", "--output", type=Path,
        help="Save figure to image instead of opening an interactive window")
    parser.add_argument("--title", help="Custom plot title")
    parser.add_argument(
        "--show-points", action="store_true",
        help="Draw recorded samples as scatter points")
    args = parser.parse_args()

    record_path = args.record.expanduser()
    if not record_path.is_file():
        raise FileNotFoundError(f"Record file not found: {record_path}")

    output_path = args.output.expanduser() if args.output else None
    plot_trajectory(record_path, output_path, args.title, args.show_points)


if __name__ == "__main__":
    main()
