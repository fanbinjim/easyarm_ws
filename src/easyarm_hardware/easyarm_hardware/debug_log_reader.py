"""Read EasyArm hardware binary debug logs."""

from __future__ import annotations

import csv
import struct
from pathlib import Path
from typing import Iterable


HEADER_FORMAT = "<8s4I2q32s"
SAMPLE_PREFIX_FORMAT = "<Qqd4B5I"
JOINT_FORMAT = "<13d8B"
MAGIC = b"EAHDBG1\0"

HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
SAMPLE_PREFIX_SIZE = struct.calcsize(SAMPLE_PREFIX_FORMAT)
JOINT_SIZE = struct.calcsize(JOINT_FORMAT)

DEFAULT_LOG_GLOB = "/dev/shm/easyarm_log_*.bin"

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

DERIVED_FIELDS = [
    "position_error",
    "smoothed_position_error",
    "velocity_error",
    "motor_velocity_error",
]


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
    candidates = sorted(
        Path("/dev/shm").glob("easyarm_log_*.bin"),
        key=lambda path: path.stat().st_mtime,
    )
    if not candidates:
        raise FileNotFoundError(
            f"No debug log found at {DEFAULT_LOG_GLOB}. "
            "Start hardware with debug_enable=true first, or pass a .bin log path explicitly."
        )
    return candidates[-1]


def iter_samples(path: Path) -> Iterable[dict]:
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
