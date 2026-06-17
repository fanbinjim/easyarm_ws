#!/usr/bin/env python3
"""Mass/COM calibration tool for EasyArm gravity compensation.

The collection flow moves the arm through static joint configurations, samples
actual joint position/effort from /joint_states, then fits link mass and Y/Z
center of mass with Pinocchio RNEA. COM.x is fixed to the URDF value. This tool
only writes YAML results; it does not change the runtime hardware configuration.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - only used outside ROS environments.
    get_package_share_directory = None

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - dependency error path.
    raise SystemExit("python3-numpy is required for mass_com_calibration") from exc

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover - dependency error path.
    raise SystemExit("pinocchio is required for mass_com_calibration") from exc

try:
    import yaml
except ImportError as exc:  # pragma: no cover - dependency error path.
    raise SystemExit("python3-yaml is required for mass_com_calibration") from exc


JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
LINK_NAMES = ["Link2", "Link3", "Link4", "Link5", "Link6"]
FITTED_JOINTS = ["Joint2", "Joint3", "Joint4", "Joint5"]
HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_DATA_FILE = "mass_com_calibration_data.jsonl"
HALF_PI = math.pi / 2.0
DESCRIPTION_CONFIG_NAME = "h0617"


@dataclass(frozen=True)
class MotionConfig:
    samples_per_config: int
    settle_time: float
    sample_interval: float
    motion_duration: float


@dataclass
class LinkPrior:
    link_name: str
    joint_name: str
    joint_id: int
    mass: float
    com: np.ndarray


class FlowList(list):
    pass


class ConfigYamlDumper(yaml.SafeDumper):
    pass


def represent_flow_list(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)


ConfigYamlDumper.add_representer(FlowList, represent_flow_list)


def flow_style_lists(value):
    if isinstance(value, list):
        return FlowList(flow_style_lists(item) for item in value)
    if isinstance(value, dict):
        return {key: flow_style_lists(item) for key, item in value.items()}
    return value


def package_source_dir() -> Path:
    def valid_package_dir(path: Path) -> bool:
        return (path / "CMakeLists.txt").is_file() and (path / "package.xml").is_file()

    cwd = Path.cwd().resolve()
    for parent in (cwd, *cwd.parents):
        if parent.name == "easyarm_dynamics" and valid_package_dir(parent):
            return parent

        candidate = parent / "src" / "easyarm_dynamics"
        if valid_package_dir(candidate):
            return candidate

    script_path = Path(__file__).resolve()
    for parent in (script_path.parent, *script_path.parents):
        if parent.name == "easyarm_dynamics" and valid_package_dir(parent):
            return parent

    for env_name in ("COLCON_PREFIX_PATH", "AMENT_PREFIX_PATH"):
        for raw_prefix in os.environ.get(env_name, "").split(os.pathsep):
            if not raw_prefix:
                continue
            prefix = Path(raw_prefix).resolve()
            workspace_roots: List[Path] = []
            if prefix.name == "install":
                workspace_roots.append(prefix.parent)
            elif prefix.parent.name == "install":
                workspace_roots.append(prefix.parent.parent)

            for root in workspace_roots:
                candidate = root / "src" / "easyarm_dynamics"
                if valid_package_dir(candidate):
                    return candidate

    return cwd / "src" / "easyarm_dynamics"


def results_dir() -> Path:
    return package_source_dir() / "calibration_results"


def description_config_dir(config_name: str = DESCRIPTION_CONFIG_NAME) -> Path:
    def valid_config_dir(path: Path) -> bool:
        return (
            (path / "inertials.yaml").is_file() and
            (path / "links.yaml").is_file() and
            (path / "joints.yaml").is_file()
        )

    cwd = Path.cwd().resolve()
    for parent in (cwd, *cwd.parents):
        if parent.name == "easyarm_description":
            candidate = parent / "config" / config_name
            if valid_config_dir(candidate):
                return candidate

        candidate = parent / "src" / "easyarm_description" / "config" / config_name
        if valid_config_dir(candidate):
            return candidate

    if get_package_share_directory is not None:
        try:
            candidate = Path(get_package_share_directory("easyarm_description")) / "config" / config_name
            if valid_config_dir(candidate):
                return candidate
        except Exception:
            pass

    return cwd / "src" / "easyarm_description" / "config" / config_name


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def default_urdf_path() -> Path:
    if get_package_share_directory is not None:
        try:
            share = Path(get_package_share_directory("easyarm_description"))
            return share / "urdf" / "easyarm_a1_h0616.urdf"
        except Exception:
            pass

    workspace_guess = Path.cwd() / "src" / "easyarm_description" / "urdf" / "easyarm_a1_h0616.urdf"
    return workspace_guess


def write_jsonl_meta(path: Path, mode: str, configs: Sequence[Sequence[float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    meta = {
        "meta": True,
        "mode": mode,
        "joint_names": JOINT_NAMES,
        "home": HOME_POSITION,
        "total_points": len(configs),
        "configs": [[float(v) for v in config] for config in configs],
        "start_time": datetime.now().isoformat(),
    }
    with path.open("w", encoding="utf-8") as f:
        f.write(json.dumps(meta) + "\n")


def append_jsonl_record(path: Path, record: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")
        f.flush()
        os.fsync(f.fileno())


def read_jsonl(path: Path) -> Tuple[Optional[Dict], List[Dict]]:
    meta = None
    records: List[Dict] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if obj.get("meta"):
                meta = obj
            else:
                records.append(obj)
    return meta, records


def dedup_records(records: Iterable[Dict]) -> List[Dict]:
    by_idx: Dict[int, Dict] = {}
    for record in records:
        by_idx[int(record["idx"])] = record
    return [by_idx[idx] for idx in sorted(by_idx)]


def add_unique_config(configs: List[List[float]], config: Sequence[float]) -> None:
    candidate = np.asarray(config, dtype=float)
    for existing in configs:
        if np.allclose(candidate, existing, atol=1e-9):
            return
    configs.append(candidate.tolist())


def radians(values: Sequence[float]) -> List[float]:
    return [math.radians(value) for value in values]


def degree_sweep(step_deg: int) -> List[float]:
    return radians(range(-90, 91, step_deg))


def joint23_pair_allowed(joint2: float, joint3: float) -> bool:
    eps = 1e-9
    if joint3 < -HALF_PI - eps or joint3 > HALF_PI + eps:
        return False
    if joint2 > eps and joint2 - joint3 >= HALF_PI - eps:
        return False
    if joint2 < -eps and joint2 - joint3 <= -HALF_PI + eps:
        return False
    return True


def generate_test_configurations(mode: str) -> List[List[float]]:
    configs: List[List[float]] = []
    home = np.asarray(HOME_POSITION, dtype=float)

    if mode == "quick":
        joint2_values = radians([-90.0, 0.0, 90.0])
        joint3_values = radians([-90.0, 0.0, 90.0])
        joint23_joint2 = radians([-90.0, 0.0, 90.0])
        joint23_joint3 = radians([-90.0, 0.0, 90.0])
        joint45_joint4 = radians([-90.0, 0.0, 90.0])
        joint45_joint5 = radians([-90.0, 0.0, 90.0])
    elif mode == "high":
        joint2_values = radians([-90.0, -67.5, -45.0, -22.5, 0.0, 22.5, 45.0, 67.5, 90.0])
        joint3_values = radians([-90.0, -67.5, -45.0, -22.5, 0.0, 22.5, 45.0, 67.5, 90.0])
        joint23_joint2 = radians([-90.0, -67.5, -45.0, -22.5, 0.0, 22.5, 45.0, 67.5, 90.0])
        joint23_joint3 = radians([-90.0, -67.5, -45.0, -22.5, 0.0, 22.5, 45.0, 67.5, 90.0])
        joint45_joint4 = radians([-90.0, -67.5, -45.0, -22.5, 0.0, 22.5, 45.0, 67.5, 90.0])
        joint45_joint5 = radians([-90.0, -67.5, -45.0, -22.5, 0.0, 22.5, 45.0, 67.5, 90.0])
    else:
        joint2_values = degree_sweep(10)
        joint3_values = degree_sweep(10)
        joint23_joint2 = degree_sweep(10)
        joint23_joint3 = degree_sweep(10)
        joint45_joint4 = degree_sweep(10)
        joint45_joint5 = degree_sweep(10)

    add_unique_config(configs, home)

    for value in joint2_values:
        cfg = home.copy()
        cfg[1] = value
        add_unique_config(configs, cfg)

    for value in joint3_values:
        cfg = home.copy()
        cfg[2] = value
        add_unique_config(configs, cfg)

    for joint2 in joint23_joint2:
        for joint3 in joint23_joint3:
            if not joint23_pair_allowed(joint2, joint3):
                continue
            cfg = home.copy()
            cfg[1] = joint2
            cfg[2] = joint3
            add_unique_config(configs, cfg)

    for joint4 in joint45_joint4:
        for joint5 in joint45_joint5:
            cfg = home.copy()
            cfg[3] = joint4
            cfg[4] = joint5
            add_unique_config(configs, cfg)

    return configs


def print_configurations(configs: Sequence[Sequence[float]]) -> None:
    print(f"Generated {len(configs)} calibration configurations:")
    for idx, cfg in enumerate(configs):
        values = " ".join(f"{name}={value:+.3f}" for name, value in zip(JOINT_NAMES, cfg))
        print(f"  [{idx:03d}] {values}")


def finite_vector(values: Sequence[float], length: int, field: str, idx: int) -> np.ndarray:
    if len(values) < length:
        raise ValueError(f"record {idx} field '{field}' has {len(values)} values, expected {length}")
    vector = np.asarray(values[:length], dtype=float)
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"record {idx} field '{field}' contains non-finite values")
    return vector


class MassComOptimizer:
    def __init__(
        self,
        urdf_path: Path,
        effort_sign: float,
        prior_weight: float = 0.03,
    ) -> None:
        self.urdf_path = urdf_path
        self.effort_sign = effort_sign
        self.prior_weight = prior_weight
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.link_priors = self._read_link_priors()
        self.fitted_joint_indices = [JOINT_NAMES.index(name) for name in FITTED_JOINTS]

    def _read_link_priors(self) -> List[LinkPrior]:
        priors: List[LinkPrior] = []
        for link_name in LINK_NAMES:
            joint_name = f"Joint{link_name[-1]}"
            joint_id = int(self.model.getJointId(joint_name))
            if joint_id <= 0 or joint_id >= self.model.njoints:
                names = ", ".join(str(name) for name in self.model.names)
                raise RuntimeError(f"joint '{joint_name}' for {link_name} not found in Pinocchio model: {names}")

            inertia = self.model.inertias[joint_id]
            priors.append(
                LinkPrior(
                    link_name=link_name,
                    joint_name=joint_name,
                    joint_id=joint_id,
                    mass=float(inertia.mass),
                    com=np.asarray(inertia.lever, dtype=float).copy(),
                )
            )
        return priors

    def initial_params(self) -> np.ndarray:
        values: List[float] = []
        for prior in self.link_priors:
            values.append(prior.mass)
            values.extend([float(prior.com[1]), float(prior.com[2])])
        return np.asarray(values, dtype=float)

    def bounds(self) -> List[Tuple[float, float]]:
        result: List[Tuple[float, float]] = []
        for prior in self.link_priors:
            mass_lower = max(0.001, prior.mass * 0.2)
            mass_upper = max(mass_lower + 0.001, prior.mass * 5.0)
            result.append((mass_lower, mass_upper))

            for value in prior.com[1:3]:
                lower = max(-0.25, float(value) - 0.12)
                upper = min(0.25, float(value) + 0.12)
                if upper <= lower:
                    upper = lower + 0.01
                result.append((lower, upper))
        return result

    def _apply_params(self, params: np.ndarray) -> None:
        offset = 0
        for prior in self.link_priors:
            mass = float(params[offset])
            inertia = self.model.inertias[prior.joint_id]
            inertia.mass = mass
            inertia.lever[0] = float(prior.com[0])
            inertia.lever[1] = float(params[offset + 1])
            inertia.lever[2] = float(params[offset + 2])
            offset += 3

    def _gravity(self, q: np.ndarray, params: np.ndarray) -> np.ndarray:
        self._apply_params(params)
        q_pin = np.zeros(self.model.nq)
        count = min(len(q), self.model.nq)
        q_pin[:count] = q[:count]
        v = np.zeros(self.model.nv)
        a = np.zeros(self.model.nv)
        data = pin.Data(self.model)
        tau = pin.rnea(self.model, data, q_pin, v, a)
        return np.asarray(tau, dtype=float)[:len(JOINT_NAMES)]

    def torque_residuals(self, params: np.ndarray, samples: Sequence[Tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
        residuals: List[float] = []
        for q, measured_effort in samples:
            predicted = self._gravity(q, params)
            measured = measured_effort * self.effort_sign
            for idx in self.fitted_joint_indices:
                residuals.append(float(predicted[idx] - measured[idx]))
        return np.asarray(residuals, dtype=float)

    def prior_residuals(self, params: np.ndarray) -> np.ndarray:
        residuals: List[float] = []
        offset = 0
        for prior in self.link_priors:
            mass = float(params[offset])
            com_yz = params[offset + 1: offset + 3]
            mass_scale = max(abs(prior.mass), 1e-6)
            residuals.append(self.prior_weight * (mass - prior.mass) / mass_scale)
            residuals.extend((self.prior_weight * (com_yz - prior.com[1:3]) / 0.05).tolist())
            offset += 3
        return np.asarray(residuals, dtype=float)

    def objective(self, params: np.ndarray, samples: Sequence[Tuple[np.ndarray, np.ndarray]]) -> float:
        torque = self.torque_residuals(params, samples)
        prior = self.prior_residuals(params)
        return float(np.dot(torque, torque) + np.dot(prior, prior))

    def optimize(self, records: Sequence[Dict]) -> Dict:
        try:
            from scipy.optimize import minimize
        except ImportError as exc:  # pragma: no cover - dependency error path.
            raise RuntimeError("python3-scipy is required for optimization") from exc

        samples = records_to_samples(records)
        if len(samples) < 5:
            raise RuntimeError(f"too few valid samples ({len(samples)}), need at least 5")

        initial = self.initial_params()
        initial_torque_res = self.torque_residuals(initial, samples)
        initial_rmse = rmse(initial_torque_res)

        result = minimize(
            lambda params: self.objective(params, samples),
            initial,
            method="L-BFGS-B",
            bounds=self.bounds(),
            options={"maxiter": 800, "ftol": 1e-10},
        )

        optimized = np.asarray(result.x, dtype=float)
        torque_res = self.torque_residuals(optimized, samples)
        final_rmse = rmse(torque_res)
        r_squared = compute_r_squared(optimized, samples, self)

        return {
            "success": bool(result.success),
            "message": str(result.message),
            "params": optimized,
            "initial_rmse": float(initial_rmse),
            "rmse": float(final_rmse),
            "r_squared": float(r_squared),
            "num_samples": len(samples),
        }

    def params_to_yaml(self, params: np.ndarray) -> Dict:
        output: Dict[str, Dict] = {}
        offset = 0
        for prior in self.link_priors:
            mass = float(params[offset])
            com = [
                float(prior.com[0]),
                float(params[offset + 1]),
                float(params[offset + 2]),
            ]
            output[prior.link_name] = {
                "mass": mass,
                "com": com,
                "urdf_mass": float(prior.mass),
                "urdf_com": [float(v) for v in prior.com.tolist()],
                "first_moment": [float(mass * v) for v in com],
                "fixed_com_axes": ["x"],
                "optimized_com_axes": ["y", "z"],
            }
            offset += 3
        return output


def rmse(residuals: np.ndarray) -> float:
    if residuals.size == 0:
        return 0.0
    return float(math.sqrt(float(np.mean(np.square(residuals)))))


def records_to_samples(records: Sequence[Dict]) -> List[Tuple[np.ndarray, np.ndarray]]:
    samples: List[Tuple[np.ndarray, np.ndarray]] = []
    for record in records:
        idx = int(record.get("idx", len(samples)))
        q = finite_vector(record["position"], len(JOINT_NAMES), "position", idx)
        effort = finite_vector(record["effort"], len(JOINT_NAMES), "effort", idx)
        samples.append((q, effort))
    return samples


def compute_r_squared(params: np.ndarray, samples: Sequence[Tuple[np.ndarray, np.ndarray]], optimizer: MassComOptimizer) -> float:
    measured_values: List[float] = []
    predicted_values: List[float] = []
    for q, measured_effort in samples:
        predicted = optimizer._gravity(q, params)
        measured = measured_effort * optimizer.effort_sign
        for idx in optimizer.fitted_joint_indices:
            measured_values.append(float(measured[idx]))
            predicted_values.append(float(predicted[idx]))

    if len(measured_values) < 2:
        return 0.0

    measured_arr = np.asarray(measured_values, dtype=float)
    predicted_arr = np.asarray(predicted_values, dtype=float)
    ss_res = float(np.sum(np.square(predicted_arr - measured_arr)))
    ss_tot = float(np.sum(np.square(measured_arr - float(np.mean(measured_arr)))))
    if ss_tot <= 0.0:
        return 0.0
    return 1.0 - ss_res / ss_tot


def load_records_for_optimization(data_file: Path) -> List[Dict]:
    if not data_file.exists():
        raise RuntimeError(f"data file does not exist: {data_file}")
    _, records = read_jsonl(data_file)
    records = dedup_records(records)
    if not records:
        raise RuntimeError(f"no calibration records found in {data_file}")
    return records


def write_config_yaml(path: Path, data: Dict) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.dump(
            flow_style_lists(data),
            f,
            Dumper=ConfigYamlDumper,
            sort_keys=False,
            allow_unicode=False,
            default_flow_style=False,
        )


def export_description_config_from_result(result_yaml_path: Path, config_output_dir: Optional[Path] = None) -> Path:
    with result_yaml_path.open("r", encoding="utf-8") as f:
        result_doc = yaml.safe_load(f)

    if not isinstance(result_doc, dict) or "mass_com_params" not in result_doc:
        raise RuntimeError(f"invalid calibration result YAML: {result_yaml_path}")

    template_dir = description_config_dir()
    if config_output_dir is None:
        config_output_dir = result_yaml_path.with_suffix("")
        config_output_dir = config_output_dir.parent / f"{config_output_dir.name}_config"

    with (template_dir / "inertials.yaml").open("r", encoding="utf-8") as f:
        inertials = yaml.safe_load(f)
    with (template_dir / "links.yaml").open("r", encoding="utf-8") as f:
        links = yaml.safe_load(f)
    with (template_dir / "joints.yaml").open("r", encoding="utf-8") as f:
        joints = yaml.safe_load(f)

    mass_com_params = result_doc["mass_com_params"]
    for link_name in LINK_NAMES:
        if link_name not in mass_com_params:
            raise RuntimeError(f"calibration result missing {link_name}")
        if link_name not in inertials:
            raise RuntimeError(f"description config missing {link_name}")

        link_result = mass_com_params[link_name]
        mass = float(link_result["mass"])
        com = finite_vector(link_result["com"], 3, f"{link_name}.com", 0)
        if mass <= 0.0 or not math.isfinite(mass):
            raise RuntimeError(f"{link_name} has invalid calibrated mass: {mass}")

        inertials[link_name]["mass"] = mass
        inertials[link_name]["origin"]["xyz"] = [float(value) for value in com.tolist()]

    config_output_dir.mkdir(parents=True, exist_ok=True)
    write_config_yaml(config_output_dir / "inertials.yaml", inertials)
    write_config_yaml(config_output_dir / "links.yaml", links)
    write_config_yaml(config_output_dir / "joints.yaml", joints)

    metadata = {
        "source_calibration_yaml": str(result_yaml_path),
        "source_template_config": str(template_dir),
        "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "note": "Config export for easyarm_description h0617 xacro; inertia matrices are copied from the template config.",
    }
    write_config_yaml(config_output_dir / "calibration_info.yaml", metadata)
    return config_output_dir


def validate_resume_data(meta: Optional[Dict], records: Sequence[Dict], configs: Sequence[Sequence[float]], data_file: Path) -> None:
    if meta:
        total_points = meta.get("total_points")
        if total_points is not None and int(total_points) != len(configs):
            raise RuntimeError(
                f"data file {data_file} was collected with {total_points} configurations, "
                f"current mode generates {len(configs)}; use --restart or a different --data-file"
            )

        meta_configs = meta.get("configs")
        if meta_configs:
            if len(meta_configs) != len(configs):
                raise RuntimeError(
                    f"data file {data_file} has {len(meta_configs)} stored configurations, "
                    f"current mode generates {len(configs)}; use --restart or a different --data-file"
                )
            for idx, (recorded, expected) in enumerate(zip(meta_configs, configs)):
                recorded_config = np.asarray(recorded, dtype=float)
                expected_config = np.asarray(expected, dtype=float)
                if recorded_config.shape != expected_config.shape or not np.allclose(recorded_config, expected_config, atol=1e-6):
                    raise RuntimeError(
                        f"data file {data_file} configuration {idx} does not match current sampling grid; "
                        "use --restart or a different --data-file"
                    )

    for record in records:
        idx = int(record["idx"])
        if idx >= len(configs):
            raise RuntimeError(
                f"data file {data_file} has record index {idx}, but current mode only has {len(configs)} configurations; "
                "use --restart or a different --data-file"
            )
        if "config" not in record:
            continue
        recorded_config = finite_vector(record["config"], len(JOINT_NAMES), "config", idx)
        expected_config = np.asarray(configs[idx], dtype=float)
        if not np.allclose(recorded_config, expected_config, atol=1e-6):
            raise RuntimeError(
                f"data file {data_file} record {idx} does not match current sampling grid; "
                "use --restart or a different --data-file"
            )


def save_yaml_result(
    output_path: Path,
    optimizer: MassComOptimizer,
    optimization: Dict,
    mode: str,
    data_file: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    params = np.asarray(optimization["params"], dtype=float)
    doc = {
        "use_calibrated_params": True,
        "source_urdf": str(optimizer.urdf_path),
        "joint_names": JOINT_NAMES,
        "fitted_joints": FITTED_JOINTS,
        "mass_com_params": optimizer.params_to_yaml(params),
        "calibration_info": {
            "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "mode": mode,
            "data_file": str(data_file),
            "num_samples": int(optimization["num_samples"]),
            "initial_rmse": float(optimization["initial_rmse"]),
            "rmse": float(optimization["rmse"]),
            "r_squared": float(optimization["r_squared"]),
            "optimizer_success": bool(optimization["success"]),
            "optimizer_message": str(optimization["message"]),
            "fixed_com_axes": ["x"],
            "note": "gravity-only calibration; mass and COM.y/z are constrained by URDF priors; COM.x is fixed to the URDF value",
        },
    }

    with output_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=False)

    config_output_dir = export_description_config_from_result(output_path)
    print(f"Description config saved: {config_output_dir}")


class CollectionNode:
    def __init__(self, motion_config: MotionConfig) -> None:
        import rclpy
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState
        from control_msgs.action import FollowJointTrajectory

        self.rclpy = rclpy
        self.FollowJointTrajectory = FollowJointTrajectory
        self.node = rclpy.create_node("easyarm_mass_com_calibrator")
        self.motion_config = motion_config
        self.positions = [float("nan")] * len(JOINT_NAMES)
        self.efforts = [float("nan")] * len(JOINT_NAMES)
        self.received_state = False
        self.subscription = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )
        self.action_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

    def destroy(self) -> None:
        self.node.destroy_node()

    def _joint_state_callback(self, msg) -> None:
        for i, joint_name in enumerate(JOINT_NAMES):
            if joint_name not in msg.name:
                continue
            idx = msg.name.index(joint_name)
            if idx < len(msg.position):
                self.positions[i] = float(msg.position[idx])
            if idx < len(msg.effort):
                self.efforts[i] = float(msg.effort[idx])
        self.received_state = True

    def wait_for_joint_states(self, timeout_sec: float = 10.0) -> bool:
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.received_state and self._state_is_complete():
                return True
        return False

    def _state_is_complete(self) -> bool:
        return all(math.isfinite(v) for v in self.positions) and all(math.isfinite(v) for v in self.efforts)

    def move_to_position(self, target: Sequence[float]) -> bool:
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Action server /arm_controller/follow_joint_trajectory is unavailable")
            return False

        goal_msg = self.FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in target]
        point.velocities = [0.0] * len(JOINT_NAMES)
        duration = self.motion_config.motion_duration
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        goal_msg.trajectory.points = [point]

        future = self.action_client.send_goal_async(goal_msg)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("Trajectory goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        self.rclpy.spin_until_future_complete(
            self.node,
            result_future,
            timeout_sec=duration + 10.0,
        )
        if not result_future.done():
            self.node.get_logger().error("Trajectory goal timed out")
            return False
        return True

    def collect_samples(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        positions: List[List[float]] = []
        efforts: List[List[float]] = []
        interval = self.motion_config.sample_interval

        for _ in range(self.motion_config.samples_per_config):
            start = time.monotonic()
            self.rclpy.spin_once(self.node, timeout_sec=interval)
            if self._state_is_complete():
                positions.append(list(self.positions))
                efforts.append(list(self.efforts))
            elapsed = time.monotonic() - start
            if elapsed < interval:
                time.sleep(interval - elapsed)

        if not positions:
            raise RuntimeError("no complete /joint_states samples were collected")

        pos_arr = np.asarray(positions, dtype=float)
        eff_arr = np.asarray(efforts, dtype=float)
        return (
            np.mean(pos_arr, axis=0),
            np.mean(eff_arr, axis=0),
            np.std(pos_arr, axis=0),
            np.std(eff_arr, axis=0),
        )


def confirm_collection(args: argparse.Namespace, configs: Sequence[Sequence[float]]) -> None:
    if args.yes:
        return
    print("")
    print("This will move the real robot through calibration configurations.")
    print(f"Mode: {args.mode}, points: {len(configs)}")
    print("The first commanded position is home: [0, 0, 0, 0, 0, 0].")
    answer = input("Type YES to continue: ")
    if answer.strip() != "YES":
        raise SystemExit("aborted")


def collect_data(args: argparse.Namespace, configs: Sequence[Sequence[float]], data_file: Path) -> None:
    import rclpy

    motion_config = MotionConfig(
        samples_per_config=args.samples_per_config,
        settle_time=args.settle_time,
        sample_interval=args.sample_interval,
        motion_duration=args.motion_duration,
    )

    if args.restart and data_file.exists():
        data_file.unlink()

    if data_file.exists():
        meta, records = read_jsonl(data_file)
        if not args.resume:
            raise RuntimeError(f"data file already exists: {data_file}; use --resume or --restart")
        if meta and meta.get("mode") != args.mode:
            raise RuntimeError(f"data file mode is {meta.get('mode')}, requested {args.mode}; use --restart")
        records = dedup_records(records)
        validate_resume_data(meta, records, configs, data_file)
        completed = len(records)
    else:
        write_jsonl_meta(data_file, args.mode, configs)
        completed = 0

    if completed >= len(configs):
        print(f"All {completed} configurations already collected in {data_file}")
        return

    rclpy.init(args=None)
    collector = CollectionNode(motion_config)
    try:
        if not collector.wait_for_joint_states():
            raise RuntimeError("failed to receive complete /joint_states with effort data")

        print(f"Collecting {len(configs) - completed} remaining points into {data_file}")
        print("Moving to home before calibration...")
        if not collector.move_to_position(HOME_POSITION):
            raise RuntimeError("failed to move to home position")
        time.sleep(motion_config.settle_time)

        for idx in range(completed, len(configs)):
            target = configs[idx]
            print(f"[{idx + 1}/{len(configs)}] target={format_vector(target)}")
            if not collector.move_to_position(target):
                print("  move failed, skipping")
                continue

            time.sleep(motion_config.settle_time)
            pos, eff, pos_std, eff_std = collector.collect_samples()

            append_jsonl_record(
                data_file,
                {
                    "idx": idx,
                    "config": [float(v) for v in target],
                    "position": [float(v) for v in pos.tolist()],
                    "effort": [float(v) for v in eff.tolist()],
                    "position_std": [float(v) for v in pos_std.tolist()],
                    "effort_std": [float(v) for v in eff_std.tolist()],
                    "timestamp": datetime.now().isoformat(),
                },
            )

            print(
                "  position="
                f"{format_vector(pos)} effort={format_vector(eff)} "
                f"effort_std={format_vector(eff_std)}"
            )

        print("Returning to home...")
        collector.move_to_position(HOME_POSITION)
    finally:
        collector.destroy()
        rclpy.shutdown()


def format_vector(values: Sequence[float]) -> str:
    return "[" + ", ".join(f"{float(v):+.4f}" for v in values) + "]"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate EasyArm link mass and COM from static gravity data.")
    parser.add_argument("--urdf", type=Path, default=default_urdf_path(), help="URDF path; defaults to easyarm_a1_h0616.urdf")
    parser.add_argument("--output", type=Path, default=None, help="Output YAML path")
    parser.add_argument("--data-file", type=Path, default=None, help="JSONL data path")
    parser.add_argument("--mode", choices=["quick", "full", "high"], default="full", help="Calibration sample density")
    parser.add_argument("--restart", action="store_true", help="Delete existing JSONL data and collect from scratch")
    parser.add_argument("--resume", action="store_true", help="Resume from an existing JSONL data file")
    parser.add_argument("--optimize-only", action="store_true", help="Only fit parameters from an existing JSONL data file")
    parser.add_argument("--dry-run-configs", action="store_true", help="Print generated configurations and exit")
    parser.add_argument("--samples-per-config", type=int, default=40, help="Number of /joint_states samples per point")
    parser.add_argument("--settle-time", type=float, default=1.5, help="Settling time after each motion, seconds")
    parser.add_argument("--sample-interval", type=float, default=0.02, help="Sampling interval, seconds")
    parser.add_argument("--motion-duration", type=float, default=5.0, help="Trajectory duration per point, seconds")
    parser.add_argument("--effort-sign", type=float, choices=[1.0, -1.0], default=1.0, help="Measured effort sign")
    parser.add_argument("--yes", action="store_true", help="Skip interactive safety confirmation")
    args = parser.parse_args(argv)

    if args.samples_per_config <= 0:
        parser.error("--samples-per-config must be positive")
    if args.settle_time < 0.0:
        parser.error("--settle-time must be non-negative")
    if args.sample_interval <= 0.0:
        parser.error("--sample-interval must be positive")
    if args.motion_duration <= 0.0:
        parser.error("--motion-duration must be positive")
    if args.restart and args.resume:
        parser.error("--restart and --resume cannot be used together")

    if args.data_file is None:
        args.data_file = results_dir() / DEFAULT_DATA_FILE
    if args.output is None:
        args.output = results_dir() / f"mass_com_params_{timestamp()}.yaml"

    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    configs = generate_test_configurations(args.mode)

    if args.dry_run_configs:
        try:
            print_configurations(configs)
        except BrokenPipeError:
            devnull = os.open(os.devnull, os.O_WRONLY)
            os.dup2(devnull, sys.stdout.fileno())
        return 0

    if not args.urdf.exists():
        print(f"URDF does not exist: {args.urdf}", file=sys.stderr)
        return 2

    try:
        if not args.optimize_only:
            confirm_collection(args, configs)
            collect_data(args, configs, args.data_file)

        records = load_records_for_optimization(args.data_file)
        optimizer = MassComOptimizer(args.urdf, effort_sign=args.effort_sign)
        optimization = optimizer.optimize(records)
        save_yaml_result(args.output, optimizer, optimization, args.mode, args.data_file)

        print("")
        print(f"Optimization success: {optimization['success']} ({optimization['message']})")
        print(f"Samples: {optimization['num_samples']}")
        print(f"Initial RMSE: {optimization['initial_rmse']:.5f} Nm")
        print(f"Final RMSE:   {optimization['rmse']:.5f} Nm")
        print(f"R^2:          {optimization['r_squared']:.5f}")
        print(f"YAML saved:   {args.output}")
        return 0 if optimization["success"] else 1
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
