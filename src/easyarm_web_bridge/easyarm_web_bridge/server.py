import argparse
import asyncio
import os
import shlex
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import ListControllers
from easyarm_interfaces.action import MoveJ, MoveL, MoveNamedState
from easyarm_interfaces.srv import GetJoints, GetPose, GetState, ListNamedState, SetMode, Stop
from fastapi import Body, Depends, FastAPI, Header, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import uvicorn


JOINT_NAMES = [f"Joint{index}" for index in range(1, 7)]


@dataclass
class ActionSnapshot:
    kind: str = ""
    state: str = "idle"
    accepted: bool = False
    done: bool = True
    success: Optional[bool] = None
    message: str = ""
    feedback: List[str] = field(default_factory=list)


def _now() -> float:
    return time.time()


def _to_float_list(values, size: Optional[int] = None) -> List[float]:
    result = [float(value) for value in values]
    if size is not None and len(result) != size:
        raise ValueError(f"expected {size} values, got {len(result)}")
    return result


def _pose_to_dict(message: PoseStamped) -> Dict[str, Any]:
    return {
        "frame_id": message.header.frame_id,
        "position": {
            "x": message.pose.position.x,
            "y": message.pose.position.y,
            "z": message.pose.position.z,
        },
        "orientation": {
            "x": message.pose.orientation.x,
            "y": message.pose.orientation.y,
            "z": message.pose.orientation.z,
            "w": message.pose.orientation.w,
        },
    }


def _joint_state_to_dict(message: JointState) -> Dict[str, Any]:
    return {
        "names": list(message.name),
        "positions": list(message.position),
        "velocities": list(message.velocity),
        "efforts": list(message.effort),
        "stamp": {
            "sec": int(message.header.stamp.sec),
            "nanosec": int(message.header.stamp.nanosec),
        },
    }


class EasyArmWebBridge(Node):
    def __init__(self):
        super().__init__("easyarm_web_bridge")

        self.declare_parameter("host", os.environ.get("EASYARM_WEB_BACKEND_HOST", "127.0.0.1"))
        self.declare_parameter("port", int(os.environ.get("EASYARM_WEB_BACKEND_PORT", "8000")))
        self.declare_parameter("token", os.environ.get("EASYARM_WEB_TOKEN", ""))
        self.declare_parameter("request_timeout_sec", 5.0)
        self.declare_parameter("stream_idle_timeout_sec", 0.75)
        self.declare_parameter(
            "safe_shutdown_command",
            "ros2 run easyarm_a1_bringup safe_shutdown.sh",
        )
        self.declare_parameter("safe_shutdown_log_dir", "")

        self._lock = threading.Lock()
        self._safe_shutdown_lock = threading.Lock()
        self._latest_joint_state: Optional[JointState] = None
        self._latest_joint_state_time = 0.0
        self._rosout: List[Dict[str, Any]] = []
        self._active_action = ActionSnapshot()
        self._active_goal_handle = None
        self._active_stream_kind = ""
        self._last_stream_command_time = 0.0
        self._safe_shutdown_process: Optional[subprocess.Popen] = None
        self._safe_shutdown_log_path = ""

        self.movej_client = ActionClient(self, MoveJ, "/easyarm/movej")
        self.movel_client = ActionClient(self, MoveL, "/easyarm/movel")
        self.move_named_state_client = ActionClient(self, MoveNamedState, "/easyarm/move_named_state")

        self.set_mode_client = self.create_client(SetMode, "/easyarm/set_mode")
        self.stop_client = self.create_client(Stop, "/easyarm/stop")
        self.get_state_client = self.create_client(GetState, "/easyarm/get_state")
        self.get_joints_client = self.create_client(GetJoints, "/easyarm/get_joints")
        self.get_pose_client = self.create_client(GetPose, "/easyarm/get_pose")
        self.list_named_state_client = self.create_client(ListNamedState, "/easyarm/list_named_state")
        self.list_controllers_client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
        )

        self.speedj_pub = self.create_publisher(JointJog, "/easyarm/speedj_cmd", 10)
        self.speedl_pub = self.create_publisher(TwistStamped, "/easyarm/speedl_cmd", 10)
        self.servoj_pub = self.create_publisher(JointTrajectory, "/easyarm/servoj_cmd", 10)
        self.servol_pub = self.create_publisher(PoseStamped, "/easyarm/servol_cmd", 10)

        self.create_subscription(JointState, "/joint_states", self._handle_joint_state, 10)
        self.create_subscription(Log, "/rosout", self._handle_rosout, 50)

    @property
    def host(self) -> str:
        return str(self.get_parameter("host").value)

    @property
    def port(self) -> int:
        return int(str(self.get_parameter("port").value))

    @property
    def token(self) -> str:
        return str(self.get_parameter("token").value)

    @property
    def request_timeout_sec(self) -> float:
        return float(self.get_parameter("request_timeout_sec").value)

    @property
    def stream_idle_timeout_sec(self) -> float:
        return float(self.get_parameter("stream_idle_timeout_sec").value)

    @property
    def safe_shutdown_command(self) -> str:
        return str(self.get_parameter("safe_shutdown_command").value)

    @property
    def safe_shutdown_log_dir(self) -> str:
        return str(self.get_parameter("safe_shutdown_log_dir").value)

    def _handle_joint_state(self, message: JointState) -> None:
        with self._lock:
            self._latest_joint_state = message
            self._latest_joint_state_time = _now()

    def _handle_rosout(self, message: Log) -> None:
        if not (
            message.name.startswith("easyarm")
            or "move_group" in message.name
            or "controller" in message.name
        ):
            return
        item = {
            "stamp": {
                "sec": int(message.stamp.sec),
                "nanosec": int(message.stamp.nanosec),
            },
            "level": int(message.level),
            "name": message.name,
            "message": message.msg,
        }
        with self._lock:
            self._rosout.append(item)
            self._rosout = self._rosout[-100:]

    def _wait_future(self, future, timeout: Optional[float] = None):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise TimeoutError("Timed out waiting for ROS response")
        return future.result()

    def _call_service(self, client, request, timeout: Optional[float] = None):
        timeout = self.request_timeout_sec if timeout is None else timeout
        if not client.wait_for_service(timeout_sec=timeout):
            service_name = getattr(client, "srv_name", "requested")
            raise RuntimeError(f"{service_name} service is not available")
        return self._wait_future(client.call_async(request), timeout)

    def get_state(self) -> Dict[str, Any]:
        response = self._call_service(self.get_state_client, GetState.Request())
        return {
            "success": bool(response.success),
            "message": response.message,
            "mode": response.mode,
            "busy": bool(response.busy),
            "active_task": response.active_task,
        }

    def get_joints(self) -> Dict[str, Any]:
        response = self._call_service(self.get_joints_client, GetJoints.Request())
        return {
            "success": bool(response.success),
            "message": response.message,
            "names": list(response.names),
            "positions": list(response.positions),
            "velocities": list(response.velocities),
            "efforts": list(response.efforts),
        }

    def get_pose(self, target_frame: str = "", source_frame: str = "") -> Dict[str, Any]:
        request = GetPose.Request()
        request.target_frame = target_frame
        request.source_frame = source_frame
        response = self._call_service(self.get_pose_client, request)
        data = _pose_to_dict(response.pose)
        data.update({"success": bool(response.success), "message": response.message})
        return data

    def list_named_states(self) -> Dict[str, Any]:
        response = self._call_service(self.list_named_state_client, ListNamedState.Request())
        width = len(response.joint_names)
        states = []
        for index, name in enumerate(response.names):
            start = index * width
            values = list(response.positions[start:start + width])
            states.append({"name": name, "positions": values})
        return {
            "success": bool(response.success),
            "message": response.message,
            "joint_names": list(response.joint_names),
            "states": states,
        }

    def list_controllers(self) -> Dict[str, Any]:
        response = self._call_service(self.list_controllers_client, ListControllers.Request())
        controllers = []
        for controller in response.controller:
            controllers.append({
                "name": controller.name,
                "state": controller.state,
                "type": controller.type,
                "claimed_interfaces": list(controller.claimed_interfaces),
                "required_command_interfaces": list(controller.required_command_interfaces),
                "required_state_interfaces": list(controller.required_state_interfaces),
            })
        return {"success": True, "message": "OK", "controllers": controllers}

    def set_mode(self, mode: str) -> Dict[str, Any]:
        request = SetMode.Request()
        request.mode = mode
        response = self._call_service(self.set_mode_client, request)
        return {"success": bool(response.success), "message": response.message}

    def stop(self) -> Dict[str, Any]:
        response = self._call_service(self.stop_client, Stop.Request())
        with self._lock:
            self._active_action.state = "stopped"
            self._active_action.done = True
        return {"success": bool(response.success), "message": response.message}

    def cancel_active_action(self) -> Dict[str, Any]:
        with self._lock:
            goal_handle = self._active_goal_handle
        if goal_handle is None:
            return {"success": False, "message": "No active action goal"}
        self._wait_future(goal_handle.cancel_goal_async(), self.request_timeout_sec)
        return {"success": True, "message": "Cancel requested"}

    def _set_action_snapshot(self, **updates) -> None:
        with self._lock:
            for key, value in updates.items():
                setattr(self._active_action, key, value)

    def _append_action_feedback(self, state: str) -> None:
        with self._lock:
            self._active_action.state = state
            self._active_action.feedback.append(state)
            self._active_action.feedback = self._active_action.feedback[-20:]

    def send_movej(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        goal = MoveJ.Goal()
        goal.joints = _to_float_list(payload.get("joints", []), 6)
        goal.velocity_scale = float(payload.get("velocity_scale", 0.2))
        goal.acceleration_scale = float(payload.get("acceleration_scale", 0.2))
        goal.execute = bool(payload.get("execute", False))
        return self._send_action("MoveJ", self.movej_client, goal)

    def send_movel(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        pose = payload.get("pose", payload)
        orientation = pose.get("orientation", {})
        position = pose.get("position", {})
        goal = MoveL.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = str(payload.get("frame_id", pose.get("frame_id", "base_link")))
        goal.target_pose.pose.position.x = float(position.get("x", payload.get("x", 0.0)))
        goal.target_pose.pose.position.y = float(position.get("y", payload.get("y", 0.0)))
        goal.target_pose.pose.position.z = float(position.get("z", payload.get("z", 0.0)))
        goal.target_pose.pose.orientation.x = float(orientation.get("x", payload.get("qx", 0.0)))
        goal.target_pose.pose.orientation.y = float(orientation.get("y", payload.get("qy", 0.0)))
        goal.target_pose.pose.orientation.z = float(orientation.get("z", payload.get("qz", 0.0)))
        goal.target_pose.pose.orientation.w = float(orientation.get("w", payload.get("qw", 1.0)))
        goal.velocity_scale = float(payload.get("velocity_scale", 0.1))
        goal.acceleration_scale = float(payload.get("acceleration_scale", 0.1))
        goal.execute = bool(payload.get("execute", False))
        return self._send_action("MoveL", self.movel_client, goal)

    def send_move_named_state(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        goal = MoveNamedState.Goal()
        goal.name = str(payload.get("name", ""))
        goal.velocity_scale = float(payload.get("velocity_scale", 0.2))
        goal.acceleration_scale = float(payload.get("acceleration_scale", 0.2))
        goal.execute = bool(payload.get("execute", False))
        if not goal.name:
            raise ValueError("name is required")
        return self._send_action("MoveNamedState", self.move_named_state_client, goal)

    def _send_action(self, kind: str, client, goal) -> Dict[str, Any]:
        if not client.wait_for_server(timeout_sec=self.request_timeout_sec):
            raise RuntimeError(f"{kind} action server is not available")

        self._set_action_snapshot(
            kind=kind,
            state="sending",
            accepted=False,
            done=False,
            success=None,
            message="",
            feedback=[],
        )

        send_future = client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._append_action_feedback(feedback.feedback.state),
        )
        goal_handle = self._wait_future(send_future, self.request_timeout_sec)
        if goal_handle is None or not goal_handle.accepted:
            self._set_action_snapshot(state="rejected", accepted=False, done=True, success=False, message="Goal rejected")
            return {"success": False, "message": "Goal rejected", "accepted": False, "feedback": []}

        with self._lock:
            self._active_goal_handle = goal_handle
        self._set_action_snapshot(state="accepted", accepted=True)

        result_future = goal_handle.get_result_async()
        wrapped_result = self._wait_future(result_future, None)
        result = wrapped_result.result
        with self._lock:
            feedback = list(self._active_action.feedback)
            self._active_goal_handle = None
        self._set_action_snapshot(
            state="done" if result.success else "failed",
            done=True,
            success=bool(result.success),
            message=result.message,
        )
        return {
            "success": bool(result.success),
            "message": result.message,
            "accepted": True,
            "feedback": feedback,
        }

    def make_joint_jog(self, velocities: List[float]) -> JointJog:
        message = JointJog()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.joint_names = JOINT_NAMES
        message.velocities = _to_float_list(velocities, 6)
        return message

    def make_twist(self, values: List[float], frame_id: str = "base_link") -> TwistStamped:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        values = _to_float_list(values, 6)
        message.twist.linear.x = values[0]
        message.twist.linear.y = values[1]
        message.twist.linear.z = values[2]
        message.twist.angular.x = values[3]
        message.twist.angular.y = values[4]
        message.twist.angular.z = values[5]
        return message

    def make_servoj(self, joints: List[float]) -> JointTrajectory:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = _to_float_list(joints, 6)
        message.points = [point]
        return message

    def make_servol(self, payload: Dict[str, Any]) -> PoseStamped:
        pose = payload.get("pose", payload)
        position = pose.get("position", {})
        orientation = pose.get("orientation", {})
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(payload.get("frame_id", pose.get("frame_id", "base_link")))
        message.pose.position.x = float(position.get("x", payload.get("x", 0.0)))
        message.pose.position.y = float(position.get("y", payload.get("y", 0.0)))
        message.pose.position.z = float(position.get("z", payload.get("z", 0.0)))
        message.pose.orientation.x = float(orientation.get("x", payload.get("qx", 0.0)))
        message.pose.orientation.y = float(orientation.get("y", payload.get("qy", 0.0)))
        message.pose.orientation.z = float(orientation.get("z", payload.get("qz", 0.0)))
        message.pose.orientation.w = float(orientation.get("w", payload.get("qw", 1.0)))
        return message

    def _mark_stream_command(self, kind: str) -> None:
        with self._lock:
            self._active_stream_kind = kind
            self._last_stream_command_time = _now()

    def _consume_active_stream_kind(self) -> str:
        with self._lock:
            kind = self._active_stream_kind
            age = _now() - self._last_stream_command_time
            if not kind or age > self.stream_idle_timeout_sec:
                self._active_stream_kind = ""
                self._last_stream_command_time = 0.0
                return ""
            self._active_stream_kind = ""
            self._last_stream_command_time = 0.0
            return kind

    def publish_stream_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        command_type = str(payload.get("type", "")).lower()
        if command_type == "speedj":
            message = self.make_joint_jog(payload.get("velocities", []))
            self._mark_stream_command(command_type)
            self.speedj_pub.publish(message)
        elif command_type == "speedl":
            message = self.make_twist(
                payload.get("twist", payload.get("values", [])),
                payload.get("frame_id", "base_link"),
            )
            self._mark_stream_command(command_type)
            self.speedl_pub.publish(message)
        elif command_type == "servoj":
            message = self.make_servoj(payload.get("joints", []))
            self._mark_stream_command(command_type)
            self.servoj_pub.publish(message)
        elif command_type == "servol":
            message = self.make_servol(payload)
            self._mark_stream_command(command_type)
            self.servol_pub.publish(message)
        elif command_type == "halt":
            halted = self.stop_stream()
            return {"success": True, "message": "halted active stream" if halted else "no active stream"}
        else:
            raise ValueError(f"Unknown stream command type '{command_type}'")
        return {"success": True, "message": "published"}

    def publish_halt(self) -> bool:
        kind = self._consume_active_stream_kind()
        if not kind:
            self.get_logger().debug("Ignore stream halt: no active stream command")
            return False
        if kind == "speedj":
            self.speedj_pub.publish(self.make_joint_jog([0.0] * 6))
        elif kind == "speedl":
            self.speedl_pub.publish(self.make_twist([0.0] * 6))
        return True

    def stop_stream(self) -> bool:
        halted = self.publish_halt()
        if halted:
            self.stop()
        return halted

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            joint_state = self._latest_joint_state
            joint_state_time = self._latest_joint_state_time
            rosout = list(self._rosout)
            action = {
                "kind": self._active_action.kind,
                "state": self._active_action.state,
                "accepted": self._active_action.accepted,
                "done": self._active_action.done,
                "success": self._active_action.success,
                "message": self._active_action.message,
                "feedback": list(self._active_action.feedback),
            }
        latest_joints = _joint_state_to_dict(joint_state) if joint_state is not None else None
        return {
            "stamp": _now(),
            "latest_joints": latest_joints,
            "latest_joint_age_sec": None if joint_state is None else max(0.0, _now() - joint_state_time),
            "active_action": action,
            "rosout": rosout[-30:],
        }

    def health(self) -> Dict[str, Any]:
        with self._lock:
            joint_ok = self._latest_joint_state is not None and (_now() - self._latest_joint_state_time) < 1.0
        return {
            "success": True,
            "message": "OK",
            "motion_server": {
                "get_state": self.get_state_client.service_is_ready(),
                "movej": self.movej_client.server_is_ready(),
                "movel": self.movel_client.server_is_ready(),
                "move_named_state": self.move_named_state_client.server_is_ready(),
            },
            "controller_manager": self.list_controllers_client.service_is_ready(),
            "joint_state_recent": joint_ok,
            "is_mock_hardware": "unknown",
            "servo_state": "reserved",
            "trajectory_preview": "reserved",
        }

    def _safe_shutdown_log_file(self) -> Path:
        base_dir = self.safe_shutdown_log_dir
        if not base_dir:
            base_dir = os.environ.get("ROS_LOG_DIR", str(Path.home() / ".ros" / "log"))
        log_dir = Path(base_dir).expanduser() / "easyarm_web_bridge"
        log_dir.mkdir(parents=True, exist_ok=True)
        return log_dir / f"safe_shutdown_{time.strftime('%Y%m%d-%H%M%S')}.log"

    @staticmethod
    def _bool_env(value: Any) -> str:
        if isinstance(value, bool):
            return "1" if value else "0"
        if isinstance(value, (int, float)):
            return "1" if value else "0"
        text = str(value).strip().lower()
        return "1" if text in {"1", "true", "yes", "on"} else "0"

    def _safe_shutdown_env(self, payload: Dict[str, Any]) -> Dict[str, str]:
        env = os.environ.copy()
        bool_fields = {
            "skip_stop": "SKIP_STOP",
            "skip_set_position": "SKIP_SET_POSITION",
            "skip_move_ready": "SKIP_MOVE_READY",
            "skip_hardware_disable": "SKIP_HARDWARE_DISABLE",
            "skip_kill_launch": "SKIP_KILL_LAUNCH",
            "force_kill_on_disable_failure": "FORCE_KILL_ON_DISABLE_FAILURE",
        }
        for payload_key, env_key in bool_fields.items():
            if payload_key in payload:
                env[env_key] = self._bool_env(payload[payload_key])

        value_fields = {
            "motion_timeout": "MOTION_TIMEOUT",
            "ready_velocity_scale": "READY_VELOCITY_SCALE",
            "ready_acceleration_scale": "READY_ACCELERATION_SCALE",
            "term_timeout_seconds": "TERM_TIMEOUT_SECONDS",
            "kill_timeout_seconds": "KILL_TIMEOUT_SECONDS",
            "controller_manager": "CONTROLLER_MANAGER",
            "arm_controller": "ARM_CONTROLLER",
            "hardware_component": "HARDWARE_COMPONENT",
            "launch_targets": "EASYARM_LAUNCH_TARGETS",
        }
        for payload_key, env_key in value_fields.items():
            if payload_key in payload:
                env[env_key] = str(payload[payload_key])

        if "ready_joints" in payload:
            ready_joints = payload["ready_joints"]
            if not isinstance(ready_joints, list) or len(ready_joints) != 6:
                raise ValueError("ready_joints must be a list with 6 values")
            env["READY_JOINTS"] = " ".join(str(float(value)) for value in ready_joints)

        return env

    def start_safe_shutdown(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._safe_shutdown_lock:
            if self._safe_shutdown_process is not None and self._safe_shutdown_process.poll() is None:
                return {
                    "success": False,
                    "message": "safe shutdown is already running",
                    "pid": self._safe_shutdown_process.pid,
                    "log_path": self._safe_shutdown_log_path,
                }

            command = shlex.split(self.safe_shutdown_command)
            if not command:
                raise ValueError("safe_shutdown_command is empty")

            log_path = self._safe_shutdown_log_file()
            env = self._safe_shutdown_env(payload)
            with log_path.open("a", encoding="utf-8") as log_file:
                log_file.write(f"# {' '.join(command)}\n")
                process = subprocess.Popen(
                    command,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    env=env,
                    start_new_session=True,
                )

            self._safe_shutdown_process = process
            self._safe_shutdown_log_path = str(log_path)

            return {
                "success": True,
                "message": "safe shutdown started",
                "pid": process.pid,
                "log_path": self._safe_shutdown_log_path,
            }


def create_app(bridge: EasyArmWebBridge) -> FastAPI:
    app = FastAPI(title="EasyArm Web Bridge", version="0.1.0")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    def check_token(
        authorization: str = Header(default=""),
        x_easyarm_token: str = Header(default=""),
        token: str = Query(default=""),
    ) -> None:
        expected = bridge.token
        if not expected:
            raise HTTPException(status_code=503, detail="web_token is not configured")
        candidate = token or x_easyarm_token
        if authorization.lower().startswith("bearer "):
            candidate = authorization.split(" ", 1)[1]
        if candidate != expected:
            raise HTTPException(status_code=401, detail="invalid token")

    def run_blocking(callable_, *args):
        return callable_(*args)

    async def call_ros(callable_, *args):
        loop = asyncio.get_running_loop()
        try:
            return await loop.run_in_executor(None, run_blocking, callable_, *args)
        except TimeoutError as exc:
            raise HTTPException(status_code=504, detail=str(exc)) from exc
        except (RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.get("/api/health", dependencies=[Depends(check_token)])
    async def health():
        return await call_ros(bridge.health)

    @app.get("/api/state", dependencies=[Depends(check_token)])
    async def state():
        return await call_ros(bridge.get_state)

    @app.get("/api/joints", dependencies=[Depends(check_token)])
    async def joints():
        return await call_ros(bridge.get_joints)

    @app.get("/api/pose", dependencies=[Depends(check_token)])
    async def pose(target_frame: str = "", source_frame: str = ""):
        return await call_ros(bridge.get_pose, target_frame, source_frame)

    @app.get("/api/named-states", dependencies=[Depends(check_token)])
    async def named_states():
        return await call_ros(bridge.list_named_states)

    @app.get("/api/controllers", dependencies=[Depends(check_token)])
    async def controllers():
        return await call_ros(bridge.list_controllers)

    @app.post("/api/set-mode", dependencies=[Depends(check_token)])
    async def set_mode(payload: Dict[str, Any]):
        return await call_ros(bridge.set_mode, str(payload.get("mode", "")))

    @app.post("/api/stop", dependencies=[Depends(check_token)])
    async def stop():
        return await call_ros(bridge.stop)

    @app.post("/api/safe-shutdown", dependencies=[Depends(check_token)])
    async def safe_shutdown(payload: Optional[Dict[str, Any]] = Body(default=None)):
        return await call_ros(bridge.start_safe_shutdown, payload or {})

    @app.post("/api/actions/active/cancel", dependencies=[Depends(check_token)])
    async def cancel_action():
        return await call_ros(bridge.cancel_active_action)

    @app.post("/api/movej", dependencies=[Depends(check_token)])
    async def movej(payload: Dict[str, Any]):
        return await call_ros(bridge.send_movej, payload)

    @app.post("/api/movel", dependencies=[Depends(check_token)])
    async def movel(payload: Dict[str, Any]):
        return await call_ros(bridge.send_movel, payload)

    @app.post("/api/move-named-state", dependencies=[Depends(check_token)])
    async def move_named_state(payload: Dict[str, Any]):
        return await call_ros(bridge.send_move_named_state, payload)

    async def accept_ws(websocket: WebSocket) -> bool:
        token = websocket.query_params.get("token", "")
        if not bridge.token or token != bridge.token:
            await websocket.close(code=1008)
            return False
        await websocket.accept()
        return True

    @app.websocket("/ws/telemetry")
    async def telemetry(websocket: WebSocket):
        if not await accept_ws(websocket):
            return
        try:
            while True:
                await websocket.send_json(bridge.snapshot())
                await asyncio.sleep(0.5)
        except WebSocketDisconnect:
            return

    @app.websocket("/ws/command-stream")
    async def command_stream(websocket: WebSocket):
        if not await accept_ws(websocket):
            return
        try:
            while True:
                payload = await websocket.receive_json()
                try:
                    response = bridge.publish_stream_command(payload)
                    await websocket.send_json(response)
                except Exception as exc:
                    await websocket.send_json({"success": False, "message": str(exc)})
        except WebSocketDisconnect:
            try:
                bridge.stop_stream()
            except Exception:
                pass

    return app


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run EasyArm Web Bridge.")
    parser.parse_known_args(argv)

    rclpy.init(args=argv)
    bridge = EasyArmWebBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = create_app(bridge)
    config = uvicorn.Config(app, host=bridge.host, port=bridge.port, log_level="info")
    server = uvicorn.Server(config)

    try:
        server.run()
    finally:
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
