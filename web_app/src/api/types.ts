export type BasicResponse = {
  success: boolean;
  message: string;
};

export type Pose = {
  frame_id: string;
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number; w: number };
};

export type HealthResponse = BasicResponse & {
  motion_server: {
    get_state: boolean;
    movej: boolean;
    movel: boolean;
    move_named_state: boolean;
    cancel_active_action: boolean;
  };
  controller_manager: boolean;
  joint_state_recent: boolean;
  is_mock_hardware: string;
  servo_state: string;
  trajectory_preview: string;
};

export type StateResponse = BasicResponse & {
  mode: string;
  busy: boolean;
  active_task: string;
};

export type JointResponse = BasicResponse & {
  names: string[];
  positions: number[];
  velocities: number[];
  efforts: number[];
};

export type JointTarget = {
  names: string[];
  positions: number[];
};

export type PoseResponse = BasicResponse & Pose;

export type NamedStateResponse = BasicResponse & {
  joint_names: string[];
  states: Array<{ name: string; positions: number[] }>;
};

export type ControllerInfo = {
  name: string;
  state: string;
  type: string;
  claimed_interfaces?: string[];
  required_command_interfaces?: string[];
  required_state_interfaces?: string[];
};

export type ControllerResponse = BasicResponse & {
  controllers: ControllerInfo[];
};

export type ActionResponse = BasicResponse & {
  accepted: boolean;
  feedback: string[];
};

export type CancelActionResponse = BasicResponse;

export type StopResponse = BasicResponse;

export type SafeShutdownResponse = BasicResponse;

export type RobotModelResponse = BasicResponse & {
  urdf_url: string;
  asset_base_url: string;
  joint_state_source: string;
  joint_names: string[];
  root_link: string;
  description_source: string;
};

export type TelemetryJointState = {
  names: string[];
  positions: number[];
  velocities: number[];
  efforts: number[];
  stamp: { sec: number; nanosec: number };
};

export type TelemetryActiveAction = {
  kind: string;
  state: string;
  accepted: boolean;
  done: boolean;
  success: boolean | null;
  message: string;
  feedback: string[];
  termination_reason: string;
};

export type TelemetryRosout = {
  stamp: { sec: number; nanosec: number };
  level: number;
  name: string;
  message: string;
};

export type Telemetry = {
  stamp: number;
  latest_joints: TelemetryJointState | null;
  latest_joint_age_sec: number | null;
  active_action: TelemetryActiveAction;
  rosout: TelemetryRosout[];
};

export type StreamCommand =
  | { type: "speedj"; velocities: number[] }
  | { type: "speedl"; twist: number[]; frame_id?: string }
  | { type: "servoj"; joints: number[] }
  | {
      type: "servol";
      frame_id?: string;
      x: number;
      y: number;
      z: number;
      qx: number;
      qy: number;
      qz: number;
      qw: number;
    }
  | { type: "halt" };

export type BackendStatus = "connected" | "disconnected" | "unauthorized" | "error";

export type MotionServerStatus = "ready" | "unavailable" | "degraded";

export type ActionState =
  | "idle"
  | "sending"
  | "accepted"
  | "planning"
  | "executing"
  | "canceling"
  | "canceled"
  | "stopped"
  | "failed"
  | "done";

export type TelemetryFreshness = "fresh" | "stale" | "missing";

export type ConfirmDialogState = {
  title: string;
  message: string;
  confirmLabel: string;
  tone: "danger" | "warn";
  onConfirm: () => void;
};
