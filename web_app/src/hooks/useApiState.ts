import { useCallback, useEffect, useRef, useState } from "react";
import { api, ApiError, isRecoverableError, recoverableMessage, errorMessage } from "../api/client";
import type {
  StateResponse,
  JointResponse,
  PoseResponse,
  NamedStateResponse,
  ControllerResponse,
  HealthResponse,
} from "../api/types";

export type ApiStateSnapshot = {
  state: StateResponse | null;
  joints: JointResponse | null;
  pose: PoseResponse | null;
  namedStates: NamedStateResponse | null;
  controllers: ControllerResponse | null;
  error: string;
  warning: string;
  loading: boolean;
  selectedNamedState: string;
  setSelectedNamedState: (name: string) => void;
};

export function useApiState(token: string, health: HealthResponse | null): ApiStateSnapshot {
  const [state, setState] = useState<StateResponse | null>(null);
  const [joints, setJoints] = useState<JointResponse | null>(null);
  const [pose, setPose] = useState<PoseResponse | null>(null);
  const [namedStates, setNamedStates] = useState<NamedStateResponse | null>(null);
  const [controllers, setControllers] = useState<ControllerResponse | null>(null);
  const [error, setError] = useState("");
  const [warning, setWarning] = useState("");
  const [loading, setLoading] = useState(false);
  const [selectedNamedState, setSelectedNamedState] = useState("");
  const lastControllersRefresh = useRef(0);
  const authRef = useRef(false);
  const namedSetRef = useRef(false);

  const poll = useCallback(async () => {
    if (!health || !token || authRef.current) return;
    setLoading(true);

    const tasks: Array<Promise<void>> = [];

    if (health.motion_server?.get_state) {
      tasks.push(
        api.state.then((d) => setState(d)).catch(() => undefined),
        api.joints.then((d) => setJoints(d)).catch(() => undefined),
        api.pose.then((d) => setPose(d)).catch(() => undefined),
        api.namedStates.then((d) => {
          setNamedStates(d);
          if (!namedSetRef.current && d.states.length > 0) {
            setSelectedNamedState(d.states[0].name);
            namedSetRef.current = true;
          }
        }).catch(() => undefined),
      );
    }

    const now = Date.now();
    if (health.controller_manager && now - lastControllersRefresh.current > 5000) {
      tasks.push(
        api.controllers.then((d) => {
          setControllers(d);
          lastControllersRefresh.current = now;
        }).catch(() => undefined),
      );
    }

    if (tasks.length === 0) {
      setError("");
      setWarning("");
      setLoading(false);
      return;
    }

    const results = await Promise.allSettled(tasks);
    let fatalError: unknown = null;
    let warningText = "";

    for (const result of results) {
      if (result.status !== "rejected") continue;
      if (result.reason instanceof ApiError && result.reason.isUnauthorized) {
        fatalError = result.reason;
        break;
      }
      const msg = isRecoverableError(result.reason) ? recoverableMessage(result.reason) : null;
      if (msg) {
        warningText ||= msg;
        continue;
      }
      fatalError = result.reason;
      break;
    }

    if (fatalError) {
      if (fatalError instanceof ApiError && fatalError.isUnauthorized) {
        authRef.current = true;
      }
      setError(errorMessage(fatalError));
      setWarning("");
    } else {
      setError("");
      setWarning(warningText);
      authRef.current = false;
    }
    setLoading(false);
  }, [health, token]);

  useEffect(() => {
    authRef.current = false;
    namedSetRef.current = false;
    poll();
    const interval = window.setInterval(poll, 1500);
    return () => window.clearInterval(interval);
  }, [poll]);

  return {
    state, joints, pose, namedStates, controllers,
    error, warning, loading,
    selectedNamedState, setSelectedNamedState,
  };
}
