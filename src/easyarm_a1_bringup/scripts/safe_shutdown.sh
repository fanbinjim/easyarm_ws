#!/usr/bin/env bash
set -euo pipefail

CONTROLLER_MANAGER="${CONTROLLER_MANAGER:-/controller_manager}"
ARM_CONTROLLER="${ARM_CONTROLLER:-arm_controller}"
HARDWARE_COMPONENT="${HARDWARE_COMPONENT:-EasyArmHardware}"
TERM_TIMEOUT_SECONDS="${TERM_TIMEOUT_SECONDS:-10}"
KILL_TIMEOUT_SECONDS="${KILL_TIMEOUT_SECONDS:-3}"
FORCE_KILL_ON_DISABLE_FAILURE="${FORCE_KILL_ON_DISABLE_FAILURE:-0}"
SKIP_STOP="${SKIP_STOP:-0}"
SKIP_SET_POSITION="${SKIP_SET_POSITION:-0}"
SKIP_MOVE_READY="${SKIP_MOVE_READY:-0}"
SKIP_HARDWARE_DISABLE="${SKIP_HARDWARE_DISABLE:-0}"
SKIP_KILL_LAUNCH="${SKIP_KILL_LAUNCH:-0}"
MOTION_TIMEOUT="${MOTION_TIMEOUT:-5.0}"

READY_JOINTS=(${READY_JOINTS:-0 1.85005 2.68781 0.9599 1.57 0})
READY_VELOCITY_SCALE="${READY_VELOCITY_SCALE:-0.2}"
READY_ACCELERATION_SCALE="${READY_ACCELERATION_SCALE:-0.2}"

DEFAULT_LAUNCH_TARGETS=(
  "easyarm_a1_bringup:bringup.launch.py"
  "easyarm_a1_moveit_config:demo.launch.py"
)
LAUNCH_TARGETS_TEXT="${EASYARM_LAUNCH_TARGETS:-${DEFAULT_LAUNCH_TARGETS[*]}}"
read -r -a LAUNCH_TARGETS <<< "${LAUNCH_TARGETS_TEXT}"

log() {
  printf '[safe_shutdown] %s\n' "$*"
}

run_step() {
  log "$*"
  "$@"
}

run_motion_shutdown() {
  local args=(
    --timeout "${MOTION_TIMEOUT}"
    --ready-joints "${READY_JOINTS[@]}"
    --velocity-scale "${READY_VELOCITY_SCALE}"
    --acceleration-scale "${READY_ACCELERATION_SCALE}"
  )

  if [[ "${SKIP_STOP}" == "1" ]]; then
    args+=(--skip-stop)
  fi
  if [[ "${SKIP_SET_POSITION}" == "1" ]]; then
    args+=(--skip-set-position)
  fi
  if [[ "${SKIP_MOVE_READY}" == "1" ]]; then
    args+=(--skip-move-ready)
  fi

  run_step ros2 run easyarm_a1_bringup safe_shutdown_motion "${args[@]}"
}

find_easyarm_launch_pids() {
  local package_name
  local launch_file
  local target

  for target in "${LAUNCH_TARGETS[@]}"; do
    package_name="${target%%:*}"
    launch_file="${target#*:}"
    [[ -z "${package_name}" || -z "${launch_file}" || "${package_name}" == "${launch_file}" ]] && continue
    pgrep -f "ros2 launch ${package_name} ${launch_file}" || true
    pgrep -f "${package_name}.*${launch_file}" || true
  done

  pgrep -f "ros2 launch easyarm.*bringup bringup.launch.py" || true
  pgrep -f "easyarm.*bringup.*bringup.launch.py" || true
  pgrep -f "ros2 launch easyarm.*moveit_config demo.launch.py" || true
  pgrep -f "easyarm.*moveit_config.*demo.launch.py" || true
}

collect_descendants() {
  local parent_pid="$1"
  local child_pid

  while read -r child_pid; do
    [[ -z "${child_pid}" ]] && continue
    printf '%s\n' "${child_pid}"
    collect_descendants "${child_pid}"
  done < <(pgrep -P "${parent_pid}" || true)
}

alive_pids() {
  local pid

  for pid in "$@"; do
    if kill -0 "${pid}" 2>/dev/null; then
      printf '%s\n' "${pid}"
    fi
  done
}

wait_for_exit() {
  local timeout_seconds="$1"
  shift
  local deadline=$((SECONDS + timeout_seconds))
  local remaining

  while (( SECONDS < deadline )); do
    mapfile -t remaining < <(alive_pids "$@")
    if (( ${#remaining[@]} == 0 )); then
      return 0
    fi
    sleep 0.5
  done

  return 1
}

disable_hardware() {
  log "Deactivating ${ARM_CONTROLLER}..."
  if ! ros2 control switch_controllers \
    --controller-manager "${CONTROLLER_MANAGER}" \
    --deactivate "${ARM_CONTROLLER}" \
    --strict; then
    log "Warning: failed to deactivate ${ARM_CONTROLLER}; continuing with hardware deactivation."
  fi

  log "Deactivating hardware component ${HARDWARE_COMPONENT}..."
  if ! ros2 control set_hardware_component_state \
    --controller-manager "${CONTROLLER_MANAGER}" \
    "${HARDWARE_COMPONENT}" inactive; then
    return 1
  fi

  log "Cleaning up hardware component ${HARDWARE_COMPONENT}..."
  if ! ros2 control set_hardware_component_state \
    --controller-manager "${CONTROLLER_MANAGER}" \
    "${HARDWARE_COMPONENT}" unconfigured; then
    return 1
  fi
}

stop_launch_tree() {
  local launch_pids=()
  local all_pids=()
  local child_pids=()
  local remaining_pids=()
  local pid

  mapfile -t launch_pids < <(find_easyarm_launch_pids | sort -u)
  if (( ${#launch_pids[@]} == 0 )); then
    log "No running EasyArm launch process found."
    return 0
  fi

  for pid in "${launch_pids[@]}"; do
    if [[ "${pid}" == "$$" ]]; then
      continue
    fi
    mapfile -t child_pids < <(collect_descendants "${pid}" | sort -u)
    all_pids+=("${child_pids[@]}" "${pid}")
  done

  mapfile -t all_pids < <(printf '%s\n' "${all_pids[@]}" | sort -rn -u)
  if (( ${#all_pids[@]} == 0 )); then
    log "No EasyArm launch process tree remains."
    return 0
  fi

  log "Sending SIGTERM to EasyArm launch process tree: ${all_pids[*]}"
  kill -TERM "${all_pids[@]}" 2>/dev/null || true

  if wait_for_exit "${TERM_TIMEOUT_SECONDS}" "${all_pids[@]}"; then
    log "EasyArm launch process tree stopped cleanly."
    return 0
  fi

  mapfile -t remaining_pids < <(alive_pids "${all_pids[@]}")
  log "Some processes did not exit after ${TERM_TIMEOUT_SECONDS}s. Sending SIGKILL: ${remaining_pids[*]}"
  kill -KILL "${remaining_pids[@]}" 2>/dev/null || true

  if wait_for_exit "${KILL_TIMEOUT_SECONDS}" "${remaining_pids[@]}"; then
    log "EasyArm launch process tree stopped."
    return 0
  fi

  log "Warning: some processes are still alive. Please check manually."
  return 1
}

if (( ${#READY_JOINTS[@]} != 6 )); then
  log "READY_JOINTS must contain 6 joint values, got ${#READY_JOINTS[@]}."
  exit 1
fi

run_motion_shutdown

if [[ "${SKIP_HARDWARE_DISABLE}" == "1" ]]; then
  log "SKIP_HARDWARE_DISABLE=1 set. Skipping ros2_control hardware shutdown."
else
  if ! disable_hardware; then
    log "Failed to disable hardware through ros2_control."
    if [[ "${FORCE_KILL_ON_DISABLE_FAILURE}" != "1" ]]; then
      log "Shutdown aborted before killing launch processes. Set FORCE_KILL_ON_DISABLE_FAILURE=1 to force process cleanup anyway."
      exit 1
    fi
    log "FORCE_KILL_ON_DISABLE_FAILURE=1 set. Continuing to kill launch processes."
  fi
fi

if [[ "${SKIP_KILL_LAUNCH}" == "1" ]]; then
  log "SKIP_KILL_LAUNCH=1 set. Skipping launch process cleanup."
else
  stop_launch_tree
fi
