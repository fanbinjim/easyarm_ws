#!/usr/bin/env bash
set -euo pipefail

PACKAGE_NAME="easyarm_a1_moveit_config"
LAUNCH_FILE="demo.launch.py"
CONTROLLER_MANAGER="/controller_manager"
ARM_CONTROLLER="arm_controller"
HARDWARE_COMPONENT="EasyArmHardware"
TERM_TIMEOUT_SECONDS=10
KILL_TIMEOUT_SECONDS=3
FORCE_KILL_ON_DISABLE_FAILURE="${FORCE_KILL_ON_DISABLE_FAILURE:-0}"

log() {
  printf '[safe_shutdown_demo] %s\n' "$*"
}

find_demo_launch_pids() {
  pgrep -f "ros2 launch ${PACKAGE_NAME} ${LAUNCH_FILE}" || true
  pgrep -f "${PACKAGE_NAME}.*${LAUNCH_FILE}" || true
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

log "Moving arm to ready before shutdown..."
if ! ros2 run "${PACKAGE_NAME}" move_to_ready; then
  log "move_to_ready failed. Shutdown aborted so the arm is not left in an unknown state."
  exit 1
fi

if ! disable_hardware; then
  log "Failed to disable hardware through ros2_control."
  if [[ "${FORCE_KILL_ON_DISABLE_FAILURE}" != "1" ]]; then
    log "Shutdown aborted before killing demo processes. Set FORCE_KILL_ON_DISABLE_FAILURE=1 to force process cleanup anyway."
    exit 1
  fi
  log "FORCE_KILL_ON_DISABLE_FAILURE=1 set. Continuing to kill demo processes."
fi

mapfile -t launch_pids < <(find_demo_launch_pids | sort -u)
if (( ${#launch_pids[@]} == 0 )); then
  log "No running ${PACKAGE_NAME} ${LAUNCH_FILE} process found."
  exit 0
fi

all_pids=()
for pid in "${launch_pids[@]}"; do
  if [[ "${pid}" == "$$" ]]; then
    continue
  fi
  mapfile -t child_pids < <(collect_descendants "${pid}" | sort -u)
  all_pids+=("${child_pids[@]}" "${pid}")
done

mapfile -t all_pids < <(printf '%s\n' "${all_pids[@]}" | sort -rn -u)
if (( ${#all_pids[@]} == 0 )); then
  log "No demo process tree remains after moving to ready."
  exit 0
fi

log "Sending SIGTERM to demo process tree: ${all_pids[*]}"
kill -TERM "${all_pids[@]}" 2>/dev/null || true

if wait_for_exit "${TERM_TIMEOUT_SECONDS}" "${all_pids[@]}"; then
  log "Demo process tree stopped cleanly."
  exit 0
fi

mapfile -t remaining_pids < <(alive_pids "${all_pids[@]}")
log "Some processes did not exit after ${TERM_TIMEOUT_SECONDS}s. Sending SIGKILL: ${remaining_pids[*]}"
kill -KILL "${remaining_pids[@]}" 2>/dev/null || true

if wait_for_exit "${KILL_TIMEOUT_SECONDS}" "${remaining_pids[@]}"; then
  log "Demo process tree stopped."
  exit 0
fi

log "Warning: some processes are still alive. Please check manually."
exit 1
