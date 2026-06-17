#!/usr/bin/env bash
#
# Usage:
#   ./scripts/switch_controller_mode.sh <IDLE|POSITION|DRAG>
#
# Examples:
#   ./scripts/switch_controller_mode.sh IDLE
#   ./scripts/switch_controller_mode.sh DRAG
#   ./scripts/switch_controller_mode.sh POSITION
#
# Notes:
#   IDLE     Pure damping mode.
#   DRAG     Gravity-compensated drag mode.
#   POSITION Normal trajectory/position mode. The underlying tool first sends
#            a hold trajectory at the current joint positions before switching.
#
# This wrapper enters the workspace, sources install/setup.bash when present,
# and forwards the mode to:
#   ros2 run easyarm_move_task switch_controller_mode <mode>

set -euo pipefail

usage() {
  sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -ne 1 ]]; then
  usage >&2
  exit 1
fi

MODE="${1^^}"
case "${MODE}" in
  IDLE|POSITION|DRAG)
    ;;
  *)
    printf 'Error: unknown mode "%s". Expected IDLE, POSITION, or DRAG.\n\n' "$1" >&2
    usage >&2
    exit 1
    ;;
esac

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${WORKSPACE_DIR}"

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
fi

exec ros2 run easyarm_move_task switch_controller_mode "${MODE}"
