#!/usr/bin/env bash
#
# Usage:
#   ./scripts/decode_debug_log.sh [log] [options]
#
# Examples:
#   ./scripts/decode_debug_log.sh
#   ./scripts/decode_debug_log.sh /dev/shm/easyarm_log_xxx.bin
#   ./scripts/decode_debug_log.sh --start 2 --end 8 --split
#   ./scripts/decode_debug_log.sh --help
#
# This wrapper enters the workspace, sources install/setup.bash when present,
# and forwards all arguments to:
#   ros2 run easyarm_hardware decode_debug_log "$@"

set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${WORKSPACE_DIR}"

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
fi

exec ros2 run easyarm_hardware decode_debug_log "$@"
