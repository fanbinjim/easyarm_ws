#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SHUTDOWN_SCRIPT="${WORKSPACE_DIR}/src/easyarm_a1_moveit_config/scripts/safe_shutdown_demo.sh"

cd "${WORKSPACE_DIR}"

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
fi

exec "${SHUTDOWN_SCRIPT}" "$@"
