#!/usr/bin/env bash
#
# Usage:
#   ./scripts/launch_h0616_demo.sh [launch arguments]
#
# Examples:
#   ./scripts/launch_h0616_demo.sh
#   ./scripts/launch_h0616_demo.sh debug_enable:=true
#   ./scripts/launch_h0616_demo.sh use_mock_hardware:=true
#   ./scripts/launch_h0616_demo.sh use_mock_hardware:=true debug_enable:=true
#   ./scripts/launch_h0616_demo.sh --show-args
#
# Warning:
#   Without use_mock_hardware:=true, this launch file uses real EasyArm
#   hardware, matching:
#     ros2 launch easyarm_a1_h0616_moveit_config demo.launch.py
#
# This wrapper enters the workspace, sources install/setup.bash when present,
# and forwards all arguments to:
#   ros2 launch easyarm_a1_h0616_moveit_config demo.launch.py "$@"

set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${WORKSPACE_DIR}"

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
fi

exec ros2 launch easyarm_a1_h0616_moveit_config demo.launch.py "$@"
