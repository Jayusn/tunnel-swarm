#!/usr/bin/env bash
set -o pipefail
export ROS_MASTER_URI=http://localhost:11311
unset ROS_IP ROS_HOSTNAME
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
if [[ -f "$SCRIPT_DIR/crack_capture.env" ]]; then
  source "$SCRIPT_DIR/crack_capture.env" || exit 1
fi
if [[ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
  echo "未找到编译环境：$WORKSPACE_DIR/devel/setup.bash，请先编译工作空间。"
  exit 1
fi
source "$WORKSPACE_DIR/devel/setup.bash" || exit 1
exec /usr/bin/python3 "$SCRIPT_DIR/experiment_control.py" start "$@"
