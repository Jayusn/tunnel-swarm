#!/usr/bin/env bash
set -o pipefail
export ROS_MASTER_URI=http://localhost:11311
unset ROS_IP ROS_HOSTNAME
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$SCRIPT_DIR/experiment_control.py" stop
