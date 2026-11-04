#!/usr/bin/env bash
# Only open the saved-photo / pixel viewer; no camera, ROS or detector startup.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /bin/bash "$SCRIPT_DIR/open_crack_pixels.sh" "$@"
