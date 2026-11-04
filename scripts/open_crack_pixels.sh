#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
if [[ -f "$SCRIPT_DIR/crack_capture.env" ]]; then
  source "$SCRIPT_DIR/crack_capture.env"
fi
exec /usr/bin/python3 "$SCRIPT_DIR/crack_pixel_viewer.py" \
  --root "${CRACK_CAPTURE_DIR:-$WORKSPACE_DIR/captures/crack_live}" "$@"
