#!/usr/bin/env bash

# UAV1 颜色、二维码、热源检测及四分屏一键入口。
# 独立于原有传感器、检测和规划启动脚本，不改变它们的默认行为。

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${UAV1_FOUR_VIEW_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
STACK_PID=""
VIEW_PID=""

usage() {
  cat <<'EOF'
用法：
  bash shfiles/start_uav1_four_view.sh

作用：
  启动 UAV1 D435、颜色标签、普通二维码、热源检测、热源-D435 映射，
  然后打开只显示四分屏的 RViz。

停止：
  在本终端按 Ctrl+C，会同时关闭本脚本启动的检测和四分屏进程。
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  usage >&2
  exit 2
fi

[[ -r "${ROS_SETUP}" ]] || { echo "找不到 ROS 环境：${ROS_SETUP}" >&2; exit 1; }
[[ -r "${MATCH_WS}/devel/setup.bash" ]] || {
  echo "找不到 ${MATCH_WS}/devel/setup.bash，请先编译工作区。" >&2
  exit 1
}
[[ -n "${DISPLAY:-}" ]] || {
  echo "DISPLAY 未设置，请在图形桌面终端运行。" >&2
  exit 1
}

set +u
source "${ROS_SETUP}"
source "${MATCH_WS}/devel/setup.bash"
set -u

cleanup() {
  trap - INT TERM EXIT
  if [[ -n "${VIEW_PID}" ]] && kill -0 "${VIEW_PID}" 2>/dev/null; then
    kill -INT "${VIEW_PID}" 2>/dev/null || true
  fi
  if [[ -n "${STACK_PID}" ]] && kill -0 "${STACK_PID}" 2>/dev/null; then
    kill -INT "${STACK_PID}" 2>/dev/null || true
  fi
  wait "${VIEW_PID}" 2>/dev/null || true
  wait "${STACK_PID}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "[UAV1 four-view] 启动 D435/颜色/二维码/热源/映射检测..."
roslaunch uav0_competition_bringup uav1_detection_stack.launch \
  enable_realsense:=true \
  enable_color_tag:=true \
  enable_qr:=true \
  enable_front_aruco:=false \
  enable_thermal:=true \
  enable_thermal_d435_fusion:=true \
  enable_target_reporting:=false &
STACK_PID=$!

sleep 2
echo "[UAV1 four-view] 打开四分屏..."
roslaunch target_reporting uav1_four_view.launch &
VIEW_PID=$!

wait "${VIEW_PID}"
