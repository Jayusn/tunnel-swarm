#!/usr/bin/env bash

# UAV1 颜色、二维码、热源检测及四分屏一键入口。
# 独立于原有传感器、检测和规划启动脚本，不改变它们的默认行为。

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${UAV1_FOUR_VIEW_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
LOCAL_MASTER_URI="${UAV1_FOUR_VIEW_MASTER_URI:-http://127.0.0.1:11311}"
LOCAL_ROS_IP="${UAV1_FOUR_VIEW_ROS_IP:-127.0.0.1}"
ROSCORE_PID=""
MAVROS_PID=""
MID360_PID=""
FASTLIO_PID=""
STACK_PID=""
VIEW_PID=""

usage() {
  cat <<'EOF'
用法：
  bash shfiles/start_uav1_four_view.sh

作用：
  依次启动 UAV1 MAVROS、MID360、FAST-LIO、D435/颜色/二维码/热源检测、
  目标上报和只显示四分屏的 RViz。

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
export ROS_MASTER_URI="${LOCAL_MASTER_URI}"
export ROS_IP="${LOCAL_ROS_IP}"
unset ROS_HOSTNAME
set -u

cleanup() {
  trap - INT TERM EXIT
  if [[ -n "${VIEW_PID}" ]] && kill -0 "${VIEW_PID}" 2>/dev/null; then
    kill -INT "${VIEW_PID}" 2>/dev/null || true
  fi
  if [[ -n "${STACK_PID}" ]] && kill -0 "${STACK_PID}" 2>/dev/null; then
    kill -INT "${STACK_PID}" 2>/dev/null || true
  fi
  for pid in "${FASTLIO_PID}" "${MID360_PID}" "${MAVROS_PID}" "${ROSCORE_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  wait "${VIEW_PID}" 2>/dev/null || true
  wait "${STACK_PID}" 2>/dev/null || true
  wait "${FASTLIO_PID}" 2>/dev/null || true
  wait "${MID360_PID}" 2>/dev/null || true
  wait "${MAVROS_PID}" 2>/dev/null || true
  wait "${ROSCORE_PID}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

wait_for_topic() {
  local topic="$1"
  local timeout="${UAV1_FOUR_VIEW_WAIT_TIMEOUT:-180}"
  echo "[UAV1 four-view] 等待话题 ${topic} ..."
  timeout "${timeout}" bash -c \
    "source '${ROS_SETUP}' && source '${MATCH_WS}/devel/setup.bash' && rostopic wait '${topic}'" \
    || { echo "等待话题超时：${topic}" >&2; exit 1; }
}

if ! rostopic list >/dev/null 2>&1; then
  echo "[UAV1 four-view] 启动本机 roscore..."
  roscore >/tmp/uav1_four_view_roscore.log 2>&1 &
  ROSCORE_PID=$!
  wait_for_topic /rosout
else
  echo "[UAV1 four-view] 使用已有本机 ROS Master：${ROS_MASTER_URI}"
fi

echo "[UAV1 four-view] 启动 MAVROS..."
roslaunch cxr_ego_ctrl mavros_uav.launch vehicle_ns:=UAV1 &
MAVROS_PID=$!
sleep 2

echo "[UAV1 four-view] 启动 MID360..."
roslaunch livox_ros_driver2 msg_MID360.launch \
  vehicle_ns:=UAV1 msg_frame_id:=UAV1/livox_frame publish_freq:=30.0 &
MID360_PID=$!
wait_for_topic /UAV1/livox/lidar
wait_for_topic /UAV1/livox/imu

echo "[UAV1 four-view] 启动 FAST-LIO..."
roslaunch fast_lio mapping_mid360.launch \
  vehicle_ns:=UAV1 odom_topic:=/UAV1/fast_lio/Odom_high_freq rviz:=false &
FASTLIO_PID=$!
wait_for_topic /UAV1/fast_lio/Odom_high_freq

echo "[UAV1 four-view] 启动 D435/颜色/二维码/热源/映射检测和目标上报..."
roslaunch uav0_competition_bringup uav1_detection_stack.launch \
  enable_realsense:=true \
  enable_color_tag:=true \
  enable_qr:=true \
  enable_front_aruco:=false \
  enable_thermal:=true \
  enable_thermal_d435_fusion:=true \
  enable_target_reporting:=true &
STACK_PID=$!

sleep 3
echo "[UAV1 four-view] 打开四分屏..."
roslaunch target_reporting uav1_four_view.launch &
VIEW_PID=$!

wait "${VIEW_PID}"
