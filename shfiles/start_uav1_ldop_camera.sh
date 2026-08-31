#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
RH_WS="${UAV1_LDOP_RH_WS:-/home/oem/rh_ws}"
ROS_SETUP="${UAV1_LDOP_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
RVIZ_CONFIG="${MATCH_WS}/src/LDOP/launch/rviz/ldop_camera_rviz.rviz"
RH_LDOP_CONFIG="${RH_WS}/src/LDOP/config/ldop.yaml"
RH_LDOP_NODE="${RH_WS}/devel/lib/ldop/ldop_node"
RUN_DIR="/tmp/uav1_ldop_camera_${UID:-$(id -u)}"
ROSCORE_LOG="${RUN_DIR}/roscore.log"
PIDS=()
NAMES=()

fail() {
  echo "[UAV1 LDOP camera][错误] $*" >&2
  exit 1
}

cleanup() {
  local index
  trap - EXIT INT TERM
  echo
  echo "[UAV1 LDOP camera] 正在停止本脚本启动的节点..."
  for ((index=${#PIDS[@]}-1; index>=0; index--)); do
    if kill -0 "${PIDS[index]}" 2>/dev/null; then
      echo "  停止 ${NAMES[index]} (PID ${PIDS[index]})"
      kill -INT "${PIDS[index]}" 2>/dev/null || true
    fi
  done
  wait "${PIDS[@]}" 2>/dev/null || true
}

start_component() {
  local name="$1"
  local log_file="$2"
  shift 2
  echo "[UAV1 LDOP camera] 启动 ${name}，日志：${log_file}"
  "$@" >"${log_file}" 2>&1 &
  PIDS+=("$!")
  NAMES+=("${name}")
  sleep 2
  if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    tail -n 30 "${log_file}" >&2 || true
    fail "${name} 启动失败"
  fi
}

wait_for_topic() {
  local topic="$1"
  local timeout_seconds="$2"
  local elapsed=0
  echo "[UAV1 LDOP camera] 等待话题 ${topic} ..."
  until rostopic info "${topic}" >/dev/null 2>&1; do
    sleep 1
    elapsed=$((elapsed + 1))
    if ((elapsed >= timeout_seconds)); then
      fail "等待话题超时：${topic}"
    fi
  done
}

run_rh_ldop() {
  local private_ns="/UAV1/ldop_node"

  set +u
  source "${ROS_SETUP}"
  source "${MATCH_WS}/devel/setup.bash"
  source "${RH_WS}/devel/setup.bash"
  set -u
  export LD_LIBRARY_PATH="/home/oem/ldop_ws/external/install/ufomap/lib:/home/oem/.local/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

  [[ "$(rospack find ldop 2>/dev/null || true)" == "${RH_WS}/src/LDOP" ]] ||
    fail "ROS 未解析到 rh_ws 的 LDOP"

  rosparam load "${RH_LDOP_CONFIG}" "${private_ns}"
  rosparam set "${private_ns}/input_cloud_topic" /UAV1/ldop/cloud_registered_5hz
  rosparam set "${private_ns}/odom_topic" /UAV1/fast_lio/Odom_high_freq
  rosparam set "${private_ns}/static_cloud_topic" /UAV1/ldop/static_cloud
  rosparam set "${private_ns}/dynamic_cloud_topic" /UAV1/ldop/dynamic_cloud
  rosparam set "${private_ns}/dynamic_objects_topic" /UAV1/ldop/dynamic_objects
  rosparam set "${private_ns}/dynamic_object_markers_topic" /UAV1/ldop/dynamic_object_markers
  rosparam set "${private_ns}/dynamic_track_markers_topic" /UAV1/ldop/dynamic_track_markers
  rosparam set "${private_ns}/dynamic_predictions_topic" /UAV1/ldop/dynamic_object_predictions
  rosparam set "${private_ns}/dynamic_prediction_markers_topic" /UAV1/ldop/dynamic_prediction_markers
  rosparam set "${private_ns}/static_map_markers_topic" /UAV1/ldop/static_map_markers
  rosparam set "${private_ns}/fusion_objects_topic" /UAV1/ldop/fused_dynamic_objects
  rosparam set "${private_ns}/fusion_virtual_imu_topic" /UAV1/ldop/fused_dynamic_object_virtual_imu
  rosparam set "${private_ns}/fusion_marker_topic" /UAV1/ldop/fused_dynamic_object_markers

  exec "${RH_LDOP_NODE}" __ns:=/UAV1 __name:=ldop_node
}

[[ -r "${ROS_SETUP}" ]] || fail "找不到 ${ROS_SETUP}"
[[ -r "${MATCH_WS}/devel/setup.bash" ]] || fail "找不到 ${MATCH_WS}/devel/setup.bash"
[[ -r "${RH_WS}/devel/setup.bash" ]] || fail "找不到 rh_ws/devel/setup.bash"
[[ -r "${RH_LDOP_CONFIG}" ]] || fail "找不到 rh_ws LDOP 配置"
[[ -x "${RH_LDOP_NODE}" ]] || fail "rh_ws LDOP 尚未编译：${RH_LDOP_NODE}"
[[ -r "${RVIZ_CONFIG}" ]] || fail "找不到 RViz 配置 ${RVIZ_CONFIG}"
[[ -n "${DISPLAY:-}" ]] || fail "DISPLAY 未设置，请在图形桌面终端运行"

set +u
source "${ROS_SETUP}"
source "${MATCH_WS}/devel/setup.bash"
set -u

# 此入口只用于 UAV1 本机测试，默认不依赖当前 WiFi 地址。
export ROS_MASTER_URI="${UAV1_LDOP_ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_IP="${UAV1_LDOP_ROS_IP:-127.0.0.1}"
unset ROS_HOSTNAME

mkdir -p "${RUN_DIR}"
trap cleanup EXIT INT TERM

if rosparam get /run_id >/dev/null 2>&1; then
  echo "[UAV1 LDOP camera] 复用已有 ROS master：${ROS_MASTER_URI}"
else
  start_component roscore "${ROSCORE_LOG}" roscore
  for _ in {1..30}; do
    rosparam get /run_id >/dev/null 2>&1 && break
    sleep 1
  done
  rosparam get /run_id >/dev/null 2>&1 || fail "roscore 启动超时"
fi

start_component MID360 "${RUN_DIR}/mid360.log" \
  roslaunch livox_ros_driver2 msg_MID360.launch \
  vehicle_ns:=UAV1 msg_frame_id:=UAV1/livox_frame publish_freq:=30.0
wait_for_topic /UAV1/livox/lidar 60
wait_for_topic /UAV1/livox/imu 60

# LDOP 的输入是 FAST-LIO 注册点云和高频里程计，必须保留这一前置处理。
start_component FAST-LIO "${RUN_DIR}/fast_lio.log" \
  roslaunch fast_lio mapping_mid360.launch vehicle_ns:=UAV1 rviz:=false
wait_for_topic /UAV1/fast_lio/cloud_registered 120
wait_for_topic /UAV1/fast_lio/Odom_high_freq 120

start_component "LDOP 输入限频 (5 Hz)" "${RUN_DIR}/ldop_rate_limit.log" \
  rosrun topic_tools drop \
  /UAV1/fast_lio/cloud_registered 1 2 /UAV1/ldop/cloud_registered_5hz
wait_for_topic /UAV1/ldop/cloud_registered_5hz 30

start_component D435 "${RUN_DIR}/d435.log" \
  roslaunch realsense2_camera rs_camera.launch \
  camera:=UAV1/camera tf_prefix:=UAV1_camera \
  align_depth:=true enable_color:=true enable_depth:=true \
  color_width:=1280 color_height:=720 color_fps:=30 \
  depth_width:=1280 depth_height:=720 depth_fps:=30 \
  enable_pointcloud:=false
wait_for_topic /UAV1/camera/color/image_raw 60

start_component "LDOP (rh_ws)" "${RUN_DIR}/ldop.log" run_rh_ldop

echo "[UAV1 LDOP camera] 启动 LDOP RViz（含 D435 前视画面）"
echo "[UAV1 LDOP camera] 所有日志位于：${RUN_DIR}"
rviz -d "${RVIZ_CONFIG}"
