#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="${SCRIPT_DIR}/$(basename -- "${BASH_SOURCE[0]}")"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${UAV1_FOUR_VIEW_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
LAYOUT_CONFIG="${SCRIPT_DIR}/terminator_uav1_four_view.conf"
RUNTIME_CONFIG="/tmp/uav1_four_view_${UID:-$(id -u)}.conf"
LOG_FILE="/tmp/uav1_four_view_${UID:-$(id -u)}.log"
PID_FILE="/tmp/uav1_four_view_${UID:-$(id -u)}.pid"

usage() {
  cat <<'EOF'
用法：
  bash shfiles/start_uav1_four_view.sh
  bash shfiles/start_uav1_four_view.sh stop

打开 Terminator 五分屏：MAVROS、MID360、FAST-LIO、检测/上报、四分屏画面。
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then usage; exit 0; fi
if [[ "${1:-}" == "stop" ]]; then
  if [[ -r "${PID_FILE}" ]]; then kill "$(cat "${PID_FILE}")" 2>/dev/null || true; rm -f "${PID_FILE}"; fi
  exit 0
fi
command -v terminator >/dev/null 2>&1 || { echo '未安装 terminator' >&2; exit 1; }
[[ -r "${ROS_SETUP}" && -r "${MATCH_WS}/devel/setup.bash" ]] || { echo 'ROS 或 devel/setup.bash 不存在' >&2; exit 1; }
[[ -n "${DISPLAY:-}" ]] || { echo 'DISPLAY 未设置，请在图形桌面终端运行' >&2; exit 1; }

if [[ "${1:-}" == "--pane" ]]; then
  [[ $# -eq 2 ]] || { usage >&2; exit 2; }
  set +u; source "${ROS_SETUP}"; source "${MATCH_WS}/devel/setup.bash"; set -u
  case "${2:-}" in
    mavros) exec roslaunch cxr_ego_ctrl mavros_uav.launch vehicle_ns:=UAV1 ;;
    mid360) exec roslaunch livox_ros_driver2 msg_MID360.launch vehicle_ns:=UAV1 msg_frame_id:=UAV1/livox_frame publish_freq:=30.0 ;;
    fastlio) exec roslaunch fast_lio mapping_mid360.launch vehicle_ns:=UAV1 odom_topic:=/UAV1/fast_lio/Odom_high_freq rviz:=false ;;
    detection) exec roslaunch uav0_competition_bringup uav1_detection_stack.launch enable_realsense:=true enable_color_tag:=true enable_qr:=true enable_front_aruco:=false enable_thermal:=true enable_thermal_d435_fusion:=true enable_target_reporting:=true ;;
    view) exec roslaunch target_reporting uav1_four_view.launch ;;
    *) usage >&2; exit 2 ;;
  esac
fi
[[ $# -eq 0 ]] || { usage >&2; exit 2; }

sed "s|__UAV1_FOUR_VIEW_SCRIPT__|${SCRIPT_PATH}|g" "${LAYOUT_CONFIG}" > "${RUNTIME_CONFIG}"
nohup terminator --no-dbus --maximise --config="${RUNTIME_CONFIG}" --layout=uav1_four_view >"${LOG_FILE}" 2>&1 &
printf '%s\n' "$!" > "${PID_FILE}"
echo "[UAV1 four-view] 已打开 Terminator 五分屏，停止：bash shfiles/start_uav1_four_view.sh stop"
