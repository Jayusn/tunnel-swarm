#!/usr/bin/env bash
# 前视 D435 + 下视 USB 相机 ArUco 调试，不启动飞行控制。
set -e -o pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
用法：start_aruco_debug.sh [roslaunch 参数]

默认启动前视、下视相机及两个 ArUco 检测窗口，画面显示检测框和 ID。
仅启动纯观察入口，不启动 MAVROS、规划器、精降控制器或起飞节点。
没有 FAST-LIO 时出现 WAITING FOR SYNCED FAST-LIO ODOM 属正常现象，
仍可查看检测框和 ID；此时不提供世界坐标。
码边长沿用 front_aruco_hint.yaml 和 landing_search.yaml 中的 marker.size_m。
在本终端按 Ctrl+C 停止相机、检测节点和窗口。

示例：
  start_aruco_debug.sh
  start_aruco_debug.sh open_views:=false
  start_aruco_debug.sh down_camera_device:=/dev/video0

环境变量：
  ARUCO_DEBUG_WS                  工作区目录，默认使用脚本所属 match_ws
  ARUCO_DEBUG_DOWN_CAMERA_DEVICE  下视设备，默认自动查找 Generic USB Camera
HELP
  exit 0
fi

script_file="$(readlink -f -- "${BASH_SOURCE[0]}")"
debug_ws="${ARUCO_DEBUG_WS:-$(cd -- "$(dirname -- "$script_file")/../../.." && pwd)}"
if [[ ! -f "$debug_ws/devel/setup.bash" ]]; then
  printf '找不到工作区环境：%s/devel/setup.bash\n' "$debug_ws" >&2
  exit 1
fi

# ROS 的环境脚本可能访问未设置的变量，因此不启用 nounset。
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "$debug_ws/devel/setup.bash"

down_device="${ARUCO_DEBUG_DOWN_CAMERA_DEVICE:-}"
for launch_arg in "$@"; do
  if [[ "$launch_arg" == down_camera_device:=* ]]; then
    down_device="${launch_arg#down_camera_device:=}"
  fi
done
if [[ -z "$down_device" ]]; then
  shopt -s nullglob
  down_devices=(/dev/v4l/by-id/usb-Generic_USB_Camera_*-video-index0)
  down_device="${down_devices[0]:-}"
fi
if [[ ! -c "$down_device" ]]; then
  printf '未找到下视相机，请通过 down_camera_device:=/dev/videoX 指定设备。\n' >&2
  exit 1
fi

printf 'ArUco 纯检测调试：下视设备 %s\n' "$down_device"
printf '前视：/UAV0/aruco_observer/front/debug_image\n'
printf '下视：/UAV0/aruco_observer/down/debug_image\n'
printf '没有 FAST-LIO 时仍可看检测框和 ID；按 Ctrl+C 停止。\n'
exec roslaunch uav0_competition_bringup uav0_dual_aruco_observer.launch \
  "down_camera_device:=$down_device" "$@"
