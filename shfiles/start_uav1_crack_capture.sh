#!/usr/bin/env bash
# 手持采集裂缝训练原图：启动 D435 彩色相机、实时预览和定时存图节点。
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ "${1:-}" == --help ]]; then
  cat <<'EOF'
连续采集 D435 彩色原图（默认每秒一张 PNG，Ctrl+C 停止）：
  bash shfiles/start_uav1_crack_capture.sh
收到相机画面后自动存图，并自动打开 rqt_image_view 显示实时彩色画面。
关闭预览窗口不会停止采集；在启动终端按 Ctrl+C 停止全部采集程序。
每半秒一张：
  bash shfiles/start_uav1_crack_capture.sh interval_sec:=0.5
自定义保存根目录：
  bash shfiles/start_uav1_crack_capture.sh output_root:=/home/oem/crack_photos

默认保存到当前工作空间 datasets/crack_capture/日期_时间_微秒/。
同一次采集的所有图片直接放在同一个文件夹，不按图片创建文件夹。
运行前关闭其他占用 D435 的相机节点。此入口使用本机 11319 端口，
不受无线 IP 变化影响；不启动 MAVROS、控制器、规划器或检测模型。
EOF
  exit 0
fi

source /opt/ros/noetic/setup.bash
source "${MATCH_WS}/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11319
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
exec roslaunch --port=11319 uav1_bridge_inspection crack_capture.launch \
  "output_root:=${MATCH_WS}/datasets/crack_capture" "$@"
