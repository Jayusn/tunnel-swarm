#!/bin/sh
# 与多机相同的检测/定位程序，独立本机 ROS master，不启动飞行控制节点。
if [ -z "${BASH_VERSION:-}" ]; then
    exec bash "$0" "$@"
fi
set -e

test_script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
test_workspace=$(dirname -- "$test_script_dir")
if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    cat <<'EOF'
用法：bash shfiles/test_uav1_detection.sh [roslaunch 参数]
  默认启动 MID360、FAST-LIO、D435、热成像、三类检测和多机同款 RViz。
  RViz 显示配准点云、移动轨迹、机体模型、检测图像和目标位置。
  open_rviz:=false       不打开 RViz
  enable_thermal:=false  本次不接热成像相机时使用
  record_bag:=false      本次不录包（默认录制）
  --dump-params          只解析配置，不启动设备

图片：match_ws/target_reports/uav1_detection_test/日期/test_时间/images/annotated/
按 qr_code、color_tag、thermal_source 分类，JSON 位于该次测试目录的 reports.jsonl。
录包：同次测试目录的 bags/detection_*.bag，按 1 GiB 分包。
使用独立本机 ROS master http://127.0.0.1:11321；Ctrl+C 结束本次启动的节点。
启动前关闭占用同一相机/雷达的多机程序，FAST-LIO 初始化时先保持机体静止。
EOF
    exit 0
fi

. /opt/ros/noetic/setup.bash
. "$test_workspace/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
test_mission_id=$(date +%Y%m%d/test_%H%M%S_%N)
test_record_root="$test_workspace/target_reports/uav1_detection_test"
printf '[后机检测测试] 默认保存目录：%s/%s\n' "$test_record_root" "$test_mission_id" >&2
printf '[后机检测测试] ROS master：%s；Ctrl+C 结束。\n' "$ROS_MASTER_URI" >&2
exec roslaunch --port 11321 uav0_competition_bringup uav1_detection_test.launch \
    record_root:="$test_record_root" mission_id:="$test_mission_id" "$@"
