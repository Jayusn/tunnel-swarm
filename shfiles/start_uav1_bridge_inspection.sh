#!/usr/bin/env bash
# 单机桥梁巡检入口；小车由外部操作，本脚本不连接 UAV0。
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SCRIPT_PATH="${SCRIPT_DIR}/start_uav1_bridge_inspection.sh"
export UAV1_SINGLE_ROS_IP="${UAV1_SINGLE_ROS_IP:-127.0.0.1}"
export UAV1_SINGLE_ROS_MASTER_IP="${UAV1_SINGLE_ROS_MASTER_IP:-${UAV1_SINGLE_ROS_IP}}"

if [[ "${1:-}" == --help || $# == 0 ]]; then
  cat <<'EOF'
单机桥梁巡检（先填写现场坐标）：
  bash shfiles/start_uav1_bridge_inspection.sh \
    ground_z:=地面Z entrance_x:=入口X entrance_y:=入口Y \
    bridge_yaw:=沿桥方向弧度 landing_x:=出口停车X landing_y:=出口停车Y

全部坐标使用 UAV1/camera_init（FAST-LIO），ground_z 不是车面高度。
桥底离地1.5m、净高1.2m，初始航点高度为桥底以上0.5m（离地2.0m）。
Diff 上下边界各向洞内留15cm；可追加 floor_margin:=0.15 ceiling_margin:=0.15 调整。
航点在 src/uav1_bridge_inspection/config/route.yaml 中修改。
启动后：车停稳，手动解锁，CH8低位并切OFFBOARD起飞；到起飞高度后CH8高位执行航点。
到最后的车上方航点后自动触发18cm精降；沿用CH10人工降落。
左侧显示裂缝/热源/二维码/ArUco，右侧显示点云及规划轨迹。
裂缝带框图、确认后的热源和二维码证据图保存到 ~/match_ws/target_reports/bridge。
其他 roslaunch 参数可直接追加，如 enable_thermal:=false。
EOF
  exit 0
fi

if [[ "$1" == --pane ]]; then
  pane="$2"
  shift 2
  case "$pane" in
    mavros|mid360|fastlio|pose)
      exec bash "${SCRIPT_DIR}/start_uav1_seven_terminator.sh" --pane "$pane"
      ;;
    camera|mission)
      source /opt/ros/noetic/setup.bash
      source "${MATCH_WS}/devel/setup.bash"
      export ROS_MASTER_URI="http://${UAV1_SINGLE_ROS_MASTER_IP}:11311"
      export ROS_IP="${UAV1_SINGLE_ROS_IP}"
      unset ROS_HOSTNAME
      until rosnode list >/dev/null 2>&1; do sleep 1; done
      if [[ "$pane" == camera ]]; then
        exec bash "${SCRIPT_DIR}/run_uav1_sensor_stack.sh" landing
      fi
      exec roslaunch uav1_bridge_inspection bridge.launch "$@"
      ;;
    *) echo "未知分屏：$pane" >&2; exit 2 ;;
  esac
fi

source /opt/ros/noetic/setup.bash
source "${MATCH_WS}/devel/setup.bash"
# 只解析 launch 参数；不启动任何硬件或飞行节点。
roslaunch --nodes uav1_bridge_inspection bridge.launch "$@" >/dev/null
runtime_config="$(mktemp /tmp/uav1_bridge_XXXXXX.conf)"
trap 'rm -f -- "$runtime_config"' EXIT
printf -v quoted_script '%q' "$SCRIPT_PATH"
printf -v quoted_args '%q ' "$@"
cat > "$runtime_config" <<EOF
[global_config]
  suppress_multiple_term_dialog = True
[keybindings]
[profiles]
  [[default]]
[layouts]
  [[uav1_bridge]]
    [[[window0]]]
      type = Window
      parent = ""
      title = UAV1 单机桥梁巡检
      size = 1600,900
    [[[root]]]
      type = VPaned
      parent = window0
      ratio = 0.5
    [[[top]]]
      type = HPaned
      parent = root
      ratio = 0.333
    [[[top_right]]]
      type = HPaned
      parent = top
      ratio = 0.5
    [[[bottom]]]
      type = HPaned
      parent = root
      ratio = 0.333
    [[[bottom_right]]]
      type = HPaned
      parent = bottom
      ratio = 0.5
EOF
panes=(mavros mid360 fastlio pose camera mission)
parents=(top top_right top_right bottom bottom_right bottom_right)
titles=('本机Master/MAVROS' 'MID360' 'FAST-LIO' '视觉位姿回传' '下视相机' '航点/Diff/NMPC/检测显示')
for i in "${!panes[@]}"; do
  cat >> "$runtime_config" <<EOF
    [[[pane${i}]]]
      type = Terminal
      parent = ${parents[$i]}
      profile = default
      title = ${titles[$i]}
      command = bash ${quoted_script} --pane ${panes[$i]} ${quoted_args}
EOF
done
printf '[plugins]\n' >> "$runtime_config"
terminator --no-dbus --maximise --config="$runtime_config" --layout=uav1_bridge
