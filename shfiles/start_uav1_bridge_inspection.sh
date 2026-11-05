#!/usr/bin/env bash
# 单机桥梁巡检入口；小车由外部操作，本脚本不连接 UAV0。
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SCRIPT_PATH="${SCRIPT_DIR}/start_uav1_bridge_inspection.sh"
export UAV1_SINGLE_ROS_IP="${UAV1_SINGLE_ROS_IP:-127.0.0.1}"
export UAV1_SINGLE_ROS_MASTER_IP="${UAV1_SINGLE_ROS_MASTER_IP:-${UAV1_SINGLE_ROS_IP}}"

if [[ "${1:-}" == --help ]]; then
  cat <<'EOF'
单机桥梁巡检（直接打开六分屏和检测/RViz窗口）：
  bash shfiles/start_uav1_bridge_inspection.sh

地面Z和降落点XY在 src/uav1_bridge_inspection/launch/bridge.launch 开头修改。
也可临时覆盖：ground_z:=实测值 landing_x:=实测值 landing_y:=实测值。

全部坐标使用 UAV1/camera_init（FAST-LIO），ground_z 不是车面高度。
桥底离地1.5m、净高1.2m，默认起飞高度为 ground_z+2.0。
Diff 上下边界各向洞内留15cm；可追加 floor_margin:=0.15 ceiling_margin:=0.15 调整。
航点在 src/uav1_bridge_inspection/config/route.yaml 中直接填写 [x,y,z]，满足到点条件后切点，无额外停留。
航点 z 不再叠加底板高度；示例 z=2.0 只在 ground_z=0 时表示离地2米。
启动后：车停稳，手动解锁，CH8低位并切OFFBOARD起飞；到起飞高度后CH8高位执行航点。
到最后的车上方航点后自动触发18cm精降；沿用CH10人工降落。
左侧显示裂缝/热源/二维码/ArUco，右侧显示点云及规划轨迹。
裂缝带框图、确认后的热源和二维码证据图保存到 ~/match_ws/target_reports/bridge。
其他 roslaunch 参数可直接追加，如 enable_thermal:=false。
外力估计默认 NDOB（aerothrow_ndob）；切回原滑动窗口方法：
  bash shfiles/start_uav1_bridge_inspection.sh force_estimator_method:=window_opt
切回 NDOB：追加 force_estimator_method:=aerothrow_ndob。
NDOB 参数：ndob_rate:=20.0 ndob_cutoff_freq:=5.0；修改后重启任务生效。
EOF
  exit 0
fi

if [[ "${1:-}" == --pane ]]; then
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
quoted_args=""
if (( $# > 0 )); then
  printf -v quoted_args '%q ' "$@"
fi
cat > "$runtime_config" <<EOF
[global_config]
  suppress_multiple_term_dialog = True
[keybindings]
[profiles]
  [[default]]
    exit_action = hold
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
      order = 0
      ratio = 0.333
    [[[top_right]]]
      type = HPaned
      parent = top
      order = 1
      ratio = 0.5
    [[[bottom]]]
      type = HPaned
      parent = root
      order = 1
      ratio = 0.333
    [[[bottom_right]]]
      type = HPaned
      parent = bottom
      order = 1
      ratio = 0.5
EOF
panes=(mavros mid360 fastlio pose camera mission)
parents=(top top_right top_right bottom bottom_right bottom_right)
orders=(0 0 1 0 0 1)
titles=('本机Master/MAVROS' 'MID360' 'FAST-LIO' '视觉位姿回传' '下视相机' '航点/Diff/NMPC/检测显示')
for i in "${!panes[@]}"; do
  cat >> "$runtime_config" <<EOF
    [[[pane${i}]]]
      type = Terminal
      parent = ${parents[$i]}
      order = ${orders[$i]}
      profile = default
      title = ${titles[$i]}
      command = bash ${quoted_script} --pane ${panes[$i]} ${quoted_args}
EOF
done
printf '[plugins]\n' >> "$runtime_config"
printf '[桥梁巡检] 打开六分屏：MAVROS / MID360 / FAST-LIO / 位姿回传 / 下视相机 / 巡检任务\n'
printf '[桥梁巡检] 本机 ROS master 就绪后自动打开检测/RViz窗口；无传感器数据时显示等待画面。\n'
printf '[桥梁巡检] 地面Z、降落点XY读取 launch/bridge.launch；当前配置数值需按现场确认。\n'
terminator --no-dbus --maximise --config="$runtime_config" --layout=uav1_bridge
