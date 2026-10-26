#!/bin/sh

# UAV1 传感器链路启动器：启动 MAVROS、MID360、FAST-LIO 或下视相机。
# 不启动控制器、不自动解锁、不切换 OFFBOARD，也不发布目标点。

set -eu

# 用户可能用 sh 调用；ROS setup.bash 需要 Bash 语法，自动切换到 Bash 执行本脚本。
if [ -z "${BASH_VERSION:-}" ]; then
    exec bash "$0" "$@"
fi

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MATCH_WS=$(dirname -- "$SCRIPT_DIR")
WAIT_TIMEOUT=$(printenv WAIT_TIMEOUT 2>/dev/null || echo 45)
DOWN_CAMERA_START_TIMEOUT=$(printenv UAV1_DOWN_CAMERA_START_TIMEOUT 2>/dev/null || echo 20)

usage()
{
    cat <<'EOF'
用法：
  sh shfiles/run_uav1_sensor_stack.sh mavros
  sh shfiles/run_uav1_sensor_stack.sh mid360
  sh shfiles/run_uav1_sensor_stack.sh fastlio
  sh shfiles/run_uav1_sensor_stack.sh landing

说明：
  mavros  启动 /UAV1/mavros，并设置 ESC 状态频率为 200 Hz。
  mid360  启动 /UAV1/livox/lidar 和 /UAV1/livox/imu。
  fastlio 先启动原始点云诊断录包，再启动 /UAV1/fast_lio。
          起飞前、停桨后各保留约 20 秒，再关闭本终端。
  landing 只启动 /UAV1/down_camera；搜索和精降节点由
          leader_safe_path_follower.launch 统一启动，避免重复节点。
EOF
}

fail()
{
    echo "[UAV1 sensor stack][错误] $*" >&2
    exit 1
}

source_ros()
{
    [ -r /opt/ros/noetic/setup.bash ] || fail "未找到 /opt/ros/noetic/setup.bash。"
    [ -r "$MATCH_WS/devel/setup.bash" ] || fail "未找到 $MATCH_WS/devel/setup.bash。"

    # 用 POSIX '.' 兼容用户现有的 sh 调用方式。
    # ROS setup 脚本可能访问尚未定义的变量，加载期间暂时关闭 nounset。
    set +u
    . /opt/ros/noetic/setup.bash
    . "$MATCH_WS/devel/setup.bash"
    set -u
}

wait_for_mavros()
{
    local deadline=$((SECONDS + WAIT_TIMEOUT)) check_timeout
    echo "[UAV1 MAVROS] 等待 /UAV1/mavros/state connected: True ..."

    while [ "$SECONDS" -lt "$deadline" ]; do
        check_timeout=$((deadline - SECONDS))
        [ "$check_timeout" -gt 0 ] || break
        if [ "$check_timeout" -gt 5 ]; then check_timeout=5; fi
        if timeout --kill-after=1s "${check_timeout}s" rostopic echo -n 1 /UAV1/mavros/state 2>/dev/null |
            grep -q 'connected: True'; then
            return 0
        fi
        if [ "$SECONDS" -lt "$deadline" ]; then sleep 1; fi
    done
    fail "等待 UAV1 MAVROS 连接超时。"
}

set_interval()
{
    message_id=$1
    interval_us=$2
    description=$3
    attempt=1

    while [ "$attempt" -le 3 ]; do
        echo "[UAV1 MAVROS] 设置 $description: $interval_us us（第 $attempt 次）"
        if timeout --kill-after=1s 5s rosrun mavros mavcmd -n /UAV1/mavros long 511 "$message_id" "$interval_us" 0 0 0 0 0; then
            return 0
        fi
        attempt=$((attempt + 1))
        sleep 1
    done

    fail "设置 $description 频率失败。"
}

run_mavros()
{
    mavros_pid=
    cleanup()
    {
        if [ -n "$mavros_pid" ] && kill -0 "$mavros_pid" 2>/dev/null; then
            echo "[UAV1 MAVROS] 正在关闭 MAVROS ..."
            kill -INT "$mavros_pid" 2>/dev/null || true
        fi
    }
    trap cleanup INT TERM EXIT

    roslaunch cxr_ego_ctrl mavros_uav.launch vehicle_ns:=UAV1 &
    mavros_pid=$!
    wait_for_mavros

    # MAV_CMD_SET_MESSAGE_INTERVAL(511) 的间隔单位为微秒。
    # 六/七分屏入口指定原始 IMU 间隔，对应 /UAV1/mavros/imu/data_raw。
    if [ -n "${UAV1_HIGHRES_IMU_INTERVAL_US:-}" ]; then
        set_interval 105 "$UAV1_HIGHRES_IMU_INTERVAL_US" "HIGHRES_IMU"
    fi
    set_interval 291 5000 "ESC_STATUS 目标 200 Hz"

    echo "[UAV1 MAVROS] 频率请求已发送，保持当前终端显示 MAVROS 日志。"
    wait "$mavros_pid"
}

run_mid360()
{
    exec roslaunch livox_ros_driver2 msg_MID360.launch
}

run_fastlio()
{
    local record_dir recorder_pid mapping_pid=""
    mkdir -p "$HOME/.ros/fast_lio_diagnostics"
    record_dir=$(mktemp -d "$HOME/.ros/fast_lio_diagnostics/$(date +%Y%m%d_%H%M%S)_XXXXXX")
    echo "[UAV1 FAST-LIO] 原始点云/诊断录包：$record_dir"
    echo "[UAV1 FAST-LIO] 缓存 64 MB，1 GB 分包；起飞前、停桨后各保留约 20 秒。"
    git -C "$MATCH_WS" rev-parse HEAD > "$record_dir/source_revision.txt" 2>/dev/null || true
    sha256sum "$MATCH_WS/devel/lib/fast_lio/fastlio_mapping" > "$record_dir/binary.sha256"
    git -C "$MATCH_WS" diff -- src/FAST_LIO shfiles/run_uav1_sensor_stack.sh > "$record_dir/source_diff.patch"
    timeout 3 rosparam dump "$record_dir/params_before.yaml" /UAV1 2>/dev/null || true
    cp "$MATCH_WS/src/FAST_LIO/config/mid360.yaml" "$record_dir/"
    cp "$MATCH_WS/src/FAST_LIO/launch/mapping_mid360.launch" "$record_dir/"
    cp "$MATCH_WS/src/FAST_LIO/launch/record_lio_diagnostics.launch" "$record_dir/"
    cleanup_fastlio_recording()
    {
        # Keep the master alive while writing the final parameter snapshot and bag index.
        timeout 3 rosparam dump "$record_dir/params_end.yaml" /UAV1 2>/dev/null || true
        if [ -n "$mapping_pid" ]; then
            kill -INT "$mapping_pid" 2>/dev/null || true
            wait "$mapping_pid" 2>/dev/null || true
        fi
        kill -INT "$recorder_pid" 2>/dev/null || true
        wait "$recorder_pid" 2>/dev/null || true
    }
    roslaunch fast_lio record_lio_diagnostics.launch \
        bag_prefix:="$record_dir/raw" > "$record_dir/recorder.log" 2>&1 &
    recorder_pid=$!
    trap cleanup_fastlio_recording EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    # rosbag's latched begin_write confirms that the file is open before LIO starts.
    if ! timeout 8 rostopic echo -n 1 /UAV1/lio_recording_started > "$record_dir/recording_started.txt"; then
        echo "[UAV1 FAST-LIO][警告] 未确认录包就绪，初始化数据可能缺失；检查 $record_dir/recorder.log" >&2
    fi
    # 七分屏入口提前准备录包，将原有雷达就绪检查移到建图前。
    # 直接调用本脚本时维持原有启动方式。
    if [ -n "${UAV1_FASTLIO_LIDAR_WAIT_TIMEOUT:-}" ]; then
        local lidar_wait_pid imu_wait_pid lidar_wait_status=0
        echo "[UAV1 FAST-LIO] 等待雷达点云和 IMU 实际消息 ..."
        timeout --kill-after=1s "${UAV1_FASTLIO_LIDAR_WAIT_TIMEOUT}s" \
            rostopic echo --noarr --nostr -n 1 /UAV1/livox/lidar >/dev/null 2>&1 &
        lidar_wait_pid=$!
        timeout --kill-after=1s "${UAV1_FASTLIO_LIDAR_WAIT_TIMEOUT}s" \
            rostopic echo --noarr --nostr -n 1 /UAV1/livox/imu >/dev/null 2>&1 &
        imu_wait_pid=$!
        wait "$lidar_wait_pid" || lidar_wait_status=1
        wait "$imu_wait_pid" || lidar_wait_status=1
        if [ "$lidar_wait_status" -ne 0 ]; then
            cleanup_fastlio_recording
            trap - EXIT INT TERM
            fail "等待雷达点云或 IMU 超时，未启动 FAST-LIO 建图。"
        fi
        echo "[UAV1 FAST-LIO] 雷达点云和 IMU 已就绪，启动建图。"
    fi
    roslaunch fast_lio mapping_mid360.launch \
        rviz_goal_topic:=/UAV1/planning/goal \
        vehicle_ns:=UAV1 \
        odom_topic:=/UAV1/fast_lio/Odometry \
        high_freq_odom_topic:=/UAV1/fast_lio/Odom_high_freq \
        performance_logging:=false \
        publish_odometry:=true &
    mapping_pid=$!
    wait "$mapping_pid"
    cleanup_fastlio_recording
    trap - EXIT INT TERM
}

find_down_camera()
{
    device=$(find /dev/v4l/by-id -maxdepth 1 -type l \
        -name 'usb-Generic_USB_Camera_*-video-index0' 2>/dev/null |
        sort | head -n 1)
    [ -n "$device" ] ||
        fail "未发现下视相机：/dev/v4l/by-id/usb-Generic_USB_Camera_*-video-index0。"
    printf '%s\n' "$device"
}

run_landing()
{
    camera_device=$(find_down_camera)
    camera_info="$HOME/.ros/camera_info/down_camera.yaml"
    [ -r "$camera_info" ] || fail "未找到下视相机标定文件：$camera_info。"
    case "$DOWN_CAMERA_START_TIMEOUT" in
        ''|*[!0-9]*)
            fail "UAV1_DOWN_CAMERA_START_TIMEOUT 必须是正整数秒。"
            ;;
    esac
    [ "$DOWN_CAMERA_START_TIMEOUT" -gt 0 ] ||
        fail "UAV1_DOWN_CAMERA_START_TIMEOUT 必须大于 0。"

    echo "[UAV1 Landing] 下视相机：$camera_device"
    echo "[UAV1 Landing] 仅启动下视相机；搜索和精降由后机接力入口启动。"
    camera_start_deadline=$(($(date +%s) + DOWN_CAMERA_START_TIMEOUT))
    camera_launch_attempt=0
    landing_stop_requested=0
    trap 'landing_stop_requested=1' INT TERM

    while [ "$(date +%s)" -lt "$camera_start_deadline" ]; do
        # usb_cam尚未建立ROS话题时，设备也可能被预览软件或上一次残留进程
        # 短暂占用。只等待并报告PID，不擅自终止未知进程。
        camera_holder_pids=$(fuser "$camera_device" 2>/dev/null || true)
        if [ -n "$camera_holder_pids" ]; then
            echo "[UAV1 Landing][等待] 相机设备忙，占用 PID:${camera_holder_pids}；2 秒后重试。"
            sleep 2
            continue
        fi

        camera_launch_attempt=$((camera_launch_attempt + 1))
        echo "[UAV1 Landing] 启动 usb_cam（第 $camera_launch_attempt 次）。"
        if roslaunch precision_landing landing_stack.launch \
            vehicle_ns:=UAV1 \
            video_device:="$camera_device" \
            camera_info_url:="file://$camera_info" \
            enable_precision_landing:=false; then
            camera_launch_status=0
        else
            camera_launch_status=$?
        fi

        if [ "$landing_stop_requested" -eq 1 ]; then
            trap - INT TERM
            return 0
        fi
        echo "[UAV1 Landing][重试] usb_cam 已退出（状态 $camera_launch_status），设备可能刚刚释放。"
        sleep 1
    done

    trap - INT TERM
    camera_holder_pids=$(fuser "$camera_device" 2>/dev/null || true)
    if [ -n "$camera_holder_pids" ]; then
        fail "等待下视相机设备释放超时，占用 PID:${camera_holder_pids}。"
    fi
    fail "下视相机在 ${DOWN_CAMERA_START_TIMEOUT}s 内连续启动失败；请查看最近的 UAV1-down_camera 日志。"
}

[ "$#" -eq 1 ] || {
    usage
    exit 1
}

source_ros

case "$1" in
    mavros)
        run_mavros
        ;;
    mid360)
        run_mid360
        ;;
    fastlio)
        run_fastlio
        ;;
    landing)
        run_landing
        ;;
    -h|--help)
        usage
        ;;
    *)
        usage >&2
        fail "未知子命令：$1"
        ;;
esac
