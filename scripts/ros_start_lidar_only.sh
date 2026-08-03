#!/bin/bash
# IMU+LiDAR Fast-LIO2 简易启动脚本（雷达+FastLIO+MAVROS+桥接节点）
WORKSPACE="$HOME/rh_ws"
start_terminal() {
    local win_name=$1
    local cmd=$2
    gnome-terminal --tab --title="$win_name" -- bash -c "
        cd $WORKSPACE;
        source devel/setup.bash;
        echo ========== $win_name ==========;
        echo \$cmd;
        $cmd;
        exec bash
    "
}
start_terminal "1_livox_ros_driver2" "roslaunch livox_ros_driver2 msg_MID360.launch"
sleep 3
start_terminal "2_MAVROS_PX4_FCU" "roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 gcs_url:=udp://@192.168.31.46:14550"
sleep 3
start_terminal "3_Start fastlio2" "roslaunch fast_lio mapping_mid360.launch rviz:=false"
sleep 3
start_terminal "4_Start VRPN to MAVROS bridge" "rosrun external_pos_fusion fastlio_fusion"
sleep 3
echo "如需关闭全部程序：./ros_stop_all.sh"
echo "  rostopic echo /Odometry/pose/pose"
echo "  rostopic echo /mavros/vision_pose/pose/pose"
echo "  rostopic echo /mavros/local_position/pose/pose"
echo "  rostopic echo /mavros/state"
