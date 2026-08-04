#!/bin/bash
# 批量关闭所有ROS相关进程
pkill -f "roslaunch"
pkill -f "rosrun"
pkill -f "roscore"
pkill -f "rosmaster"

echo "=================================="
echo "所有 ROS 主节点、运行程序已终止"
echo "=================================="
