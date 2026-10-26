cmake_minimum_required(VERSION 3.0.2)
project(match_ws)

# 复用 ROS Noetic 的标准 catkin 工作空间入口，扩展仅作用于本工作空间。
include(/opt/ros/noetic/share/catkin/cmake/toplevel.cmake)

# 前机正式流程使用的 ROS Noetic 航点节点不能继承工作区 C++20。
if(TARGET waypoint_generator)
  set_target_properties(waypoint_generator PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)
endif()
