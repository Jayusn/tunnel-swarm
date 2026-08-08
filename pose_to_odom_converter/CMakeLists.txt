cmake_minimum_required(VERSION 3.0.2)
project(pose_to_odom_converter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_to_odom_converter_node src/pose_to_odom_converter_node.cpp)

target_link_libraries(pose_to_odom_converter_node
  ${catkin_LIBRARIES}
)
