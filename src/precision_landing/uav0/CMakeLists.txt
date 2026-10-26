# 前机独立目标；不导出或覆盖后机的库与包含目录。

cmake_minimum_required(VERSION 3.0.2)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs mavros_msgs
  cv_bridge image_transport camera_info_manager rostest
  tf2 tf2_ros tf2_geometry_msgs message_generation)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

find_package(Threads REQUIRED)

include_directories(BEFORE include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(landing_controller_uav0
  ../src/landing_controller.cpp)

target_link_libraries(landing_controller_uav0 ${catkin_LIBRARIES})

add_library(landing_state_machine_uav0
  src/landing_state_machine.cpp)

target_link_libraries(landing_state_machine_uav0 ${catkin_LIBRARIES})

add_library(aruco_tracker_uav0
  ../src/aruco_tracker.cpp)

target_link_libraries(aruco_tracker_uav0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(landing_search_core_uav0
  ../src/landing_search_core.cpp)

target_link_libraries(landing_search_core_uav0 ${catkin_LIBRARIES})

add_executable(landing_search_node_uav0
  src/landing_search_node.cpp)

add_dependencies(landing_search_node_uav0 ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(landing_search_node_uav0
  landing_search_core_uav0 aruco_tracker_uav0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(front_aruco_hint_node_uav0
  src/front_aruco_hint_node.cpp)

add_dependencies(front_aruco_hint_node_uav0 ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(front_aruco_hint_node_uav0
  landing_search_core_uav0 aruco_tracker_uav0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(precision_landing_node_uav0
  src/precision_landing_node.cpp)

target_link_libraries(precision_landing_node_uav0
  landing_controller_uav0 landing_state_machine_uav0 aruco_tracker_uav0
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(landing_setpoint_arbiter_node_uav0
  src/landing_setpoint_arbiter_node.cpp)

target_link_libraries(landing_setpoint_arbiter_node_uav0 ${catkin_LIBRARIES})

add_executable(landing_debug_image_mux_node_uav0
  ../src/landing_debug_image_mux_node.cpp)

target_link_libraries(landing_debug_image_mux_node_uav0 ${catkin_LIBRARIES})

add_executable(dual_uav_landing_coordinator_node_uav0
  src/dual_uav_landing_coordinator_node.cpp)

add_dependencies(dual_uav_landing_coordinator_node_uav0 ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(dual_uav_landing_coordinator_node_uav0 ${catkin_LIBRARIES})

include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_dependencies(landing_controller_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(landing_state_machine_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(aruco_tracker_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(landing_search_core_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(landing_search_node_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(front_aruco_hint_node_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(precision_landing_node_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(landing_setpoint_arbiter_node_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(landing_debug_image_mux_node_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(dual_uav_landing_coordinator_node_uav0 ${catkin_EXPORTED_TARGETS})

# ROS Noetic 及其 Boost 接口使用 C++14，避免继承工作区 C++20。
set_target_properties(landing_controller_uav0 landing_state_machine_uav0 aruco_tracker_uav0 landing_search_core_uav0 landing_search_node_uav0 front_aruco_hint_node_uav0 precision_landing_node_uav0 landing_setpoint_arbiter_node_uav0 landing_debug_image_mux_node_uav0 dual_uav_landing_coordinator_node_uav0
  PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)

# 每个前机目标彻底替换继承的后机头文件路径，避免同名类的 ABI 混用。
foreach(_target IN ITEMS landing_controller_uav0 landing_state_machine_uav0 aruco_tracker_uav0 landing_search_core_uav0 landing_search_node_uav0 front_aruco_hint_node_uav0 precision_landing_node_uav0 landing_setpoint_arbiter_node_uav0 landing_debug_image_mux_node_uav0 dual_uav_landing_coordinator_node_uav0)
  get_target_property(_includes ${_target} INCLUDE_DIRECTORIES)
  set(_private_includes "${CMAKE_CURRENT_SOURCE_DIR}/include")
  foreach(_include IN LISTS _includes)
    if(EXISTS "${_include}/../uav0/include")
      get_filename_component(_include "${_include}/../uav0/include" REALPATH)
    endif()
    list(APPEND _private_includes "${_include}")
  endforeach()
  list(REMOVE_DUPLICATES _private_includes)
  set_property(TARGET ${_target} PROPERTY INCLUDE_DIRECTORIES "${_private_includes}")
endforeach()
