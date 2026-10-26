# 前机独立目标；不导出或覆盖后机的库与包含目录。

cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs

  sensor_msgs
  mavros_msgs

  visualization_msgs
  quadrotor_msgs
  fuel_plan_env
  fuel_path_searching
  bspline
  bspline_opt
  fuel_traj_utils
  active_perception
  lkh_tsp_solver
  plan_manage
  cv_bridge
  tf
  tf2)

set(_front_libraries "")
foreach(_library IN LISTS catkin_LIBRARIES)
  get_filename_component(_name "${_library}" NAME_WE)
  string(REGEX REPLACE "^lib" "" _name "${_name}")
  if(_name STREQUAL "fuel_plan_env")
    list(APPEND _front_libraries fuel_plan_env_uav0)
  elseif(_name STREQUAL "fuel_path_searching")
    list(APPEND _front_libraries fuel_path_searching_uav0)
  elseif(_name STREQUAL "active_perception")
    list(APPEND _front_libraries active_perception_uav0)
  elseif(_name STREQUAL "bspline_opt")
    list(APPEND _front_libraries bspline_opt_uav0)
  elseif(_name STREQUAL "plan_manage")
    list(APPEND _front_libraries plan_manage_uav0)
  elseif(_name STREQUAL "bspline")
    list(APPEND _front_libraries bspline_uav0)
  elseif(_name STREQUAL "fuel_traj_utils")
    list(APPEND _front_libraries fuel_traj_utils_uav0)
  else()
    list(APPEND _front_libraries "${_library}")
  endif()
endforeach()
set(catkin_LIBRARIES ${_front_libraries})

include_directories(BEFORE include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})

add_executable(exploration_node_uav0
  ../src/exploration_node.cpp
  src/fast_exploration_fsm.cpp
  src/fast_exploration_manager.cpp
  src/safe_relay_publisher.cpp
  src/task_search_manager.cpp)

target_link_libraries(exploration_node_uav0
  ${catkin_LIBRARIES})

include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_dependencies(exploration_node_uav0 ${catkin_EXPORTED_TARGETS})

# ROS Noetic 及其 Boost 接口使用 C++14，避免继承工作区 C++20。
set_target_properties(exploration_node_uav0
  PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)

# 每个前机目标彻底替换继承的后机头文件路径，避免同名类的 ABI 混用。
foreach(_target IN ITEMS exploration_node_uav0)
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

if(CATKIN_ENABLE_TESTING)
  get_target_property(_front_test_includes exploration_node_uav0 INCLUDE_DIRECTORIES)
  foreach(_test IN ITEMS safe_relay_policy_test route_width_policy_test passage_centering_policy_test vertical_detour_policy_test frontier_clearance_policy_test inflation_history_escape_policy_test)
    add_executable(${_test}_uav0 test/${_test}.cpp)
    set_property(TARGET ${_test}_uav0 PROPERTY INCLUDE_DIRECTORIES "${_front_test_includes}")
    target_link_libraries(${_test}_uav0 ${catkin_LIBRARIES})
    set_target_properties(${_test}_uav0 PROPERTIES CXX_STANDARD 14)
    add_test(NAME ${_test}_uav0 COMMAND ${_test}_uav0)
  endforeach()
endif()
