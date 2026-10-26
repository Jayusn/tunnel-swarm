# 前机独立目标；不导出或覆盖后机的库与包含目录。

cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-std=c++11)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  cv_bridge
  pcl_conversions)

set(_front_libraries "")
foreach(_library IN LISTS catkin_LIBRARIES)
  get_filename_component(_name "${_library}" NAME_WE)
  string(REGEX REPLACE "^lib" "" _name "${_name}")
  if(_name STREQUAL "diff_plan_env_target")
    list(APPEND _front_libraries diff_plan_env_target_uav0)
  elseif(_name STREQUAL "plan_env")
    list(APPEND _front_libraries diff_plan_env_target_uav0)
  elseif(_name STREQUAL "diff_path_searching_target")
    list(APPEND _front_libraries diff_path_searching_target_uav0)
  elseif(_name STREQUAL "path_searching")
    list(APPEND _front_libraries diff_path_searching_target_uav0)
  elseif(_name STREQUAL "diff_traj_utils")
    list(APPEND _front_libraries diff_traj_utils_uav0)
  elseif(_name STREQUAL "traj_opt")
    list(APPEND _front_libraries traj_opt_uav0)
  elseif(_name STREQUAL "bspline")
    list(APPEND _front_libraries bspline_uav0)
  elseif(_name STREQUAL "fuel_traj_utils")
    list(APPEND _front_libraries fuel_traj_utils_uav0)
  else()
    list(APPEND _front_libraries "${_library}")
  endif()
endforeach()
set(catkin_LIBRARIES ${_front_libraries})

find_package(ldop REQUIRED)

include_directories(BEFORE include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${ldop_INCLUDE_DIRS})

add_executable(diff_planner_node_uav0
  ../src/diff_planner_node.cpp
  src/diff_replan_fsm.cpp
  src/planner_manager.cpp)

target_link_libraries(diff_planner_node_uav0
  ${catkin_LIBRARIES}

  diff_traj_utils_uav0)

add_executable(diff_traj_server_target_uav0
  src/traj_server.cpp)

set_target_properties(diff_traj_server_target_uav0 PROPERTIES OUTPUT_NAME traj_server_uav0)

target_link_libraries(diff_traj_server_target_uav0 ${catkin_LIBRARIES})

add_executable(corridor_search_manager_uav0
  src/corridor_search_manager.cpp)

target_link_libraries(corridor_search_manager_uav0 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(landing_diff_search_manager_uav0
  src/landing_diff_search_manager.cpp)

target_link_libraries(landing_diff_search_manager_uav0 ${catkin_LIBRARIES})

add_executable(planner_command_arbiter_uav0
  src/planner_command_arbiter.cpp)

add_dependencies(planner_command_arbiter_uav0 ${ldop_EXPORTED_TARGETS})

target_link_libraries(planner_command_arbiter_uav0 ${catkin_LIBRARIES})

include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_dependencies(diff_planner_node_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(diff_traj_server_target_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(corridor_search_manager_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(landing_diff_search_manager_uav0 ${catkin_EXPORTED_TARGETS})

add_dependencies(planner_command_arbiter_uav0 ${catkin_EXPORTED_TARGETS})

# ROS Noetic 及其 Boost 接口使用 C++14，避免继承工作区 C++20。
set_target_properties(diff_planner_node_uav0 diff_traj_server_target_uav0 corridor_search_manager_uav0 landing_diff_search_manager_uav0 planner_command_arbiter_uav0
  PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)

# 每个前机目标彻底替换继承的后机头文件路径，避免同名类的 ABI 混用。
foreach(_target IN ITEMS diff_planner_node_uav0 diff_traj_server_target_uav0 corridor_search_manager_uav0 landing_diff_search_manager_uav0 planner_command_arbiter_uav0)
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
