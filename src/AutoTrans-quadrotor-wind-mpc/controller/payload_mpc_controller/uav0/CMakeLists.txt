# 前机独立目标；不导出或覆盖后机的库与包含目录。

cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")

message("enabling ARM neon optimizations")

endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3 -march=native")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  rosgraph_msgs
  quadrotor_msgs
  uav_utils)

add_executable(mpc_controller_node_uav0
  src/mpc_controller_node.cpp
  src/mpc_controller.cpp
  src/mpc_input.cpp
  src/mpc_fsm.cpp
  src/mpc_wrapper.cpp
  ../externals/qpoases/SRC/Bounds.cpp
  ../externals/qpoases/SRC/Constraints.cpp
  ../externals/qpoases/SRC/CyclingManager.cpp
  ../externals/qpoases/SRC/Indexlist.cpp
  ../externals/qpoases/SRC/MessageHandling.cpp
  ../externals/qpoases/SRC/QProblem.cpp
  ../externals/qpoases/SRC/QProblemB.cpp
  ../externals/qpoases/SRC/SubjectTo.cpp
  ../externals/qpoases/SRC/Utils.cpp
  ../externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  ../model/quadrotor_payload_mpc/acado_qpoases_interface.cpp
  ../model/quadrotor_payload_mpc/acado_integrator.c
  ../model/quadrotor_payload_mpc/acado_solver.c
  ../model/quadrotor_payload_mpc/acado_auxiliary_functions.c)

target_include_directories(mpc_controller_node_uav0 PUBLIC
  include/payload_mpc_controller
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}

  ../model/quadrotor_payload_mpc/
  ../externals/qpoases
  ../externals/qpoases/INCLUDE
  ../externals/qpoases/SRC)

target_link_libraries(mpc_controller_node_uav0

  ${catkin_LIBRARIES})

include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_dependencies(mpc_controller_node_uav0 ${catkin_EXPORTED_TARGETS})

# ROS Noetic 及其 Boost 接口使用 C++14，避免继承工作区 C++20。
set_target_properties(mpc_controller_node_uav0
  PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_trajectory_input_uav0 test/test_trajectory_input_uav0.cpp src/mpc_input.cpp)
  if(TARGET test_trajectory_input_uav0)
    get_target_property(_controller_includes mpc_controller_node_uav0 INCLUDE_DIRECTORIES)
    target_include_directories(test_trajectory_input_uav0 PRIVATE ${_controller_includes})
    target_link_libraries(test_trajectory_input_uav0 ${catkin_LIBRARIES})
    add_dependencies(test_trajectory_input_uav0 ${catkin_EXPORTED_TARGETS})
    set_target_properties(test_trajectory_input_uav0 PROPERTIES CXX_STANDARD 14)
  endif()
endif()

# 每个前机目标彻底替换继承的后机头文件路径，避免同名类的 ABI 混用。
foreach(_target IN ITEMS mpc_controller_node_uav0)
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
