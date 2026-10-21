cmake_minimum_required(VERSION 2.8.3)
project(payload_mpc_controller)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# ARM NEON flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

# flags for speed (should already be enabled by default)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3 -march=native")

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  rosgraph_msgs
  quadrotor_msgs
  traj_utils
  uav_utils

)

catkin_package()

# Record the configured source revision and content fingerprint, including local edits.
execute_process(COMMAND git describe --always --dirty WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE FORCE_GIT_REV OUTPUT_STRIP_TRAILING_WHITESPACE)
set(FORCE_SOURCE_HASHES "")
foreach(source include/payload_mpc_controller/aerothrow_ndob.h
    include/payload_mpc_controller/selected_force_estimator.h
    include/payload_mpc_controller/multi_optimization_based_force_estimator.hpp
    include/payload_mpc_controller/mpc_params.h include/payload_mpc_controller/mpc_fsm.h
    include/payload_mpc_controller/lowpassfilter2p.h
    include/payload_mpc_controller/force_observer_input_synchronizer.h
    include/payload_mpc_controller/vertical_disturbance_low_pass.h
    include/payload_mpc_controller/disturbance_slew_limiter.h
    src/mpc_input.cpp src/mpc_fsm.cpp src/force_estimator_replay.cpp)
  file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${source}" hash)
  string(APPEND FORCE_SOURCE_HASHES "${hash}")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${source}")
endforeach()
string(SHA256 FORCE_SOURCE_HASH "${FORCE_SOURCE_HASHES}")
configure_file(include/force_build_info.h.in ${CMAKE_CURRENT_BINARY_DIR}/force_build_info.h @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#include dir
# include_directories(
#   include
#   include/payload_mpc_controller
# )

# add_library(mpc_solver
#   externals/qpoases/SRC/Bounds.cpp
#   externals/qpoases/SRC/Constraints.cpp
#   externals/qpoases/SRC/CyclingManager.cpp
#   externals/qpoases/SRC/Indexlist.cpp
#   externals/qpoases/SRC/MessageHandling.cpp
#   externals/qpoases/SRC/QProblem.cpp
#   externals/qpoases/SRC/QProblemB.cpp
#   externals/qpoases/SRC/SubjectTo.cpp
#   externals/qpoases/SRC/Utils.cpp
#   externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
#   model/quadrotor_payload_mpc/acado_qpoases_interface.cpp
#   model/quadrotor_payload_mpc/acado_integrator.c
#   model/quadrotor_payload_mpc/acado_solver.c
#   model/quadrotor_payload_mpc/acado_auxiliary_functions.c)

# target_include_directories(mpc_solver PUBLIC
#   model/quadrotor_payload_mpc/
#   externals/qpoases
#   externals/qpoases/INCLUDE
#   externals/qpoases/SRC
#   )

add_executable(mpc_controller_node 
  src/mpc_controller_node.cpp
  src/mpc_controller.cpp
  src/mpc_input.cpp
  src/mpc_fsm.cpp
  src/mpc_wrapper.cpp
  # MPC solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_payload_mpc/acado_qpoases_interface.cpp
  model/quadrotor_payload_mpc/acado_integrator.c
  model/quadrotor_payload_mpc/acado_solver.c
  model/quadrotor_payload_mpc/acado_auxiliary_functions.c
  )
target_include_directories(mpc_controller_node PUBLIC
  include/payload_mpc_controller
  include 
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  # MPC solver
  model/quadrotor_payload_mpc/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
  )
target_link_libraries(mpc_controller_node
  # mpc_solver
  ${catkin_LIBRARIES}
)

add_dependencies(mpc_controller_node ${catkin_EXPORTED_TARGETS})

get_target_property(force_estimator_includes mpc_controller_node INCLUDE_DIRECTORIES)
add_executable(force_estimator_replay src/force_estimator_replay.cpp)
# ROS Noetic / Boost 1.71 rosbag headers require the pre-C++20 allocator API.
set_target_properties(force_estimator_replay PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)
target_include_directories(force_estimator_replay PRIVATE ${force_estimator_includes})
target_link_libraries(force_estimator_replay ${catkin_LIBRARIES} yaml-cpp)
install(TARGETS mpc_controller_node force_estimator_replay
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_input_callback_priority.py)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_selected_force_estimator test/test_selected_force_estimator.cpp)
  if(TARGET test_selected_force_estimator)
    target_include_directories(test_selected_force_estimator PRIVATE ${force_estimator_includes})
    target_link_libraries(test_selected_force_estimator ${catkin_LIBRARIES})
  endif()
  get_target_property(locked_yaw_test_sources mpc_controller_node SOURCES)
  list(REMOVE_ITEM locked_yaw_test_sources src/mpc_controller_node.cpp)
  add_rostest_gtest(test_locked_exit_yaw test/locked_exit_yaw.test
    test/test_locked_exit_yaw.cpp test/test_tracking_replan.cpp test/test_waypoint_yaw.cpp test/test_force_controller_integration.cpp
    test/test_vertical_velocity_input.cpp ${locked_yaw_test_sources})
  if(TARGET test_locked_exit_yaw)
    get_target_property(locked_yaw_test_includes mpc_controller_node INCLUDE_DIRECTORIES)
    target_include_directories(test_locked_exit_yaw PRIVATE ${locked_yaw_test_includes})
    add_dependencies(test_locked_exit_yaw ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_locked_exit_yaw ${catkin_LIBRARIES})
  endif()
  foreach(force_test force_observer_input_synchronizer disturbance_slew_limiter vertical_disturbance_low_pass)
    catkin_add_gtest(test_${force_test} test/test_${force_test}.cpp)
    if(TARGET test_${force_test})
      target_include_directories(test_${force_test} PRIVATE
        include/payload_mpc_controller
        ${EIGEN3_INCLUDE_DIR}
      )
    endif()
  endforeach()

  # 使用与实机相同的 ACADO/qpOASES，回归旋转矩阵转四元数的符号边界。
  get_target_property(quaternion_test_sources mpc_controller_node SOURCES)
  list(REMOVE_ITEM quaternion_test_sources
    src/mpc_controller_node.cpp src/mpc_controller.cpp src/mpc_input.cpp src/mpc_fsm.cpp)
  catkin_add_gtest(test_quaternion_reference
    test/test_quaternion_reference.cpp ${quaternion_test_sources})
  if(TARGET test_quaternion_reference)
    get_target_property(quaternion_test_includes mpc_controller_node INCLUDE_DIRECTORIES)
    target_include_directories(test_quaternion_reference PRIVATE ${quaternion_test_includes})
    target_link_libraries(test_quaternion_reference ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_odom_spike_guard
    test/test_odom_spike_guard.cpp
  )
  if(TARGET test_odom_spike_guard)
    target_include_directories(test_odom_spike_guard PRIVATE
      include/payload_mpc_controller
      ${EIGEN3_INCLUDE_DIR}
    )
  endif()

  catkin_add_gtest(test_recovery_control
    test/test_recovery_control.cpp
  )
  if(TARGET test_recovery_control)
    target_include_directories(test_recovery_control PRIVATE
      include/payload_mpc_controller
      ${EIGEN3_INCLUDE_DIR}
    )
  endif()
endif()

catkin_install_python(PROGRAMS
  scripts/autotrans_mpc_logger.py
  scripts/generate_evo_report.py
  scripts/compare_force_estimators.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  scripts/analyze_autotrans_wind_mpc.m
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
