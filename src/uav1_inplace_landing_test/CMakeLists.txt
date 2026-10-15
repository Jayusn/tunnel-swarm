cmake_minimum_required(VERSION 3.0.2)
project(uav1_inplace_landing_test)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs mavros_msgs rosgraph_msgs quadrotor_msgs uav_utils cv_bridge image_transport)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
find_package(Threads REQUIRED)
catkin_package()
include_directories(precision/include mpc/include mpc/include/payload_mpc_controller ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_library(uav1_test_landing_core precision/src/aruco_tracker.cpp precision/src/landing_controller.cpp precision/src/landing_state_machine.cpp)
target_link_libraries(uav1_test_landing_core ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
foreach(node precision_landing_node landing_test_mission_node)
  add_executable(uav1_test_${node} precision/src/${node}.cpp)
  target_link_libraries(uav1_test_${node} uav1_test_landing_core ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
endforeach()
add_executable(uav1_test_mpc_controller_node
  mpc/src/mpc_controller_node.cpp
  mpc/src/mpc_controller.cpp
  mpc/src/mpc_input.cpp
  mpc/src/mpc_fsm.cpp
  mpc/src/mpc_wrapper.cpp
  # MPC solver
  mpc/externals/qpoases/SRC/Bounds.cpp
  mpc/externals/qpoases/SRC/Constraints.cpp
  mpc/externals/qpoases/SRC/CyclingManager.cpp
  mpc/externals/qpoases/SRC/Indexlist.cpp
  mpc/externals/qpoases/SRC/MessageHandling.cpp
  mpc/externals/qpoases/SRC/QProblem.cpp
  mpc/externals/qpoases/SRC/QProblemB.cpp
  mpc/externals/qpoases/SRC/SubjectTo.cpp
  mpc/externals/qpoases/SRC/Utils.cpp
  mpc/externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  mpc/model/quadrotor_payload_mpc/acado_qpoases_interface.cpp
  mpc/model/quadrotor_payload_mpc/acado_integrator.c
  mpc/model/quadrotor_payload_mpc/acado_solver.c
  mpc/model/quadrotor_payload_mpc/acado_auxiliary_functions.c
  )
target_include_directories(uav1_test_mpc_controller_node PUBLIC
  mpc/include/payload_mpc_controller
  mpc/include
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  # MPC solver
  mpc/model/quadrotor_payload_mpc/
  mpc/externals/qpoases
  mpc/externals/qpoases/INCLUDE
  mpc/externals/qpoases/SRC
  )
target_link_libraries(uav1_test_mpc_controller_node
  # mpc_solver
  ${catkin_LIBRARIES}
)


catkin_install_python(PROGRAMS scripts/landing_nmpc_test_router.py scripts/autotrans_mpc_logger.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_uav1_landing_state_machine
    test/test_landing_state_machine.cpp test/gtest_main.cpp)
  if(TARGET test_uav1_landing_state_machine)
    target_link_libraries(test_uav1_landing_state_machine uav1_test_landing_core ${catkin_LIBRARIES})
  endif()
endif()
