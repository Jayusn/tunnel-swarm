cmake_minimum_required(VERSION 2.8.3)
project(exploration_manager)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  # 地图与规划节点使用PointCloud2传感数据。
  sensor_msgs
  mavros_msgs
  # 2026-07-13: 第二阶段任务搜索发布覆盖和三类作业目标的 RViz 标记。
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
  tf2
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES exploration_manager
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs mavros_msgs visualization_msgs fuel_plan_env fuel_path_searching bspline bspline_opt active_perception fuel_traj_utils lkh_tsp_solver plan_manage
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(exploration_node
  src/exploration_node.cpp 
  src/fast_exploration_fsm.cpp
  src/fast_exploration_manager.cpp
  # 2026-07-13: 编译比赛第二阶段任务搜索模块，frontier 仅作为辅助候选。
  src/task_search_manager.cpp
  )
target_link_libraries(exploration_node 
  ${catkin_LIBRARIES}
  # -ldw
  )

# add_library(exploration_manager
#   src/fast_exploration_manager.cpp
# )
# target_link_libraries(exploration_manager
#   ${catkin_LIBRARIES}
# )

add_executable(exit_detection_node src/exit_detection_node.cpp src/task_search_manager.cpp)
target_link_libraries(exit_detection_node ${catkin_LIBRARIES})
add_dependencies(exit_detection_node ${catkin_EXPORTED_TARGETS})

add_executable(write_tsp test/write_tsp.cpp)
target_link_libraries(write_tsp
  ${catkin_LIBRARIES} 
)

add_executable(fuel_nav src/fuel_nav.cpp)
target_link_libraries(fuel_nav
  ${catkin_LIBRARIES} 
)

add_executable(load_obj test/load_obj.cpp)
target_link_libraries(load_obj
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(turn_escape_test test/turn_escape.test test/turn_escape_test.cpp
    src/fast_exploration_manager.cpp src/task_search_manager.cpp)
  target_link_libraries(turn_escape_test ${catkin_LIBRARIES})
  add_dependencies(turn_escape_test ${catkin_EXPORTED_TARGETS})
  add_executable(corridor_curve_heading_test test/corridor_curve_heading_test.cpp)
  add_test(NAME corridor_curve_heading_test COMMAND corridor_curve_heading_test)
  add_executable(open_space_exit_detection_test
    test/open_space_exit_detection_test.cpp src/task_search_manager.cpp)
  target_link_libraries(open_space_exit_detection_test ${catkin_LIBRARIES})
  add_test(NAME open_space_exit_detection_test COMMAND open_space_exit_detection_test)
  add_executable(entry_forward_policy_test test/entry_forward_policy_test.cpp)
  add_test(NAME entry_forward_policy_test COMMAND entry_forward_policy_test)
  add_executable(odometry_review_test test/odometry_review_test.cpp)
  target_link_libraries(odometry_review_test ${catkin_LIBRARIES})
  add_test(NAME odometry_review_test COMMAND odometry_review_test)
  add_executable(motion_direction_rules_test test/motion_direction_rules_test.cpp)
  add_test(NAME motion_direction_rules_test COMMAND motion_direction_rules_test)
  add_executable(vertical_detour_policy_test test/vertical_detour_policy_test.cpp)
  add_test(NAME vertical_detour_policy_test COMMAND vertical_detour_policy_test)
  add_executable(frontier_clearance_policy_test test/frontier_clearance_policy_test.cpp)
  add_test(NAME frontier_clearance_policy_test COMMAND frontier_clearance_policy_test)
  add_executable(path_shortening_policy_test test/path_shortening_policy_test.cpp)
  add_test(NAME path_shortening_policy_test COMMAND path_shortening_policy_test)
  add_executable(manual_waypoint_path_test test/manual_waypoint_path_test.cpp)
  add_test(NAME manual_waypoint_path_test COMMAND manual_waypoint_path_test)
  add_executable(replan_execution_policy_test test/replan_execution_policy_test.cpp)
  add_test(NAME replan_execution_policy_test COMMAND replan_execution_policy_test)
  add_executable(inflation_history_escape_policy_test
    test/inflation_history_escape_policy_test.cpp)
  add_test(NAME inflation_history_escape_policy_test
    COMMAND inflation_history_escape_policy_test)
endif()
