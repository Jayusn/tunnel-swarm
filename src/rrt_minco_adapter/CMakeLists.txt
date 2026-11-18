cmake_minimum_required(VERSION 3.0.2)
project(rrt_minco_adapter)
find_package(catkin REQUIRED COMPONENTS roscpp fuel_plan_env)
catkin_package()
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_executable(rrt_fuel_map src/fuel_map_node.cpp)
set_property(TARGET rrt_fuel_map PROPERTY CXX_STANDARD 14)
target_link_libraries(rrt_fuel_map ${catkin_LIBRARIES})
install(TARGETS rrt_fuel_map RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/rrt_minco_node.py scripts/cloud_to_scene_node.py
  scripts/point_to_point_mission.py scripts/ground_plan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/start_flight.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config rrt_core scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
