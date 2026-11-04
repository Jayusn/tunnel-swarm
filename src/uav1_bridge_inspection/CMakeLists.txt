cmake_minimum_required(VERSION 3.0.2)
project(uav1_bridge_inspection)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/bridge_mission.py scripts/bridge_view.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
