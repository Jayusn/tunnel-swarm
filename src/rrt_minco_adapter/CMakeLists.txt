cmake_minimum_required(VERSION 3.0.2)
project(rrt_minco_adapter)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/rrt_minco_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config rrt_core scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
