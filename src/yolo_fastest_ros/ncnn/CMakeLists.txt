cmake_minimum_required(VERSION 3.10)
project(yolo_detector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib std_msgs sensor_msgs image_transport cv_bridge target_ros_msgs
)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

# Set -Dncnn_DIR=... to use an NCNN build matching the deployment machine.
if(NOT ncnn_DIR AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/install/lib/cmake/ncnn/ncnnConfig.cmake")
  set(ncnn_DIR "${CMAKE_CURRENT_SOURCE_DIR}/install/lib/cmake/ncnn" CACHE PATH "NCNN CMake package")
endif()
find_package(ncnn CONFIG REQUIRED)

catkin_package(CATKIN_DEPENDS
  roscpp roslib std_msgs sensor_msgs image_transport cv_bridge target_ros_msgs
)
include_directories(src/include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(rosnode_yolo src/rosnode_yolo.cpp src/yolo-fastestv2.cpp)
add_dependencies(rosnode_yolo ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosnode_yolo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ncnn OpenMP::OpenMP_CXX)

if(CATKIN_ENABLE_TESTING)
  add_executable(test_sliced_inference EXCLUDE_FROM_ALL tests/test_sliced_inference.cpp)
  target_link_libraries(test_sliced_inference ${OpenCV_LIBRARIES} ncnn OpenMP::OpenMP_CXX)
endif()

install(TARGETS rosnode_yolo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
