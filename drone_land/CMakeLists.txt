cmake_minimum_required(VERSION 3.0.2)
project(drone_land)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/drone_land.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
