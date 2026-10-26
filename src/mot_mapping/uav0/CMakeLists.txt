# 前机独立目标；不导出或覆盖后机的库与包含目录。

cmake_minimum_required(VERSION 3.0.2)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_conversions message_filters sensor_msgs nav_msgs geometry_msgs visualization_msgs obj_state_msgs)

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)

find_package(Threads REQUIRED)

add_executable(mapping_node_uav0
  ../src/mapping_node.cpp
  src/mapping_manger.cpp
  ../src/Hungarian.cpp)

target_compile_features(mapping_node_uav0 PRIVATE cxx_std_14)

set_target_properties(mapping_node_uav0 PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

target_include_directories(mapping_node_uav0 PRIVATE include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

target_link_libraries(mapping_node_uav0 ${catkin_LIBRARIES} ${PCL_LIBRARIES} Threads::Threads)

add_dependencies(mapping_node_uav0 ${catkin_EXPORTED_TARGETS})

include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/include")

# ROS Noetic 及其 Boost 接口使用 C++14，避免继承工作区 C++20。
set_target_properties(mapping_node_uav0
  PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)

# 每个前机目标彻底替换继承的后机头文件路径，避免同名类的 ABI 混用。
foreach(_target IN ITEMS mapping_node_uav0)
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
