cmake_minimum_required(VERSION 3.0.2)
project(ldop)

set(CMAKE_CXX_STANDARD 14)

# 要求编译器必须支持上面的标准；如果不支持就直接报错，
# 而不是偷偷降级成更老的标准。
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 关闭编译器的 GNU 扩展，尽量使用更标准的 C++ 语法。
# 简单理解：让代码更“标准 C++”，减少平台差异。
set(CMAKE_CXX_EXTENSIONS OFF)

# 生成 compile_commands.json，clangd 会用它来知道
# “每个源文件到底是怎么编译的”。
# 没有这个文件时，clangd 的跳转、补全、报错经常不准。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 给编译器添加常用警告选项。
# -Wall/-Wextra: 打开更多常见警告
# -Wpedantic: 对不够规范的写法更严格
# 学习阶段建议保留，能尽早发现代码问题。
add_compile_options(-Wall -Wextra -Wpedantic)

# find_package(catkin REQUIRED COMPONENTS ...) 的作用是：
# 告诉 catkin“这个包编译时依赖哪些 ROS 包”。
# 这里列出的包，通常也应该出现在 package.xml 里。
# 这些依赖会提供头文件、库、消息类型等。
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
find_package(Threads REQUIRED)

add_message_files(
  FILES
  PredictedState.msg
  DynamicObject.msg
  DynamicObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

# catkin_package(...) 不是“编译当前包”，而是“向外声明当前包能提供什么”。
# 如果未来别的 ROS 包要依赖 ldop，它们会从这里获取导出信息。
#
# INCLUDE_DIRS include
# 表示：本包对外暴露的头文件在 include/ 目录里。
#
# LIBRARIES ldop_core
# 表示：本包对外提供一个名为 ldop_core 的库。
#
# CATKIN_DEPENDS ...
# 表示：依赖 ldop 的其他包，也会间接依赖这些 catkin 包。
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ldop_core
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp sensor_msgs std_msgs visualization_msgs
)

# include_directories(...) 指定编译时去哪些目录找头文件。
#
# include
# 对应你自己包里的头文件目录，例如：
#   #include <LDOP/ldop.h>
#
# ${catkin_INCLUDE_DIRS}
# 对应上面 find_package(catkin ...) 找到的 ROS 依赖头文件目录。
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(...) 定义一个“库”目标。
# 这里把真正的核心逻辑放进 ldop_core 里，而不是全塞进 main 函数。
# 这样做的好处是：
# 1. 结构更清晰
# 2. 后续更容易拆分模块
# 3. 别的可执行程序将来也能复用这部分代码
add_library(ldop_core
  src/ldop.cpp
)

# add_dependencies(...) 用来声明“编译顺序依赖”。
# ${catkin_EXPORTED_TARGETS} 常用于消息/服务自动生成的场景。
# 你这个包现在还没自定义 msg/srv，但保留这行是个好习惯，
# 以后一旦加了消息或服务，通常不用再回头补这类依赖。
add_dependencies(ldop_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(...) 表示链接阶段要把哪些库连进来。
# 这里 ldop_core 需要使用 roscpp 等 ROS 功能，所以要链接 catkin 提供的库。
target_link_libraries(ldop_core
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

# add_executable(...) 定义一个可执行文件。
# 这里生成的是节点程序 ldop_node，
# 对应的源码入口就是 src/ldop_node.cpp 里的 main 函数。
add_executable(ldop_node
  src/ldop_node.cpp
)

# 和上面的库一样，节点也声明一下对 catkin 生成目标的依赖。
add_dependencies(ldop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 节点最终需要链接两部分：
# 1. ldop_core：你自己写的核心逻辑库
# 2. ${catkin_LIBRARIES}：ROS 相关库
#
# 这样 main 函数就可以调用你自己的类，同时也能正常使用 ROS API。
target_link_libraries(ldop_node
  ldop_core
  ${catkin_LIBRARIES}
)
