cmake_minimum_required(VERSION 3.16)
project(mechdog_navigation_ros)

# ROS2 版本兼容: Jazzy (Ubuntu 24.04) 默认; Humble 需改 ROS_DISTRO 环境变量, 代码无需改动
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 纯算法库源码目录 (上游 mechdog_navigation 仓库, 作为子目录引用)
set(MECHDOG_ALGO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../mechdog_navigation")
if(NOT EXISTS "${MECHDOG_ALGO_DIR}/sensor_fusion.cpp")
  message(FATAL_ERROR "未找到纯算法库源码: ${MECHDOG_ALGO_DIR} (需要与 mechdog_navigation 仓库同级)")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 注: /fusion_result 使用 std_msgs/String (JSON), 避免自定义消息的 typesupport 链接复杂性;
#     如需强类型消息可后续引入 rosidl_generate_interfaces (见 README 备注)

# 算法库对象库 (纯算法, 不生成可执行文件)
add_library(mechdog_algo OBJECT
  ${MECHDOG_ALGO_DIR}/sensor_astra.cpp
  ${MECHDOG_ALGO_DIR}/sensor_ultrasonic.cpp
  ${MECHDOG_ALGO_DIR}/sensor_ir.cpp
  ${MECHDOG_ALGO_DIR}/sensor_fusion.cpp
  ${MECHDOG_ALGO_DIR}/path_planner.cpp
)
target_include_directories(mechdog_algo PRIVATE ${MECHDOG_ALGO_DIR})
target_compile_features(mechdog_algo PRIVATE cxx_std_17)

# 主节点: 局部安全层 (融合 + 规划 + 速度仲裁)
add_executable(safety_node src/safety_node.cpp)
target_include_directories(safety_node PRIVATE
  ${MECHDOG_ALGO_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
target_link_libraries(safety_node
  mechdog_algo
  rclcpp::rclcpp
  std_msgs::std_msgs
  geometry_msgs::geometry_msgs
  sensor_msgs::sensor_msgs
)
target_compile_features(safety_node PRIVATE cxx_std_17)

install(TARGETS safety_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
