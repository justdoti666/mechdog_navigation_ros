cmake_minimum_required(VERSION 3.16)
project(mechdog_navigation_ros)

# ROS2 版本兼容: Jazzy (Ubuntu 24.04) 默认; Humble 需改 ROS_DISTRO 环境变量, 代码无需改动
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 纯算法库源码目录 (上游 mechdog_navigation 仓库, 作为子目录引用)
# ROS-1 (v2.2): 改为 CACHE PATH 可由 -DMECHDOG_ALGO_DIR=<path> 覆盖;
# FATAL 守卫已存在 (下方), 仅补可配置性, 解耦同级目录强耦合
set(MECHDOG_ALGO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../mechdog_navigation"
    CACHE PATH "mechdog_navigation 纯算法库源码目录 (可由 -DMECHDOG_ALGO_DIR=<path> 覆盖; 默认同级目录)")
if(NOT EXISTS "${MECHDOG_ALGO_DIR}/sensor_fusion.cpp")
  message(FATAL_ERROR "未找到纯算法库源码: ${MECHDOG_ALGO_DIR} (需要与 mechdog_navigation 仓库同级, 或传 -DMECHDOG_ALGO_DIR=<path>)")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 真机 Astra Pro: 与上游算法库同名开关, 透传编译定义与 SDK 链接 (FIX-6/ROS-1)
option(USE_ASTRA_SDK "Enable Astra Pro via Orbbec Astra SDK (真机模式)" OFF)
set(ASTRA_SDK_ROOT "" CACHE PATH "Orbbec Astra SDK 安装根目录")

# 注: /fusion_result 使用 std_msgs/String (JSON), 避免自定义消息的 typesupport 链接复杂性;
#     如需强类型消息可后续引入 rosidl_generate_interfaces (见 README 备注)

# 算法库对象库 (纯算法, 不生成可执行文件)
add_library(mechdog_algo OBJECT
  ${MECHDOG_ALGO_DIR}/sensor_astra.cpp
  ${MECHDOG_ALGO_DIR}/sensor_ultrasonic.cpp
  ${MECHDOG_ALGO_DIR}/sensor_ir.cpp
  ${MECHDOG_ALGO_DIR}/sensor_fusion.cpp
  ${MECHDOG_ALGO_DIR}/path_planner.cpp
  ${MECHDOG_ALGO_DIR}/point_cloud.cpp
)
target_include_directories(mechdog_algo PRIVATE ${MECHDOG_ALGO_DIR})
target_compile_features(mechdog_algo PRIVATE cxx_std_20)

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
target_compile_features(safety_node PRIVATE cxx_std_20)

# 真机 Astra Pro: 与上游算法库同名开关, 透传编译定义与 SDK 链接 (FIX-6/ROS-1)
# 置于 safety_node 定义之后 (target_link_* 需要 target 已存在)
if(USE_ASTRA_SDK)
    if(NOT ASTRA_SDK_ROOT)
        message(FATAL_ERROR "USE_ASTRA_SDK=ON 需要指定 -DASTRA_SDK_ROOT=<sdk根目录>")
    endif()
    if(NOT EXISTS "${ASTRA_SDK_ROOT}/include/astra/astra.hpp")
        message(FATAL_ERROR "未找到 Astra SDK: ${ASTRA_SDK_ROOT}")
    endif()
    target_compile_definitions(mechdog_algo PRIVATE USE_ASTRA_SDK)
    target_include_directories(mechdog_algo PRIVATE "${ASTRA_SDK_ROOT}/include")
    target_link_directories(safety_node PRIVATE "${ASTRA_SDK_ROOT}/lib")
    # 保持与 50 行一致的 plain 签名 (CMake 禁止同一 target 混用两种签名)
    target_link_libraries(safety_node astra_core astra_core_api astra)
    message(STATUS "USE_ASTRA_SDK: 真机深度驱动已启用 (${ASTRA_SDK_ROOT})")
endif()

# 底盘通信节点: 订阅 /cmd_vel -> ChassisBridge (simulated/stm32)
add_executable(chassis_bridge_node src/chassis_bridge_node.cpp)
target_include_directories(chassis_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(chassis_bridge_node
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
)
target_compile_features(chassis_bridge_node PRIVATE cxx_std_20)

install(TARGETS safety_node chassis_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# 注: 融合结果以 std_msgs/String (JSON) 发布、节点参数内联在 launch 中 (FIX-16/ROS-6),
#     已删除无消费方的 msg/FusionResult.msg 与 config/safety_params.yaml

# ROS-10 (v2.2): lint + smoke test (chassis_bridge.hpp 是纯 C++ 头, smoke 验证链接+构造)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_smoke test/test_smoke.cpp)
  target_include_directories(test_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_compile_features(test_smoke PRIVATE cxx_std_20)
endif()

ament_package()
