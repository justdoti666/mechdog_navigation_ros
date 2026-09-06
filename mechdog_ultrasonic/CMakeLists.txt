cmake_minimum_required(VERSION 3.16)
project(mechdog_ultrasonic)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 真实 GPIO 读取开关: 树莓派上 ON (需 libgpiod), WSL/PC 模拟 OFF
option(USE_GPIO "Enable real HC-SR04 GPIO read via libgpiod (Raspberry Pi)" OFF)
# libgpiod 版本: v1 (原 gpiod_*) 或 v2 (新 gpiod_line_*)。树上多为 v1, 24.04 可能 v2
set(LIBGPIOD_VERSION "v1" CACHE STRING "libgpiod API version: v1 or v2")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

# ---- 消息生成 ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UltrasonicArray.msg"
  DEPENDENCIES builtin_interfaces
)

# ---- 节点 ----
add_executable(ultrasonic_node src/ultrasonic_node.cpp)
target_include_directories(ultrasonic_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(ultrasonic_node
  rclcpp::rclcpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
target_compile_features(ultrasonic_node PRIVATE cxx_std_20)

if(USE_GPIO)
  # 树莓派: libgpiod 真读
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
  target_compile_definitions(ultrasonic_node PRIVATE USE_GPIO=1)
  if(LIBGPIOD_VERSION STREQUAL "v2")
    target_compile_definitions(ultrasonic_node PRIVATE LIBGPIOD_V2=1)
  endif()
  target_include_directories(ultrasonic_node PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
  target_link_libraries(ultrasonic_node ${LIBGPIOD_LIBRARIES})
  message(STATUS "ultrasonic_node: 启用 libgpiod 真读 (${LIBGPIOD_VERSION})")
else()
  message(STATUS "ultrasonic_node: 无 GPIO (模拟模式, 开关 USE_GPIO=ON 启用树莓派真读)")
endif()

install(TARGETS ultrasonic_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
