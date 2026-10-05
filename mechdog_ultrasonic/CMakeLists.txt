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
# libgpiod API 开关 (U-2, 2026-10-05): auto|v1|v2 (auto = 按库版本 >=2 自动选 v2)
#   修复: 旧名 LIBGPIOD_VERSION 与 pkg_check_modules 产生的同名普通变量冲突 ——
#   库版本号(如 1.6.3) 会遮蔽 cache 变量, 致 -DLIBGPIOD_VERSION=v2 静默失效、落回 v1 编译.
set(MECHDOG_GPIOD_API "auto" CACHE STRING "libgpiod API: auto|v1|v2 (auto=按库版本 >=2 选 v2)")
# 旧开关名一次性兼容: 非版本号值(v1/v2)告警并转义; 随后清除同名变量防遮蔽
if(DEFINED LIBGPIOD_VERSION AND NOT LIBGPIOD_VERSION MATCHES "^[0-9]")
  message(WARNING "LIBGPIOD_VERSION='${LIBGPIOD_VERSION}' 为已废弃开关名 (与 pkg-config 同名变量冲突): 请改用 -DMECHDOG_GPIOD_API=auto|v1|v2; 本次按旧值兼容处理")
  if(LIBGPIOD_VERSION MATCHES "^v[12]$" AND MECHDOG_GPIOD_API STREQUAL "auto")
    set(MECHDOG_GPIOD_API "${LIBGPIOD_VERSION}" CACHE STRING "libgpiod API: auto|v1|v2" FORCE)
  endif()
endif()
if(DEFINED LIBGPIOD_VERSION)
  unset(LIBGPIOD_VERSION CACHE)
  unset(LIBGPIOD_VERSION)
endif()

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

# ---- 单测 (U-1, 2026-10-05): publish-mode seam 先红后绿 ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ultra_publish_mode test/test_ultra_publish_mode.cpp)
  target_include_directories(test_ultra_publish_mode PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()

if(USE_GPIO)
  # 树莓派: libgpiod 真读
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
  target_compile_definitions(ultrasonic_node PRIVATE USE_GPIO=1)
  # U-2: MECHDOG_GPIOD_API (auto|v1|v2) + 实测库版本 双确认; 显式矛盾直接 FATAL, 不静默回落
  if(MECHDOG_GPIOD_API STREQUAL "v2")
    if(LIBGPIOD_VERSION VERSION_LESS 2)
      message(FATAL_ERROR "MECHDOG_GPIOD_API=v2 但 libgpiod=${LIBGPIOD_VERSION} (<2): v2 API 不存在; 请用 auto 或安装 libgpiod>=2")
    endif()
    set(_mdw_gpiod_v2 ON)
  elseif(MECHDOG_GPIOD_API STREQUAL "v1")
    if(NOT LIBGPIOD_VERSION VERSION_LESS 2)
      message(FATAL_ERROR "MECHDOG_GPIOD_API=v1 但 libgpiod=${LIBGPIOD_VERSION} (>=2): v1 API 已移除; 请用 auto 或 v2")
    endif()
    set(_mdw_gpiod_v2 OFF)
  elseif(MECHDOG_GPIOD_API STREQUAL "auto")
    if(LIBGPIOD_VERSION VERSION_GREATER_EQUAL 2)
      set(_mdw_gpiod_v2 ON)
    else()
      set(_mdw_gpiod_v2 OFF)
    endif()
  else()
    message(FATAL_ERROR "MECHDOG_GPIOD_API='${MECHDOG_GPIOD_API}' 非法 (仅 auto|v1|v2)")
  endif()
  if(_mdw_gpiod_v2)
    target_compile_definitions(ultrasonic_node PRIVATE LIBGPIOD_V2=1)
  endif()
  target_include_directories(ultrasonic_node PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
  target_link_libraries(ultrasonic_node ${LIBGPIOD_LIBRARIES})
  if(_mdw_gpiod_v2)
    message(STATUS "ultrasonic_node: libgpiod 真读启用 (库=${LIBGPIOD_VERSION}, API=v2, 开关=${MECHDOG_GPIOD_API})")
  else()
    message(STATUS "ultrasonic_node: libgpiod 真读启用 (库=${LIBGPIOD_VERSION}, API=v1, 开关=${MECHDOG_GPIOD_API})")
  endif()
else()
  message(STATUS "ultrasonic_node: 无 GPIO (模拟模式, 开关 USE_GPIO=ON 启用树莓派真读)")
endif()

install(TARGETS ultrasonic_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
