cmake_minimum_required(VERSION 3.22)
project(control_demo_driver LANGUAGES CXX)

# Zsibot SDK 头文件和预编译静态库使用 C++20。
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# ============================================================
# Zsibot SDK（直接链接预编译静态库，不 add_subdirectory）
# ============================================================
get_filename_component(_zsibot_default_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../zsibot-sdk/genisom_L1_sdk" ABSOLUTE)
set(ZSIBOT_SDK_DIR "${_zsibot_default_dir}"
  CACHE PATH "genisom_L1_sdk 根目录（包含 include/ 和 lib/）")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(_zsibot_default_arch "aarch64")
else()
  set(_zsibot_default_arch "x86_64")
endif()
set(ZSIBOT_ARCH "${_zsibot_default_arch}"
  CACHE STRING "Zsibot 预编译库架构（x86_64 或 aarch64）")
set_property(CACHE ZSIBOT_ARCH PROPERTY STRINGS x86_64 aarch64)

set(_zsibot_include_dir "${ZSIBOT_SDK_DIR}/include")
set(_zsibot_lib "${ZSIBOT_SDK_DIR}/lib/${ZSIBOT_ARCH}/libzsibot.a")

if(NOT EXISTS "${_zsibot_lib}")
  message(FATAL_ERROR
    "未找到 Zsibot 静态库: ${_zsibot_lib}\n"
    "请检查 ZSIBOT_SDK_DIR（当前为 '${ZSIBOT_SDK_DIR}'）和 ZSIBOT_ARCH"
    "（当前为 '${ZSIBOT_ARCH}'）。可通过以下方式覆盖：\n"
    "  colcon build --packages-select control_demo_driver --cmake-args "
    "-DZSIBOT_SDK_DIR=/abs/path/to/genisom_L1_sdk -DZSIBOT_ARCH=x86_64")
endif()
if(NOT EXISTS "${_zsibot_include_dir}/zsibot_sdk/zsibot_api.h")
  message(FATAL_ERROR
    "未找到 Zsibot 头文件: ${_zsibot_include_dir}/zsibot_sdk/zsibot_api.h")
endif()

add_library(zsibot_sdk STATIC IMPORTED)
set_target_properties(zsibot_sdk PROPERTIES
  IMPORTED_LOCATION "${_zsibot_lib}"
  INTERFACE_INCLUDE_DIRECTORIES "${_zsibot_include_dir}"
  INTERFACE_LINK_LIBRARIES "Threads::Threads")

message(STATUS "control_demo_driver: zsibot_sdk lib = ${_zsibot_lib}")
message(STATUS "control_demo_driver: zsibot_sdk include = ${_zsibot_include_dir}")

# ============================================================
# 可执行文件
# ============================================================
add_executable(control_demo_driver
  src/control_node.cpp
  src/executor_holder.cpp
  src/status_builder.cpp)

target_compile_features(control_demo_driver PRIVATE cxx_std_20)
target_include_directories(control_demo_driver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(control_demo_driver rclcpp sensor_msgs std_msgs)
target_link_libraries(control_demo_driver zsibot_sdk)
target_compile_options(control_demo_driver PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS control_demo_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
