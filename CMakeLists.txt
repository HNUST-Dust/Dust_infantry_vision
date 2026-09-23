cmake_minimum_required(VERSION 3.16.3)

project(dust-vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# CMAKE_EXPORT_COMPILE_COMMANDS 必须用普通变量赋值：本仓 build-ninja 的缓存中已存在同名
# 空值项（CMAKE_EXPORT_COMPILE_COMMANDS:BOOL=），而 set(... CACHE ...) 不会覆盖已存在的
# 缓存项，原写法因此静默失效、从未生成 compile_commands.json。普通变量会遮蔽缓存项，
# 保证 clangd/IDE 能拿到编译数据库。如需关闭：注释掉本行。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(ENABLE_CCACHE "Use ccache when it is available" ON)
if(ENABLE_CCACHE)
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}" CACHE STRING "C++ compiler launcher")
  else()
    message(STATUS "ccache not found; building without a compiler cache")
  endif()
endif()

option(ENABLE_WARNINGS "Enable extra compiler warnings" OFF)
if(ENABLE_WARNINGS)
  # 有意保留为目录级设置，不做成 INTERFACE 目标：警告是全项目策略而非 target 间依赖关系，
  # 且改成 INTERFACE 会让 vendored 的 io/serial 丢掉这些选项，属无关的编译命令变更。
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(
  ENABLE_FOXGLOVE_CALIBRATION
  "Enable the native Foxglove WebSocket preview and services in capture"
  OFF
)
option(
  ENABLE_FOXGLOVE_VISION
  "Enable the native Foxglove WebSocket image and telemetry stream in infantry / infantry_debug"
  OFF
)

message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(
  OpenCV REQUIRED
  COMPONENTS core imgproc imgcodecs videoio highgui calib3d dnn features2d flann
)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/" CACHE PATH "OpenVINO CMake package directory")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# 仓库根即 include 根：全仓 198 处 #include "tools/logger.hpp" 这类形式依赖它
# （tools 121 / tasks 46 / io 23 / visualization 6 / calibration 2）。
# 用 INTERFACE 目标表达，由各 target 显式链接，取代原来的全局
# include_directories(${PROJECT_SOURCE_DIR}) / (${EIGEN3_INCLUDE_DIR}) / (${OpenCV_INCLUDE_DIRS})。
add_library(dust_include INTERFACE)
target_include_directories(dust_include INTERFACE ${PROJECT_SOURCE_DIR})
add_library(dust::include ALIAS dust_include)

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)

###################src###################
add_executable(infantry_debug src/infantry_debug.cpp visualization/vision_overlay.cpp)
add_executable(infantry src/infantry.cpp visualization/vision_overlay.cpp)

# 只声明各自直接使用的库：OpenCV / Eigen / yaml-cpp / nlohmann / spdlog / OpenVINO /
# Threads 与仓库根 include 由 tools / io / auto_aim 的 PUBLIC（及 PRIVATE 的 LINK_ONLY）
# 需求传播而来，不再逐个手抄。
target_link_libraries(infantry_debug PRIVATE fmt::fmt auto_aim tools io)
target_link_libraries(infantry PRIVATE fmt::fmt auto_aim tools io)

#################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture PRIVATE fmt::fmt tools io)
target_link_libraries(calibrate_camera PRIVATE fmt::fmt tools)
target_link_libraries(calibrate_handeye PRIVATE fmt::fmt tools)
target_link_libraries(calibrate_robotworld_handeye PRIVATE fmt::fmt tools)
target_link_libraries(split_video PRIVATE fmt::fmt tools)

if(ENABLE_FOXGLOVE_CALIBRATION OR ENABLE_FOXGLOVE_VISION)
  # 两个功能共用同一份 SDK：只下载、只 find_package、只定义 dust_foxglove_cpp 一次。
  # 离线环境可用 -DFETCHCONTENT_SOURCE_DIR_FOXGLOVE_SDK_DIST=<已解压目录> 跳过下载。
  include(FetchContent)
  FetchContent_Declare(
    foxglove_sdk_dist
    URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.27.0/foxglove-v0.27.0-cpp-x86_64-unknown-linux-gnu.zip"
    URL_HASH SHA256=4790dad26adaced1a5f81530ed406bab7b60aa202a868eb78cfc04976b9ce6d7
  )
  FetchContent_GetProperties(foxglove_sdk_dist)
  if(NOT foxglove_sdk_dist_POPULATED)
    FetchContent_Populate(foxglove_sdk_dist)
  endif()

  find_package(
    foxglove-sdk CONFIG REQUIRED
    HINTS "${foxglove_sdk_dist_SOURCE_DIR}"
  )
  foxglove_sdk_add_cpp_library(dust_foxglove_cpp TYPE STATIC REMOTE_ACCESS OFF)
endif()

if(ENABLE_FOXGLOVE_CALIBRATION)
  # 本库的 cpp 用相对 include 包含自己的头，故不需要 dust::include。
  add_library(calibration_foxglove STATIC calibration/foxglove_calibration.cpp)
  target_link_libraries(
    calibration_foxglove
    PUBLIC ${OpenCV_LIBS} Eigen3::Eigen nlohmann_json::nlohmann_json dust_foxglove_cpp
  )

  target_compile_definitions(capture PRIVATE DUST_ENABLE_FOXGLOVE)
  target_link_libraries(capture PRIVATE calibration_foxglove)
endif()

if(ENABLE_FOXGLOVE_VISION)
  # 传输库只依赖 OpenCV / nlohmann_json / Foxglove：auto_aim 与 tools 都是 OBJECT 库，
  # 把它们链进静态库会复制一份目标文件，与可执行程序自身链接的那份冲突。
  add_library(visualization_foxglove STATIC visualization/foxglove_vision.cpp)
  target_link_libraries(
    visualization_foxglove
    PUBLIC ${OpenCV_LIBS} nlohmann_json::nlohmann_json dust_foxglove_cpp
  )
  # 本库的 cpp 用根相对路径包含自己的头（#include "visualization/foxglove_vision.hpp"）。
  target_link_libraries(visualization_foxglove PRIVATE dust::include)

  # 叠加层已在上面直接编进两个可执行程序（不依赖 Foxglove），这里只补上编译定义与传输库。
  foreach(vision_target infantry infantry_debug)
    target_compile_definitions(${vision_target} PRIVATE DUST_ENABLE_FOXGLOVE)
    target_link_libraries(${vision_target} PRIVATE visualization_foxglove)
  endforeach()
endif()

#################tests##################
# 测试目标全部在 tests/CMakeLists.txt，便于本文件只描述"产品"目标。
enable_testing()
add_subdirectory(tests)
