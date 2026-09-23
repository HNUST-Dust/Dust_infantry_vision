cmake_minimum_required(VERSION 3.16.3)

project(dust-vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile_commands.json")

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

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/" CACHE PATH "OpenVINO CMake package directory")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)

###################src###################
add_executable(infantry_debug src/infantry_debug.cpp visualization/vision_overlay.cpp)
add_executable(infantry src/infantry.cpp visualization/vision_overlay.cpp)

target_link_libraries(infantry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
target_link_libraries(infantry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

#################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

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
  add_library(calibration_foxglove STATIC calibration/foxglove_calibration.cpp)
  target_link_libraries(
    calibration_foxglove
    PUBLIC ${OpenCV_LIBS} Eigen3::Eigen nlohmann_json::nlohmann_json dust_foxglove_cpp
  )

  target_compile_definitions(capture PRIVATE DUST_ENABLE_FOXGLOVE)
  target_link_libraries(capture calibration_foxglove)

  add_executable(foxglove_calibration_test tests/foxglove_calibration_test.cpp)
  target_link_libraries(foxglove_calibration_test calibration_foxglove)
endif()

if(ENABLE_FOXGLOVE_VISION)
  # 传输库只依赖 OpenCV / nlohmann_json / Foxglove：auto_aim 与 tools 都是 OBJECT 库，
  # 把它们链进静态库会复制一份目标文件，与可执行程序自身链接的那份冲突。
  add_library(visualization_foxglove STATIC visualization/foxglove_vision.cpp)
  target_link_libraries(
    visualization_foxglove
    PUBLIC ${OpenCV_LIBS} nlohmann_json::nlohmann_json dust_foxglove_cpp
  )

  # 叠加层已在上面直接编进两个可执行程序（不依赖 Foxglove），这里只补上编译定义与传输库。
  foreach(vision_target infantry infantry_debug)
    target_compile_definitions(${vision_target} PRIVATE DUST_ENABLE_FOXGLOVE)
    target_link_libraries(${vision_target} visualization_foxglove)
  endforeach()

  add_executable(foxglove_vision_test tests/foxglove_vision_test.cpp)
  target_link_libraries(foxglove_vision_test visualization_foxglove)
endif()

#################tests##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)

target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

enable_testing()

add_executable(async_detector_test tests/async_detector_test.cpp)
target_link_libraries(async_detector_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(ordered_delivery_test tests/ordered_delivery_test.cpp)

find_package(Threads REQUIRED)
add_executable(quaternion_buffer_test tests/quaternion_buffer_test.cpp)
target_link_libraries(quaternion_buffer_test Eigen3::Eigen Threads::Threads)
add_test(NAME quaternion_buffer COMMAND quaternion_buffer_test)

add_executable(dynamic_roi_test tests/dynamic_roi_test.cpp)
target_link_libraries(dynamic_roi_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)

add_executable(detector_roi_test tests/detector_roi_test.cpp)
target_link_libraries(detector_roi_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(openvino_benchmark_test tests/openvino_benchmark_test.cpp)
target_link_libraries(openvino_benchmark_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

if(ENABLE_FOXGLOVE_CALIBRATION OR ENABLE_FOXGLOVE_VISION)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
endif()

if(ENABLE_FOXGLOVE_CALIBRATION)
  add_test(
    NAME foxglove_calibration_protocol
    COMMAND
      ${Python3_EXECUTABLE}
      ${PROJECT_SOURCE_DIR}/tests/foxglove_calibration_protocol_test.py
      $<TARGET_FILE:foxglove_calibration_test>
  )
  set_tests_properties(foxglove_calibration_protocol PROPERTIES TIMEOUT 20)
endif()

if(ENABLE_FOXGLOVE_VISION)
  add_test(
    NAME foxglove_vision_protocol
    COMMAND
      ${Python3_EXECUTABLE}
      ${PROJECT_SOURCE_DIR}/tests/foxglove_vision_protocol_test.py
      $<TARGET_FILE:foxglove_vision_test>
  )
  set_tests_properties(foxglove_vision_protocol PROPERTIES TIMEOUT 60)
endif()
