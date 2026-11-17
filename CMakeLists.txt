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
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}" CACHE STRING "C++ compiler launcher" FORCE)
  else()
    message(STATUS "ccache not found; building without a compiler cache")
    unset(CMAKE_CXX_COMPILER_LAUNCHER CACHE)
    unset(CMAKE_CXX_COMPILER_LAUNCHER)
  endif()
else()
  unset(CMAKE_CXX_COMPILER_LAUNCHER CACHE)
  unset(CMAKE_CXX_COMPILER_LAUNCHER)
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
  "Enable the native Foxglove WebSocket image and telemetry stream in infantry"
  OFF
)

message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

option(BUILD_APPLICATIONS "Build the infantry application" ON)
option(BUILD_CALIBRATION_TOOLS "Build the calibration programs" ON)
option(BUILD_HARDWARE_TOOLS "Build manual hardware diagnostics" OFF)
option(BUILD_PERFORMANCE_TOOLS "Build inference benchmarks" OFF)
include(CTest)
option(BUILD_OFFLINE_TESTS "Build offline algorithm regression tests" ${BUILD_APPLICATIONS})
option(ENABLE_HIKROBOT "Build the HikRobot driver" ON)
option(ENABLE_MINDVISION "Build the MindVision driver" ON)
option(ENABLE_TENSORRT "Build the TensorRT 10 inference backend" OFF)
option(ENABLE_SIMULATOR "Build the Talos simulator IO backend" OFF)

set(build_offline OFF)
if(BUILD_TESTING AND BUILD_OFFLINE_TESTS)
  set(build_offline ON)
endif()
set(build_inference OFF)
if(BUILD_APPLICATIONS OR BUILD_HARDWARE_TOOLS OR BUILD_PERFORMANCE_TOOLS)
  set(build_inference ON)
endif()
set(build_pipeline OFF)
if(BUILD_APPLICATIONS OR BUILD_HARDWARE_TOOLS OR build_offline)
  set(build_pipeline ON)
endif()
set(build_camera OFF)
if(BUILD_APPLICATIONS OR BUILD_CALIBRATION_TOOLS OR BUILD_HARDWARE_TOOLS)
  set(build_camera ON)
endif()
set(build_foxglove_calibration OFF)
if(BUILD_CALIBRATION_TOOLS AND ENABLE_FOXGLOVE_CALIBRATION)
  set(build_foxglove_calibration ON)
endif()
set(build_foxglove_vision OFF)
if(BUILD_APPLICATIONS AND ENABLE_FOXGLOVE_VISION)
  set(build_foxglove_vision ON)
endif()

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
if(build_camera OR build_pipeline OR build_inference)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui calib3d dnn)
  find_package(fmt REQUIRED)
  find_package(spdlog REQUIRED)
  find_package(yaml-cpp REQUIRED)
  find_package(nlohmann_json REQUIRED)
endif()

if(ENABLE_SIMULATOR AND BUILD_APPLICATIONS)
  set(TALOS_IPC_ROOT "${PROJECT_SOURCE_DIR}/../bevy_robomaster_simulator" CACHE PATH "Bevy Talos IPC crate root")
  foreach(TALOS_IPC_REQUIRED_FILE
      Cargo.toml Cargo.lock crates/talos-ipc/Cargo.toml crates/talos-ipc/include/talos_ipc.h)
    if(NOT EXISTS "${TALOS_IPC_ROOT}/${TALOS_IPC_REQUIRED_FILE}")
      message(FATAL_ERROR
        "Talos IPC source missing: ${TALOS_IPC_ROOT}/${TALOS_IPC_REQUIRED_FILE}; "
        "set TALOS_IPC_ROOT to the simulator checkout or use -DENABLE_SIMULATOR=OFF")
    endif()
  endforeach()
  find_program(CARGO_EXECUTABLE cargo)
  if(NOT CARGO_EXECUTABLE)
    message(FATAL_ERROR "ENABLE_SIMULATOR=ON requires cargo to build the Talos IPC C ABI")
  endif()
  set(TALOS_IPC_LIBRARY "${TALOS_IPC_ROOT}/target/release/libtalos_ipc.so")
  file(GLOB_RECURSE TALOS_IPC_SOURCES CONFIGURE_DEPENDS "${TALOS_IPC_ROOT}/crates/talos-ipc/src/*.rs")
  add_custom_command(
    OUTPUT "${TALOS_IPC_LIBRARY}"
    COMMAND "${CARGO_EXECUTABLE}" build --release --manifest-path "${TALOS_IPC_ROOT}/Cargo.toml" -p talos-ipc
    WORKING_DIRECTORY "${TALOS_IPC_ROOT}"
    DEPENDS "${TALOS_IPC_ROOT}/Cargo.toml" "${TALOS_IPC_ROOT}/Cargo.lock"
      "${TALOS_IPC_ROOT}/crates/talos-ipc/Cargo.toml" ${TALOS_IPC_SOURCES}
    VERBATIM
  )
  add_custom_target(talos_ipc_build DEPENDS "${TALOS_IPC_LIBRARY}")
  add_library(talos_ipc SHARED IMPORTED GLOBAL)
  set_target_properties(talos_ipc PROPERTIES
    IMPORTED_LOCATION "${TALOS_IPC_LIBRARY}"
    # Rust cdylibs have no SONAME; link by library name so RUNPATH resolves them.
    IMPORTED_NO_SONAME TRUE
    INTERFACE_INCLUDE_DIRECTORIES "${TALOS_IPC_ROOT}/crates/talos-ipc/include"
  )
endif()
add_library(dust_include INTERFACE)
target_include_directories(dust_include INTERFACE ${PROJECT_SOURCE_DIR})
add_library(dust::include ALIAS dust_include)

add_subdirectory(tools)
if(build_camera OR build_pipeline)
  add_subdirectory(io)
endif()
if(build_pipeline OR build_inference)
  add_subdirectory(tasks/auto_aim)
endif()

if(BUILD_APPLICATIONS OR build_offline)
  add_library(planner_diagnostics STATIC src/planner_diagnostics.cpp)
  target_link_libraries(planner_diagnostics PUBLIC aim_planner nlohmann_json::nlohmann_json
    PRIVATE tools)
endif()

if(BUILD_APPLICATIONS)
  add_library(infantry_runtime STATIC
    src/app_config.cpp src/auto_aim_runtime.cpp src/diagnostics/telemetry.cpp
    src/visualization/runtime_visualizer.cpp visualization/vision_overlay.cpp)
  target_link_libraries(infantry_runtime
    PUBLIC dust::include
    PRIVATE aim_detection aim_tracking planner_diagnostics camera_io gimbal_io tools
            opencv_highgui opencv_imgproc)
  if(ENABLE_SIMULATOR)
    target_link_libraries(infantry_runtime PRIVATE talos_io)
  endif()

  add_executable(infantry src/infantry.cpp)
  target_link_libraries(infantry PRIVATE infantry_runtime tools opencv_core)
endif()

if(BUILD_CALIBRATION_TOOLS)
  add_executable(capture calibration/capture.cpp)
  add_executable(calibrate_camera calibration/calibrate_camera.cpp)
  add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
  add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
  add_executable(split_video calibration/split_video.cpp)

  target_link_libraries(
    capture PRIVATE fmt::fmt nlohmann_json::nlohmann_json Eigen3::Eigen ${OpenCV_LIBS} tools camera_io gimbal_io
  )
  target_link_libraries(calibrate_camera PRIVATE fmt::fmt yaml-cpp ${OpenCV_LIBS} tools)
  target_link_libraries(
    calibrate_handeye PRIVATE fmt::fmt yaml-cpp Eigen3::Eigen ${OpenCV_LIBS} tools
  )
  target_link_libraries(
    calibrate_robotworld_handeye PRIVATE fmt::fmt yaml-cpp Eigen3::Eigen ${OpenCV_LIBS} tools
  )
  target_link_libraries(split_video PRIVATE fmt::fmt ${OpenCV_LIBS} tools)
endif()

if(build_foxglove_calibration OR build_foxglove_vision)
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

if(build_foxglove_calibration)
  # 本库的 cpp 用相对 include 包含自己的头，故不需要 dust::include。
  add_library(calibration_foxglove STATIC calibration/foxglove_calibration.cpp)
  target_link_libraries(
    calibration_foxglove
    PUBLIC ${OpenCV_LIBS} Eigen3::Eigen nlohmann_json::nlohmann_json dust_foxglove_cpp
  )

  target_compile_definitions(capture PRIVATE DUST_ENABLE_FOXGLOVE)
  target_link_libraries(capture PRIVATE calibration_foxglove)
endif()

if(build_foxglove_vision)
  add_library(visualization_foxglove STATIC visualization/foxglove_vision.cpp)
  target_link_libraries(visualization_foxglove
    PUBLIC dust::include opencv_core nlohmann_json::nlohmann_json
    PRIVATE opencv_imgcodecs opencv_imgproc dust_foxglove_cpp Threads::Threads
  )
  target_compile_definitions(infantry_runtime PUBLIC DUST_ENABLE_FOXGLOVE)
  target_link_libraries(infantry_runtime PRIVATE visualization_foxglove)
endif()

if(BUILD_TESTING OR BUILD_HARDWARE_TOOLS OR BUILD_PERFORMANCE_TOOLS)
  add_subdirectory(tests)
endif()
