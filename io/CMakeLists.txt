find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
)

# hikrobot
set(HIKROBOT_MVS_ROOT "/opt/MVS" CACHE PATH "HikRobot MVS SDK installation root")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(HIKROBOT_BUNDLED_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/amd64")
  set(HIKROBOT_MVS_LIBRARY_DIR "${HIKROBOT_MVS_ROOT}/lib/64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(HIKROBOT_BUNDLED_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64")
  set(HIKROBOT_MVS_LIBRARY_DIR "${HIKROBOT_MVS_ROOT}/lib/64")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
endif()

set(HIKROBOT_MVS_LIBRARY "${HIKROBOT_MVS_LIBRARY_DIR}/libMvCameraControl.so")

if(EXISTS "${HIKROBOT_MVS_ROOT}/include/MvCameraControl.h" AND
   EXISTS "${HIKROBOT_MVS_LIBRARY}")
  message(STATUS "Using HikRobot MVS SDK from ${HIKROBOT_MVS_ROOT}")
  target_include_directories(io PUBLIC "${HIKROBOT_MVS_ROOT}/include")
  target_link_libraries(io PUBLIC "${HIKROBOT_MVS_LIBRARY}")
else()
  message(WARNING
    "Complete HikRobot MVS SDK not found under ${HIKROBOT_MVS_ROOT}; "
    "falling back to the bundled camera control library. USB cameras may fail "
    "to enumerate without a matching MVS runtime."
  )
  target_include_directories(io PUBLIC hikrobot/include)
  target_link_directories(io PUBLIC "${HIKROBOT_BUNDLED_LIBRARY_DIR}")
  target_link_libraries(io PUBLIC MvCameraControl)
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
endif()

# io 的公共头暴露 OpenCV（camera.hpp / hikrobot.hpp / mindvision.hpp / usbcamera.hpp 含
# <opencv2/opencv.hpp>）、Eigen（dm_imu/gimbal 含 <Eigen/Geometry>）与 serial
# （dm_imu.hpp / gimbal.hpp 含 <serial/serial.h>）；gimbal.hpp 等用根相对路径包含
# tools/quaternion_buffer.hpp，故 dust::include 也必需。
target_link_libraries(io
  PUBLIC
    dust::include
    Eigen3::Eigen
    MVSDK
    usb-1.0
    yaml-cpp
    serial
    ${OpenCV_LIBS}
  PRIVATE
    # io 的源文件与 PCH 都经 tools/logger.hpp 包含 spdlog，而 io 并不链接 tools，
    # 必须自己声明一次：spdlog 是 header-only 内联（SPDLOG_INLINE inline），
    # 两个翻译单元若拿到不同的 SPDLOG_FMT_EXTERNAL 定义即构成 ODR 违规。
    spdlog::spdlog_header_only
    Threads::Threads
)

target_precompile_headers(io PRIVATE
  <Eigen/Geometry>
  <opencv2/opencv.hpp>
  <spdlog/spdlog.h>
)
