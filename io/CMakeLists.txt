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


# 厂商 SDK 一律以 imported target 表达，取代 target_link_directories + 裸库名：后者会把厂商
# SDK 的 -L 路径 PUBLIC 传播给全部下游可执行程序，且库名只能靠编译器的默认搜索路径解析，
# 失败时报错也不指明来源。
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(DUST_SDK_ARCH amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(DUST_SDK_ARCH arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
endif()

# MindVision：仓库内自带的 libMVSDK.so（裸实体文件，无版本化符号链接，但带 SONAME
# libMVSDK.so，故按绝对路径链接后 DT_NEEDED 仍是 libMVSDK.so）。
# mindvision.hpp 用 #include "CameraApi.h"，故 include 目录必须传播给使用者。
add_library(MVSDK SHARED IMPORTED)
set_target_properties(MVSDK PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/mindvision/lib/${DUST_SDK_ARCH}/libMVSDK.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/mindvision/include"
)

# libusb 固定使用 MVS SDK 自带版本，避免 MVS 相机库与系统 libusb 版本不一致。
# mindvision.cpp 用 <libusb-1.0/libusb.h>，该头由默认的 /usr/include 解析，无需额外 include 目录。

# hikrobot：优先用 HIKROBOT_MVS_ROOT 下的完整 SDK，缺失时回退到仓库内 bundled 库。
# 两条分支共用一个 imported target，只替换 IMPORTED_LOCATION 与 include 目录。
set(HIKROBOT_MVS_ROOT "/opt/MVS" CACHE PATH "HikRobot MVS SDK installation root")
set(HIKROBOT_BUNDLED_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/${DUST_SDK_ARCH}")
set(HIKROBOT_MVS_LIBRARY_DIR "${HIKROBOT_MVS_ROOT}/lib/64")
set(HIKROBOT_MVS_LIBRARY "${HIKROBOT_MVS_LIBRARY_DIR}/libMvCameraControl.so")
set(HIKROBOT_MVS_USB_LIBRARY "${HIKROBOT_MVS_LIBRARY_DIR}/libusb-1.0.so.0")

if(NOT EXISTS "${HIKROBOT_MVS_USB_LIBRARY}")
  message(FATAL_ERROR "MVS libusb not found: ${HIKROBOT_MVS_USB_LIBRARY}")
endif()
add_library(MVS::usb SHARED IMPORTED)
set_target_properties(MVS::usb PROPERTIES IMPORTED_LOCATION "${HIKROBOT_MVS_USB_LIBRARY}")

add_library(MvCameraControl SHARED IMPORTED)
if(EXISTS "${HIKROBOT_MVS_ROOT}/include/MvCameraControl.h" AND
   EXISTS "${HIKROBOT_MVS_LIBRARY}")
  message(STATUS "Using HikRobot MVS SDK from ${HIKROBOT_MVS_ROOT}")
  set_target_properties(MvCameraControl PROPERTIES
    IMPORTED_LOCATION "${HIKROBOT_MVS_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${HIKROBOT_MVS_ROOT}/include")
else()
  message(WARNING
    "Complete HikRobot MVS SDK not found under ${HIKROBOT_MVS_ROOT}; "
    "falling back to the bundled camera control library. USB cameras may fail "
    "to enumerate without a matching MVS runtime."
  )
  set_target_properties(MvCameraControl PROPERTIES
    IMPORTED_LOCATION "${HIKROBOT_BUNDLED_LIBRARY_DIR}/libMvCameraControl.so"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/include")
endif()

# io 的公共头暴露 OpenCV（camera.hpp / hikrobot.hpp / mindvision.hpp / usbcamera.hpp 含
# <opencv2/opencv.hpp>）、Eigen（dm_imu/gimbal 含 <Eigen/Geometry>）与 serial
# （dm_imu.hpp / gimbal.hpp 含 <serial/serial.h>）；gimbal.hpp 等用根相对路径包含
# tools/quaternion_buffer.hpp，故 dust::include 也必需。
target_link_libraries(io
  PUBLIC
    dust::include
    Eigen3::Eigen
    MvCameraControl
    MVSDK
    MVS::usb
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
