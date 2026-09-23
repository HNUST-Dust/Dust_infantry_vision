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

target_link_libraries(io PUBLIC MVSDK usb-1.0 yaml-cpp serial)

target_precompile_headers(io PRIVATE
  <Eigen/Geometry>
  <opencv2/opencv.hpp>
  <spdlog/spdlog.h>
)
