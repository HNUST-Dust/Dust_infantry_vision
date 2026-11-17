add_subdirectory(serial)

add_library(gimbal_io STATIC gimbal/gimbal.cpp)
target_link_libraries(gimbal_io PUBLIC tools_core serial PRIVATE tools yaml-cpp)

if(build_camera)
  add_library(camera_interface INTERFACE)
  target_link_libraries(camera_interface INTERFACE dust::include opencv_core)
  add_library(camera_io STATIC camera.cpp)
  target_link_libraries(camera_io PUBLIC camera_interface PRIVATE tools yaml-cpp)

  if(ENABLE_HIKROBOT OR ENABLE_MINDVISION)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
      set(DUST_SDK_ARCH amd64)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
      set(DUST_SDK_ARCH arm64)
    else()
      message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
    endif()
  endif()

  if(ENABLE_HIKROBOT)
    set(HIKROBOT_MVS_ROOT "/opt/MVS" CACHE PATH "HikRobot MVS SDK installation root")
    set(HIKROBOT_MVS_LIBRARY_DIR "${HIKROBOT_MVS_ROOT}/lib/64")
    set(HIKROBOT_MVS_USB_LIBRARY "${HIKROBOT_MVS_LIBRARY_DIR}/libusb-1.0.so.0")

    if(NOT EXISTS "${HIKROBOT_MVS_USB_LIBRARY}")
      message(FATAL_ERROR "MVS libusb not found: ${HIKROBOT_MVS_USB_LIBRARY}")
    endif()
    add_library(MVS::usb SHARED IMPORTED)
    set_target_properties(MVS::usb PROPERTIES IMPORTED_LOCATION "${HIKROBOT_MVS_USB_LIBRARY}")

    set(HIKROBOT_BUNDLED_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/${DUST_SDK_ARCH}")
    set(HIKROBOT_MVS_LIBRARY "${HIKROBOT_MVS_LIBRARY_DIR}/libMvCameraControl.so")
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

    add_library(hikrobot_camera STATIC hikrobot/hikrobot.cpp)
    target_link_libraries(hikrobot_camera
      PUBLIC camera_interface MvCameraControl tools_core PRIVATE tools MVS::usb)
    target_link_libraries(camera_io PRIVATE hikrobot_camera)
    target_compile_definitions(camera_io PRIVATE DUST_ENABLE_HIKROBOT)
  endif()

  if(ENABLE_MINDVISION)
    add_library(MVSDK SHARED IMPORTED)
    set_target_properties(MVSDK PROPERTIES
      IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/mindvision/lib/${DUST_SDK_ARCH}/libMVSDK.so"
      INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/mindvision/include"
    )
    if(ENABLE_HIKROBOT)
      # Match the USB runtime used by MVS when both drivers are enabled.
      set(mindvision_usb MVS::usb)
    else()
      find_library(SYSTEM_LIBUSB NAMES usb-1.0)
      if(NOT SYSTEM_LIBUSB)
        message(FATAL_ERROR "MindVision requires system libusb when HikRobot is disabled")
      endif()
      set(mindvision_usb ${SYSTEM_LIBUSB})
    endif()
    add_library(mindvision_camera STATIC mindvision/mindvision.cpp)
    target_link_libraries(mindvision_camera
      PUBLIC camera_interface MVSDK tools_core PRIVATE tools ${mindvision_usb})
    target_link_libraries(camera_io PRIVATE mindvision_camera)
    target_compile_definitions(camera_io PRIVATE DUST_ENABLE_MINDVISION)
  endif()

  add_library(usb_camera STATIC usbcamera/usbcamera.cpp)
  target_link_libraries(usb_camera PUBLIC camera_interface opencv_videoio PRIVATE tools)

  if(BUILD_HARDWARE_TOOLS)
    add_library(dm_imu_io STATIC dm_imu/dm_imu.cpp)
    target_link_libraries(dm_imu_io PUBLIC tools_core serial PRIVATE tools)
  endif()
endif()

if(ENABLE_SIMULATOR AND BUILD_APPLICATIONS)
  add_library(talos_io STATIC talos/talos.cpp)
  target_link_libraries(talos_io PUBLIC dust::include tools_core opencv_core talos_ipc
    PRIVATE tools)
  target_compile_definitions(talos_io PUBLIC DUST_ENABLE_SIMULATOR)
  add_dependencies(talos_io talos_ipc_build)
endif()
