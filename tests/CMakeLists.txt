# Keep the documented binary paths under the build directory root.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

function(add_regression test_name target source category)
  add_executable(${target} ${source})
  # Some regressions perform operations inside assert(). Keep them enabled in Release.
  target_compile_options(${target} PRIVATE -UNDEBUG)
  add_test(NAME ${test_name} COMMAND ${target} ${ARGN})
  set_tests_properties(${test_name} PROPERTIES LABELS ${category} TIMEOUT 30)
endfunction()

if(BUILD_TESTING)
  add_regression(ordered_delivery ordered_delivery_test tools/ordered_delivery_test.cpp unit)
  target_link_libraries(ordered_delivery_test PRIVATE tools_core)
  add_regression(quaternion_buffer quaternion_buffer_test tools/quaternion_buffer_test.cpp unit)
  target_link_libraries(quaternion_buffer_test PRIVATE tools_core)

  if(build_offline)
    add_regression(planner_reference planner_reference_test planner/planner_reference_test.cpp offline)
    target_link_libraries(planner_reference_test PRIVATE tools_core)

    add_regression(planner_feedback planner_feedback_test planner/planner_feedback_test.cpp offline)
    target_link_libraries(planner_feedback_test PRIVATE aim_planner gimbal_io tools)

    add_regression(dynamic_roi dynamic_roi_test detection/dynamic_roi_test.cpp offline)
    target_link_libraries(dynamic_roi_test PRIVATE aim_roi)

    foreach(test planner_replay mpc_cost planner_diagnostics planner_preaim planner_axis_strategy)
      set(arguments)
      if(test STREQUAL "planner_replay")
        set(arguments --self-test)
      elseif(test STREQUAL "planner_preaim" OR test STREQUAL "planner_axis_strategy")
        set(arguments ${PROJECT_SOURCE_DIR}/configs/standard3.yaml)
      endif()
      add_regression(${test} ${test}_test planner/${test}_test.cpp offline ${arguments})
      target_link_libraries(${test}_test PRIVATE planner_diagnostics tools)
    endforeach()
  endif()

  if(BUILD_APPLICATIONS)
    add_regression(runtime_data runtime_data_test integration/runtime_data_test.cpp offline
      ${PROJECT_SOURCE_DIR}/configs/standard3.yaml ${CMAKE_CURRENT_BINARY_DIR}/invalid-runtime.yaml)
    target_link_libraries(runtime_data_test PRIVATE infantry_runtime aim_detection
      planner_diagnostics gimbal_io tools)
    set_tests_properties(runtime_data PROPERTIES WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
  endif()

  if(BUILD_APPLICATIONS AND NOT ENABLE_SIMULATOR)
    add_test(NAME simulator_disabled COMMAND ${CMAKE_COMMAND}
      -DINFANTRY=$<TARGET_FILE:infantry>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/integration/simulator_disabled_test.cmake)
    set_tests_properties(simulator_disabled PROPERTIES LABELS offline TIMEOUT 10)
  endif()

  if(build_foxglove_calibration OR build_foxglove_vision)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
  endif()

  if(build_foxglove_calibration)
    add_executable(foxglove_calibration_test calibration/foxglove_calibration_test.cpp)
    target_link_libraries(
      foxglove_calibration_test PRIVATE dust::include Eigen3::Eigen ${OpenCV_LIBS} calibration_foxglove
    )

    add_test(
      NAME foxglove_calibration_protocol
      COMMAND
        ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/calibration/foxglove_calibration_protocol_test.py
        $<TARGET_FILE:foxglove_calibration_test>
    )
    set_tests_properties(foxglove_calibration_protocol PROPERTIES LABELS offline TIMEOUT 20)
  endif()

  if(build_foxglove_vision)
    add_executable(foxglove_vision_test visualization/foxglove_vision_test.cpp)
    target_link_libraries(
      foxglove_vision_test PRIVATE dust::include ${OpenCV_LIBS} visualization_foxglove
    )

    add_test(
      NAME foxglove_vision_protocol
      COMMAND
        ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/visualization/foxglove_vision_protocol_test.py
        $<TARGET_FILE:foxglove_vision_test>
    )
    set_tests_properties(foxglove_vision_protocol PROPERTIES LABELS offline TIMEOUT 60)
  endif()

endif()

# Offline programs with user-supplied media or interactive windows.
if(BUILD_TESTING AND build_inference AND build_pipeline)
  add_executable(auto_aim_test integration/auto_aim_test.cpp)
  target_link_libraries(auto_aim_test PRIVATE aim_detection aim_tracking aim_planner tools
    fmt::fmt opencv_calib3d opencv_highgui)
  add_executable(detector_video_test detection/detector_video_test.cpp)
  target_link_libraries(detector_video_test PRIVATE aim_detection tools fmt::fmt opencv_highgui)
  add_executable(async_detector_test detection/async_detector_test.cpp)
  target_compile_options(async_detector_test PRIVATE -UNDEBUG)
  target_link_libraries(async_detector_test PRIVATE aim_detection tools)
  if(ENABLE_TENSORRT)
    add_executable(tensorrt_detector_test detection/tensorrt_detector_test.cpp)
    target_link_libraries(tensorrt_detector_test PRIVATE aim_detection aim_geometry tools)
  endif()
endif()

if(build_offline)
  # Requires the classifier model from the YAML, relative to the repo working directory.
  add_regression(detector_roi detector_roi_test detection/detector_roi_test.cpp offline
    ${PROJECT_SOURCE_DIR}/configs/standard3.yaml)
  target_link_libraries(detector_roi_test PRIVATE aim_traditional_detector tools opencv_imgproc)
  set_tests_properties(detector_roi PROPERTIES WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
endif()

# Manual hardware diagnostics. These must be started explicitly.
if(BUILD_HARDWARE_TOOLS)
  add_executable(camera_test camera/camera_test.cpp)
  target_link_libraries(camera_test PRIVATE camera_io tools opencv_highgui)
  add_executable(camera_thread_test camera/camera_thread_test.cpp)
  target_link_libraries(camera_thread_test PRIVATE camera_io aim_tracking aim_detection tools
    opencv_highgui)
  add_executable(fire_test gimbal/fire_test.cpp)
  target_link_libraries(fire_test PRIVATE gimbal_io tools fmt::fmt opencv_highgui)
  add_executable(gimbal_response_test gimbal/gimbal_response_test.cpp)
  target_link_libraries(gimbal_response_test PRIVATE gimbal_io tools opencv_highgui)
  add_executable(handeye_test calibration/handeye_test.cpp)
  target_link_libraries(handeye_test PRIVATE camera_io gimbal_io aim_geometry tools
    opencv_calib3d opencv_highgui)
  add_executable(minimum_vision_system integration/minimum_vision_system.cpp)
  target_link_libraries(minimum_vision_system PRIVATE camera_io dm_imu_io aim_detection aim_tracking
    aim_planner tools fmt::fmt opencv_highgui)
  add_executable(gimbal_test gimbal/gimbal_test.cpp)
  target_link_libraries(gimbal_test PRIVATE gimbal_io tools opencv_highgui)
endif()

if(BUILD_PERFORMANCE_TOOLS)
  add_executable(openvino_benchmark_test performance/openvino_benchmark_test.cpp)
  target_link_libraries(openvino_benchmark_test PRIVATE aim_detection tools)
endif()
