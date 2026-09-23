# 测试目标集中在此。链接只声明各自"直接"使用的库：
# OpenCV / Eigen / yaml-cpp / nlohmann / spdlog / OpenVINO / Threads 与仓库根 include
# 由 tools / io / auto_aim 的 PUBLIC（及 PRIVATE 的 LINK_ONLY）需求传播而来，不再逐个手抄。
# 少数不链接这三个库的测试（ordered_delivery / quaternion_buffer / 两个 foxglove 协议测试）
# 需要自己显式声明 dust::include。

# 测试二进制仍输出到构建目录根，而不是随本文件所在子目录变成 build-ninja/tests/<name>：
# readme.md 与 .codex/skills/ 都按 build-ninja/<name> 这种路径调用它们。
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#################tests##################
add_executable(auto_aim_test auto_aim_test.cpp)
add_executable(camera_test camera_test.cpp)
add_executable(camera_thread_test camera_thread_test.cpp)
add_executable(fire_test fire_test.cpp)
add_executable(detector_video_test detector_video_test.cpp)
add_executable(gimbal_response_test gimbal_response_test.cpp)
add_executable(handeye_test handeye_test.cpp)
add_executable(minimum_vision_system minimum_vision_system.cpp)
add_executable(gimbal_test gimbal_test.cpp)
add_executable(async_detector_test async_detector_test.cpp)
add_executable(dynamic_roi_test dynamic_roi_test.cpp)
add_executable(detector_roi_test detector_roi_test.cpp)
add_executable(openvino_benchmark_test openvino_benchmark_test.cpp)

target_link_libraries(auto_aim_test PRIVATE fmt::fmt tools io auto_aim)
target_link_libraries(camera_test PRIVATE fmt::fmt tools io)
target_link_libraries(camera_thread_test PRIVATE fmt::fmt auto_aim tools io)
target_link_libraries(fire_test PRIVATE fmt::fmt tools io)
target_link_libraries(detector_video_test PRIVATE fmt::fmt tools auto_aim)
target_link_libraries(gimbal_response_test PRIVATE fmt::fmt tools io)
target_link_libraries(handeye_test PRIVATE fmt::fmt tools io auto_aim)
target_link_libraries(minimum_vision_system PRIVATE fmt::fmt tools io auto_aim)
target_link_libraries(gimbal_test PRIVATE fmt::fmt auto_aim tools io)
target_link_libraries(async_detector_test PRIVATE fmt::fmt auto_aim tools io)
target_link_libraries(dynamic_roi_test PRIVATE fmt::fmt auto_aim tools io)
target_link_libraries(detector_roi_test PRIVATE fmt::fmt auto_aim tools io)
target_link_libraries(openvino_benchmark_test PRIVATE fmt::fmt auto_aim tools io)

# ordered_delivery.hpp 是纯头（只用 std），但它用 #include "tools/ordered_delivery.hpp"
# 根相对路径，故仍需 dust::include；其源码用 std::async/std::future/std::mutex，补上 Threads。
add_executable(ordered_delivery_test ordered_delivery_test.cpp)
target_link_libraries(ordered_delivery_test PRIVATE dust::include Threads::Threads)

add_executable(quaternion_buffer_test quaternion_buffer_test.cpp)
target_link_libraries(quaternion_buffer_test PRIVATE dust::include Eigen3::Eigen Threads::Threads)
add_test(NAME quaternion_buffer COMMAND quaternion_buffer_test)

if(ENABLE_FOXGLOVE_CALIBRATION OR ENABLE_FOXGLOVE_VISION)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
endif()

if(ENABLE_FOXGLOVE_CALIBRATION)
  add_executable(foxglove_calibration_test foxglove_calibration_test.cpp)
  target_link_libraries(foxglove_calibration_test PRIVATE dust::include calibration_foxglove)

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
  add_executable(foxglove_vision_test foxglove_vision_test.cpp)
  target_link_libraries(foxglove_vision_test PRIVATE dust::include visualization_foxglove)

  add_test(
    NAME foxglove_vision_protocol
    COMMAND
      ${Python3_EXECUTABLE}
      ${PROJECT_SOURCE_DIR}/tests/foxglove_vision_protocol_test.py
      $<TARGET_FILE:foxglove_vision_test>
  )
  set_tests_properties(foxglove_vision_protocol PROPERTIES TIMEOUT 60)
endif()
