add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
    model/armor.cpp
    detection/classifier.cpp
    detection/detector.cpp
    detection/dynamic_roi.cpp
    geometry/solver.cpp
    tracking/target.cpp
    tracking/tracker.cpp
    detection/net_detector.cpp
    detection/yolo.cpp
    detection/yolov5.cpp
    detection/mt_detector.cpp
    planner/planner.cpp
    planner/mpc_axis.cpp
)

file(GLOB mpc_build_sources CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/planner/mpc_axis.*
  ${CMAKE_CURRENT_SOURCE_DIR}/planner/tinympc/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/planner/tinympc/*.hpp)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${mpc_build_sources})
set(mpc_build_hashes "${CMAKE_CXX_COMPILER_ID};${CMAKE_CXX_COMPILER_VERSION};${CMAKE_BUILD_TYPE};${CMAKE_CXX_FLAGS};${CMAKE_CXX_FLAGS_RELEASE};${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
foreach(source IN LISTS mpc_build_sources)
  file(SHA256 ${source} source_hash)
  string(APPEND mpc_build_hashes ${source_hash})
endforeach()
string(SHA256 mpc_build_id "${mpc_build_hashes}")
target_compile_definitions(auto_aim PUBLIC DUST_MPC_BUILD_ID="${mpc_build_id}")

# Target-reference replay also depends on prediction, selection, ballistics and differentiation.
file(GLOB planner_build_sources CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/planner/planner.*
  ${CMAKE_CURRENT_SOURCE_DIR}/planner/reference.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tracking/target.*
  ${CMAKE_CURRENT_SOURCE_DIR}/model/armor.hpp
  ${PROJECT_SOURCE_DIR}/tools/math/trajectory.*
  ${PROJECT_SOURCE_DIR}/tools/math/math_tools.*
  ${PROJECT_SOURCE_DIR}/tools/runtime/yaml.*)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${planner_build_sources})
set(planner_build_hashes "${mpc_build_id}")
foreach(source IN LISTS planner_build_sources)
  file(SHA256 ${source} source_hash)
  string(APPEND planner_build_hashes ${source_hash})
endforeach()
string(SHA256 planner_build_id "${planner_build_hashes}")
target_compile_definitions(auto_aim PUBLIC DUST_PLANNER_BUILD_ID="${planner_build_id}")


# auto_aim 的公共头暴露 OpenCV（model/armor.hpp / detection/detector.hpp / detection/yolo.hpp / detection/classifier.hpp 等）、
# Eigen（geometry/solver.hpp / planner.hpp / tracking/tracker.hpp / tracking/target.hpp）、
# OpenVINO（detection/classifier.hpp 含 <openvino/openvino.hpp>）；多个公共头用根相对路径包含
# tools/ 与 tasks/auto_aim/ 下的头（tracking/target.hpp→tools/estimation/extended_kalman_filter.hpp 等），
# 故 tools 与 dust::include 都是 PUBLIC。
# yaml-cpp / fmt 只出现在实现文件里（detection/detector.cpp / config 读取），Threads 同理，故 PRIVATE。
target_link_libraries(auto_aim
  PUBLIC
    dust::include
    io
    openvino::runtime
    tinympcstatic
    tools
    Eigen3::Eigen
    ${OpenCV_LIBS}
  PRIVATE
    fmt::fmt
    yaml-cpp
    Threads::Threads
)

if(ENABLE_TENSORRT)
  target_compile_definitions(auto_aim PRIVATE DUST_ENABLE_TENSORRT)
  target_link_libraries(auto_aim PRIVATE dust_tensorrt)
endif()

target_precompile_headers(auto_aim PRIVATE
  <Eigen/Geometry>
  <opencv2/opencv.hpp>
  <spdlog/spdlog.h>
  <openvino/openvino.hpp>
)
