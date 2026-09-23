add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    dynamic_roi.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    shooter.cpp
    net_detector.cpp
    yolo.cpp
    yolos/yolov5.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

# auto_aim 的公共头暴露 OpenCV（armor.hpp / detector.hpp / yolo.hpp / classifier.hpp 等）、
# Eigen（aimer.hpp / solver.hpp / planner.hpp / tracker.hpp / target.hpp）、
# OpenVINO（net_detector.hpp 含 <openvino/openvino.hpp>）；多个公共头用根相对路径包含
# tools/ 与 tasks/auto_aim/ 下的头（target.hpp→tools/extended_kalman_filter.hpp 等），
# 故 tools 与 dust::include 都是 PUBLIC。
# yaml-cpp / fmt 只出现在实现文件里（detector.cpp / config 读取），Threads 同理，故 PRIVATE。
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

target_precompile_headers(auto_aim PRIVATE
  <Eigen/Geometry>
  <opencv2/opencv.hpp>
  <spdlog/spdlog.h>
  <openvino/openvino.hpp>
)
