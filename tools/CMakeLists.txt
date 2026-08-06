cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)

target_precompile_headers(tools PRIVATE
  <Eigen/Geometry>
  <opencv2/opencv.hpp>
  <spdlog/spdlog.h>
)
