add_library(tools_core INTERFACE)
target_link_libraries(tools_core INTERFACE dust::include Eigen3::Eigen Threads::Threads)

if(build_camera OR build_pipeline OR build_inference)
  add_library(tools STATIC
    runtime/exiter.cpp
    estimation/extended_kalman_filter.cpp
    image/img_tools.cpp
    math/math_tools.cpp
    diagnostics/plotter.cpp
    math/trajectory.cpp
    diagnostics/recorder.cpp
    diagnostics/logger.cpp
    math/pid.cpp
    protocol/crc.cpp
  )
  target_link_libraries(tools
    PUBLIC tools_core nlohmann_json::nlohmann_json yaml-cpp spdlog::spdlog_header_only
           opencv_core opencv_videoio
    PRIVATE fmt::fmt opencv_imgproc
  )
endif()
