add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)

# tools 的公共头直接暴露这些库，故必须是 PUBLIC：img_tools.hpp / recorder.hpp 含
# <opencv2/opencv.hpp>；math_tools.hpp / extended_kalman_filter.hpp / quaternion_buffer.hpp /
# thread_pool.hpp 含 Eigen；logger.hpp 含 <spdlog/spdlog.h>（全仓 31 处包含它）；
# yaml.hpp 含 <yaml-cpp/yaml.h>；plotter.hpp 含 <nlohmann/json.hpp>。
# dust::include 也是 PUBLIC：recorder.hpp 等用 #include "tools/thread_safe_queue.hpp" 根相对路径。
# logger.hpp 用 spdlog/spdlog.h 并以 header-only 方式内联实现，故 spdlog 必须是 PUBLIC：
# 所有包含 logger.hpp 的翻译单元都要拿到同一份定义（含 SPDLOG_FMT_EXTERNAL），否则是 ODR 违规。
# fmt 与 Threads 只出现在实现文件里（recorder.cpp / thread_pool.cpp），故 PRIVATE。
target_link_libraries(tools
  PUBLIC
    dust::include
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    yaml-cpp
    spdlog::spdlog_header_only
    ${OpenCV_LIBS}
  PRIVATE
    fmt::fmt
    Threads::Threads
)

target_precompile_headers(tools PRIVATE
  <Eigen/Geometry>
  <opencv2/opencv.hpp>
  <spdlog/spdlog.h>
)
