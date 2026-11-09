add_library(tools OBJECT 
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

# tools 的公共头直接暴露这些库，故必须是 PUBLIC：image/img_tools.hpp / diagnostics/recorder.hpp 含
# <opencv2/opencv.hpp>；math/math_tools.hpp / estimation/extended_kalman_filter.hpp / estimation/quaternion_buffer.hpp /
# concurrency/thread_pool.hpp 含 Eigen；diagnostics/logger.hpp 含 <spdlog/spdlog.h>（全仓 31 处包含它）；
# runtime/yaml.hpp 含 <yaml-cpp/yaml.h>；diagnostics/plotter.hpp 含 <nlohmann/json.hpp>。
# dust::include 也是 PUBLIC：diagnostics/recorder.hpp 等用 #include "tools/concurrency/thread_safe_queue.hpp" 根相对路径。
# diagnostics/logger.hpp 用 spdlog/spdlog.h 并以 header-only 方式内联实现，故 spdlog 必须是 PUBLIC：
# 所有包含 diagnostics/logger.hpp 的翻译单元都要拿到同一份定义（含 SPDLOG_FMT_EXTERNAL），否则是 ODR 违规。
# fmt 与 Threads 只出现在实现文件里（diagnostics/recorder.cpp / thread_pool.cpp），故 PRIVATE。
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
