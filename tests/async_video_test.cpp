#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <atomic>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/thread_safe_queue.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(1);

  tools::Exiter exiter;

  auto_aim::multithread::MultiThreadDetector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);
  if (!video.isOpened() || !text) {
    tools::logger()->error("Failed to open {} / {}", video_path, text_path);
    return 1;
  }

  auto t0 = std::chrono::steady_clock::now();
  std::atomic<bool> capture_done{false};
  tools::ThreadSafeQueue<std::chrono::steady_clock::time_point, true> push_wall_queue(128);
  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    while (!exiter.exit()) {
      video.read(img);
      if (img.empty()) break;

      double t, w, x, y, z;
      text >> t >> w >> x >> y >> z;
      auto timestamp = t0 + std::chrono::microseconds(int64_t(t * 1e6));
      auto push_wall = std::chrono::steady_clock::now();
      if (detector.push(img, timestamp)) {
        push_wall_queue.push(push_wall);
      }
    }
    capture_done = true;
  });

  auto max_latency = 0.0;
  auto sum_latency = 0.0;
  auto frame_total = 0;
  for (int frame_count = 0; !exiter.exit(); frame_count++) {
    if (capture_done.load() && detector.empty()) break;

    frame_total++;
    auto pop_start = std::chrono::steady_clock::now();
    auto [img_det, armors, t_det] = detector.debug_pop();
    auto pop_end = std::chrono::steady_clock::now();
    auto push_wall = push_wall_queue.pop();
    auto latency_ms = tools::delta_time(pop_end, push_wall) * 1e3;
    (void)img_det;

    auto targets = tracker.track(armors, t_det);
    max_latency = std::max(max_latency, latency_ms);
    sum_latency += latency_ms;

    tools::logger()->info(
      "[{}] pop: {:.1f}ms, latency: {:.1f}ms, targets: {}, armors: {}", frame_count,
      tools::delta_time(pop_end, pop_start) * 1e3,
      latency_ms, targets.size(), armors.size());
  }

  if (detect_thread.joinable()) detect_thread.join();
  tools::logger()->info(
    "[Summary] frames: {}, avg latency: {:.1f}ms, max latency: {:.1f}ms", frame_total,
    sum_latency / frame_total, max_latency);
  return 0;
}
