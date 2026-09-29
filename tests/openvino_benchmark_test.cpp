#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/yolo.hpp"

namespace
{
double elapsed_ms(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void warm_up(auto_aim::YOLO & yolo, const cv::Mat & image)
{
  for (int i = 0; i < 10; ++i) {
    yolo.detect(image);
  }
}

}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::cerr << "Usage: openvino_benchmark_test <config.yaml> <video.avi> [iterations]" << std::endl;
    return 1;
  }

  const int iterations = argc >= 4 ? std::atoi(argv[3]) : 120;
  if (iterations <= 0) return 1;

  cv::VideoCapture video(argv[2]);
  cv::Mat image;
  if (!video.read(image) || image.empty()) return 1;

  auto_aim::YOLO sync_yolo(argv[1], false);
  warm_up(sync_yolo, image);
  const auto sync_start = std::chrono::steady_clock::now();
  for (int i = 0; i < iterations; ++i) {
    sync_yolo.detect(image);
  }
  const double sync_ms = elapsed_ms(sync_start);

  auto_aim::YOLO async_yolo(argv[1], false);
  warm_up(async_yolo, image);
  std::deque<auto_aim::NetDetector::TicketPtr> pending;
  std::deque<std::chrono::steady_clock::time_point> pending_started;
  std::vector<double> async_latencies;
  async_latencies.reserve(iterations);
  const auto async_start = std::chrono::steady_clock::now();
  for (int i = 0; i < iterations; ++i) {
    const auto started = std::chrono::steady_clock::now();
    auto ticket = async_yolo.try_start_async(image);
    if (!ticket) {
      async_yolo.postprocess(pending.front());
      async_latencies.push_back(elapsed_ms(pending_started.front()));
      pending.pop_front();
      pending_started.pop_front();
      ticket = async_yolo.try_start_async(image);
    }
    if (!ticket) return 2;
    pending.push_back(std::move(ticket));
    pending_started.push_back(started);
  }
  while (!pending.empty()) {
    async_yolo.postprocess(pending.front());
    async_latencies.push_back(elapsed_ms(pending_started.front()));
    pending.pop_front();
    pending_started.pop_front();
  }

  std::sort(async_latencies.begin(), async_latencies.end());
  const auto percentile = [&](double quantile) {
    const auto index = static_cast<std::size_t>(
      std::ceil(quantile * async_latencies.size()) - 1.0);
    return async_latencies[index];
  };
  const double async_ms = elapsed_ms(async_start);

  std::cout << std::fixed << std::setprecision(2)
            << "sync: " << iterations / (sync_ms / 1000.0) << " FPS, " << sync_ms / iterations
            << " ms/frame\n"
            << "async: " << iterations / (async_ms / 1000.0) << " FPS, " << async_ms / iterations
            << " ms/frame, p50: " << percentile(0.50) << " ms, p95: " << percentile(0.95)
            << " ms, p99: " << percentile(0.99) << " ms, max: " << async_latencies.back()
            << " ms\n";
  return 0;
}
