#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>

#include "visualization/foxglove_vision.hpp"

// Foxglove 图像/遥测协议测试的发布端，配合 tests/foxglove_vision_protocol_test.py 使用。
//   foxglove_vision_test <image_port> [data_port] [duration_ms]
// data_port 省略或为 0 表示单端口模式；只在 visualization_foxglove 上运行，不需要相机或模型。
int main(int argc, char * argv[])
{
  if (argc < 2 || argc > 4) {
    std::cerr << "Usage: " << argv[0] << " <image_port> [data_port] [duration_ms]\n";
    return 2;
  }

  const int image_port = std::stoi(argv[1]);
  const int data_port = argc > 2 ? std::stoi(argv[2]) : 0;
  const int duration_ms = argc > 3 ? std::stoi(argv[3]) : 8000;
  if (image_port < 1 || image_port > 65535 || data_port < 0 || data_port > 65535 || duration_ms <= 0)
    throw std::runtime_error("invalid port or duration");

  visualization::FoxgloveVisionOptions options;
  options.host = "127.0.0.1";
  options.image_port = static_cast<uint16_t>(image_port);
  options.data_port = static_cast<uint16_t>(data_port);
  options.image_fps = 20.0;
  options.image_scale = 0.5;  // 1280x960 -> 640x480，协议测试据此断言缩放生效
  options.jpeg_quality = 85;

  visualization::FoxgloveVision foxglove(options);

  // 输入固定为 1280x960，缩放 0.5 后协议测试应解析出 640x480
  cv::Mat image(960, 1280, CV_8UC3, cv::Scalar(32, 32, 32));
  cv::putText(
    image, "Foxglove vision protocol test", {42, 480}, cv::FONT_HERSHEY_SIMPLEX, 1.2,
    cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

  std::cout << "FOXGLOVE_VISION_READY image=" << foxglove.image_port()
            << " data=" << foxglove.data_port() << std::endl;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms);
  const auto started_at = std::chrono::steady_clock::now();
  uint64_t counter = 0;
  auto next_telemetry = started_at;

  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();

    // 移动色块：保证相邻帧的 JPEG 不同，便于区分新旧帧
    const int offset = static_cast<int>(counter % 40) * 8;
    image(cv::Rect(100 + offset, 100, 120, 120)).setTo(cv::Scalar(40, 180, 240));
    foxglove.publish_image(image, now);

    if (now >= next_telemetry) {
      next_telemetry = now + std::chrono::milliseconds(10);  // 100 Hz，与计划线程同量级
      counter++;
      foxglove.publish_telemetry(
        {{"t", 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(now - started_at).count()},
         {"counter", counter},
         {"tracker_state", 2},
         {"has_target", 1},
         {"mode", 1},
         {"gimbal_yaw", 1.25}},
        now);
      foxglove.publish_status({{"tracker_state_name", "tracking"}}, now);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  std::cout << "FOXGLOVE_VISION_DONE telemetry=" << counter << std::endl;
  return 0;
}
