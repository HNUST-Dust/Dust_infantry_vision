#include <Eigen/Core>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "calibration/foxglove_calibration.hpp"

int main(int argc, char * argv[])
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <port>\n";
    return 2;
  }

  int port = std::stoi(argv[1]);
  if (port < 1 || port > 65535) throw std::runtime_error("port must be 1..65535");

  calibration::FoxgloveCalibration foxglove({
    "127.0.0.1", static_cast<uint16_t>(port), 20.0, 85});

  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(32, 32, 32));
  for (int y = 0; y < image.rows; y += 40) {
    for (int x = 0; x < image.cols; x += 40) {
      const bool light = ((x / 40) + (y / 40)) % 2 == 0;
      cv::rectangle(
        image, {x, y, 40, 40}, light ? cv::Scalar(40, 180, 240) : cv::Scalar(180, 70, 40),
        cv::FILLED);
    }
  }
  cv::putText(
    image, "Foxglove calibration protocol test", {42, 245}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
    cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

  std::cout << "FOXGLOVE_TEST_READY=" << port << std::endl;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(12);
  int saved_count = 0;
  std::string last_event = "ready";
  while (std::chrono::steady_clock::now() < deadline) {
    if (foxglove.take_save_request()) {
      saved_count++;
      last_event = "saved_" + std::to_string(saved_count);
    }
    foxglove.publish(image, true, Eigen::Vector3d(12.5, -3.25, 1.75), saved_count, last_event);
    if (foxglove.quit_requested()) {
      std::cout << "FOXGLOVE_TEST_QUIT saved_count=" << saved_count << std::endl;
      return saved_count == 1 ? 0 : 3;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cerr << "Timed out waiting for /calibration/quit\n";
  return 4;
}
