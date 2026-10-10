#ifndef IO__TALOS_HPP
#define IO__TALOS_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <vector>

#include <talos_ipc.h>

namespace io
{
struct TalosFrame
{
  cv::Mat image_bgr;
  std::chrono::steady_clock::time_point timestamp;
  Eigen::Quaterniond gimbal_to_world;
  uint64_t sequence = 0;
  TalosFrameInfo camera{};
};

class Talos
{
public:
  Talos();
  ~Talos();
  Talos(const Talos &) = delete;
  Talos & operator=(const Talos &) = delete;

  // Returns 1 for a frame, 0 when no frame is ready, and a negative value on protocol errors.
  int read(TalosFrame & frame);
  int send(float yaw_deg, float pitch_deg, float distance_m, bool fire_advice);

private:
  void * handle_ = nullptr;
  std::vector<uint8_t> rgb_;
  std::chrono::system_clock::time_point wall_epoch_;
  std::chrono::steady_clock::time_point steady_epoch_;
  std::mutex mutex_;
};
}  // namespace io

#endif
