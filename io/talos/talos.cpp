#include "io/talos/talos.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace io
{
static_assert(sizeof(TalosFrameInfo) == 72);

Talos::Talos()
: rgb_(1440U * 1080U * 3U), wall_epoch_(std::chrono::system_clock::now()),
  steady_epoch_(std::chrono::steady_clock::now())
{
  handle_ = talos_ipc_connect();
  if (!handle_) throw std::runtime_error("Unable to connect to Talos IPC; is the simulator running?");
}

Talos::~Talos()
{
  if (handle_) talos_ipc_close(handle_);
}

int Talos::read(TalosFrame & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  TalosFrameInfo info{};
  const int result = talos_ipc_read_frame(handle_, rgb_.data(), rgb_.size(), &info);
  if (result != 1) return result;
  if (info.width != 1440 || info.height != 1080) return -10;
  if (!(std::isfinite(info.fx) && std::isfinite(info.fy) && std::isfinite(info.cx) &&
        std::isfinite(info.cy)) || info.fx <= 0.0 || info.fy <= 0.0)
    return -12;
  const Eigen::Quaterniond q(
    info.quaternion_wxyz[0], info.quaternion_wxyz[1], info.quaternion_wxyz[2],
    info.quaternion_wxyz[3]);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6) return -11;

  cv::Mat rgb(static_cast<int>(info.height), static_cast<int>(info.width), CV_8UC3, rgb_.data());
  cv::cvtColor(rgb, frame.image_bgr, cv::COLOR_RGB2BGR);
  const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    wall_epoch_.time_since_epoch()).count();
  const auto frame_delta = std::chrono::nanoseconds(
    static_cast<int64_t>(info.timestamp_ns) - wall_ns);
  frame.timestamp = steady_epoch_ + frame_delta;
  frame.gimbal_to_world = q.normalized();
  frame.sequence = info.frame_seq;
  frame.camera = info;
  return 1;
}

int Talos::send(float yaw_deg, float pitch_deg, float distance_m, bool fire_advice)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return talos_ipc_send_command(
    handle_, yaw_deg, pitch_deg, distance_m, static_cast<uint8_t>(fire_advice));
}
}  // namespace io
