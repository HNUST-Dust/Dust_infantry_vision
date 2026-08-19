#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <tuple>

#include "usb_interrupt_transport.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) == 43);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) == 29);

enum class GimbalMode
{
  IDLE,      // 空闲
  AUTO_AIM   // 自瞄
};

struct GimbalState
{
  // Note: values are in radians and radians/second
  float yaw;       // yaw angle in radians
  float yaw_vel;   // yaw velocity in radians/sec
  float pitch;     // pitch angle in radians
  float pitch_vel; // pitch velocity in radians/sec
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path, bool simulate = false);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc,uint8_t camera_flag);
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

private:
  std::unique_ptr<UsbInterruptTransport> usb_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;
  std::mutex send_mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;
  bool simulate_ = false;
  bool skip_crc_ = false;
  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_{};
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  void read_thread();
  bool reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
