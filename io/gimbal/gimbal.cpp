#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <stdexcept>

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  skip_crc_ = false;  // 默认
  if (yaml["skip_gimbal_crc"])
    skip_crc_ = tools::read<bool>(yaml, "skip_gimbal_crc");
  else if (yaml["skip_cboard_crc"])
    skip_crc_ = tools::read<bool>(yaml, "skip_cboard_crc");

  try {
    serial_.setPort(com_port);
    auto timeout = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  if (!queue_.wait_pop_for(std::chrono::seconds(5))) {
    quit_ = true;
    queue_.close();
    if (thread_.joinable()) thread_.join();
    serial_.close();
    throw std::runtime_error("[Gimbal] Timed out waiting for first quaternion");
  }
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  queue_.close();
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (!quit_) {
    auto first = queue_.wait_pop_for(std::chrono::milliseconds(20));
    if (!first) return Eigen::Quaterniond::Identity();
    auto second = queue_.wait_front_for(std::chrono::milliseconds(20));
    if (!second) return std::get<0>(*first).normalized();

    auto [q_a, t_a] = *first;
    auto [q_b, t_b] = *second;
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    if (t_ab <= 0) return q_b.normalized();
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }

  return Eigen::Quaterniond::Identity();
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  // // 打印十六进制数据
  // std::string hex_str;
  // uint8_t * data_ptr = reinterpret_cast<uint8_t *>(&tx_data_);
  // for (size_t i = 0; i < sizeof(tx_data_); ++i) {
  //   hex_str += fmt::format("{:02x} ", data_ptr[i]);
  // }
  // tools::logger()->info("[Gimbal Send HEX] {}", hex_str);

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    uint16_t crc = rx_data_.crc16;
    // tools::logger()->info("[Gimbal] Received data, CRC: 0x{:04X}", crc);

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      if (!skip_crc_) {
        uint16_t received_crc = rx_data_.crc16;
        uint16_t calculated_crc = tools::get_crc16(
          reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_) - sizeof(rx_data_.crc16));
        tools::logger()->info("[Gimbal] CRC16 check failed. Received: 0x{:04X}, Calculated: 0x{:04X}", received_crc, calculated_crc);
        error_count++;
        continue;
      }
      // else: skip CRC check silently
    }

    uint16_t received_crc = rx_data_.crc16;
    uint16_t calculated_crc = tools::get_crc16(
      reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_) - sizeof(rx_data_.crc16));
    // tools::logger()->info("[Gimbal] CRC16 check passed. Received: 0x{:04X}, Calculated: 0x{:04X}", received_crc, calculated_crc);

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, t});
    // 按一般情况解算欧拉角：ZYX（Yaw-Pitch-Roll）
    Eigen::Vector3d ypr = q.toRotationMatrix().eulerAngles(2, 1, 0);
    double yaw_angle   = ypr[0];  // Z
    double pitch_angle = ypr[1];  // Y
    double roll_angle  = ypr[2];  // X

    // tools::logger()->info(
    //   "[Gimbal] Euler from q (ZYX) -> Yaw(Z): {:.4f} rad ({:.2f} deg), Pitch(Y): {:.4f} rad ({:.2f} deg), Roll(X): {:.4f} rad ({:.2f} deg)",
    //   yaw_angle, yaw_angle * 180.0 / M_PI,
    //   pitch_angle, pitch_angle * 180.0 / M_PI,
    //   roll_angle, roll_angle * 180.0 / M_PI);
    // 打印原始数据
    std::string raw_hex;
    for (size_t i = 0; i < sizeof(rx_data_); ++i) {
      raw_hex += fmt::format("{:02x} ", reinterpret_cast<uint8_t*>(&rx_data_)[i]);
    }
    // tools::logger()->debug("[Gimbal] Raw data: {}", raw_hex);

    // 打印四元数
    // tools::logger()->debug("[Gimbal] Quaternion: q=[{}, {}, {}, {}]", q.w(), q.x(), q.y(), q.z());

    std::lock_guard<std::mutex> lock(mutex_);

  // convert deg->rad for storage in state
    state_.yaw = yaw_angle;
    state_.yaw_vel = rx_data_.yaw_vel * static_cast<float>(M_PI / 180.0);
    state_.pitch = pitch_angle;
    state_.pitch_vel = rx_data_.pitch_vel * static_cast<float>(M_PI / 180.0);
    state_.bullet_speed = rx_data_.bullet_speed;
    // tools::logger()->info("[Gimbal] bullet_speed: {:.2f}", state_.bullet_speed);
    state_.bullet_count = rx_data_.bullet_count;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
