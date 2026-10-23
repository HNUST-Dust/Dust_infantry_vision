#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <cmath>
#include <stdexcept>

namespace io
{
Gimbal::Gimbal(const std::string & config_path, bool simulate)
: simulate_(simulate)
{
  if (simulate_) {
    state_.bullet_speed = 23.0F;
    feedback_timestamp_ = std::chrono::steady_clock::now();
    feedback_valid_ = true;
    tools::logger()->warn("[Gimbal] Using simulated feedback and serial output.");
    return;
  }

  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  const uint32_t baudrate = yaml["baudrate"] ? yaml["baudrate"].as<uint32_t>() : 460800;
  skip_crc_ = false;  // 默认
  if (yaml["skip_gimbal_crc"])
    skip_crc_ = tools::read<bool>(yaml, "skip_gimbal_crc");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    tools::logger()->info("[Gimbal] Serial {} at {} baud", com_port, baudrate);
    auto timeout = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  if (!orientations_.wait_first(std::chrono::seconds(5))) {
    quit_ = true;
    orientations_.close();
    if (thread_.joinable()) thread_.join();
    serial_.close();
    throw std::runtime_error("[Gimbal] Timed out waiting for first quaternion");
  }
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  orientations_.close();
  if (thread_.joinable()) thread_.join();
  if (!simulate_) serial_.close();
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

std::chrono::steady_clock::time_point Gimbal::feedback_timestamp() const
{
  return feedback().received_at;
}

GimbalFeedback Gimbal::feedback() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto at = simulate_ && !external_feedback_ ? std::chrono::steady_clock::now()
                                                 : feedback_timestamp_;
  return {state_, feedback_q_, at, feedback_sequence_, feedback_valid_,
          !simulate_ && !external_feedback_};
}

void Gimbal::set_external_feedback(
  const Eigen::Quaterniond & q, std::chrono::steady_clock::time_point t)
{
  std::lock_guard<std::mutex> lock(mutex_);
  external_feedback_ = true;
  feedback_valid_ = false;
  if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6) {
    external_rates_ = tools::AngleRateEstimator{};
    return;
  }
  const auto angles = tools::eulers(q.normalized(), 2, 0, 1);
  const auto rates = external_rates_.update(angles.head<2>(), t);
  state_.yaw = static_cast<float>(angles[0]);
  state_.pitch = static_cast<float>(angles[1]);
  state_.yaw_vel = rates ? (*rates)[0] : 0.0F;
  state_.pitch_vel = rates ? (*rates)[1] : 0.0F;
  state_.bullet_speed = 23.0F;
  feedback_timestamp_ = t;
  feedback_q_ = q.normalized();
  ++feedback_sequence_;
  feedback_valid_ = rates.has_value() && rates->allFinite();
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

std::optional<tools::QuaternionSample> Gimbal::orientation_at(std::chrono::steady_clock::time_point t)
{
  if (simulate_) return tools::QuaternionSample{Eigen::Quaterniond::Identity(), t, t};
  return orientations_.at(t);
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

  if (simulate_) return;

  write_tx_data();
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

  if (simulate_) return;

  // // 打印十六进制数据
  // std::string hex_str;
  // uint8_t * data_ptr = reinterpret_cast<uint8_t *>(&tx_data_);
  // for (size_t i = 0; i < sizeof(tx_data_); ++i) {
  //   hex_str += fmt::format("{:02x} ", data_ptr[i]);
  // }
  // tools::logger()->info("[Gimbal Send HEX] {}", hex_str);

  write_tx_data();
}

bool Gimbal::write_tx_data()
{
  try {
    const auto written = serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    if (written == sizeof(tx_data_)) return true;
    const auto failures = ++write_failure_count_;
    if (failures <= 3 || failures % 100 == 0)
      tools::logger()->warn(
        "[Gimbal] Short serial write: {}/{} bytes (failures={})", written, sizeof(tx_data_),
        failures);
  } catch (const std::exception & e) {
    const auto failures = ++write_failure_count_;
    if (failures <= 3 || failures % 100 == 0)
      tools::logger()->warn("[Gimbal] Failed to write serial (failures={}): {}", failures, e.what());
  }
  return false;
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
  auto last_rate_log = std::chrono::steady_clock::time_point{};

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

    // uint16_t crc = rx_data_.crc16;
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

    // uint16_t received_crc = rx_data_.crc16;
    // uint16_t calculated_crc = tools::get_crc16(
    //   reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_) - sizeof(rx_data_.crc16));
    // tools::logger()->info("[Gimbal] CRC16 check passed. Received: 0x{:04X}, Calculated: 0x{:04X}", received_crc, calculated_crc);

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6 ||
        !std::isfinite(rx_data_.yaw_vel) || !std::isfinite(rx_data_.pitch_vel)) {
      std::lock_guard<std::mutex> lock(mutex_);
      feedback_valid_ = false;
      continue;
    }
    if (!orientations_.push(q, t)) continue;
    q.normalize();
    // 云台关节轴为 Z(yaw)-X(pitch)-Y(roll)，按同一顺序提取反馈角。
    Eigen::Vector3d zxy = tools::eulers(q, 2, 0, 1);
    double yaw_angle   = zxy[0];  // Z
    double pitch_angle = zxy[1];  // X
    // double roll_angle  = zxy[2];  // Y

    // tools::logger()->info(
    //   "[Gimbal] Euler from q (ZXY) -> Yaw(Z): {:.4f} rad ({:.2f} deg), Pitch(X): {:.4f} rad ({:.2f} deg), Roll(Y): {:.4f} rad ({:.2f} deg)",
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

    // if (last_rate_log == std::chrono::steady_clock::time_point{} ||
    //     t - last_rate_log >= std::chrono::seconds(1)) {
    //   tools::logger()->info(
    //     "[GimbalRxRate] feedback (rad/s): yaw={:.6f}, pitch={:.6f}",
    //     static_cast<float>(rx_data_.yaw_vel), static_cast<float>(rx_data_.pitch_vel));
    //   last_rate_log = t;
    // }

    std::lock_guard<std::mutex> lock(mutex_);

    // MCU angular rates already use rad/s, matching the planner state.
    state_.yaw = yaw_angle;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = pitch_angle;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    // tools::logger()->info("[Gimbal] bullet_speed: {:.2f}", state_.bullet_speed);
    state_.bullet_count = rx_data_.bullet_count;
    feedback_timestamp_ = t;
    feedback_q_ = q;
    ++feedback_sequence_;
    feedback_valid_ = true;

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
  {
    std::lock_guard<std::mutex> lock(mutex_);
    feedback_valid_ = false;
  }
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
      orientations_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
