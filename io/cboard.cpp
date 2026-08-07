#include "cboard.hpp"

#include "tools/crc.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <limits>

namespace io
{
CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  queue_(5000)
{
  auto yaml = tools::load(config_path);
  com_port_ = "/dev/gimbal";  // 默认
  if (yaml["cboard_com_port"]) com_port_ = tools::read<std::string>(yaml, "cboard_com_port");
  baudrate_ = 115200;  // 默认
  if (yaml["cboard_baudrate"]) baudrate_ = tools::read<int>(yaml, "cboard_baudrate");
  skip_crc_ = false;  // 默认
  if (yaml["skip_cboard_crc"]) skip_crc_ = tools::read<bool>(yaml, "skip_cboard_crc");

  try {
    serial_.setPort(com_port_);
    serial_.setBaudrate(baudrate_);
    auto timeout = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    ::tools::logger()->error("[CBoard] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&CBoard::read_thread, this);

  if (!skip_crc_) {
    ::tools::logger()->info("[CBoard] Waiting for q...");
    auto first = queue_.wait_pop_for(std::chrono::seconds(5));
    auto second = queue_.wait_pop_for(std::chrono::seconds(5));
    if (first && second) {
      data_ahead_ = std::move(*first);
      data_behind_ = std::move(*second);
    } else {
      ::tools::logger()->error("[CBoard] Timed out waiting for IMU data");
      data_ahead_ = {Eigen::Quaterniond::Identity(), std::chrono::steady_clock::now()};
      data_behind_ = data_ahead_;
    }
  } else {
    // 跳过CRC时，不等待初始数据，使用默认值
    data_ahead_ = {Eigen::Quaterniond::Identity(), std::chrono::steady_clock::now()};
    data_behind_ = {Eigen::Quaterniond::Identity(), std::chrono::steady_clock::now()};
  }
  ::tools::logger()->info("[CBoard] Opened.");
}

CBoard::~CBoard()
{
  quit_ = true;
  queue_.close();
  if (thread_.joinable()) thread_.join();
  serial_.close();
}Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    if (!queue_.try_pop(data_behind_)) {
      // 队列为空，返回当前可用的四元数
      break;
    }
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;

  if (t_a == t_b) {
    return q_a;
  }

  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command)
{
  auto encode = [](double value) {
    auto scaled = value * 1e4;
    scaled = std::clamp(
      scaled, static_cast<double>(std::numeric_limits<int16_t>::min()),
      static_cast<double>(std::numeric_limits<int16_t>::max()));
    return static_cast<int16_t>(scaled);
  };

  VisionToCBoard tx_data;
  tx_data.control = command.control ? 1 : 0;
  tx_data.shoot = command.shoot ? 1 : 0;
  tx_data.yaw = encode(command.yaw);
  tx_data.pitch = encode(command.pitch);
  tx_data.horizon_distance = encode(command.horizon_distance);
  tx_data.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data), sizeof(tx_data) - sizeof(tx_data.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data), sizeof(tx_data));
  } catch (const std::exception & e) {
    ::tools::logger()->warn("[CBoard] Failed to write serial: {}", e.what());
  }
}

void CBoard::read_thread()
{
  ::tools::logger()->info("[CBoard] read_thread started.");
  ::CBoardToVision rx_data;

  while (!quit_) {
    if (!serial_.read(reinterpret_cast<uint8_t *>(&rx_data), sizeof(rx_data.head))) {
      continue;
    }

    if (rx_data.head[0] != 'S' || rx_data.head[1] != 'P') continue;

    if (!serial_.read(
          reinterpret_cast<uint8_t *>(&rx_data) + sizeof(rx_data.head),
          sizeof(rx_data) - sizeof(rx_data.head))) {
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data), sizeof(rx_data))) {
      if (!skip_crc_) {
        ::tools::logger()->debug("[CBoard] CRC16 check failed.");
        continue;
      }
      // else: skip CRC check silently
    }

    // 打印原始数据
    std::string raw_hex;
    for (size_t i = 0; i < sizeof(rx_data); ++i) {
      raw_hex += fmt::format("{:02x} ", reinterpret_cast<uint8_t*>(&rx_data)[i]);
    }
    // ::tools::logger()->debug("[CBoard] Raw data: {}", raw_hex);

    auto timestamp = std::chrono::steady_clock::now();

    if (rx_data.type == 1) {  // IMU
      Eigen::Quaterniond q(rx_data.q[0], rx_data.q[1], rx_data.q[2], rx_data.q[3]);
      queue_.push({q, timestamp});

      // // 打印四元数
      // ::tools::logger()->debug("[CBoard] Quaternion: q=[{}, {}, {}, {}]", q.w(), q.x(), q.y(), q.z());
    } else if (rx_data.type == 0) {  // Bullet speed
      bullet_speed = rx_data.bullet_speed;
      auto valid_mode = rx_data.mode < MODES.size();
      auto valid_shoot_mode = rx_data.shoot_mode < SHOOT_MODES.size();
      mode = valid_mode ? static_cast<Mode>(rx_data.mode) : Mode::idle;
      shoot_mode = valid_shoot_mode ? static_cast<ShootMode>(rx_data.shoot_mode)
                                    : ShootMode::left_shoot;
      ft_angle = rx_data.ft_angle;

      if (!valid_mode || !valid_shoot_mode) {
        ::tools::logger()->warn(
          "[CBoard] Invalid mode values: mode={}, shoot_mode={}", rx_data.mode,
          rx_data.shoot_mode);
        continue;
      }

      // 限制日志输出频率为1Hz
      static auto last_log_time = std::chrono::steady_clock::time_point::min();
      auto now = std::chrono::steady_clock::now();

      if (rx_data.bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
        ::tools::logger()->info(
          "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
          rx_data.bullet_speed, MODES[rx_data.mode], SHOOT_MODES[rx_data.shoot_mode],
          rx_data.ft_angle);
        last_log_time = now;
      }
    }
  }

  ::tools::logger()->info("[CBoard] read_thread stopped.");
}

// 移除read_yaml方法，不再需要CAN配置

}  // namespace io
