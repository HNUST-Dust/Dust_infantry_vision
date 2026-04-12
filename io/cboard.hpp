#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "io/command.hpp"
#include "serial/serial.h"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

#pragma pack(push, 1)

struct CBoardToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t type;  // 0: IMU, 1: Bullet speed
  float q[4];    // IMU四元数
  float bullet_speed;
  uint8_t mode;
  uint8_t shoot_mode;
  float ft_angle;  // 无人机专有
  uint16_t crc16;
};

struct VisionToCBoard
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t control;  // 0: 不控制, 1: 控制
  uint8_t shoot;    // 0: 不射击, 1: 射击
  int16_t yaw;      // 角度 * 1e4
  int16_t pitch;
  int16_t horizon_distance;  // 水平距离 * 1e4
  uint16_t crc16;
};

#pragma pack(pop)

static_assert(sizeof(CBoardToVision) <= 64);
static_assert(sizeof(VisionToCBoard) <= 64);

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有

  CBoard(const std::string & config_path);

  ~CBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command);

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在serial_之前初始化，否则存在死锁的可能
  serial::Serial serial_;
  IMUData data_ahead_;
  IMUData data_behind_;

  std::string com_port_;
  int baudrate_;
  bool skip_crc_;
  std::thread thread_;
  std::atomic<bool> quit_{false};

  void read_thread();
};

}  // namespace io

#endif  // IO__CBOARD_HPP