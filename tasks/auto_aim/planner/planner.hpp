#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <mutex>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 30;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control = false;
  bool fire = false;
  float target_yaw = 0;
  float target_pitch = 0;
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float pitch_acc = 0;

  // 云台控制帧的 mode：0=IDLE(不控制), 1=AUTO_AIM(控制云台但不开火), 2=FIRE
  // 唯一实现处；Gimbal::send(bool,bool,...) 也按同一规则算 mode。
  int mode() const { return control ? (fire ? 2 : 1) : 0; }
};

// 装甲板选择结果；id 是 Target::armor_xyza_list() 的下标
struct AimPoint
{
  bool valid = false;
  int id = 0;
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
};

class Planner
{
public:
  Planner(const std::string & config_path);
  ~Planner();

  Eigen::Vector4d debug_xyza() const;
  bool debug_aim_valid() const;

  // 核心重载：只做相对预测，不碰墙钟。target 自身的时刻就是规划的基准时刻 T1。
  Plan plan(
    Target target, double bullet_speed,
    const Eigen::Matrix3d & R_gimbal2world = Eigen::Matrix3d::Identity(),
    std::optional<double> gimbal_yaw = std::nullopt);
  // 生产入口：基准时刻取 steady_clock::now()。gimbal_yaw 有值时才启用"云台已到位"的开火判定。
  Plan plan(
    std::optional<Target> target, double bullet_speed,
    const Eigen::Matrix3d & R_gimbal2world = Eigen::Matrix3d::Identity(),
    std::optional<double> gimbal_yaw = std::nullopt);
  // 回放/单测入口：显式给出"现在"，让离线回放与单测不依赖墙钟（对应旧 Aimer 的 to_now=false）。
  Plan plan_at(
    std::optional<Target> target, std::chrono::steady_clock::time_point now, double bullet_speed,
    const Eigen::Matrix3d & R_gimbal2world = Eigen::Matrix3d::Identity(),
    std::optional<double> gimbal_yaw = std::nullopt);

private:
  double yaw_offset_;
  double pitch_offset_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  double fire_thresh_high_speed_;
  double fire_thresh_low_speed_;
  // 装甲板选择策略参数（原 Aimer）
  double comming_angle_;
  double leaving_angle_;
  // 开火判定参数（原 Shooter）
  double first_tolerance_;
  double second_tolerance_;
  double judge_distance_;
  bool auto_fire_;

  TinySolver * yaw_solver_ = nullptr;
  TinySolver * pitch_solver_ = nullptr;
  Eigen::Vector4d debug_xyza_ = Eigen::Vector4d::Zero();
  bool debug_aim_valid_ = false;
  mutable std::mutex debug_mutex_;

  int lock_id_ = -1;                // 原 Aimer 的锁定装甲板，防止在两块 45 度板之间来回切
  bool have_last_command_ = false;  // 原 Shooter 的 last_command_ 是否已经存在
  double last_plan_yaw_ = 0.0;      // 上一条下发的 yaw（Plan::yaw）
  double last_target_yaw_ = 0.0;    // 上一条参考的 yaw（Plan::target_yaw）

  // 一次规划最终采用的瞄准决策
  struct AimDecision
  {
    bool valid = false;  // 策略真正选中了装甲板；false 表示走了回退
    int id = 0;
    double fly_time = 0.0;
  };
  struct AimSolution
  {
    double fly_time = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
  };

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  int nearest_armor_id(const Target & target) const;

  // 击打策略（原 Aimer::choose_aim_point）。lock_id 按引用进出：horizon 前瞻用副本调用，
  // 不会污染成员状态。
  AimPoint choose_aim_point(const Target & target, int & lock_id) const;

  // 单块装甲板 -> 弹道飞行时间与云台角。保留 Planner 原有的"世界系水平距离 + 云台系高度"约定。
  std::optional<AimSolution> solve_xyza(
    const Eigen::Vector4d & xyza, double bullet_speed,
    const Eigen::Matrix3d & R_gimbal2world) const;

  // 用原 Aimer 的迭代法收敛飞行时间，同时定下决策时刻要打的那块装甲板。
  bool resolve_aim_time(
    Target base, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world,
    AimDecision & decision);

  bool get_trajectory(
    Target & target, double yaw0, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world,
    int lookahead_lock, int decision_id, Trajectory & out);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
