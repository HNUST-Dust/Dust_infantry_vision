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
  // 生产入口：基准时刻取 steady_clock::now()。
  // gimbal_yaw 传云台当前绝对 yaw（GimbalState::yaw）。开火判定里只有"云台已到位"这一项
  // 依赖它；auto_fire、决策有效性和指令跳变检查与是否提供反馈无关。
  Plan plan(
    std::optional<Target> target, double bullet_speed,
    const Eigen::Matrix3d & R_gimbal2world = Eigen::Matrix3d::Identity(),
    std::optional<double> gimbal_yaw = std::nullopt);
  // 回放/单测入口：显式给出"现在"，让离线回放与单测不依赖墙钟（对应旧 Aimer 的 to_now=false）。
  Plan plan_at(
    std::optional<Target> target, std::chrono::steady_clock::time_point now, double bullet_speed,
    const Eigen::Matrix3d & R_gimbal2world = Eigen::Matrix3d::Identity(),
    std::optional<double> gimbal_yaw = std::nullopt);

  // Runtime hook: replace the feedback sample recorded for the command after planning has finished.
  // The planner uses this as the baseline for the next settled check.
  void record_command_gimbal_yaw(std::optional<double> gimbal_yaw);

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
  double last_plan_yaw_ = 0.0;      // 上一条下发的 yaw（Plan::yaw），是相对云台系的增量
  double last_target_yaw_ = 0.0;    // 上一条参考的 yaw（Plan::target_yaw），同样在云台系
  // 上一条指令生成时刻的云台绝对 yaw（GimbalState::yaw，来自姿态四元数）。
  // 下发值 plan.yaw 与它同框：下位机按「目标 = 当前云台 yaw + 下发 yaw」解释控制帧，
  // 所以"云台已到位"要比较的是这一对（转过多少 vs 命令转多少），而不是两个绝对角直接相减。
  // 没有反馈的那次规划会把这里清成 nullopt，避免用过期角度判定。
  std::optional<double> last_plan_gimbal_yaw_;

  // 一次规划最终采用的瞄准决策
  struct AimDecision
  {
    bool valid = false;  // 策略真正选中了装甲板；false 表示走了回退
    bool converged = false;  // 飞行时间迭代在容差内收敛；未收敛时禁止开火
    int id = 0;
    int lock = -1;  // 决策时刻的局部锁，作为参考轨迹的锚点，不回流成员 lock_id_
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

  // 单块装甲板 -> 弹道飞行时间与云台角。弹道的水平距离和高度均取世界系；yaw 使用云台系。
  std::optional<AimSolution> solve_xyza(
    const Eigen::Vector4d & xyza, double bullet_speed,
    const Eigen::Matrix3d & R_gimbal2world) const;

  // 用原 Aimer 的迭代法收敛飞行时间，同时定下决策时刻要打的那块装甲板。
  bool resolve_aim_time(
    Target base, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world,
    AimDecision & decision);

  // 以命中时刻的决策为锚点构造 60 列参考轨迹：第 HALF_HORIZON 列就是 decision_id 那块板，
  // 再由它向两侧（未来 / 过去）各自演进一个锁副本，保证中心板 ID、飞行时间与参考方向一致。
  // hit_target 必须已经推进到命中时刻 T1 + fly_time。
  bool get_trajectory(
    const Target & hit_target, double yaw0, double bullet_speed,
    const Eigen::Matrix3d & R_gimbal2world, int decision_lock, int decision_id, Trajectory & out);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
