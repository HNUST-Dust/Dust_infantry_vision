#include "planner.hpp"

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_high_speed_ = tools::read<double>(yaml, "fire_thresh_high_speed");
  fire_thresh_low_speed_ = tools::read<double>(yaml, "fire_thresh_low_speed");
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");
  // 装甲板选择策略（原 Aimer）
  comming_angle_ = tools::read<double>(yaml, "comming_angle") / 57.3;
  leaving_angle_ = tools::read<double>(yaml, "leaving_angle") / 57.3;
  // 开火判定（原 Shooter）
  first_tolerance_ = tools::read<double>(yaml, "first_tolerance") / 57.3;
  second_tolerance_ = tools::read<double>(yaml, "second_tolerance") / 57.3;
  judge_distance_ = tools::read<double>(yaml, "judge_distance");
  auto_fire_ = tools::read<bool>(yaml, "auto_fire");

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Planner::~Planner()
{
  tiny_cleanup(yaw_solver_);
  tiny_cleanup(pitch_solver_);
}

Eigen::Vector4d Planner::debug_xyza() const
{
  std::lock_guard<std::mutex> lock(debug_mutex_);
  return debug_xyza_;
}

bool Planner::debug_aim_valid() const
{
  std::lock_guard<std::mutex> lock(debug_mutex_);
  return debug_aim_valid_;
}

int Planner::nearest_armor_id(const Target & target) const
{
  const auto armor_xyza_list = target.armor_xyza_list();
  int best = -1;
  double min_dist = 1e10;
  for (int i = 0; i < static_cast<int>(armor_xyza_list.size()); i++) {
    const double dist = armor_xyza_list[i].head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      best = i;
    }
  }
  return best;
}

AimPoint Planner::choose_aim_point(const Target & target, int & lock_id) const
{
  const Eigen::VectorXd ekf_x = target.ekf_x();
  const std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  const int armor_num = static_cast<int>(armor_xyza_list.size());
  // 默认构造的 Target 一块装甲板都没有；原实现在这里会越界访问 armor_xyza_list[0]
  if (armor_num == 0) return AimPoint{};

  // 装甲板未发生过跳变时，只有当前装甲板的位置已知
  if (!target.jumped) return {true, 0, armor_xyza_list[0]};

  // 整车旋转中心的球坐标 yaw
  const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // delta_angle 为 0 时，该装甲板中心和整车中心的连线在世界系 xy 平面过原点
  std::vector<double> delta_angle_list(armor_num);
  for (int i = 0; i < armor_num; i++)
    delta_angle_list[i] = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);

  // 不考虑小陀螺：|w| <= 2 rad/s 走下面的锁定分支，转得更快才走 coming/leaving 分支。
  // 索引别写错 —— 状态排列是 x vx y vy z vz a w r l h，w 在 [7]，r 在 [8]。
  // 原 Aimer 这里误用了 ekf_x[8]（旋转半径 r，恒在 0.05-0.5），条件恒真，使该分支对非前哨站
  // 目标成了死代码；2026-10-03 合并进 Planner 时按原样移植，随后修正为 ekf_x[7]。
  if (std::abs(ekf_x[7]) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      // 每个规划周期最多被调用 70 次，这里只留 debug 级别
      tools::logger()->debug("[Planner] Empty id list!");
      return {false, 0, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈 45 度的装甲板之间来回切换
    if (id_list.size() > 1) {
      const int id0 = id_list[0], id1 = id_list[1];
      // 未处于锁定模式时，选择 delta_angle 绝对值较小的装甲板，进入锁定模式
      if (lock_id != id0 && lock_id != id1)
        lock_id = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;
      return {true, lock_id, armor_xyza_list[lock_id]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id = -1;
    return {true, id_list[0], armor_xyza_list[id_list[0]]};
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 小陀螺时一侧的装甲板不断出现、另一侧不断消失，前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle)
      return {true, i, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle)
      return {true, i, armor_xyza_list[i]};
  }

  return {false, 0, armor_xyza_list[0]};
}

std::optional<Planner::AimSolution> Planner::solve_xyza(
  const Eigen::Vector4d & xyza, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world) const
{
  const Eigen::Vector3d xyz = xyza.head<3>();
  // 保留 Planner 原有约定：水平距离取世界系，高度取云台系
  const double dist = xyz.head<2>().norm();
  const Eigen::Vector3d xyz_in_gimbal = R_gimbal2world.transpose() * xyz;
  const double azim = std::atan2(xyz_in_gimbal.y(), xyz_in_gimbal.x());

  const tools::Trajectory bullet_traj(bullet_speed, dist, xyz_in_gimbal.z());
  if (bullet_traj.unsolvable) return std::nullopt;

  AimSolution solution;
  solution.fly_time = bullet_traj.fly_time;
  solution.yaw = tools::limit_rad(azim + yaw_offset_);
  // 世界坐标系下 pitch 向上为负
  solution.pitch = -bullet_traj.pitch - pitch_offset_;
  return solution;
}

bool Planner::resolve_aim_time(
  Target base, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world, AimDecision & decision)
{
  // lock_id_ 每次规划只写回一次；迭代内部用局部副本，60 步前瞻不会污染持久状态
  int lock = lock_id_;
  int last_id = -1;
  bool selected = false;
  double used = 0.0;

  {
    const AimPoint p = choose_aim_point(base, lock);
    const auto armor_xyza_list = base.armor_xyza_list();
    const int id = p.valid ? p.id : nearest_armor_id(base);
    if (id < 0 || id >= static_cast<int>(armor_xyza_list.size())) return false;

    const auto solution = solve_xyza(armor_xyza_list[id], bullet_speed, R_gimbal2world);
    if (!solution) return false;

    used = solution->fly_time;
    last_id = id;
    selected = p.valid;
  }

  // 迭代求解飞行时间：每次都从 base 副本重新预测，误差不累积，只有标量 fly_time 在收敛
  for (int iter = 0; iter < 10; iter++) {
    Target predicted = base;
    predicted.predict(used);  // 绝对时刻 T1 + used

    const AimPoint p = choose_aim_point(predicted, lock);
    const auto armor_xyza_list = predicted.armor_xyza_list();
    const int id = p.valid ? p.id : (last_id >= 0 ? last_id : nearest_armor_id(predicted));
    if (id < 0 || id >= static_cast<int>(armor_xyza_list.size())) break;

    const auto solution = solve_xyza(armor_xyza_list[id], bullet_speed, R_gimbal2world);
    // 弹道不可解：保留上一轮的结果，不让一次失败毁掉整次规划
    if (!solution) break;

    last_id = id;
    selected = p.valid;
    if (std::abs(solution->fly_time - used) < 0.001) break;
    used = solution->fly_time;
  }

  if (last_id < 0) return false;

  lock_id_ = lock;
  decision.valid = selected;
  decision.id = last_id;
  decision.fly_time = used;
  return true;
}

bool Planner::get_trajectory(
  Target & target, double yaw0, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world,
  int lookahead_lock, int decision_id, Trajectory & out)
{
  // lookahead_lock 是按值传入的副本：60 列内部自己演进，不回写 lock_id_
  int lock = lookahead_lock;
  int last_id = decision_id;
  bool any_solved = false;
  Eigen::Matrix<double, 2, 1> last_yaw_pitch = Eigen::Matrix<double, 2, 1>::Zero();

  // 逐列重新过一遍击打策略。策略给不出结果时沿用上一列的装甲板，弹道不可解时沿用上一列的
  // 角度，两者都不让参考轨迹断掉。
  auto solve_column = [&]() -> Eigen::Matrix<double, 2, 1> {
    const auto armor_xyza_list = target.armor_xyza_list();
    if (armor_xyza_list.empty()) return last_yaw_pitch;

    const AimPoint p = choose_aim_point(target, lock);
    int id = p.valid ? p.id : last_id;
    if (id < 0 || id >= static_cast<int>(armor_xyza_list.size())) id = nearest_armor_id(target);
    if (id < 0 || id >= static_cast<int>(armor_xyza_list.size())) return last_yaw_pitch;

    const auto solution = solve_xyza(armor_xyza_list[id], bullet_speed, R_gimbal2world);
    if (!solution) return last_yaw_pitch;

    last_id = id;
    any_solved = true;
    last_yaw_pitch << solution->yaw, solution->pitch;
    return last_yaw_pitch;
  };

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = solve_column();

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HALF_HORIZON] = 0
  auto yaw_pitch = solve_column();

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    const auto yaw_pitch_next = solve_column();

    const auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    const auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    out.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return any_solved;
}

Plan Planner::plan(
  Target target, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world,
  std::optional<double> gimbal_yaw)
{
  {
    std::lock_guard<std::mutex> guard(debug_mutex_);
    debug_aim_valid_ = false;
  }

  const Eigen::VectorXd ekf_x = target.ekf_x();
  const double fire_thresh =
    std::abs(ekf_x[7]) > decision_speed_ ? fire_thresh_high_speed_ : fire_thresh_low_speed_;

  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) bullet_speed = 23;

  if (target.armor_xyza_list().empty()) return Plan{};

  // 1. 迭代收敛飞行时间，并定下决策时刻要打的那块装甲板
  AimDecision decision;
  if (!resolve_aim_time(target, bullet_speed, R_gimbal2world, decision)) return Plan{};

  Target aim_target = target;
  aim_target.predict(decision.fly_time);  // 推进到 T1 + fly_time，即子弹到达时刻

  const auto armor_xyza_list = aim_target.armor_xyza_list();
  if (decision.id < 0 || decision.id >= static_cast<int>(armor_xyza_list.size())) return Plan{};
  const Eigen::Vector4d aim_xyza = armor_xyza_list[decision.id];

  const auto aim_solution = solve_xyza(aim_xyza, bullet_speed, R_gimbal2world);
  if (!aim_solution) return Plan{};

  // 2. 参考轨迹：每一列都重新过一遍击打策略
  const double yaw0 = aim_solution->yaw;
  Trajectory traj;
  if (!get_trajectory(aim_target, yaw0, bullet_speed, R_gimbal2world, lock_id_, decision.id, traj))
    return Plan{};

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);
  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  // 5. 开火 = Planner 原有的"参考可达"判据 AND 原 Shooter 的"指令未跳变 + 云台已到位"
  const auto shoot_offset = 2;
  const bool track_ok = std::hypot(
                          traj(0, HALF_HORIZON + shoot_offset) -
                            yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset),
                          traj(2, HALF_HORIZON + shoot_offset) -
                            pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset)) < fire_thresh;
  plan.fire = track_ok;

  if (gimbal_yaw) {
    // 原 Shooter：指令未突变 + 云台已到达上一条指令 + 瞄准点有效
    bool settled = auto_fire_ && have_last_command_ && decision.valid;
    if (settled) {
      const double tolerance =
        std::hypot(ekf_x[0], ekf_x[2]) > judge_distance_ ? second_tolerance_ : first_tolerance_;
      settled = std::abs(last_target_yaw_ - plan.target_yaw) < tolerance * 2 &&
                std::abs(*gimbal_yaw - last_plan_yaw_) < tolerance;
    }
    plan.fire = plan.fire && settled;
  }

  if (plan.control) {
    last_plan_yaw_ = plan.yaw;
    last_target_yaw_ = plan.target_yaw;
    have_last_command_ = true;
  }

  {
    std::lock_guard<std::mutex> guard(debug_mutex_);
    debug_xyza_ = aim_xyza;
    debug_aim_valid_ = decision.valid;
  }

  return plan;
}

Plan Planner::plan(
  std::optional<Target> target, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world,
  std::optional<double> gimbal_yaw)
{
  return plan_at(target, std::chrono::steady_clock::now(), bullet_speed, R_gimbal2world, gimbal_yaw);
}

Plan Planner::plan_at(
  std::optional<Target> target, std::chrono::steady_clock::time_point now, double bullet_speed,
  const Eigen::Matrix3d & R_gimbal2world, std::optional<double> gimbal_yaw)
{
  if (!target.has_value()) {
    std::lock_guard<std::mutex> guard(debug_mutex_);
    debug_aim_valid_ = false;
    return Plan{};
  }

  const double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
  target->predict(now + std::chrono::microseconds(static_cast<int>(delay_time * 1e6)));

  return plan(*target, bullet_speed, R_gimbal2world, gimbal_yaw);
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;
}

}  // namespace auto_aim
