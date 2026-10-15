#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
const char *to_string(PlanStatus status)
{
  switch (status) {
    case PlanStatus::not_run: return "not_run";
    case PlanStatus::ok: return "ok";
    case PlanStatus::no_target: return "no_target";
    case PlanStatus::empty_armor: return "empty_armor";
    case PlanStatus::feedback_invalid: return "feedback_invalid";
    case PlanStatus::timestamp_missing: return "timestamp_missing";
    case PlanStatus::feedback_future: return "feedback_future";
    case PlanStatus::feedback_stale: return "feedback_stale";
    case PlanStatus::state_nonfinite: return "state_nonfinite";
    case PlanStatus::rotation_invalid: return "rotation_invalid";
    case PlanStatus::state_frame_mismatch: return "state_frame_mismatch";
    case PlanStatus::pitch_singular: return "pitch_singular";
    case PlanStatus::propagated_state_invalid: return "propagated_state_invalid";
    case PlanStatus::trajectory_unavailable: return "trajectory_unavailable";
    case PlanStatus::mpc_failed: return "mpc_failed";
    case PlanStatus::exception: return "exception";
  }
  return "unknown";
}

Planner::Planner(const std::string &config_path)
  : yaw_solver_(MpcConfig::load(config_path, "yaw")),
    pitch_solver_(MpcConfig::load(config_path, "pitch"))
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
  if (yaml["mpc_max_state_age_ms"])
    max_state_age_ = yaml["mpc_max_state_age_ms"].as<double>() * 1e-3;
  if (!std::isfinite(max_state_age_) || max_state_age_ <= 0.0)
    throw std::invalid_argument("mpc_max_state_age_ms must be positive");
  if (!std::isfinite(low_speed_delay_time_) || low_speed_delay_time_ < 0.0 ||
      !std::isfinite(high_speed_delay_time_) || high_speed_delay_time_ < 0.0)
    throw std::invalid_argument("Planner delay times must be finite and nonnegative");

}

int Planner::nearest_armor_id(const Target &target) const
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

AimPoint Planner::choose_aim_point(const Target &target, int &lock_id) const
{
  const Eigen::VectorXd ekf_x = target.ekf_x();
  const std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  const int armor_num = static_cast<int>(armor_xyza_list.size());
  // 默认构造的 Target 一块装甲板都没有；原实现在这里会越界访问 armor_xyza_list[0]
  if (armor_num == 0)
    return AimPoint{};

  // 装甲板未发生过跳变时，只有当前装甲板的位置已知
  if (!target.jumped)
    return {true, 0, armor_xyza_list[0]};

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
      if (std::abs(delta_angle_list[i]) > 60 / 57.3)
        continue;
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
    if (std::abs(delta_angle_list[i]) > coming_angle)
      continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle)
      return {true, i, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle)
      return {true, i, armor_xyza_list[i]};
  }

  return {false, 0, armor_xyza_list[0]};
}

std::optional<Planner::AimSolution> Planner::solve_xyza(const Eigen::Vector4d &xyza,
                                                        double bullet_speed,
                                                        const Eigen::Matrix3d &R_gimbal2world) const
{
  const Eigen::Vector3d xyz = xyza.head<3>();
  // 重力沿世界系 z 轴，弹道的水平距离和高度必须使用同一世界参考系。
  const double dist = xyz.head<2>().norm();
  const tools::Trajectory bullet_traj(bullet_speed, dist, xyz.z());
  if (bullet_traj.unsolvable || !std::isfinite(dist) || dist < 1e-6 ||
      !std::isfinite(bullet_traj.fly_time) || !std::isfinite(bullet_traj.pitch))
    return std::nullopt;

  // 先在世界系求出包含重力补偿的发射方向，再变换到当前云台系。
  // 云台的关节轴为 Z(yaw)-X(pitch)-Y(roll)，其默认瞄准方向是 -Y：
  //   dir_g = Rz(yaw) Rx(pitch) (-Y)
  // 因此 pitch 必须从变换后的方向分解，不能直接把世界系弹道仰角当作 X 轴角度。
  const Eigen::Vector2d horizontal = xyz.head<2>();
  const double cos_pitch = std::cos(bullet_traj.pitch);
  const Eigen::Vector3d ballistic_dir_world{horizontal.x() / dist * cos_pitch,
                                            horizontal.y() / dist * cos_pitch,
                                            std::sin(bullet_traj.pitch)};
  const Eigen::Vector3d ballistic_dir_gimbal = R_gimbal2world.transpose() * ballistic_dir_world;

  // 云台 -Y 为零 yaw，绕 X 轴正方向抬头为负（与控制帧约定一致）。
  const double azim = std::atan2(ballistic_dir_gimbal.x(), -ballistic_dir_gimbal.y());
  const double pitch = std::atan2(-ballistic_dir_gimbal.z(),
                                  std::hypot(ballistic_dir_gimbal.x(), ballistic_dir_gimbal.y()));

  AimSolution solution;
  solution.fly_time = bullet_traj.fly_time;
  // Preserve the calibrated local offset as a direction, then express it in absolute joint angles.
  const Eigen::Vector3d corrected =
      R_gimbal2world * (Eigen::AngleAxisd(azim + yaw_offset_, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch - pitch_offset_, Eigen::Vector3d::UnitX()) *
                        Eigen::Vector3d(0.0, -1.0, 0.0));
  solution.yaw = std::atan2(corrected.x(), -corrected.y());
  solution.pitch = std::atan2(-corrected.z(), std::hypot(corrected.x(), corrected.y()));
  return solution;
}

bool Planner::resolve_aim_time(Target base, double bullet_speed,
                               const Eigen::Matrix3d &R_gimbal2world, int initial_lock,
                               AimDecision &decision) const
{
  // lock_id_ 每次规划只写回一次；迭代内部用局部副本，60 步前瞻不会污染持久状态
  int lock = initial_lock;
  int last_id = -1;
  bool selected = false;
  double used = 0.0;
  double evaluated_time = 0.0;
  bool converged = false;

  {
    const AimPoint p = choose_aim_point(base, lock);
    const auto armor_xyza_list = base.armor_xyza_list();
    const int id = p.valid ? p.id : nearest_armor_id(base);
    if (id < 0 || id >= static_cast<int>(armor_xyza_list.size()))
      return false;

    const auto solution = solve_xyza(armor_xyza_list[id], bullet_speed, R_gimbal2world);
    if (!solution)
      return false;

    used = solution->fly_time;
    evaluated_time = used;
    last_id = id;
    selected = p.valid;
  }

  // 迭代求解飞行时间：每次都从 base 副本重新预测，误差不累积，只有标量 fly_time 在收敛
  for (int iter = 0; iter < 10; iter++) {
    const double current_time = used;
    Target predicted = base;
    predicted.predict(current_time); // 绝对时刻 T1 + current_time

    const AimPoint p = choose_aim_point(predicted, lock);
    const auto armor_xyza_list = predicted.armor_xyza_list();
    const int id = p.valid ? p.id : (last_id >= 0 ? last_id : nearest_armor_id(predicted));
    if (id < 0 || id >= static_cast<int>(armor_xyza_list.size()))
      break;

    const auto solution = solve_xyza(armor_xyza_list[id], bullet_speed, R_gimbal2world);
    // 弹道不可解：保留上一轮的结果，不让一次失败毁掉整次规划
    if (!solution)
      break;

    last_id = id;
    selected = p.valid;
    evaluated_time = current_time;
    if (std::abs(solution->fly_time - current_time) < 0.001) {
      converged = true;
      used = current_time;
      break;
    }
    used = solution->fly_time;
  }

  if (last_id < 0)
    return false;

  // On exhaustion or a failed iteration, keep the last internally consistent candidate for
  // tracking, but do not publish its lock as a settled decision or allow it to fire.
  if (!converged)
    used = evaluated_time;
  decision.valid = selected;
  decision.converged = converged;
  decision.id = last_id;
  decision.lock = lock;
  decision.fly_time = used;
  return true;
}

bool Planner::get_trajectory(const Target &launch_target, double yaw_origin, double bullet_speed,
                             const Eigen::Matrix3d &rotation, Trajectory &out,
                             AimDecision &first_decision, Eigen::Vector4d &aim_xyza) const
{
  int lock = lock_id_;
  double previous_yaw = yaw_origin;
  for (int k = 0; k < HORIZON; ++k) {
    Target base = launch_target;
    if (k > 0)
      base.predict(k * DT);
    AimDecision decision;
    std::optional<AimSolution> solution;
    Eigen::Vector4d selected_armor = Eigen::Vector4d::Zero();
    if (resolve_aim_time(base, bullet_speed, rotation, lock, decision)) {
      Target hit = base;
      hit.predict(decision.fly_time);
      const auto armors = hit.armor_xyza_list();
      if (decision.id >= 0 && decision.id < static_cast<int>(armors.size())) {
        selected_armor = armors[decision.id];
        solution = solve_xyza(selected_armor, bullet_speed, rotation);
      }
    }
    if (!solution) {
      if (k == 0)
        return false;
      out(0, k) = out(0, k - 1);
      out(2, k) = out(2, k - 1);
      if (k <= COMMAND_STEP)
        first_decision.command_valid = false;
      continue;
    }
    const double yaw = previous_yaw + tools::limit_rad(solution->yaw - previous_yaw);
    out(0, k) = yaw - yaw_origin;
    out(2, k) = solution->pitch;
    previous_yaw = yaw;
    if (decision.converged)
      lock = decision.lock;
    if (k == 0) {
      first_decision = decision;
      first_decision.command_valid = decision.valid && decision.converged;
      aim_xyza = selected_armor;
    }
    if (k > 0 && k <= COMMAND_STEP)
      first_decision.command_valid =
          first_decision.command_valid && decision.valid && decision.converged;
  }
  for (int k = 0; k < HORIZON; ++k) {
    const int left = std::max(0, k - 1);
    const int right = std::min(HORIZON - 1, k + 1);
    const double span = (right - left) * DT;
    out(1, k) = (out(0, right) - out(0, left)) / span;
    out(3, k) = (out(2, right) - out(2, left)) / span;
  }
  return out.allFinite();
}

Plan Plan::relative_to(double measured_yaw, double measured_pitch) const
{
  if (!control || !std::isfinite(measured_yaw) || !std::isfinite(measured_pitch))
    return Plan{};
  Plan result = *this;
  result.yaw = tools::limit_rad(absolute_yaw - measured_yaw);
  result.pitch = absolute_pitch - measured_pitch;
  result.target_yaw = tools::limit_rad(absolute_target_yaw - measured_yaw);
  result.target_pitch = absolute_target_pitch - measured_pitch;
  result.fire = fire && std::hypot(result.target_yaw, result.target_pitch) < fire_tolerance;
  return result;
}

bool Plan::aim_visible_at(std::chrono::steady_clock::time_point now) const
{
  return aim_debug.display_valid && planned_at != std::chrono::steady_clock::time_point{} &&
         now >= planned_at && now - planned_at <= 20ms;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed, const PlanningState &state,
                   PlanningTrace *trace)
{
  return plan_at(target, std::chrono::steady_clock::now(), bullet_speed, state, trace);
}

Plan Planner::plan_at(std::optional<Target> target, std::chrono::steady_clock::time_point now,
                      double bullet_speed, const PlanningState &state, PlanningTrace *trace)
{
  Plan result;
  result.planned_at = now;
  result.initial_state = state;
  if (trace) {
    *trace = PlanningTrace{};
    trace->received_state = state;
    trace->yaw_config = yaw_solver_.config();
    trace->pitch_config = pitch_solver_.config();
    trace->bullet_speed = bullet_speed;
    trace->yaw_offset = yaw_offset_;
    trace->pitch_offset = pitch_offset_;
    trace->target_available = target.has_value();
    if (target) {
      const auto x = target->ekf_x();
      for (int i = 0; i < std::min<int>(11, x.size()); ++i) trace->target_state[i] = x[i];
    }
  }
  auto fail = [&](PlanStatus status) {
    result.status = status;
    have_last_command_ = false;
    lock_id_ = -1;
    return result;
  };
  const double age = std::chrono::duration<double>(now - state.timestamp).count();
  result.feedback_age_ms = age * 1e3;
  const Eigen::Vector4d measured{state.yaw, state.yaw_vel, state.pitch, state.pitch_vel};
  if (!target) return fail(PlanStatus::no_target);
  if (!state.valid) return fail(PlanStatus::feedback_invalid);
  if (state.timestamp == std::chrono::steady_clock::time_point{})
    return fail(PlanStatus::timestamp_missing);
  if (age < 0.0) return fail(PlanStatus::feedback_future);
  if (age > max_state_age_) return fail(PlanStatus::feedback_stale);
  if (!measured.allFinite() || !state.R_gimbal2world.allFinite())
    return fail(PlanStatus::state_nonfinite);
  if (target->armor_xyza_list().empty()) return fail(PlanStatus::empty_armor);

  const auto &rotation = state.R_gimbal2world;
  if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-5 ||
      std::abs(rotation.determinant() - 1.0) > 1e-5)
    return fail(PlanStatus::rotation_invalid);
  const auto angles = tools::eulers(rotation, 2, 0, 1);
  if (std::abs(std::remainder(angles[0] - state.yaw, 2 * M_PI)) > 1e-4 ||
      std::abs(angles[1] - state.pitch) > 1e-4)
    return fail(PlanStatus::state_frame_mismatch);
  if (std::abs(std::cos(state.pitch)) < 0.05) return fail(PlanStatus::pitch_singular);

  PlanningState initial = state;
  initial.yaw += state.yaw_vel * age;
  initial.pitch += state.pitch_vel * age;
  if (!std::isfinite(initial.yaw) || !std::isfinite(initial.pitch) ||
      std::abs(initial.pitch) > std::acos(0.05))
    return fail(PlanStatus::propagated_state_invalid);
  initial.yaw = std::remainder(initial.yaw, 2 * M_PI);
  initial.timestamp = now;
  // Keep the sampled roll while propagating the two measured joint coordinates.
  initial.R_gimbal2world = (Eigen::AngleAxisd(initial.yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(initial.pitch, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitY()))
                               .toRotationMatrix();
  result.initial_state = initial;
  if (!std::isfinite(bullet_speed) || bullet_speed < 10 || bullet_speed > 25)
    bullet_speed = 23;
  if (trace) trace->bullet_speed = bullet_speed;
  const auto ekf_x = target->ekf_x();
  const bool high_speed = std::abs(ekf_x[7]) > decision_speed_;
  const double delay = high_speed ? high_speed_delay_time_ : low_speed_delay_time_;
  target->predict(now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(delay)));
  Trajectory reference;
  AimDecision decision;
  Eigen::Vector4d aim_xyza;
  if (!get_trajectory(*target, initial.yaw, bullet_speed, initial.R_gimbal2world, reference,
                      decision, aim_xyza))
    return fail(PlanStatus::trajectory_unavailable);

  result.aim_debug.xyza = aim_xyza;
  result.aim_debug.armor_id = decision.id;
  result.aim_debug.candidate_available = true;
  result.aim_debug.selection_valid = decision.valid;
  result.aim_debug.ballistic_converged = decision.converged;
  if (trace) {
    trace->reference = reference;
    trace->reference_available = true;
  }

  result.yaw_mpc =
      yaw_solver_.solve({0.0, initial.yaw_vel}, reference.topRows<2>());
  result.pitch_mpc = pitch_solver_.solve({initial.pitch, initial.pitch_vel},
                                        reference.bottomRows<2>());
  if (!result.yaw_mpc.solved || !result.pitch_mpc.solved)
    return fail(PlanStatus::mpc_failed);

  result.status = PlanStatus::ok;
  result.control = true;
  result.absolute_yaw = yaw_solver_.states()(0, COMMAND_STEP) + initial.yaw;
  result.absolute_pitch = pitch_solver_.states()(0, COMMAND_STEP);
  result.absolute_target_yaw = reference(0, COMMAND_STEP) + initial.yaw;
  result.absolute_target_pitch = reference(2, COMMAND_STEP);
  result.yaw_vel = yaw_solver_.states()(1, COMMAND_STEP);
  result.pitch_vel = pitch_solver_.states()(1, COMMAND_STEP);
  result.yaw_acc = yaw_solver_.inputs()(0, 0) * yaw_solver_.config().max_acc;
  result.pitch_acc = pitch_solver_.inputs()(0, 0) * pitch_solver_.config().max_acc;
  result.valid_at = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(COMMAND_STEP * DT));
  result.fire_tolerance =
      std::hypot(ekf_x[0], ekf_x[2]) > judge_distance_ ? second_tolerance_ : first_tolerance_;
  const double threshold = high_speed ? fire_thresh_high_speed_ : fire_thresh_low_speed_;
  const bool reachable =
      std::hypot(reference(0, COMMAND_STEP) - yaw_solver_.states()(0, COMMAND_STEP),
                 reference(2, COMMAND_STEP) - pitch_solver_.states()(0, COMMAND_STEP)) < threshold;
  const bool settled =
      std::hypot(reference(0, 0), reference(2, 0) - initial.pitch) < result.fire_tolerance;
  const double since_last = std::chrono::duration<double>(now - last_plan_at_).count();
  result.fire = auto_fire_ && state.rates_measured && have_last_command_ && since_last > 0.0 &&
                since_last <= max_state_age_ && decision.command_valid && reachable && settled &&
                std::abs(tools::limit_rad(last_target_yaw_ - result.absolute_target_yaw)) <
                    2 * result.fire_tolerance;

  if (decision.converged)
    lock_id_ = decision.lock;
  have_last_command_ = true;
  last_target_yaw_ = result.absolute_target_yaw;
  last_plan_at_ = now;
  result.aim_debug.display_valid = decision.valid && decision.converged;
  result = result.relative_to(initial.yaw, initial.pitch);
  return result;
}

} // namespace auto_aim
