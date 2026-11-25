#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "tools/math/math_tools.hpp"
#include "tools/math/trajectory.hpp"
#include "tools/runtime/yaml_config.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
namespace
{
// Only ordinary four-plate targets use motion relative to the center line of sight.
// This rate is for selection, not for propagating the physical target orientation.
std::optional<double> selection_rate(const Eigen::VectorXd & x, int armor_num, ArmorName name)
{
  double rate = x[7];
  if (armor_num == 4 && name != ArmorName::outpost) {
    const double distance_squared = x[0] * x[0] + x[2] * x[2];
    if (!std::isfinite(distance_squared) || distance_squared <= 1e-12)
      return std::nullopt;
    rate -= (x[0] * x[3] - x[2] * x[1]) / distance_squared;
  }
  if (!std::isfinite(rate))
    return std::nullopt;
  return rate;
}
}  // namespace

const char * to_string(MpcMode mode)
{
  return mode == MpcMode::reference_smoothing ? "reference_smoothing" : "measured_state";
}

const char * to_string(PlanStatus status)
{
  switch (status) {
  case PlanStatus::not_run:
    return "not_run";
  case PlanStatus::ok:
    return "ok";
  case PlanStatus::no_target:
    return "no_target";
  case PlanStatus::empty_armor:
    return "empty_armor";
  case PlanStatus::feedback_invalid:
    return "feedback_invalid";
  case PlanStatus::timestamp_missing:
    return "timestamp_missing";
  case PlanStatus::feedback_future:
    return "feedback_future";
  case PlanStatus::feedback_stale:
    return "feedback_stale";
  case PlanStatus::state_nonfinite:
    return "state_nonfinite";
  case PlanStatus::rotation_invalid:
    return "rotation_invalid";
  case PlanStatus::state_frame_mismatch:
    return "state_frame_mismatch";
  case PlanStatus::pitch_singular:
    return "pitch_singular";
  case PlanStatus::propagated_state_invalid:
    return "propagated_state_invalid";
  case PlanStatus::trajectory_unavailable:
    return "trajectory_unavailable";
  case PlanStatus::mpc_failed:
    return "mpc_failed";
  case PlanStatus::exception:
    return "exception";
  }
  return "unknown";
}

PlannerConfig PlannerConfig::parse(const tools::YamlConfig & yaml)
{
  PlannerConfig config;
  const auto mode = yaml.get<std::string>("mpc_mode", "measured_state");
  if (mode == "reference_smoothing")
    config.mpc_mode = MpcMode::reference_smoothing;
  else if (mode != "measured_state")
    yaml.fail("mpc_mode", "must be measured_state or reference_smoothing");
  config.yaw = MpcConfig::parse(yaml, "yaw");
  config.pitch = MpcConfig::parse(yaml, "pitch");
  config.yaw_offset = yaml.get<double>("yaw_offset") / 57.3;
  config.pitch_offset = yaml.get<double>("pitch_offset") / 57.3;
  config.fire_thresh = yaml.get<double>("fire_thresh");
  config.fire_thresh_high_speed = yaml.get("fire_thresh_high_speed", config.fire_thresh);
  config.decision_speed = yaml.get<double>("decision_speed");
  config.high_speed_delay_time = yaml.get<double>("high_speed_delay_time");
  config.low_speed_delay_time = yaml.get<double>("low_speed_delay_time");
  config.comming_angle = yaml.get<double>("comming_angle") / 57.3;
  config.leaving_angle = yaml.get<double>("leaving_angle") / 57.3;
  config.first_tolerance = yaml.get<double>("first_tolerance") / 57.3;
  config.second_tolerance = yaml.get<double>("second_tolerance") / 57.3;
  config.first_tolerance_high_speed =
    yaml.get("first_tolerance_high_speed", config.first_tolerance * 57.3) / 57.3;
  config.second_tolerance_high_speed =
    yaml.get("second_tolerance_high_speed", config.second_tolerance * 57.3) / 57.3;
  config.fire_directional_tolerance_enabled = yaml.get("fire_directional_tolerance_enabled", false);
  config.fire_lag_tolerance_ratio = yaml.get("fire_lag_tolerance_ratio", 0.5);
  config.fire_direction_min_yaw_rate = yaml.get("fire_direction_min_yaw_rate", 0.05);
  if (!std::isfinite(config.fire_lag_tolerance_ratio) || config.fire_lag_tolerance_ratio <= 0 ||
      config.fire_lag_tolerance_ratio > 1)
    yaml.fail("fire_lag_tolerance_ratio", "must be finite and in (0, 1]");
  if (!std::isfinite(config.fire_direction_min_yaw_rate) || config.fire_direction_min_yaw_rate <= 0)
    yaml.fail("fire_direction_min_yaw_rate", "must be finite and positive");
  if (config.fire_directional_tolerance_enabled) {
    for (const auto * key : {"first_tolerance", "second_tolerance", "first_tolerance_high_speed",
                             "second_tolerance_high_speed"}) {
      const double value = yaml.get(key, 1.0);
      if (!std::isfinite(value) || value <= 0)
        yaml.fail(key, "must be finite and positive when directional tolerance is enabled");
    }
  }
  config.judge_distance = yaml.get<double>("judge_distance");
  config.auto_fire = yaml.get<bool>("auto_fire");

  config.gap_preaim_enabled = yaml.get("gap_preaim_enabled", config.gap_preaim_enabled);
  config.max_state_age = yaml.get("mpc_max_state_age_ms", 20.0) * 1e-3;
  if (!std::isfinite(config.max_state_age) || config.max_state_age <= 0)
    yaml.fail("mpc_max_state_age_ms", "must be finite and positive");
  if (!std::isfinite(config.low_speed_delay_time) || config.low_speed_delay_time < 0)
    yaml.fail("low_speed_delay_time", "must be finite and nonnegative");
  if (!std::isfinite(config.high_speed_delay_time) || config.high_speed_delay_time < 0)
    yaml.fail("high_speed_delay_time", "must be finite and nonnegative");
  YAML::Node replay_config;
  for (const auto * key : {"mpc_mode",
                           "yaw_offset",
                           "pitch_offset",
                           "fire_thresh",
                           "fire_thresh_high_speed",
                           "decision_speed",
                           "high_speed_delay_time",
                           "low_speed_delay_time",
                           "comming_angle",
                           "leaving_angle",
                           "first_tolerance",
                           "second_tolerance",
                           "first_tolerance_high_speed",
                           "second_tolerance_high_speed",
                           "fire_directional_tolerance_enabled",
                           "fire_lag_tolerance_ratio",
                           "fire_direction_min_yaw_rate",
                           "judge_distance",
                           "auto_fire",
                           "gap_preaim_enabled",
                           "mpc_max_state_age_ms",
                           "max_yaw_acc",
                           "max_pitch_acc",
                           "Q_yaw",
                           "Q_pitch",
                           "R_yaw",
                           "R_pitch",
                           "mpc_cost_yaw",
                           "mpc_cost_pitch",
                           "mpc_rho",
                           "mpc_max_iter",
                           "mpc_primal_tolerance",
                           "mpc_dual_tolerance"})
    if (yaml.has(key))
      replay_config[key] = YAML::Clone(yaml.node()[key]);
  config.replay_yaml = YAML::Dump(replay_config);

  return config;
}

Planner::Planner(const std::string & config_path)
    : Planner(PlannerConfig::parse(tools::YamlConfig(config_path)))
{
}

Planner::Planner(const PlannerConfig & config)
    : planner_config_yaml_(config.replay_yaml), yaw_solver_(config.yaw), pitch_solver_(config.pitch)
{
  mpc_mode_ = config.mpc_mode;
  yaw_offset_ = config.yaw_offset;
  pitch_offset_ = config.pitch_offset;
  fire_thresh_ = config.fire_thresh;
  fire_thresh_high_speed_ = config.fire_thresh_high_speed;
  decision_speed_ = config.decision_speed;
  high_speed_delay_time_ = config.high_speed_delay_time;
  low_speed_delay_time_ = config.low_speed_delay_time;
  comming_angle_ = config.comming_angle;
  leaving_angle_ = config.leaving_angle;
  first_tolerance_ = config.first_tolerance;
  second_tolerance_ = config.second_tolerance;
  first_tolerance_high_speed_ = config.first_tolerance_high_speed;
  second_tolerance_high_speed_ = config.second_tolerance_high_speed;
  fire_directional_tolerance_enabled_ = config.fire_directional_tolerance_enabled;
  fire_lag_tolerance_ratio_ = config.fire_lag_tolerance_ratio;
  fire_direction_min_yaw_rate_ = config.fire_direction_min_yaw_rate;
  judge_distance_ = config.judge_distance;
  auto_fire_ = config.auto_fire;
  gap_preaim_enabled_ = config.gap_preaim_enabled;
  max_state_age_ = config.max_state_age;
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
  if (armor_num == 0)
    return AimPoint{};

  const auto rate = selection_rate(ekf_x, armor_num, target.name);
  if (!rate)
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

  // 相对视线变化慢时锁板；普通四板车的横移可能抵消或反转来板方向。
  if (std::abs(*rate) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3)
        continue;
      id_list.push_back(i);
    }
    if (id_list.empty())
      return {false, 0, armor_xyza_list[0]};

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
    if ((*rate > 0 && delta_angle_list[i] < leaving_angle) ||
        (*rate < 0 && delta_angle_list[i] > -leaving_angle))
      return {true, i, armor_xyza_list[i]};
  }

  return {false, 0, armor_xyza_list[0]};
}

Planner::AimDecision Planner::choose_reference(const Target & target, int & lock,
                                               int previous_id) const
{
  AimDecision result;
  const auto armors = target.armor_xyza_list();
  const auto x = target.ekf_x();
  if (armors.empty()) {
    result.id = -1;
    return result;
  }
  const auto rate = selection_rate(x, static_cast<int>(armors.size()), target.name);
  if (!rate) {
    result.id = -1;
    return result;
  }
  const auto point = choose_aim_point(target, lock);
  result.lock = lock;
  result.id = point.valid ? point.id : (previous_id >= 0 ? previous_id : nearest_armor_id(target));
  if (result.id < 0 || result.id >= static_cast<int>(armors.size())) {
    result.id = -1;
    return result;
  }
  result.valid = point.valid;
  result.mode = point.valid ? AimMode::track : AimMode::fallback;
  result.reference_xyza = armors[result.id];
  if (point.valid || !gap_preaim_enabled_ || !target.jumped || armors.size() != 4 ||
      target.name == ArmorName::outpost || std::abs(*rate) <= 2.0)
    return result;

  const double phi = std::atan2(x[2], x[0]);
  const double direction = *rate > 0.0 ? 1.0 : -1.0;
  const double boundary = -direction * comming_angle_;
  double best_phase = 2 * M_PI;
  for (int i = 0; i < static_cast<int>(armors.size()); ++i) {
    const double delta = tools::limit_rad(armors[i][3] - phi);
    double phase = std::fmod(direction * (boundary - delta), 2 * M_PI);
    if (phase < 0.0)
      phase += 2 * M_PI;
    if (phase < best_phase) {
      best_phase = phase;
      result.id = i;
    }
  }
  const auto & incoming = armors[result.id];
  const double radius = std::hypot(incoming[0] - x[0], incoming[1] - x[2]);
  result.reference_xyza = {x[0] - radius * std::cos(phi + boundary),
                           x[2] - radius * std::sin(phi + boundary), incoming[2],
                           tools::limit_rad(phi + boundary)};
  result.mode = AimMode::preaim;
  // Remaining phase selects geometry only: no extra waiting time and no fire eligibility.
  return result;
}

std::optional<Planner::AimSolution>
Planner::solve_xyza(const Eigen::Vector4d & xyza, double bullet_speed,
                    const Eigen::Matrix3d & R_gimbal2world) const
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
                               const Eigen::Matrix3d & R_gimbal2world, int initial_lock,
                               AimDecision & decision) const
{
  int lock = initial_lock;
  AimDecision candidate = choose_reference(base, lock, -1);
  if (candidate.id < 0)
    return false;
  auto solution = solve_xyza(candidate.reference_xyza, bullet_speed, R_gimbal2world);
  if (!solution)
    return false;
  double used = solution->fly_time;
  double evaluated_time = used;
  bool converged = false;

  for (int iter = 0; iter < 10; ++iter) {
    const double current_time = used;
    Target predicted = base;
    predicted.predict(current_time);
    auto next = choose_reference(predicted, lock, candidate.id);
    if (next.id < 0)
      break;
    solution = solve_xyza(next.reference_xyza, bullet_speed, R_gimbal2world);
    if (!solution)
      break;
    const bool stable = next.id == candidate.id && next.mode == candidate.mode;
    candidate = next;
    evaluated_time = current_time;
    if (std::abs(solution->fly_time - current_time) < 0.001 && (!gap_preaim_enabled_ || stable)) {
      converged = true;
      used = current_time;
      break;
    }
    used = solution->fly_time;
  }

  if (!converged)
    used = evaluated_time;
  // Preserve the old reconstruction (including its fallback timing) when disabled.
  if (!gap_preaim_enabled_) {
    Target hit = base;
    hit.predict(used);
    candidate.reference_xyza = hit.armor_xyza_list()[candidate.id];
  }
  if (!converged) {
    candidate.mode = AimMode::fallback;
    if (gap_preaim_enabled_)
      candidate.valid = false;
  }
  candidate.converged = converged;
  candidate.lock = lock;
  candidate.fly_time = used;
  decision = candidate;
  return true;
}

bool Planner::get_trajectory(const Target & launch_target, double yaw_origin, double bullet_speed,
                             const Eigen::Matrix3d & rotation, Trajectory & out,
                             ReferenceTags & tags, AimDecision & first_decision,
                             Eigen::Vector4d & aim_xyza) const
{
  tags = {};
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
      selected_armor = decision.reference_xyza;
      solution = solve_xyza(selected_armor, bullet_speed, rotation);
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
    tags[k] = {
      decision.id,       decision.converged && (decision.valid || decision.mode == AimMode::preaim),
      decision.mode,     decision.reference_xyza,
      decision.fly_time, decision.valid,
      decision.converged};
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
  }
  if (gap_preaim_enabled_)
    fill_segment_reference_velocity(out, tags);
  else
    fill_pitch_reference_velocity(out, tags);
  return out.allFinite();
}

bool Planner::get_smoothing_trajectory(const Target & launch_target, double yaw_origin,
                                       double bullet_speed, const Eigen::Matrix3d & rotation,
                                       Trajectory & out, ReferenceTags & tags,
                                       AimDecision & decision, Eigen::Vector4d & aim_xyza) const
{
  tags = {};
  if (!resolve_aim_time(launch_target, bullet_speed, rotation, lock_id_, decision))
    return false;
  const auto center = solve_xyza(decision.reference_xyza, bullet_speed, rotation);
  if (!center)
    return false;
  decision.command_valid = decision.valid && decision.converged;
  aim_xyza = decision.reference_xyza;
  const auto write = [&](int k, const AimDecision & d, const AimSolution & solution,
                         double previous_yaw) {
    const double yaw = previous_yaw + tools::limit_rad(solution.yaw - previous_yaw);
    out(0, k) = yaw - yaw_origin;
    out(2, k) = solution.pitch;
    tags[k] = {d.id,
               d.converged && (d.valid || d.mode == AimMode::preaim),
               d.mode,
               d.reference_xyza,
               solution.fly_time,
               d.valid,
               d.converged};
    return yaw;
  };
  const double center_yaw = write(SMOOTHING_CENTER, decision, *center, yaw_origin);
  Target hit = launch_target;
  hit.predict(decision.fly_time);
  // The center decision owns the lock. Each side evolves its own copy; past samples
  // are independently predicted from the hit anchor, never integrated backwards in-place.
  for (int direction : {-1, 1}) {
    int lock = decision.lock;
    int previous_id = decision.id;
    double previous_yaw = center_yaw;
    for (int k = SMOOTHING_CENTER + direction; k >= 0 && k < HORIZON; k += direction) {
      Target sample = hit;
      sample.predict((k - SMOOTHING_CENTER) * DT);
      auto candidate = choose_reference(sample, lock, previous_id);
      const auto solution = candidate.id >= 0
                              ? solve_xyza(candidate.reference_xyza, bullet_speed, rotation)
                              : std::nullopt;
      if (!solution) {
        out(0, k) = out(0, k - direction);
        out(2, k) = out(2, k - direction);
        continue;
      }
      // The shared hit-time grid requires no new flight-time iteration at each side sample.
      candidate.converged = decision.converged;
      previous_yaw = write(k, candidate, *solution, previous_yaw);
      previous_id = candidate.id;
    }
  }
  for (int k = 0; k < HORIZON; ++k) {
    const int left = std::max(0, k - 1);
    const int right = std::min(HORIZON - 1, k + 1);
    out(1, k) = (out(0, right) - out(0, left)) / ((right - left) * DT);
  }
  if (gap_preaim_enabled_)
    fill_segment_reference_velocity(out, tags);
  else
    fill_pitch_reference_velocity(out, tags);
  return out.allFinite();
}

FireWindowCheck FireWindow::evaluate(double yaw_error, double pitch_error) const
{
  FireWindowCheck check;
  check.applied_yaw_tolerance = base_tolerance;
  if (motion.state == FireDirectionState::moving && std::isfinite(yaw_error))
    check.aligned_yaw_error = motion.direction * yaw_error;
  if (enabled && lag_ratio < 1 &&
      (motion.state == FireDirectionState::unknown ||
       (check.aligned_yaw_error && *check.aligned_yaw_error < 0)))
    check.applied_yaw_tolerance *= lag_ratio;
  if (!std::isfinite(yaw_error) || !std::isfinite(pitch_error) || !std::isfinite(base_tolerance) ||
      base_tolerance <= 0 || !std::isfinite(check.applied_yaw_tolerance) ||
      check.applied_yaw_tolerance <= 0)
    return check;
  const double normalized =
    std::hypot(yaw_error / check.applied_yaw_tolerance, pitch_error / base_tolerance);
  if (std::isfinite(normalized * normalized))
    check.normalized_error_squared = normalized * normalized;
  const bool inside_original = std::hypot(yaw_error, pitch_error) < base_tolerance;
  check.passed =
    inside_original && (check.applied_yaw_tolerance == base_tolerance || normalized < 1);
  return check;
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
  result.fire_window.base_tolerance = fire_tolerance;
  result.fire_window_check = result.fire_window.evaluate(-result.target_yaw, -result.target_pitch);
  result.fire = fire && result.fire_window_check.passed;
  return result;
}

bool Plan::aim_visible_at(std::chrono::steady_clock::time_point now) const
{
  return aim_debug.display_valid && planned_at != std::chrono::steady_clock::time_point{} &&
         now >= planned_at && now - planned_at <= 20ms;
}

void Planner::restore_history(const PlannerHistory & history)
{
  lock_id_ = history.lock_id;
  have_last_command_ = history.have_last_command;
  last_target_yaw_ = history.last_target_yaw;
  last_plan_at_ = history.last_plan_at;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed, const PlanningState & state,
                   PlanningTrace * trace)
{
  return plan_at(target, std::chrono::steady_clock::now(), bullet_speed, state, trace);
}

Plan Planner::plan_at(std::optional<Target> target, std::chrono::steady_clock::time_point now,
                      double bullet_speed, const PlanningState & state, PlanningTrace * trace)
{
  Plan result;
  result.mpc_mode = mpc_mode_;
  const bool smoothing = mpc_mode_ == MpcMode::reference_smoothing;
  const int state_index = smoothing ? SMOOTHING_CENTER : COMMAND_STEP;
  const int input_index = smoothing ? SMOOTHING_CENTER : 0;
  result.planned_at = now;
  result.initial_state = state;
  if (trace) {
    *trace = PlanningTrace{};
    trace->mpc_mode = mpc_mode_;
    trace->state_index = state_index;
    trace->input_index = input_index;
    trace->received_state = state;
    trace->yaw_config = yaw_solver_.config();
    trace->pitch_config = pitch_solver_.config();
    trace->bullet_speed = bullet_speed;
    trace->yaw_offset = yaw_offset_;
    trace->pitch_offset = pitch_offset_;
    trace->planner_config_yaml = planner_config_yaml_;
    trace->history = {lock_id_, have_last_command_, last_target_yaw_, last_plan_at_};
    if (target)
      trace->target_prediction = target->prediction_state();
    trace->target_available = target.has_value();
    if (target) {
      const auto x = target->ekf_x();
      for (int i = 0; i < std::min<int>(11, x.size()); ++i)
        trace->target_state[i] = x[i];
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
  if (!target)
    return fail(PlanStatus::no_target);
  if (!state.valid)
    return fail(PlanStatus::feedback_invalid);
  if (state.timestamp == std::chrono::steady_clock::time_point{})
    return fail(PlanStatus::timestamp_missing);
  if (age < 0.0)
    return fail(PlanStatus::feedback_future);
  if (age > max_state_age_)
    return fail(PlanStatus::feedback_stale);
  if (!measured.allFinite() || !state.R_gimbal2world.allFinite())
    return fail(PlanStatus::state_nonfinite);
  if (target->armor_xyza_list().empty())
    return fail(PlanStatus::empty_armor);

  const auto & rotation = state.R_gimbal2world;
  if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-5 ||
      std::abs(rotation.determinant() - 1.0) > 1e-5)
    return fail(PlanStatus::rotation_invalid);
  const auto angles = tools::eulers(rotation, 2, 0, 1);
  if (std::abs(std::remainder(angles[0] - state.yaw, 2 * M_PI)) > 1e-4 ||
      std::abs(angles[1] - state.pitch) > 1e-4)
    return fail(PlanStatus::state_frame_mismatch);
  if (std::abs(std::cos(state.pitch)) < 0.05)
    return fail(PlanStatus::pitch_singular);

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
  if (trace)
    trace->bullet_speed = bullet_speed;
  const auto ekf_x = target->ekf_x();
  const bool high_speed = std::abs(ekf_x[7]) > decision_speed_;
  const double delay = high_speed ? high_speed_delay_time_ : low_speed_delay_time_;
  if (trace)
    trace->launch_delay = delay;
  target->predict(now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(delay)));
  Trajectory reference;
  ReferenceTags reference_tags;
  AimDecision decision;
  Eigen::Vector4d aim_xyza;
  const bool reference_ready =
    smoothing ? get_smoothing_trajectory(*target, initial.yaw, bullet_speed, initial.R_gimbal2world,
                                         reference, reference_tags, decision, aim_xyza)
              : get_trajectory(*target, initial.yaw, bullet_speed, initial.R_gimbal2world,
                               reference, reference_tags, decision, aim_xyza);
  if (!reference_ready)
    return fail(PlanStatus::trajectory_unavailable);

  result.aim_debug.xyza = aim_xyza;
  result.aim_debug.armor_id = decision.id;
  result.aim_debug.mode = decision.mode;
  const int decision_index = smoothing ? SMOOTHING_CENTER : 0;
  result.aim_debug.candidate_available = reference_tags[decision_index].armor_id >= 0;
  result.aim_debug.selection_valid = decision.valid;
  result.aim_debug.ballistic_converged = decision.converged;
  const Eigen::Vector2d yaw_x0 = smoothing ? Eigen::Vector2d(reference.block<2, 1>(0, 0))
                                           : Eigen::Vector2d(0.0, initial.yaw_vel);
  const Eigen::Vector2d pitch_x0 = smoothing ? Eigen::Vector2d(reference.block<2, 1>(2, 0))
                                             : Eigen::Vector2d(initial.pitch, initial.pitch_vel);
  if (trace) {
    trace->reference = reference;
    trace->reference_tags = reference_tags;
    trace->reference_available = true;
    trace->yaw_x0 = yaw_x0;
    trace->pitch_x0 = pitch_x0;
    trace->center_fly_time = smoothing ? decision.fly_time : 0.0;
  }

  result.yaw_mpc = yaw_solver_.solve(yaw_x0, reference.topRows<2>());
  result.pitch_mpc = pitch_solver_.solve(pitch_x0, reference.bottomRows<2>());
  if (!result.yaw_mpc.solved || !result.pitch_mpc.solved)
    return fail(PlanStatus::mpc_failed);

  result.status = PlanStatus::ok;
  result.control = true;
  result.absolute_yaw = yaw_solver_.states()(0, state_index) + initial.yaw;
  result.absolute_pitch = pitch_solver_.states()(0, state_index);
  result.absolute_target_yaw = reference(0, state_index) + initial.yaw;
  result.absolute_target_pitch = reference(2, state_index);
  result.mpc_feedforward = {yaw_solver_.states()(1, state_index),
                            yaw_solver_.inputs()(0, input_index) * yaw_solver_.config().max_acc,
                            pitch_solver_.states()(1, state_index),
                            pitch_solver_.inputs()(0, input_index) *
                              pitch_solver_.config().max_acc};
  if (!smoothing) {
    result.yaw_vel = result.mpc_feedforward[0];
    result.yaw_acc = result.mpc_feedforward[1];
    result.pitch_vel = result.mpc_feedforward[2];
    result.pitch_acc = result.mpc_feedforward[3];
  }
  result.valid_at = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(COMMAND_STEP * DT));
  result.command_at = smoothing ? now : result.valid_at;
  const bool far_distance = std::hypot(ekf_x[0], ekf_x[2]) > judge_distance_;
  result.fire_tolerance =
    high_speed ? (far_distance ? second_tolerance_high_speed_ : first_tolerance_high_speed_)
               : (far_distance ? second_tolerance_ : first_tolerance_);
  const double tracking_threshold = high_speed ? fire_thresh_high_speed_ : fire_thresh_;
  const bool reachable =
    std::hypot(reference(0, state_index) - yaw_solver_.states()(0, state_index),
               reference(2, state_index) - pitch_solver_.states()(0, state_index)) <
    tracking_threshold;
  const auto window_at = [&](int index) {
    FireWindow window;
    window.enabled = fire_directional_tolerance_enabled_;
    window.base_tolerance = result.fire_tolerance;
    window.lag_ratio = fire_lag_tolerance_ratio_;
    if (window.enabled)
      window.motion =
        fire_yaw_motion(reference, reference_tags, index, fire_direction_min_yaw_rate_);
    return window;
  };
  result.settled_reference_index = decision_index;
  result.fire_reference_index = state_index;
  result.settled_window = window_at(decision_index);
  result.fire_window = window_at(state_index);
  result.settled_check = result.settled_window.evaluate(
    -reference(0, decision_index), initial.pitch - reference(2, decision_index));
  const bool settled = result.settled_check.passed;
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

}  // namespace auto_aim
