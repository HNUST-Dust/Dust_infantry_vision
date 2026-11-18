#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tools/math/math_tools.hpp"
#include "tools/runtime/yaml_config.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
namespace
{
bool supports_axis_strategy(const TargetPredictionState & target)
{
  return target.armor_num == 4 && target.name != ArmorName::outpost &&
         target.name != ArmorName::base;
}

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
  config.yaw = MpcConfig::parse(yaml, "yaw");
  config.pitch = MpcConfig::parse(yaml, "pitch");
  config.yaw_offset = yaml.get<double>("yaw_offset") / 57.3;
  config.pitch_offset = yaml.get<double>("pitch_offset") / 57.3;
  config.fire_thresh = yaml.get<double>("fire_thresh");
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
  const auto axis = yaml.section("planner_axis_strategy");
  const auto runtime = yaml.section("runtime");
  config.axis_strategy_enabled = axis.get("enabled", false);
  config.pitch_follow_yaw_window = axis.get("pitch_follow_yaw_window", false);
  config.yaw_strategy = AxisStrategyConfig::parse(axis.section("yaw"));
  config.pitch_strategy = AxisStrategyConfig::parse(axis.section("pitch"));
  if (config.axis_strategy_enabled) {
    const auto timeout = [&](const char * key, std::chrono::milliseconds fallback) {
      const int ms = runtime.get(key, static_cast<int>(fallback.count()));
      if (ms <= 0)
        runtime.fail(key, "must be positive");
      return std::chrono::milliseconds(ms);
    };
    config.control_timeout = timeout("control_timeout_ms", config.control_timeout);
    config.fire_timeout = timeout("fire_timeout_ms", config.fire_timeout);
  }
  YAML::Node replay_config;
  if (yaml.has("planner_axis_strategy"))
    replay_config["planner_axis_strategy"] = YAML::Clone(yaml.node()["planner_axis_strategy"]);
  if (config.axis_strategy_enabled) {
    replay_config["runtime"]["control_timeout_ms"] = config.control_timeout.count();
    replay_config["runtime"]["fire_timeout_ms"] = config.fire_timeout.count();
  }
  for (const auto * key : {"yaw_offset",
                           "pitch_offset",
                           "fire_thresh",
                           "decision_speed",
                           "high_speed_delay_time",
                           "low_speed_delay_time",
                           "comming_angle",
                           "leaving_angle",
                           "first_tolerance",
                           "second_tolerance",
                           "first_tolerance_high_speed",
                           "second_tolerance_high_speed",
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
  yaw_offset_ = config.yaw_offset;
  pitch_offset_ = config.pitch_offset;
  fire_thresh_ = config.fire_thresh;
  decision_speed_ = config.decision_speed;
  high_speed_delay_time_ = config.high_speed_delay_time;
  low_speed_delay_time_ = config.low_speed_delay_time;
  comming_angle_ = config.comming_angle;
  leaving_angle_ = config.leaving_angle;
  first_tolerance_ = config.first_tolerance;
  second_tolerance_ = config.second_tolerance;
  first_tolerance_high_speed_ = config.first_tolerance_high_speed;
  second_tolerance_high_speed_ = config.second_tolerance_high_speed;
  judge_distance_ = config.judge_distance;
  auto_fire_ = config.auto_fire;
  gap_preaim_enabled_ = config.gap_preaim_enabled;
  max_state_age_ = config.max_state_age;
  axis_strategy_enabled_ = config.axis_strategy_enabled;
  pitch_follow_yaw_window_ = config.pitch_follow_yaw_window;
  yaw_strategy_ = config.yaw_strategy;
  pitch_strategy_ = config.pitch_strategy;
  control_timeout_ = config.control_timeout;
  fire_timeout_ = config.fire_timeout;
}

bool Planner::allowed_armor(int id) const
{
  return axis_history_.height_group < 0 || id % 2 == axis_history_.height_group;
}

int Planner::nearest_armor_id(const Target & target) const
{
  const auto armor_xyza_list = target.armor_xyza_list();
  int best = -1;
  double min_dist = 1e10;
  for (int i = 0; i < static_cast<int>(armor_xyza_list.size()); i++) {
    if (!allowed_armor(i))
      continue;
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
    return {allowed_armor(0), 0, armor_xyza_list[0]};

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
      if (!allowed_armor(i) || std::abs(delta_angle_list[i]) > 60 / 57.3)
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
    if (!allowed_armor(i) || std::abs(delta_angle_list[i]) > coming_angle)
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
  result.id = point.id;
  if (!point.valid)
    result.id =
      previous_id >= 0 && allowed_armor(previous_id) ? previous_id : nearest_armor_id(target);
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
    if (!allowed_armor(i))
      continue;
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
  result.reference_is_real = false;
  // Remaining phase selects geometry only: no extra waiting time and no fire eligibility.
  return result;
}

std::optional<AimSolution> Planner::solve_xyza(const Eigen::Vector4d & xyza, double bullet_speed,
                                               const Eigen::Matrix3d & rotation) const
{
  return solve_ballistic_aim(xyza, bullet_speed, rotation, yaw_offset_, pitch_offset_);
}

std::optional<AimSolution> Planner::smooth_aim(const TargetPredictionState & target, int group,
                                               double bullet_speed,
                                               const Eigen::Matrix3d & rotation) const
{
  const auto & x = target.x;
  const double radius = group < 0 ? 0.0 : x[8] + (group == 1 ? x[9] : 0.0);
  const double height = x[4] + (group == 1 ? x[10] : 0.0);
  double dt = 0.0;
  for (int iteration = 0; iteration < 10; ++iteration) {
    const double cx = x[0] + dt * x[1], cy = x[2] + dt * x[3];
    const double distance = std::hypot(cx, cy);
    if (!std::isfinite(radius) || radius < 0.0 || !(distance > radius + 1e-6))
      return std::nullopt;
    const double scale = 1.0 - radius / distance;
    const auto aim =
      solve_xyza({cx * scale, cy * scale, height + dt * x[5], 0.0}, bullet_speed, rotation);
    if (!aim || std::abs(aim->fly_time - dt) < 1e-6)
      return aim;
    dt = aim->fly_time;
  }
  return std::nullopt;
}

void Planner::update_axis_modes(const TargetPredictionState & target, const PlanningState & initial,
                                double bullet_speed)
{
  const auto now = initial.timestamp;
  if (!axis_history_.has_target || axis_history_.generation != target.generation) {
    axis_history_.has_target = true;
    axis_history_.generation = target.generation;
    axis_history_.height_group = -1;
    auto reset = [&](AxisStrategyState & axis, const AxisStrategyConfig & config) {
      axis.reduced = false;
      axis.condition_since = axis.last_observation = {};
      axis.needs_bridge = config.enabled && axis.reference_at != PlannerTime{};
    };
    reset(axis_history_.yaw, yaw_strategy_);
    reset(axis_history_.pitch, pitch_strategy_);
    lock_id_ = -1;
    have_last_command_ = false;
  }
  if (!supports_axis_strategy(target))
    return;

  const double speed = std::abs(target.x[7]);
  auto & yaw = axis_history_.yaw;
  auto & pitch = axis_history_.pitch;
  if (yaw.confirm(yaw_strategy_, speed, now, target.observed_at, control_timeout_)) {
    yaw.reduced = !yaw.reduced;
    yaw.needs_bridge = true;
    yaw.condition_since = {};
  }
  if (!pitch.confirm(pitch_strategy_, speed, now, target.observed_at, control_timeout_))
    return;
  if (pitch.reduced) {
    axis_history_.height_group = -1;
  } else {
    const double current =
      pitch.reference_at != PlannerTime{} &&
          std::chrono::duration<double>(now - pitch.reference_at).count() <= max_state_age_
        ? pitch.previous_at(now).angle
        : initial.pitch;
    int group = -1;
    double smallest_error = 0.0;
    for (int i = 0; i < (target.jumped ? 2 : 1); ++i) {
      const auto aim = smooth_aim(target, i, bullet_speed, initial.R_gimbal2world);
      if (!aim)
        continue;
      const double error = std::abs(aim->pitch - current);
      if (group == -1 || error < smallest_error) {
        group = i;
        smallest_error = error;
      }
    }
    if (group == -1)
      return;
    axis_history_.height_group = group;
  }
  pitch.reduced = !pitch.reduced;
  pitch.needs_bridge = true;
  pitch.condition_since = {};
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
  const bool center = axis_history_.yaw.reduced;
  const int group = axis_history_.height_group;
  const bool require_same_armor =
    axis_strategy_enabled_ && supports_axis_strategy(launch_target.prediction_state());
  for (int k = 0; k < HORIZON; ++k) {
    Target base = launch_target;
    if (k > 0)
      base.predict(k * DT);
    AimDecision decision;
    std::optional<AimSolution> plate;
    if (resolve_aim_time(base, bullet_speed, rotation, lock, decision))
      plate = solve_xyza(decision.reference_xyza, bullet_speed, rotation);
    const auto yaw =
      center ? smooth_aim(base.prediction_state(), -1, bullet_speed, rotation) : plate;
    const auto pitch =
      group >= 0 ? smooth_aim(base.prediction_state(), group, bullet_speed, rotation) : plate;
    if ((!yaw || !pitch) && k == 0)
      return false;
    if (yaw) {
      previous_yaw += tools::limit_rad(yaw->yaw - previous_yaw);
      out(0, k) = previous_yaw - yaw_origin;
    } else {
      out(0, k) = out(0, k - 1);
    }
    out(2, k) = pitch ? pitch->pitch : out(2, k - 1);

    if (plate) {
      tags[k] = {decision.id,
                 decision.converged && (decision.valid || decision.mode == AimMode::preaim),
                 decision.mode,
                 decision.reference_xyza,
                 decision.fly_time,
                 decision.valid,
                 decision.converged};
      if (decision.reference_is_real)
        tags[k].plate_angles = PlateAimAngles{plate->yaw, plate->pitch};
      if (decision.converged)
        lock = decision.lock;
    }
    auto & tag = tags[k];
    tag.yaw_mode = center ? YawMode::center : YawMode::plate;
    tag.pitch_mode = group >= 0 ? PitchMode::height_group : PitchMode::plate;
    tag.height_group = group;
    tag.yaw_smooth_valid = center && yaw.has_value();
    tag.pitch_smooth_valid = group >= 0 && pitch.has_value();
    const bool valid_plate = plate && decision.valid && decision.converged;
    if (k == 0) {
      first_decision = decision;
      first_decision.command_valid = valid_plate;
      aim_xyza = decision.reference_xyza;
    } else if (k <= COMMAND_STEP) {
      first_decision.command_valid = first_decision.command_valid && valid_plate &&
                                     (!require_same_armor || decision.id == first_decision.id);
    }
  }
  if (gap_preaim_enabled_ || center) {
    fill_axis_reference_velocity(out, tags, 0, true);
  } else {
    for (int k = 0; k < HORIZON; ++k) {
      const int left = std::max(0, k - 1);
      const int right = std::min(HORIZON - 1, k + 1);
      out(1, k) = (out(0, right) - out(0, left)) / ((right - left) * DT);
    }
  }
  fill_axis_reference_velocity(out, tags, 2, gap_preaim_enabled_);
  return out.allFinite();
}

void Planner::apply_transitions(PlannerTime now, const PlanningState & initial,
                                Trajectory & reference, const ReferenceTags & tags)
{
  const auto at_column = [now](int column) {
    using Duration = std::chrono::duration<double>;
    return now + std::chrono::round<PlannerTime::duration>(Duration(column * DT));
  };
  const auto apply = [&](int row, AxisStrategyState & axis, const AxisStrategyConfig & config,
                         double max_acc, double measured, double measured_rate, double origin) {
    if (!config.enabled)
      return;
    const auto continuous = [&](int left, int right) {
      return same_reference_segment(tags[left], tags[right], row, gap_preaim_enabled_);
    };
    const auto sample = [&](int k) -> ReferenceMotion {
      const int left = k > 0 && continuous(k - 1, k) ? k - 1 : k;
      const int right = k + 1 < HORIZON && continuous(k, k + 1) ? k + 1 : k;
      return {reference(row, k) + origin, reference(row + 1, k),
              left == right
                ? 0.0
                : (reference(row + 1, right) - reference(row + 1, left)) / ((right - left) * DT)};
    };
    const auto raw = sample(0);
    if (!axis.transitioning(now)) {
      axis.bridge = {};
      axis.reference = raw;
      axis.reference_at = now;
      return;
    }
    auto from = ReferenceMotion{measured, measured_rate, 0.0};
    const double age = std::chrono::duration<double>(now - axis.reference_at).count();
    if (axis.reference_at != PlannerTime{} && age >= 0.0 && age <= max_state_age_)
      from = axis.previous_at(now);
    const auto configured =
      std::max(PlannerTime::duration(1), std::chrono::duration_cast<PlannerTime::duration>(
                                           std::chrono::duration<double>(config.transition)));
    auto end = axis.needs_bridge ? now + configured : axis.bridge.end;
    const double max_jerk = 6.0 * max_acc / config.transition;
    const auto step_size = at_column(1) - now;
    const auto latest_end = std::max(at_column(HORIZON - 2), end + 8 * step_size);
    const auto last_column = at_column(HORIZON - 1);
    bool joined = false;
    double suffix_shift = 0.0;
    ReferenceBridge bridge;
    // Refit from the previous curve at now. Keep its end unless the new endpoint needs more time.
    while (end <= latest_end) {
      const int left = std::min<int>((end - now) / step_size, HORIZON - 2);
      auto to = sample(left);
      if (end >= last_column) {
        // Long configured transitions can extend beyond the MPC horizon.
        to = sample(HORIZON - 1);
        to.angle += std::chrono::duration<double>(end - last_column).count() * to.velocity;
        to.acceleration = 0.0;
      } else if (end != at_column(left)) {
        if (!continuous(left, left + 1)) {
          end = at_column(left + 1);
          to = sample(left + 1);
        } else {
          to = ReferenceBridge::connect(at_column(left), at_column(left + 1), to, sample(left + 1))
                 .at(end);
        }
      }
      if (row == 0) {
        const double unwrapped = from.angle + tools::limit_rad(to.angle - from.angle);
        suffix_shift = unwrapped - to.angle;
        to.angle = unwrapped;
      }
      bridge = ReferenceBridge::connect(now, end, from, to);
      const double limit =
        max_acc + std::max(std::abs(from.acceleration), std::abs(to.acceleration));
      if (bridge.within_limits(limit, max_jerk)) {
        joined = true;
        break;
      }
      end += step_size;
    }
    if (!joined) {
      // Ease the current acceleration toward zero while waiting for a feasible join.
      const double t = std::max(DT, std::abs(from.acceleration) / max_jerk);
      end =
        now + std::chrono::duration_cast<PlannerTime::duration>(std::chrono::duration<double>(t));
      const ReferenceMotion to{from.angle + from.velocity * t + from.acceleration * t * t / 3,
                               from.velocity + from.acceleration * t / 2, 0.0};
      bridge = ReferenceBridge::connect(now, end, from, to);
    }
    axis.bridge = bridge;
    axis.needs_bridge = !joined;
    axis.reference = from;
    axis.reference_at = now;

    // Put the suffix on the same yaw winding used by the bridge.
    for (int k = 0; k < HORIZON; ++k) {
      const auto time = at_column(k);
      if (time <= end || !joined) {
        auto motion = bridge.at(time);
        if (time > end)
          motion.angle += std::chrono::duration<double>(time - end).count() * motion.velocity;
        reference(row, k) = motion.angle - origin;
        reference(row + 1, k) = motion.velocity;
      } else {
        reference(row, k) += suffix_shift;
      }
    }
  };
  apply(0, axis_history_.yaw, yaw_strategy_, yaw_solver_.config().max_acc, initial.yaw,
        initial.yaw_vel, initial.yaw);
  apply(2, axis_history_.pitch, pitch_strategy_, pitch_solver_.config().max_acc, initial.pitch,
        initial.pitch_vel, 0.0);
}

Plan Plan::relative_to(double measured_yaw, double measured_pitch) const
{
  if (!control || !std::isfinite(measured_yaw) || !std::isfinite(measured_pitch))
    return Plan{};
  Plan result = *this;
  result.yaw = tools::limit_rad(absolute_yaw - measured_yaw);
  result.pitch = absolute_pitch - measured_pitch;
  result.target_yaw = tools::limit_rad(control_reference.absolute_yaw - measured_yaw);
  result.target_pitch = control_reference.absolute_pitch - measured_pitch;
  return result;
}

bool Plan::legacy_fire_aligned() const
{
  return FireEvaluator::angles_within(target_yaw, target_pitch, fire_tolerance);
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
  axis_history_ = history.axis_strategy;
  pitch_window_bridge_ = history.pitch_window_bridge;
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
    trace->planner_config_yaml = planner_config_yaml_;
    trace->history = {lock_id_,      have_last_command_, last_target_yaw_,
                      last_plan_at_, axis_history_,      pitch_window_bridge_};
    if (target)
      trace->target_prediction = target->prediction_state();
    trace->target_available = target.has_value();
    if (target) {
      const auto x = target->ekf_x();
      for (int i = 0; i < std::min<int>(11, x.size()); ++i)
        trace->target_state[i] = x[i];
    }
  }
  const auto previous_pitch_bridge = std::exchange(pitch_window_bridge_, std::nullopt);
  auto fail = [&](PlanStatus status) {
    result.status = status;
    have_last_command_ = false;
    lock_id_ = -1;
    if (status == PlanStatus::mpc_failed) {
      // A rejected MPC result never drove the gimbal. Resume any bridge from feedback.
      axis_history_.yaw.reference_at = {};
      axis_history_.pitch.reference_at = {};
    } else {
      axis_history_.yaw.condition_since = {};
      axis_history_.pitch.condition_since = {};
    }
    if (status == PlanStatus::no_target)
      axis_history_.has_target = false;
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
  bullet_speed = usable_bullet_speed(bullet_speed);
  if (trace)
    trace->bullet_speed = bullet_speed;
  const auto ekf_x = target->ekf_x();
  const bool high_speed = std::abs(ekf_x[7]) > decision_speed_;
  const double delay = high_speed ? high_speed_delay_time_ : low_speed_delay_time_;
  const auto prediction = target->prediction_state();
  const bool strategy_target = axis_strategy_enabled_ && supports_axis_strategy(prediction);
  const double since_last = std::chrono::duration<double>(now - last_plan_at_).count();
  const bool recent_same_target = axis_history_.has_target &&
                                  axis_history_.generation == prediction.generation &&
                                  since_last > 0.0 && since_last <= max_state_age_;
  if (axis_strategy_enabled_) {
    auto launch = prediction;
    const double dt = std::chrono::duration<double>(now - prediction.timestamp).count() + delay;
    launch.x[0] += dt * launch.x[1];
    launch.x[2] += dt * launch.x[3];
    launch.x[4] += dt * launch.x[5];
    update_axis_modes(launch, initial, bullet_speed);
  }
  target->predict(now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(delay)));
  Trajectory reference;
  ReferenceTags reference_tags;
  AimDecision decision;
  Eigen::Vector4d aim_xyza;
  if (!get_trajectory(*target, initial.yaw, bullet_speed, initial.R_gimbal2world, reference,
                      reference_tags, decision, aim_xyza))
    return fail(PlanStatus::trajectory_unavailable);

  if (axis_strategy_enabled_)
    apply_transitions(now, initial, reference, reference_tags);
  result.control_reference.yaw_mode = reference_tags[0].yaw_mode;
  result.control_reference.pitch_mode = reference_tags[0].pitch_mode;
  result.control_reference.height_group = axis_history_.height_group;
  result.control_reference.yaw_transitioning = axis_history_.yaw.transitioning(now);
  result.control_reference.pitch_transitioning = axis_history_.pitch.transitioning(now);

  result.aim_debug.xyza = aim_xyza;
  result.aim_debug.armor_id = decision.id;
  result.aim_debug.mode = decision.mode;
  result.aim_debug.candidate_available = reference_tags[0].armor_id >= 0;
  result.aim_debug.selection_valid = decision.valid;
  result.aim_debug.ballistic_converged = decision.converged;
  const bool far_distance = std::hypot(ekf_x[0], ekf_x[2]) > judge_distance_;
  const double fire_tolerance =
    high_speed ? (far_distance ? second_tolerance_high_speed_ : first_tolerance_high_speed_)
               : (far_distance ? second_tolerance_ : first_tolerance_);
  result.yaw_mpc = yaw_solver_.solve({0.0, initial.yaw_vel}, reference.topRows<2>());
  std::optional<PitchWindowBridge> next_pitch_bridge;
  if (pitch_follow_yaw_window_ && strategy_target &&
      result.control_reference.yaw_mode == YawMode::center &&
      result.control_reference.pitch_mode == PitchMode::plate) {
    auto * window_trace = trace ? &trace->pitch_yaw_window.emplace() : nullptr;
    if (result.yaw_mpc.solved && !result.control_reference.pitch_transitioning) {
      next_pitch_bridge =
        follow_yaw_windows(now, initial.yaw, yaw_solver_.states(), fire_tolerance,
                           pitch_solver_.config().max_acc, reference_tags, reference,
                           recent_same_target ? previous_pitch_bridge : std::nullopt, window_trace);
    } else if (window_trace) {
      window_trace->fallback = result.yaw_mpc.solved ? PitchWindowFallback::pitch_mode_transition
                                                     : PitchWindowFallback::yaw_mpc_failed;
    }
  }
  if (trace) {
    trace->reference = reference;
    trace->reference_tags = reference_tags;
    trace->reference_available = true;
  }

  result.pitch_mpc =
    pitch_solver_.solve({initial.pitch, initial.pitch_vel}, reference.bottomRows<2>());
  if (!result.yaw_mpc.solved || !result.pitch_mpc.solved)
    return fail(PlanStatus::mpc_failed);
  pitch_window_bridge_ = next_pitch_bridge;

  result.status = PlanStatus::ok;
  result.control = true;
  result.absolute_yaw = yaw_solver_.states()(0, COMMAND_STEP) + initial.yaw;
  result.absolute_pitch = pitch_solver_.states()(0, COMMAND_STEP);
  result.control_reference.absolute_yaw = reference(0, COMMAND_STEP) + initial.yaw;
  result.control_reference.absolute_pitch = reference(2, COMMAND_STEP);
  result.yaw_vel = yaw_solver_.states()(1, COMMAND_STEP);
  result.pitch_vel = pitch_solver_.states()(1, COMMAND_STEP);
  result.yaw_acc = yaw_solver_.inputs()(0, 0) * yaw_solver_.config().max_acc;
  result.pitch_acc = pitch_solver_.inputs()(0, 0) * pitch_solver_.config().max_acc;
  result.valid_at = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(COMMAND_STEP * DT));
  result.fire_tolerance = fire_tolerance;
  const bool reachable =
    std::hypot(reference(0, COMMAND_STEP) - yaw_solver_.states()(0, COMMAND_STEP),
               reference(2, COMMAND_STEP) - pitch_solver_.states()(0, COMMAND_STEP)) < fire_thresh_;
  const bool settled =
    std::hypot(reference(0, 0), reference(2, 0) - initial.pitch) < result.fire_tolerance;
  const bool fire_enabled = auto_fire_ && state.rates_measured && have_last_command_ &&
                            since_last > 0.0 && since_last <= max_state_age_;
  const bool transitioning =
    result.control_reference.yaw_transitioning || result.control_reference.pitch_transitioning;
  if (strategy_target) {
    FireContext context;
    context.target = prediction;
    context.valid_from = now;
    if (prediction.observed_at != PlannerTime{} && prediction.observed_at <= now)
      context.valid_until = std::min(result.valid_at, prediction.observed_at + fire_timeout_);
    context.height_group = axis_history_.height_group;
    context.armor_id = decision.command_valid ? decision.id : -1;
    context.transitioning = transitioning;
    context.enabled = fire_enabled;
    context.launch_delay = delay;
    context.yaw_offset = yaw_offset_;
    context.pitch_offset = pitch_offset_;
    context.coming_angle = comming_angle_;
    context.leaving_angle = leaving_angle_;
    context.tolerance = result.fire_tolerance;
    result.fire_context = context;
    result.fire_result = FireEvaluator::evaluate(context, initial, bullet_speed, now);
    result.fire = result.fire_result.allowed();
  } else {
    result.fire =
      fire_enabled && !transitioning && decision.command_valid && reachable && settled &&
      std::abs(tools::limit_rad(last_target_yaw_ - result.control_reference.absolute_yaw)) <
        2 * result.fire_tolerance;
  }

  if (decision.converged)
    lock_id_ = decision.lock;
  have_last_command_ = true;
  last_target_yaw_ = result.control_reference.absolute_yaw;
  last_plan_at_ = now;
  result.aim_debug.display_valid = decision.valid && decision.converged;
  result = result.relative_to(initial.yaw, initial.pitch);
  if (!result.fire_context)
    result.fire = result.fire && result.legacy_fire_aligned();
  return result;
}

}  // namespace auto_aim
