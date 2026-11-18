#include "fire_evaluator.hpp"

#include <cmath>

#include "planner.hpp"
#include "tools/math/math_tools.hpp"
#include "tools/math/trajectory.hpp"

namespace auto_aim
{
double usable_bullet_speed(double speed)
{
  return std::isfinite(speed) && speed >= 10 && speed <= 25 ? speed : 23;
}

std::optional<AimSolution> solve_ballistic_aim(const Eigen::Vector4d & xyza, double bullet_speed,
                                               const Eigen::Matrix3d & R_gimbal2world,
                                               double yaw_offset, double pitch_offset)
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
    R_gimbal2world * (Eigen::AngleAxisd(azim + yaw_offset, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(pitch - pitch_offset, Eigen::Vector3d::UnitX()) *
                      Eigen::Vector3d(0.0, -1.0, 0.0));
  solution.yaw = std::atan2(corrected.x(), -corrected.y());
  solution.pitch = std::atan2(-corrected.z(), std::hypot(corrected.x(), corrected.y()));
  return solution;
}

bool armor_in_window(double angle, double relative_rate, double coming, double leaving)
{
  if (std::abs(relative_rate) <= 2)
    return std::abs(angle) <= 60 / 57.3;
  return std::abs(angle) <= coming && (relative_rate > 0 ? angle < leaving : angle > -leaving);
}

bool FireEvaluator::angles_within(double yaw_error, double pitch_error, double tolerance)
{
  return std::hypot(yaw_error, pitch_error) < tolerance;
}

const char * to_string(FireBlock block)
{
  switch (block) {
  case FireBlock::none:
    return "none";
  case FireBlock::inhibited:
    return "inhibited";
  case FireBlock::transitioning:
    return "transitioning";
  case FireBlock::expired:
    return "expired";
  case FireBlock::no_candidate:
    return "no_candidate";
  case FireBlock::feedback_invalid:
    return "feedback_invalid";
  case FireBlock::ballistic_unavailable:
    return "ballistic_unavailable";
  case FireBlock::outside_window:
    return "outside_window";
  case FireBlock::angle_outside:
    return "angle_outside";
  }
  return "unknown";
}

FireResult FireEvaluator::evaluate(const FireContext & context, const PlanningState & feedback,
                                   double bullet_speed, PlannerTime now, PlannerTime::duration hold)
{
  if (context.transitioning)
    return {FireBlock::transitioning};
  if (!context.enabled)
    return {FireBlock::inhibited};
  if (now < context.valid_from || hold < PlannerTime::duration::zero() ||
      now + hold > context.valid_until)
    return {FireBlock::expired};
  const auto & target = context.target;
  const int id = context.armor_id;
  if (target.armor_num != 4 || id < 0 || id >= target.armor_num || (!target.jumped && id != 0) ||
      (context.height_group >= 0 && id % 2 != context.height_group))
    return {FireBlock::no_candidate};
  if (!feedback.valid || !feedback.rates_measured || feedback.timestamp == PlannerTime{} ||
      feedback.timestamp > now ||
      !Eigen::Vector4d{feedback.yaw, feedback.pitch, feedback.yaw_vel, feedback.pitch_vel}
         .allFinite() ||
      !feedback.R_gimbal2world.allFinite())
    return {FireBlock::feedback_invalid, id};
  if (target.timestamp == PlannerTime{} || target.timestamp > now || !target.x.allFinite())
    return {FireBlock::expired, id};

  bullet_speed = usable_bullet_speed(bullet_speed);
  const auto roll = tools::eulers(feedback.R_gimbal2world, 2, 0, 1)[2];
  const auto check = [&](PlannerTime time) -> FireResult {
    const double feedback_age = std::chrono::duration<double>(time - feedback.timestamp).count();
    const double yaw = feedback.yaw + feedback_age * feedback.yaw_vel;
    const double pitch = feedback.pitch + feedback_age * feedback.pitch_vel;
    if (!std::isfinite(yaw) || !std::isfinite(pitch) || std::abs(pitch) > std::acos(0.05))
      return {FireBlock::feedback_invalid, id};
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitY()))
                                       .toRotationMatrix();
    const double launch =
      std::chrono::duration<double>(time - target.timestamp).count() + context.launch_delay;
    double impact = launch;
    for (int i = 0; i < 10; ++i) {
      const auto armor = target.armor_at(id, impact);
      const auto aim = solve_ballistic_aim(armor, bullet_speed, rotation, context.yaw_offset,
                                           context.pitch_offset);
      if (!aim)
        break;
      const double next = launch + aim->fly_time;
      if (std::abs(next - impact) > 1e-6) {
        impact = next;
        continue;
      }
      FireResult result{FireBlock::none, id, true, tools::limit_rad(aim->yaw - yaw),
                        aim->pitch - pitch};
      const auto & x = target.x;
      const double cx = x[0] + impact * x[1], cy = x[2] + impact * x[3];
      const double range_sq = cx * cx + cy * cy;
      if (!(range_sq > 1e-12) || !std::isfinite(range_sq))
        return {FireBlock::ballistic_unavailable, id};
      const double phase = tools::limit_rad(armor[3] - std::atan2(cy, cx));
      const double rate = x[7] - (cx * x[3] - cy * x[1]) / range_sq;
      if (std::abs(phase) >= M_PI / 2 ||
          !armor_in_window(phase, rate, context.coming_angle, context.leaving_angle))
        result.block = FireBlock::outside_window;
      else if (!angles_within(result.yaw_error, result.pitch_error, context.tolerance))
        result.block = FireBlock::angle_outside;
      return result;
    }
    return {FireBlock::ballistic_unavailable, id};
  };

  const auto current = check(now);
  if (!current.allowed() || hold == PlannerTime::duration::zero())
    return current;

  // Recheck the same plate through the requested host permission interval.
  for (const auto time : {now + hold / 2, now + hold}) {
    const auto future = check(time);
    if (!future.allowed()) {
      auto result = current;
      result.block = future.block;
      return result;
    }
  }
  return current;
}
}  // namespace auto_aim
