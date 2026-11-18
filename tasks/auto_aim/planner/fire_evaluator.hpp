#pragma once

#include <optional>

#include "axis_strategy.hpp"
#include "tasks/auto_aim/tracking/target.hpp"

namespace auto_aim
{
struct PlanningState;

struct AimSolution
{
  double fly_time = 0.0;
  double yaw = 0.0;
  double pitch = 0.0;
};

double usable_bullet_speed(double speed);

std::optional<AimSolution> solve_ballistic_aim(const Eigen::Vector4d & xyza, double bullet_speed,
                                               const Eigen::Matrix3d & rotation, double yaw_offset,
                                               double pitch_offset);

bool armor_in_window(double angle, double relative_rate, double coming, double leaving);

struct FireContext
{
  TargetPredictionState target;
  PlannerTime valid_from{}, valid_until{};
  int armor_id = -1;
  int height_group = -1;
  bool transitioning = false;
  bool enabled = false;
  // Time from the evaluation's command instant to projectile launch. Flight is added separately.
  double launch_delay = 0.0;
  double yaw_offset = 0.0, pitch_offset = 0.0;
  double coming_angle = 0.0, leaving_angle = 0.0;
  double tolerance = 0.0;
};

enum class FireBlock
{
  none,
  inhibited,
  transitioning,
  expired,
  no_candidate,
  feedback_invalid,
  ballistic_unavailable,
  outside_window,
  angle_outside
};
const char * to_string(FireBlock block);

struct FireResult
{
  FireBlock block = FireBlock::no_candidate;
  int armor_id = -1;
  bool error_valid = false;
  double yaw_error = 0.0, pitch_error = 0.0;

  bool allowed() const
  {
    return block == FireBlock::none;
  }
};

class FireEvaluator
{
public:
  static FireResult evaluate(const FireContext & context, const PlanningState & feedback,
                             double bullet_speed, PlannerTime now,
                             PlannerTime::duration hold = PlannerTime::duration::zero());
  static bool angles_within(double yaw_error, double pitch_error, double tolerance);
};
}  // namespace auto_aim
