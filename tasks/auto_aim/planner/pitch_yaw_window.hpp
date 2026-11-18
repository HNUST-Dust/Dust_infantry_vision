#pragma once

#include <vector>

#include "reference.hpp"

namespace auto_aim
{
struct PitchYawWindow
{
  int armor_id;
  PlannerTime enter, exit;
};

struct PitchWindowBridge
{
  int from_armor_id, to_armor_id;
  ReferenceBridge curve;
};

enum class PitchWindowFallback
{
  none,
  yaw_mpc_failed,
  pitch_mode_transition,
  no_adjacent_windows,
  acceleration_limit
};

const char * to_string(PitchWindowFallback fallback);

struct PitchWindowGap
{
  int from_armor_id, to_armor_id;
  PlannerTime start, end;
  double peak_acceleration;
  PitchWindowFallback fallback;
};

struct PitchWindowTrace
{
  PitchWindowFallback fallback = PitchWindowFallback::none;
  std::vector<PitchYawWindow> windows;
  std::vector<PitchWindowGap> gaps;
};

std::optional<PitchWindowBridge>
follow_yaw_windows(PlannerTime now, double yaw_origin, const tinyMatrix & yaw_prediction,
                   double tolerance, double max_acceleration, const ReferenceTags & tags,
                   Trajectory & reference, const std::optional<PitchWindowBridge> & previous,
                   PitchWindowTrace * trace);
}  // namespace auto_aim
