#pragma once

#include <array>
#include <chrono>
#include <cstdint>

namespace tools
{
class YamlConfig;
}

namespace auto_aim
{
enum class YawMode
{
  plate,
  center
};
enum class PitchMode
{
  plate,
  height_group
};
const char * to_string(YawMode mode);
const char * to_string(PitchMode mode);

using PlannerTime = std::chrono::steady_clock::time_point;

struct AxisStrategyConfig
{
  bool enabled = false;
  double enter_speed = 0.0, exit_speed = 0.0;
  double enter_hold = 0.0, exit_hold = 0.0, transition = 0.0;

  static AxisStrategyConfig parse(const tools::YamlConfig & yaml);
};

struct ReferenceMotion
{
  double angle = 0.0, velocity = 0.0, acceleration = 0.0;
};

struct ReferenceBridge
{
  PlannerTime start{}, end{};
  std::array<double, 6> coefficients{};

  static ReferenceBridge connect(PlannerTime start, PlannerTime end, ReferenceMotion from,
                                 ReferenceMotion to);
  ReferenceMotion at(PlannerTime time) const;
  double peak_acceleration() const;
  bool within_limits(double max_acceleration, double max_jerk) const;
};

struct AxisStrategyState
{
  bool reduced = false;
  PlannerTime condition_since{}, last_observation{};
  bool needs_bridge = false;
  ReferenceBridge bridge;
  PlannerTime reference_at{};
  ReferenceMotion reference;

  bool confirm(const AxisStrategyConfig & config, double speed, PlannerTime now,
               PlannerTime observation, std::chrono::milliseconds max_age);
  bool transitioning(PlannerTime now) const;
  ReferenceMotion previous_at(PlannerTime now) const;
};

struct AxisStrategyHistory
{
  bool has_target = false;
  uint64_t generation = 0;
  int height_group = -1;
  AxisStrategyState yaw, pitch;
};
}  // namespace auto_aim
