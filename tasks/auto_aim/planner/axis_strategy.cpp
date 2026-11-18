#include "axis_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tools/runtime/yaml_config.hpp"

namespace auto_aim
{
namespace
{
double seconds(PlannerTime::duration duration)
{
  return std::chrono::duration<double>(duration).count();
}
}  // namespace

const char * to_string(YawMode mode)
{
  return mode == YawMode::center ? "center" : "plate";
}

const char * to_string(PitchMode mode)
{
  return mode == PitchMode::height_group ? "height_group" : "plate";
}

AxisStrategyConfig AxisStrategyConfig::parse(const tools::YamlConfig & yaml)
{
  AxisStrategyConfig config;
  config.enabled = yaml.get("enabled", false);
  if (!config.enabled && !yaml.has("enter_speed") && !yaml.has("exit_speed") &&
      !yaml.has("enter_hold_ms") && !yaml.has("exit_hold_ms") && !yaml.has("transition_ms"))
    return config;
  config.enter_speed = yaml.get<double>("enter_speed");
  config.exit_speed = yaml.get<double>("exit_speed");
  if (!std::isfinite(config.enter_speed) || !std::isfinite(config.exit_speed) ||
      config.exit_speed < 0 || config.enter_speed <= config.exit_speed)
    yaml.fail("enter_speed", "expected enter_speed > exit_speed >= 0");
  const auto duration = [&](const char * key) {
    const double ms = yaml.get<double>(key);
    if (!std::isfinite(ms) || ms <= 0)
      yaml.fail(key, "must be finite and positive");
    return ms * 1e-3;
  };
  config.enter_hold = duration("enter_hold_ms");
  config.exit_hold = duration("exit_hold_ms");
  config.transition = duration("transition_ms");
  return config;
}

bool AxisStrategyState::confirm(const AxisStrategyConfig & config, double speed, PlannerTime now,
                                PlannerTime observation, std::chrono::milliseconds max_age)
{
  const bool fresh = observation != PlannerTime{} && observation <= now &&
                     now - observation <= max_age && std::isfinite(speed);
  if (!config.enabled || !fresh) {
    condition_since = {};
    return false;
  }
  // Only a new real observation can start or finish a confirmation.
  if (observation <= last_observation)
    return false;
  if (observation - last_observation > max_age)
    condition_since = {};
  last_observation = observation;
  const bool qualifies = reduced ? speed < config.exit_speed : speed > config.enter_speed;
  if (!qualifies) {
    condition_since = {};
    return false;
  }
  if (condition_since == PlannerTime{})
    condition_since = observation;
  return seconds(observation - condition_since) >= (reduced ? config.exit_hold : config.enter_hold);
}

ReferenceBridge ReferenceBridge::connect(PlannerTime start, PlannerTime end, ReferenceMotion from,
                                         ReferenceMotion to)
{
  const double t = seconds(end - start);
  const double position = to.angle - from.angle;
  const double v0 = from.velocity * t, v1 = to.velocity * t;
  const double a0 = from.acceleration * t * t, a1 = to.acceleration * t * t;
  return {start,
          end,
          {from.angle, v0, a0 / 2, 10 * position - 6 * v0 - 4 * v1 - 1.5 * a0 + a1 / 2,
           -15 * position + 8 * v0 + 7 * v1 + 1.5 * a0 - a1,
           6 * position - 3 * (v0 + v1) + (a1 - a0) / 2}};
}

ReferenceMotion ReferenceBridge::at(PlannerTime time) const
{
  const double duration = seconds(end - start);
  const double u = std::clamp(seconds(time - start) / duration, 0.0, 1.0);
  const auto & c = coefficients;
  return {c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5])))),
          (c[1] + u * (2 * c[2] + u * (3 * c[3] + u * (4 * c[4] + u * 5 * c[5])))) / duration,
          (2 * c[2] + u * (6 * c[3] + u * (12 * c[4] + u * 20 * c[5]))) / (duration * duration)};
}

double ReferenceBridge::peak_acceleration() const
{
  const double t = seconds(end - start);
  if (t <= 0.0)
    return std::numeric_limits<double>::infinity();
  const auto & c = coefficients;
  const auto acceleration = [&](double u) {
    return std::abs(2 * c[2] + u * (6 * c[3] + u * (12 * c[4] + u * 20 * c[5])));
  };
  double peak = std::max(acceleration(0.0), acceleration(1.0));
  const auto include = [&](double u) {
    if (u > 0.0 && u < 1.0)
      peak = std::max(peak, acceleration(u));
  };

  // Acceleration is cubic in normalized time. Its interior extrema have zero jerk.
  const double a = 60 * c[5], b = 24 * c[4], d = 6 * c[3];
  if (a == 0.0) {
    if (b != 0.0)
      include(-d / b);
  } else {
    const double discriminant = b * b - 4 * a * d;
    if (discriminant >= 0.0) {
      const double q = -0.5 * (b + std::copysign(std::sqrt(discriminant), b));
      include(q / a);
      if (q != 0.0)
        include(d / q);
    }
  }
  return peak / (t * t);
}

bool ReferenceBridge::within_limits(double max_acceleration, double max_jerk) const
{
  const double t = seconds(end - start);
  const auto & c = coefficients;
  // Bernstein coefficients bound the derivatives on the entire interval.
  const double a0 = 2 * c[2];
  const double a1 = a0 + 2 * c[3];
  const double a2 = a1 + 2 * c[3] + 4 * c[4];
  const double a3 = a0 + 6 * c[3] + 12 * c[4] + 20 * c[5];
  return std::max({std::abs(a0), std::abs(a1), std::abs(a2), std::abs(a3)}) <=
           max_acceleration * t * t &&
         3 * std::max({std::abs(a1 - a0), std::abs(a2 - a1), std::abs(a3 - a2)}) <=
           max_jerk * t * t * t;
}

bool AxisStrategyState::transitioning(PlannerTime now) const
{
  return needs_bridge || now < bridge.end;
}

ReferenceMotion AxisStrategyState::previous_at(PlannerTime now) const
{
  if (bridge.end != PlannerTime{} && now <= bridge.end)
    return bridge.at(now);
  const bool had_bridge = bridge.end != PlannerTime{};
  auto motion = had_bridge ? bridge.at(bridge.end) : reference;
  const double dt = seconds(now - (had_bridge ? bridge.end : reference_at));
  motion.angle += dt * (motion.velocity + 0.5 * dt * motion.acceleration);
  motion.velocity += dt * motion.acceleration;
  return motion;
}
}  // namespace auto_aim
