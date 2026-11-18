#include "pitch_yaw_window.hpp"

#include <algorithm>
#include <cmath>

#include "tools/math/math_tools.hpp"

namespace auto_aim
{
const char * to_string(PitchWindowFallback fallback)
{
  switch (fallback) {
  case PitchWindowFallback::none:
    return "none";
  case PitchWindowFallback::yaw_mpc_failed:
    return "yaw_mpc_failed";
  case PitchWindowFallback::pitch_mode_transition:
    return "pitch_mode_transition";
  case PitchWindowFallback::no_adjacent_windows:
    return "no_adjacent_windows";
  case PitchWindowFallback::acceleration_limit:
    return "acceleration_limit";
  }
  return "unknown";
}

std::optional<PitchWindowBridge>
follow_yaw_windows(PlannerTime now, double yaw_origin, const tinyMatrix & yaw_prediction,
                   double tolerance, double max_acceleration, const ReferenceTags & tags,
                   Trajectory & reference, const std::optional<PitchWindowBridge> & previous,
                   PitchWindowTrace * trace)
{
  const auto time_at = [now](double k) {
    return now + std::chrono::round<PlannerTime::duration>(std::chrono::duration<double>(k * DT));
  };
  const auto valid = [&](int k) {
    return k >= 0 && k < HORIZON && tags[k].armor_id >= 0 && tags[k].selection_valid &&
           tags[k].converged && tags[k].plate_angles.has_value();
  };
  const auto same_plate = [&](int k, int other) {
    return valid(other) && tags[k].armor_id == tags[other].armor_id;
  };
  const auto error = [&](int k) {
    return tools::limit_rad(tags[k].plate_angles->yaw - (yaw_origin + yaw_prediction(0, k)));
  };
  const auto in_window = [&](int k) { return valid(k) && std::abs(error(k)) < tolerance; };
  const auto motion = [&](int k) -> ReferenceMotion {
    const int left = same_plate(k, k - 1) ? k - 1 : k;
    const int right = same_plate(k, k + 1) ? k + 1 : k;
    const double velocity = left == right
                              ? 0.0
                              : (tags[right].plate_angles->pitch - tags[left].plate_angles->pitch) /
                                  ((right - left) * DT);
    return {tags[k].plate_angles->pitch, velocity, 0.0};
  };
  struct Edge
  {
    PlannerTime time;
    ReferenceMotion pitch;
  };
  const auto edge = [&](int k, int neighbor) -> Edge {
    const auto inner = motion(k);
    if (!same_plate(k, neighbor))
      return {time_at(k), inner};
    const double inner_error = error(k);
    const double outer_error = inner_error + tools::limit_rad(error(neighbor) - inner_error);
    const double threshold = std::copysign(tolerance, outer_error);
    const double fraction =
      std::clamp((threshold - inner_error) / (outer_error - inner_error), 0.0, 1.0);
    const auto outer = motion(neighbor);
    return {time_at(k + fraction * (neighbor - k)),
            {inner.angle + fraction * (outer.angle - inner.angle),
             inner.velocity + fraction * (outer.velocity - inner.velocity), 0.0}};
  };

  std::optional<PitchWindowBridge> first_bridge;
  bool found_gap = false;
  const auto join = [&](int from_id, int to_id, const Edge & from, const Edge & to, int first,
                        int last) {
    found_gap = true;
    const auto bridge = ReferenceBridge::connect(from.time, to.time, from.pitch, to.pitch);
    const double peak = bridge.peak_acceleration();
    const auto fallback = std::isfinite(peak) && peak <= max_acceleration
                            ? PitchWindowFallback::none
                            : PitchWindowFallback::acceleration_limit;
    if (trace)
      trace->gaps.push_back({from_id, to_id, from.time, to.time, peak, fallback});
    if (fallback != PitchWindowFallback::none)
      return;
    if (!first_bridge)
      first_bridge = PitchWindowBridge{from_id, to_id, bridge};
    for (int k = first; k < last; ++k) {
      const auto pitch = bridge.at(time_at(k));
      reference(2, k) = pitch.angle;
      reference(3, k) = pitch.velocity;
    }
  };

  int last_id = -1, last_column = -1;
  Edge last_exit{};
  for (int k = 0; k < HORIZON;) {
    if (!in_window(k)) {
      ++k;
      continue;
    }
    const int first = k;
    const int id = tags[k].armor_id;
    while (k + 1 < HORIZON && same_plate(k, k + 1) && in_window(k + 1))
      ++k;
    const auto enter = edge(first, first - 1);
    const auto exit = edge(k, k + 1);
    if (trace)
      trace->windows.push_back({id, enter.time, exit.time});
    if (last_id >= 0 && last_id != id) {
      join(last_id, id, last_exit, enter, last_column + 1, first);
    } else if (last_id < 0 && previous && previous->to_armor_id == id &&
               previous->curve.start <= now && now < previous->curve.end && now < enter.time) {
      // The prior window has moved behind the receding horizon. Continue its bridge from now.
      join(previous->from_armor_id, id, {now, previous->curve.at(now)}, enter, 0, first);
    }
    last_id = id;
    last_column = k;
    last_exit = exit;
    ++k;
  }
  if (trace && !found_gap)
    trace->fallback = PitchWindowFallback::no_adjacent_windows;
  return first_bridge;
}
}  // namespace auto_aim
