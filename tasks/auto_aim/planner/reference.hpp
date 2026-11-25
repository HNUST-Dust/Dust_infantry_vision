#pragma once

#include <array>
#include <cmath>
#include <optional>

#include "mpc_axis.hpp"

namespace auto_aim
{
enum class AimMode
{
  track,
  preaim,
  fallback
};
inline const char * to_string(AimMode mode)
{
  switch (mode) {
  case AimMode::track:
    return "track";
  case AimMode::preaim:
    return "preaim";
  case AimMode::fallback:
    return "fallback";
  }
  return "fallback";
}

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;

struct ReferenceTag
{
  int armor_id = -1;
  bool rate_valid = false;
  AimMode mode = AimMode::fallback;
  Eigen::Vector4d reference_xyza = Eigen::Vector4d::Zero();
  double fly_time = 0.0;
  bool selection_valid = false;
  bool converged = false;
};
using ReferenceTags = std::array<ReferenceTag, HORIZON>;

enum class FireDirectionState
{
  unknown,
  stationary,
  moving
};
inline const char * to_string(FireDirectionState state)
{
  switch (state) {
  case FireDirectionState::stationary:
    return "stationary";
  case FireDirectionState::moving:
    return "moving";
  default:
    return "unknown";
  }
}
struct FireYawMotion
{
  FireDirectionState state = FireDirectionState::unknown;
  int direction = 0;
  std::optional<double> rate;
};

// Gate direction comes from the same selected armor, never an MPC/wire velocity.
inline FireYawMotion fire_yaw_motion(const Trajectory & reference, const ReferenceTags & tags,
                                     int k, double min_rate)
{
  const auto valid = [&](int i) {
    return i >= 0 && i < HORIZON && tags[i].armor_id >= 0 && tags[i].rate_valid &&
           tags[i].selection_valid && tags[i].converged && tags[i].mode == AimMode::track &&
           std::isfinite(reference(0, i));
  };
  if (!valid(k))
    return {};
  const auto rate = [&](int left, int right) -> std::optional<double> {
    if (!valid(left) || !valid(right) || tags[left].armor_id != tags[right].armor_id)
      return std::nullopt;
    const double value =
      std::remainder(reference(0, right) - reference(0, left), 2 * std::acos(-1.0)) / DT;
    return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
  };
  const auto left = rate(k - 1, k), right = rate(k, k + 1);
  if (!left && !right)
    return {};
  const bool left_moves = left && std::abs(*left) > min_rate;
  const bool right_moves = right && std::abs(*right) > min_rate;
  if (left_moves && right_moves && (*left > 0) != (*right > 0))
    return {};  // Reversal is not a stationary central difference.
  if (!left_moves && !right_moves)
    return {FireDirectionState::stationary, 0,
            left && right ? (*left + *right) / 2
            : left        ? *left
                          : *right};
  const double moving_rate = left_moves && right_moves ? (*left + *right) / 2
                             : left_moves              ? *left
                                                       : *right;
  return {FireDirectionState::moving, moving_rate > 0 ? 1 : -1, moving_rate};
}

// Differentiate only adjacent valid samples of the same armor. A held position or an
// unselected/unconverged candidate breaks the segment, even if its armor ID is unchanged.
// Only the pitch velocity row is written; positions and both yaw rows are preserved.
inline void fill_pitch_reference_velocity(Trajectory & reference, const ReferenceTags & tags)
{
  const auto continuous = [&](int left, int right) {
    return tags[left].rate_valid && tags[right].rate_valid && tags[left].armor_id >= 0 &&
           tags[left].armor_id == tags[right].armor_id;
  };
  for (int k = 0; k < HORIZON; ++k) {
    reference(3, k) = 0.0;
    if (!tags[k].rate_valid || tags[k].armor_id < 0)
      continue;
    const int left = k > 0 && continuous(k - 1, k) ? k - 1 : k;
    const int right = k + 1 < HORIZON && continuous(k, k + 1) ? k + 1 : k;
    if (left != right)
      reference(3, k) = (reference(2, right) - reference(2, left)) / ((right - left) * DT);
  }
}
// With preaim enabled, both axes use only samples from the same converged
// armor/mode segment. Endpoints use one-sided differences; isolated points are zero.
inline void fill_segment_reference_velocity(Trajectory & reference, const ReferenceTags & tags)
{
  const auto continuous = [&](int left, int right) {
    return tags[left].rate_valid && tags[right].rate_valid && tags[left].armor_id >= 0 &&
           tags[left].armor_id == tags[right].armor_id && tags[left].mode == tags[right].mode;
  };
  for (int k = 0; k < HORIZON; ++k) {
    reference(1, k) = reference(3, k) = 0.0;
    const int left = k > 0 && continuous(k - 1, k) ? k - 1 : k;
    const int right = k + 1 < HORIZON && continuous(k, k + 1) ? k + 1 : k;
    if (left == right)
      continue;
    for (int row : {0, 2})
      reference(row + 1, k) =
        (reference(row, right) - reference(row, left)) / ((right - left) * DT);
  }
}
}  // namespace auto_aim
