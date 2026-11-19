#pragma once

#include <array>

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
