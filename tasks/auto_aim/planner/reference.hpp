#pragma once

#include <array>
#include <optional>

#include "mpc_axis.hpp"
#include "axis_strategy.hpp"

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

struct PlateAimAngles
{
  double yaw, pitch;
};

struct ReferenceTag
{
  int armor_id = -1;
  bool rate_valid = false;
  AimMode mode = AimMode::fallback;
  Eigen::Vector4d reference_xyza = Eigen::Vector4d::Zero();
  double fly_time = 0.0;
  bool selection_valid = false;
  bool converged = false;
  YawMode yaw_mode = YawMode::plate;
  PitchMode pitch_mode = PitchMode::plate;
  int height_group = -1;
  bool yaw_smooth_valid = false, pitch_smooth_valid = false;
  // Real plate angles, distinct from virtual preaim/center/height-group references.
  std::optional<PlateAimAngles> plate_angles{};
};
using ReferenceTags = std::array<ReferenceTag, HORIZON>;

inline bool same_reference_segment(const ReferenceTag & a, const ReferenceTag & b, int row,
                                   bool match_mode)
{
  if (row == 0 && (a.yaw_mode == YawMode::center || b.yaw_mode == YawMode::center))
    return a.yaw_mode == b.yaw_mode && a.yaw_smooth_valid && b.yaw_smooth_valid;
  if (row == 2 &&
      (a.pitch_mode == PitchMode::height_group || b.pitch_mode == PitchMode::height_group))
    return a.pitch_mode == b.pitch_mode && a.pitch_smooth_valid && b.pitch_smooth_valid &&
           a.height_group == b.height_group;
  return a.rate_valid && b.rate_valid && a.armor_id >= 0 && a.armor_id == b.armor_id &&
         (!match_mode || a.mode == b.mode);
}

// Use one-sided differences at the ends of each valid segment.
inline void fill_axis_reference_velocity(Trajectory & reference, const ReferenceTags & tags,
                                         int row, bool match_mode)
{
  for (int k = 0; k < HORIZON; ++k) {
    const int left =
      k > 0 && same_reference_segment(tags[k - 1], tags[k], row, match_mode) ? k - 1 : k;
    const int right =
      k + 1 < HORIZON && same_reference_segment(tags[k], tags[k + 1], row, match_mode) ? k + 1 : k;
    reference(row + 1, k) =
      left != right ? (reference(row, right) - reference(row, left)) / ((right - left) * DT) : 0.0;
  }
}

inline void fill_pitch_reference_velocity(Trajectory & reference, const ReferenceTags & tags)
{
  fill_axis_reference_velocity(reference, tags, 2, false);
}

inline void fill_segment_reference_velocity(Trajectory & reference, const ReferenceTags & tags)
{
  fill_axis_reference_velocity(reference, tags, 0, true);
  fill_axis_reference_velocity(reference, tags, 2, true);
}
}  // namespace auto_aim
