#pragma once

#include <Eigen/Core>
#include <chrono>
#include <optional>

#include "tools/math_tools.hpp"

namespace tools
{
// Pose-only simulation/replay; hardware uses the reported joint rates.
class AngleRateEstimator
{
public:
  std::optional<Eigen::Vector2d> update(const Eigen::Vector2d &angles,
                                        std::chrono::steady_clock::time_point at)
  {
    if (!angles.allFinite()) {
      previous_.reset();
      return std::nullopt;
    }
    std::optional<Eigen::Vector2d> rates;
    if (previous_) {
      const double dt = std::chrono::duration<double>(at - previous_at_).count();
      if (dt > 0.0 && dt <= 0.1) {
        rates = Eigen::Vector2d{limit_rad(angles[0] - (*previous_)[0]) / dt,
                                (angles[1] - (*previous_)[1]) / dt};
      }
    }
    previous_ = angles;
    previous_at_ = at;
    return rates;
  }

private:
  std::optional<Eigen::Vector2d> previous_;
  std::chrono::steady_clock::time_point previous_at_{};
};
} // namespace tools
