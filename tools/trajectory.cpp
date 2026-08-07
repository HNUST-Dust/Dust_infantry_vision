#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  if (!std::isfinite(v0) || !std::isfinite(d) || !std::isfinite(h) || !(v0 > 0.0) || d < 0.0)
    return;

  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0 || std::abs(a) < 1e-12) return;

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));
  if (!std::isfinite(t_1) || !std::isfinite(t_2)) {
    unsolvable = true;
    return;
  }

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}

}  // namespace tools
