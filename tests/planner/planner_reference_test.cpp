#include <cmath>
#include <iostream>
#include <stdexcept>

#include "tasks/auto_aim/planner/reference.hpp"

using namespace auto_aim;

namespace
{
void require(bool value, const char * message)
{
  if (!value)
    throw std::runtime_error(message);
}

Trajectory ramp(double velocity)
{
  Trajectory reference;
  for (int k = 0; k < HORIZON; ++k) {
    reference(0, k) = 0.01 * k;
    reference(1, k) = 0.7;
    reference(2, k) = -0.03 + velocity * k * DT;
    reference(3, k) = 123.0;
  }
  return reference;
}

void same_armor_fast_motion()
{
  auto reference = ramp(2.0);
  const auto before = reference;
  ReferenceTags tags;
  tags.fill({2, true});
  fill_pitch_reference_velocity(reference, tags);
  require((reference.topRows<3>() - before.topRows<3>()).norm() == 0,
          "Pitch differentiation must preserve positions and both yaw rows");
  for (int k = 0; k < HORIZON; ++k)
    require(std::abs(reference(3, k) - 2.0) < 1e-12,
            "Continuous fast motion and horizon endpoints must retain their velocity");
}

void different_height_switch()
{
  auto reference = ramp(0.2);
  ReferenceTags tags;
  for (int k = 0; k < HORIZON; ++k) {
    tags[k] = {k < 30 ? 0 : 1, true};
    if (k >= 30)
      reference(2, k) += 1.2;
  }
  const auto positions = reference.row(2).eval();
  fill_pitch_reference_velocity(reference, tags);
  require((reference.row(2) - positions).norm() == 0, "Keep the target position switch");
  for (int k = 0; k < HORIZON; ++k)
    require(std::abs(reference(3, k) - 0.2) < 1e-12,
            "Use each armor's motion rather than differentiating its height switch");
}

void isolated_armor_and_invalid_samples()
{
  Trajectory reference = Trajectory::Zero();
  ReferenceTags tags;
  tags.fill({0, true});
  tags[20] = {1, true};
  reference(2, 20) = 5.0;
  fill_pitch_reference_velocity(reference, tags);
  require(reference.row(3).norm() == 0,
          "A-B-A must not connect an isolated B sample to either A neighbor");

  for (const ReferenceTag invalid :
       {ReferenceTag{0, false}, ReferenceTag{-1, false}, ReferenceTag{-1, true}}) {
    reference = ramp(0.2);
    tags.fill({0, true});
    tags[15] = invalid;
    reference(2, 15) = reference(2, 14);  // Position held after an unavailable solution.
    fill_pitch_reference_velocity(reference, tags);
    require(reference(3, 15) == 0, "Invalid/unknown samples have no velocity reference");
    require(std::abs(reference(3, 14) - 0.2) < 1e-12 && std::abs(reference(3, 16) - 0.2) < 1e-12,
            "A held sample must split even a same-ID trajectory");
  }

  reference = ramp(0.2);
  tags.fill({});
  tags[30] = {3, true};
  fill_pitch_reference_velocity(reference, tags);
  require(reference.row(3).norm() == 0,
          "One valid sample among invalid samples cannot define a continuous velocity");
}
}  // namespace

int main()
{
  try {
    same_armor_fast_motion();
    different_height_switch();
    isolated_armor_and_invalid_samples();
    std::cout << "PASS pitch reference continuity, switches, invalid samples and endpoints\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL pitch reference: " << error.what() << '\n';
    return 1;
  }
}
