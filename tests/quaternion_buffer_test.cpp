#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <future>
#include <limits>

#include "tools/quaternion_buffer.hpp"

using namespace std::chrono_literals;

namespace
{
Eigen::Quaterniond yaw(double degrees)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(degrees * M_PI / 180, Eigen::Vector3d::UnitZ()));
}

void expect_yaw(const std::optional<tools::QuaternionSample> & sample, double degrees)
{
  assert(sample);
  assert(sample->q.angularDistance(yaw(degrees)) < 1e-10);
  assert(std::abs(sample->q.norm() - 1) < 1e-12);
}
}  // namespace

int main()
{
  const auto t = tools::QuaternionBuffer::Clock::time_point{};
  tools::QuaternionBuffer history(3);
  assert(!history.at(t, 0ms));
  assert(history.push(yaw(0), t));
  assert(history.push(yaw(10), t + 10ms));
  assert(history.push(yaw(20), t + 20ms));

  // The original Gimbal::q returned 10 degrees for this query instead of 15.
  auto sample = history.at(t + 15ms, 0ms);
  expect_yaw(sample, 15);
  assert(sample->before == t + 10ms && sample->after == t + 20ms);
  expect_yaw(history.at(t + 16ms, 0ms), 16);
  expect_yaw(history.at(t + 15ms, 0ms), 15);
  expect_yaw(history.at(t + 5ms, 0ms), 5);
  expect_yaw(history.at(t, 0ms), 0);
  expect_yaw(history.at(t + 20ms, 0ms), 20);
  assert(!history.at(t - 1ms, 0ms));
  assert(!history.at(t + 21ms, 1ms));

  // A full history evicts the oldest pose, never the newly arriving pose.
  assert(history.push(yaw(30), t + 30ms));
  assert(!history.at(t + 5ms, 0ms));
  expect_yaw(history.at(t + 25ms, 0ms), 25);
  assert(!history.push(yaw(99), t + 30ms));
  assert(!history.push(yaw(99), t + 29ms));
  expect_yaw(history.at(t + 30ms, 0ms), 30);

  Eigen::Quaterniond bad(0, 0, 0, 0);
  assert(!history.push(bad, t + 40ms));
  bad.w() = std::numeric_limits<double>::quiet_NaN();
  assert(!history.push(bad, t + 40ms));
  bad.w() = std::numeric_limits<double>::infinity();
  assert(!history.push(bad, t + 40ms));
  auto scaled = yaw(40);
  scaled.coeffs() *= -2;  // Unit normalization and antipodal quaternion interpolation.
  assert(history.push(scaled, t + 40ms));
  expect_yaw(history.at(t + 35ms, 0ms), 35);

  assert(history.push(yaw(90), t + 100ms));
  assert(!history.at(t + 70ms, 0ms));
  history.clear();
  assert(!history.at(t + 35ms, 0ms));
  assert(!history.wait_first(0ms));

  assert(history.push(yaw(0), t));
  assert(history.wait_first(0ms));
  auto reader = std::async(std::launch::async, [&] { return history.at(t + 5ms, 1s); });
  assert(reader.wait_for(10ms) == std::future_status::timeout);
  assert(history.push(yaw(10), t + 10ms));
  assert(reader.wait_for(1s) == std::future_status::ready);
  expect_yaw(reader.get(), 5);

  auto first = std::async(std::launch::async, [&] { return history.at(t + 3ms, 0ms); });
  auto second = std::async(std::launch::async, [&] { return history.at(t + 7ms, 0ms); });
  expect_yaw(first.get(), 3);
  expect_yaw(second.get(), 7);

  auto waiting = std::async(std::launch::async, [&] { return history.at(t + 15ms, 1s); });
  assert(waiting.wait_for(10ms) == std::future_status::timeout);
  history.close();
  assert(waiting.wait_for(1s) == std::future_status::ready);
  assert(!waiting.get());
  assert(!history.push(yaw(20), t + 20ms));
  assert(!history.at(t + 5ms, 0ms));
  return 0;
}
