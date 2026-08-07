#ifdef NDEBUG
#undef NDEBUG
#endif

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <optional>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/crc.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/trajectory.hpp"

namespace
{
using namespace std::chrono_literals;

void test_queue_shutdown_and_latest_value()
{
  tools::ThreadSafeQueue<int> queue(1);
  std::atomic<bool> returned{false};
  std::thread waiter([&] {
    auto value = queue.wait_pop();
    assert(!value.has_value());
    returned = true;
  });
  std::this_thread::sleep_for(5ms);
  queue.close();
  waiter.join();
  assert(returned.load());
  assert(!queue.push(1));

  tools::ThreadSafeQueue<int, true> latest(2);
  assert(latest.push(1));
  assert(latest.push(2));
  assert(latest.push(3));
  auto first = latest.wait_pop_for(10ms);
  auto second = latest.wait_pop_for(10ms);
  assert(first && second && *first == 2 && *second == 3);

  bool rejected_zero_capacity = false;
  try {
    tools::ThreadSafeQueue<int> invalid(0);
  } catch (const std::invalid_argument &) {
    rejected_zero_capacity = true;
  }
  assert(rejected_zero_capacity);
}

void test_trajectory_and_crc()
{
  const auto solvable = tools::Trajectory(22.0, 3.0, 0.2);
  assert(!solvable.unsolvable);
  assert(std::isfinite(solvable.fly_time) && solvable.fly_time > 0.0);
  assert(std::isfinite(solvable.pitch));

  const auto invalid = tools::Trajectory(0.0, 3.0, 0.2);
  assert(invalid.unsolvable && invalid.fly_time == 0.0);
  assert(tools::Trajectory(NAN, 3.0, 0.2).unsolvable);

  const std::vector<uint8_t> payload{1, 2, 3, 4};
  std::vector<uint8_t> packet = payload;
  packet.push_back(tools::get_crc8(payload.data(), payload.size()));
  assert(tools::check_crc8(packet.data(), packet.size()));
  assert(!tools::check_crc8(packet.data(), 0));
  assert(!tools::check_crc8(nullptr, packet.size()));
  assert(tools::get_crc8(nullptr, 1) == 0);

  const auto crc16 = tools::get_crc16(packet.data(), packet.size());
  packet.push_back(static_cast<uint8_t>(crc16 & 0xff));
  packet.push_back(static_cast<uint8_t>(crc16 >> 8));
  assert(tools::check_crc16(packet.data(), packet.size()));
  assert(!tools::check_crc16(packet.data(), 1));
  assert(!tools::check_crc16(nullptr, packet.size()));
  assert(tools::get_crc16(nullptr, 1) == 0);
}

void test_ekf_finite_and_nis_reset()
{
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(2, 2);
  tools::ExtendedKalmanFilter filter(x0, P0);
  const Eigen::MatrixXd F = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd Q = 0.01 * Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd R = 0.1 * Eigen::MatrixXd::Identity(2, 2);
  filter.predict(F, Q);
  filter.update(Eigen::Vector2d::Constant(100.0), H, R);
  assert(filter.data["nis_fail"] == 1.0);
  filter.x = Eigen::Vector2d::Constant(100.0);
  filter.P = Eigen::MatrixXd::Identity(2, 2);
  filter.update(Eigen::Vector2d::Constant(100.0), H, R);
  assert(filter.data["nis_fail"] == 0.0);
  assert(filter.x.allFinite() && filter.P.allFinite());
}

void test_planner_overloads(const char * config_path)
{
  auto_aim::Planner planner(config_path);
  assert(!planner.plan(std::nullopt, 22.0).control);

  auto_aim::Target target(3.0, 0.2, 0.1, 0.1);
  const auto plan = planner.plan(target, 22.0);
  assert(plan.control);
  assert(std::isfinite(plan.target_yaw) && std::isfinite(plan.target_pitch));
}
}  // namespace

int main(int argc, char ** argv)
{
  assert(argc >= 2);
  test_queue_shutdown_and_latest_value();
  test_trajectory_and_crc();
  test_ekf_finite_and_nis_reset();
  test_planner_overloads(argv[1]);
  return 0;
}
