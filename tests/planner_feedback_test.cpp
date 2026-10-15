#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/angle_rate_estimator.hpp"
#include "tools/trajectory.hpp"

using namespace std::chrono_literals;

namespace
{
void require(bool condition, const std::string &message)
{
  if (!condition)
    throw std::runtime_error(message);
}

struct Config {
  std::filesystem::path directory;
  std::string path;
  explicit Config(int iterations = 0, bool auto_fire = true, double rho = 0.0)
  {
    char pattern[] = "/tmp/dust-planner-feedback-XXXXXX";
    const char *created = mkdtemp(pattern);
    if (!created)
      throw std::runtime_error("mkdtemp failed");
    directory = created;
    path = (directory / "test.yaml").string();
    std::ofstream out(path);
    out << "yaw_offset: 0\npitch_offset: 0\n"
        << "fire_thresh_high_speed: 0.01\nfire_thresh_low_speed: 0.01\n"
        << "decision_speed: 8\nhigh_speed_delay_time: 0.01\nlow_speed_delay_time: 0.01\n"
        << "comming_angle: 55\nleaving_angle: 20\nfirst_tolerance: 1\n"
        << "second_tolerance: 0.5\njudge_distance: 5\nauto_fire: " << (auto_fire ? "true" : "false")
        << "\n"
        << "max_yaw_acc: 150\nmax_pitch_acc: 100\nQ_yaw: [9e9, 0]\n"
        << "Q_pitch: [9e9, 0]\nR_yaw: [1]\nR_pitch: [1]\n"
        << "\n";
    if (iterations > 0) out << "mpc_max_iter: " << iterations << "\n";
    if (rho > 0.0) out << "mpc_rho: " << rho << "\n";
    if (!out)
      throw std::runtime_error("test config write failed");
  }
  ~Config() { std::filesystem::remove_all(directory); }
};

Eigen::Matrix3d rotation(double yaw, double pitch, double roll = 0.0)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitY()))
      .toRotationMatrix();
}

auto_aim::PlanningState state_at(std::chrono::steady_clock::time_point at, double yaw = 0.0,
                                 double pitch = 0.0, double yaw_rate = 0.0, double pitch_rate = 0.0,
                                 double roll = 0.0)
{
  auto_aim::PlanningState state;
  state.yaw = yaw;
  state.pitch = pitch;
  state.yaw_vel = yaw_rate;
  state.pitch_vel = pitch_rate;
  state.R_gimbal2world = rotation(yaw, pitch, roll);
  state.timestamp = at;
  state.valid = true;
  state.rates_measured = true;
  return state;
}

auto_aim::Target target_at(std::chrono::steady_clock::time_point at, double yaw, double height = 0)
{
  auto_aim::Armor armor(0, 1.0F, cv::Rect(0, 0, 20, 10), {{0, 0}, {20, 0}, {20, 10}, {0, 10}});
  armor.xyz_in_world = {3 * std::sin(yaw), -3 * std::cos(yaw), height};
  armor.ypr_in_world = {yaw - M_PI / 2, 0, 0};
  return auto_aim::Target(armor, at, 0.25, 4, Eigen::VectorXd::Ones(11));
}

void check_solver(const auto_aim::Plan &plan, const std::string &label)
{
  if (!plan.control) {
    std::cerr << label << ": yaw iter=" << plan.yaw_mpc.iterations
              << " pri=" << plan.yaw_mpc.primal_residual << " dua=" << plan.yaw_mpc.dual_residual
              << "; pitch iter=" << plan.pitch_mpc.iterations
              << " pri=" << plan.pitch_mpc.primal_residual
              << " dua=" << plan.pitch_mpc.dual_residual << "\n";
  }
  require(plan.control && plan.yaw_mpc.solved && plan.pitch_mpc.solved, label + ": no valid solve");
  require(std::abs(plan.yaw_acc) <= 150.002 && std::abs(plan.pitch_acc) <= 100.002,
          label + ": acceleration limit");
}

void check_tiny_costs()
{
  // Compare the first action to the analytic scalar infinite-horizon LQR gain.
  const double expected = ((1 + std::sqrt(1.4)) / 2) / (0.1 + (1 + std::sqrt(1.4)) / 2);
  for (double rho : {0.01, 0.3, 1.0}) {
    TinySolver *solver = nullptr;
    const auto scalar = [](double value) { return Eigen::MatrixXd::Constant(1, 1, value); };
    require(tiny_setup(&solver, scalar(1), scalar(1), scalar(0), scalar(1), scalar(0.1), rho, 1, 1,
                       40, 0) == 0,
            "scalar setup");
    tiny_set_bound_constraints(
        solver, Eigen::MatrixXd::Constant(1, 40, -100), Eigen::MatrixXd::Constant(1, 40, 100),
        Eigen::MatrixXd::Constant(1, 39, -10), Eigen::MatrixXd::Constant(1, 39, 10));
    solver->settings->max_iter = 1000;
    solver->settings->abs_pri_tol = 1e-8;
    solver->settings->abs_dua_tol = 1e-8;
    solver->work->Xref.setOnes();
    tiny_set_x0(solver, Eigen::VectorXd::Zero(1));
    const int status = tiny_solve(solver);
    const double action = solver->work->u(0, 0);
    tiny_cleanup(solver);
    require(status == 0 && std::abs(action - expected) < 1e-4,
            "ADMM penalty must not change the reference cost");
  }
}

void check_feedback()
{
  io::Gimbal gimbal("unused", true);
  const auto t = std::chrono::steady_clock::now();
  gimbal.set_external_feedback(Eigen::Quaterniond(rotation(0.1, -0.05)), t);
  require(!gimbal.feedback().valid, "pose-only first sample has no rate");
  gimbal.set_external_feedback(Eigen::Quaterniond(rotation(0.11, -0.055)), t + 10ms);
  const auto sample = gimbal.feedback();
  require(sample.valid && !sample.rates_measured, "estimated rate provenance");
  require(std::abs(sample.state.yaw_vel - 1) < 1e-5 &&
              std::abs(sample.state.pitch_vel + 0.5) < 1e-5,
          "external velocity");
  require(!sample.fresh_at(t + 9ms, 20ms) && !sample.fresh_at(t + 31ms, 20ms),
          "future/stale feedback");
  require(gimbal.feedback_timestamp() == t + 10ms, "external feedback age must not refresh itself");
  std::atomic<bool> done{false};
  std::thread writer([&]() {
    for (int i = 1; i <= 1000; ++i)
      gimbal.set_external_feedback(Eigen::Quaterniond(rotation(0.11 + i * 0.0001, -0.055)),
                                   t + 10ms + i * 1ms);
    done = true;
  });
  bool coherent = true;
  do {
    const auto snapshot = gimbal.feedback();
    const auto angles = tools::eulers(snapshot.q, 2, 0, 1);
    coherent = coherent && std::abs(angles[0] - snapshot.state.yaw) < 1e-6 &&
               std::abs(angles[1] - snapshot.state.pitch) < 1e-6 &&
               snapshot.received_at == t + 10ms + (snapshot.sequence - 2) * 1ms;
  } while (!done);
  writer.join();
  require(coherent, "atomic pose/state/timestamp/sequence snapshot");
  Eigen::Quaterniond invalid = Eigen::Quaterniond::Identity();
  invalid.w() = std::numeric_limits<double>::quiet_NaN();
  gimbal.set_external_feedback(invalid, t);
  require(!gimbal.feedback().valid, "invalid orientation");
  tools::AngleRateEstimator estimator;
  require(!estimator.update({M_PI - 0.01, 0}, t), "first derivative unavailable");
  const auto rates = estimator.update({-M_PI + 0.01, 0}, t + 10ms);
  require(rates && std::abs((*rates)[0] - 2) < 1e-8, "rate unwrap");
  require(!estimator.update({0, 0}, t + 10ms), "duplicate timestamp");
  require(!estimator.update({0, 0}, t + 1s), "discontinuous replay");
}
} // namespace

int main(int argc, char **argv)
{
  try {
    check_tiny_costs();
    check_feedback();
    Config config(argc > 1 ? std::stoi(argv[1]) : 0, true, argc > 2 ? std::stod(argv[2]) : 0.0);
    auto_aim::Planner planner(config.path);
    const auto t = std::chrono::steady_clock::now();
    auto target = target_at(t, 0.12);
    auto_aim::PlanningTrace trace;
    const auto traced = planner.plan_at(target, t, 23, state_at(t), &trace);
    check_solver(traced, "traced solve");
    require(traced.status == auto_aim::PlanStatus::ok && traced.aim_debug.candidate_available &&
            traced.aim_debug.selection_valid && traced.aim_debug.ballistic_converged &&
            traced.aim_visible_at(t + 2ms), "completed aim snapshot");
    require(trace.reference_available && trace.reference.allFinite() && trace.target_available &&
            trace.yaw_config.max_iterations == (argc > 1 ? std::stoi(argv[1]) : 300) &&
            trace.received_state.timestamp == t, "complete optional trace");
    auto_aim::MpcAxis replay(trace.yaw_config);
    const auto replayed = replay.solve({0, traced.initial_state.yaw_vel}, trace.reference.topRows<2>());
    require(replayed.status == traced.yaw_mpc.status &&
            replayed.iterations == traced.yaw_mpc.iterations &&
            replayed.primal_residual == traced.yaw_mpc.primal_residual,
            "planner and replay share exact solve path");
    for (double velocity : {-0.4, 0.0, 0.4}) {
      const auto measured = state_at(t, 0.07, -0.02, velocity, 0.15, 0.2);
      const auto plan = planner.plan_at(target, t + 5ms, 23, measured);
      check_solver(plan, "measured initial state");
      require(std::abs(plan.initial_state.yaw - (0.07 + velocity * 0.005)) < 1e-10 &&
                  plan.initial_state.yaw_vel == velocity &&
                  std::abs(plan.initial_state.pitch - (-0.02 + 0.15 * 0.005)) < 1e-10 &&
                  plan.initial_state.pitch_vel == 0.15,
              "measured x0 and age propagation");
      require(std::abs(plan.absolute_yaw - plan.initial_state.yaw - velocity * 0.01) < 1e-9,
              "first Euler step must start from feedback, not reference");
      require(std::abs(plan.absolute_pitch - plan.initial_state.pitch - 0.15 * 0.01) < 1e-9,
              "pitch starts from measured angle and rate");
      require(plan.valid_at == plan.planned_at + 10ms, "command time");
      require(std::abs(tools::limit_rad(plan.absolute_target_yaw - 0.12)) < 1e-6,
              "absolute reference with nonzero pitch and roll");
      const auto a = plan.relative_to(0.08, -0.01);
      const auto b = plan.relative_to(0.09, -0.02);
      require(std::abs(tools::limit_rad(0.08 + a.yaw - 0.09 - b.yaw)) < 1e-7 &&
                  std::abs(-0.01 + a.pitch + 0.02 - b.pitch) < 1e-7,
              "repeated sends preserve absolute target");
    }
    for (double angle : {-3.0, -1.5, -0.5, 0.5, 1.5, 3.0}) {
      const auto p = planner.plan_at(target_at(t, angle), t, 23, state_at(t));
      check_solver(p, "large step");
      require(p.yaw_acc * angle > 0, "first acceleration follows target");
      require(std::abs(p.absolute_yaw) < 1e-9, "must not issue the 300ms prediction");
    }
    const auto wrapped =
        planner.plan_at(target_at(t, -M_PI + 0.01), t, 23, state_at(t, M_PI - 0.01));
    check_solver(wrapped, "pi boundary");
    require(std::abs(wrapped.target_yaw - 0.02) < 1e-6, "shortest yaw branch");
    const auto multi_turn =
        planner.plan_at(target_at(t, -M_PI + 0.01), t, 23, state_at(t, M_PI - 0.01 + 20 * M_PI));
    check_solver(multi_turn, "multi-turn feedback");
    require(std::abs(multi_turn.target_yaw - wrapped.target_yaw) < 1e-6,
            "equivalent multi-turn yaw coordinate");
    for (double velocity : {-2.0, 2.0}) {
      const auto braking = planner.plan_at(target, t, 23, state_at(t, 0.12, -0.02, velocity));
      check_solver(braking, "braking");
      require(braking.yaw_acc * velocity < 0, "brake measured motion at a static target");
    }
    auto invalid = state_at(t);
    invalid.yaw_vel = std::numeric_limits<double>::infinity();
    require(planner.plan_at(target, t, 23, invalid).status == auto_aim::PlanStatus::state_nonfinite,
            "nonfinite rate reason");
    invalid = state_at(t);
    invalid.valid = false;
    require(planner.plan_at(target, t, 23, invalid).status == auto_aim::PlanStatus::feedback_invalid,
            "missing feedback reason");
    require(planner.plan_at(target, t - 1ms, 23, state_at(t)).status == auto_aim::PlanStatus::feedback_future,
            "future sample reason");
    require(planner.plan_at(target, t + 21ms, 23, state_at(t)).status == auto_aim::PlanStatus::feedback_stale,
            "stale sample reason");
    const auto absent = planner.plan_at(std::nullopt, t, 23, state_at(t), &trace);
    require(absent.status == auto_aim::PlanStatus::no_target && !absent.yaw_mpc.attempted &&
            !absent.pitch_mpc.attempted && !trace.reference_available, "upstream failure resets trace");
    invalid = state_at(t);
    invalid.timestamp = {};
    require(planner.plan_at(target, t, 23, invalid).status == auto_aim::PlanStatus::timestamp_missing,
            "missing timestamp reason");
    invalid = state_at(t);
    invalid.R_gimbal2world = rotation(0.2, 0);
    require(planner.plan_at(target, t, 23, invalid).status == auto_aim::PlanStatus::state_frame_mismatch,
            "coordinate mismatch reason");
    invalid.R_gimbal2world.setZero();
    require(planner.plan_at(target, t, 23, invalid).status == auto_aim::PlanStatus::rotation_invalid,
            "invalid rotation reason");

    tools::Trajectory ballistic(23, 3, 0);
    const auto aligned = state_at(t, 0.12, -ballistic.pitch);
    // A prior successful diagnostic solve must not affect the first-fire check.
    planner.plan_at(std::nullopt, t, 23, aligned);
    require(!planner.plan_at(target, t, 23, aligned).fire, "first plan cannot fire");
    auto next = aligned;
    next.timestamp = t + 2ms;
    const auto fire = planner.plan_at(target, t + 2ms, 23, next);
    check_solver(fire, "aligned target");
    require(fire.fire, "aligned measured feedback can fire");
    next.timestamp = t + 4ms;
    next.rates_measured = false;
    require(!planner.plan_at(target, t + 4ms, 23, next).fire,
            "estimated rates cannot authorize fire");
    require(!fire.relative_to(0.5, aligned.pitch).fire, "send-time actual position gate");
    Config disabled(200, false);
    auto_aim::Planner disabled_planner(disabled.path);
    disabled_planner.plan_at(target, t, 23, aligned);
    next = aligned;
    next.timestamp = t + 2ms;
    require(!disabled_planner.plan_at(target, t + 2ms, 23, next).fire, "auto_fire false");

    Config limited(1);
    auto_aim::Planner limited_planner(limited.path);
    const auto failed = limited_planner.plan_at(target, t, 23, state_at(t));
    require(!failed.control && !failed.fire, "iteration exhaustion must reject command");
    require(failed.status == auto_aim::PlanStatus::mpc_failed &&
            failed.yaw_mpc.status == auto_aim::MpcStatus::iteration_limit &&
            failed.yaw_mpc.attempted && failed.pitch_mpc.attempted &&
            !failed.aim_debug.display_valid && failed.aim_debug.candidate_available,
            "iteration exhaustion retains candidate and both axis diagnostics");

    std::vector<double> elapsed;
    int moving_valid = 0;
    double moving_max_ms = 0.0;
    for (int i = 0; i < 80; ++i) {
      auto moving = target_at(t, 0.12, 0.1);
      moving.set_initial_omega(i % 2 == 0 ? 6.0 : -6.0);
      moving.jumped = true;
      moving.predict(t + i * 2ms);
      const auto begin = std::chrono::steady_clock::now();
      const auto plan = planner.plan_at(moving, t + i * 2ms, 23,
                                        state_at(t + i * 2ms, 0.1, -0.06, (i % 3 - 1) * 0.5));
      check_solver(plan, "moving target/armor switches");
      moving_valid += plan.control;
      moving_max_ms = std::max(moving_max_ms, std::chrono::duration<double, std::milli>(
                                                  std::chrono::steady_clock::now() - begin)
                                                  .count());
    }
    for (int i = 0; i < 120; ++i) {
      const auto begin = std::chrono::steady_clock::now();
      const auto plan = planner.plan_at(target, t, 23, state_at(t, 0.1, -0.02));
      check_solver(plan, "timing sample");
      elapsed.push_back(
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
              .count());
    }
    std::sort(elapsed.begin(), elapsed.end());
    std::cout << "PASS planner feedback; timing p50=" << elapsed[60] << "ms p95=" << elapsed[114]
              << "ms max=" << elapsed.back() << "ms; moving valid=" << moving_valid
              << "/80 max=" << moving_max_ms << "ms\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << "\n";
    return 1;
  }
}
