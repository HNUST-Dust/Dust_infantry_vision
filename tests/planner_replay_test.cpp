#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "src/planner_diagnostics.hpp"

using nlohmann::json;
using namespace auto_aim;
using namespace auto_aim::runtime;

namespace
{
void require(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}
struct Sample {
  json record;
  AxisReference yaw_reference = AxisReference::Zero(), pitch_reference = AxisReference::Zero();
  Eigen::Vector2d yaw_x0 = Eigen::Vector2d::Zero(), pitch_x0 = Eigen::Vector2d::Zero();
  MpcConfig yaw_config, pitch_config;
};
Sample parse_sample(const json &j)
{
  require(j.at("schema_version") == 1, "Unsupported trace schema");
  require(j.at("units") == "rad,rad/s,rad/s2,s", "Trace units mismatch");
  require(j.at("solver_mode") == "cold", "Only cold-start replay is supported");
  Sample s;
  s.record = j;
  s.yaw_config = mpc_config_from_json(j.at("yaw_config"));
  s.pitch_config = mpc_config_from_json(j.at("pitch_config"));
  if (!j.at("reference_available").get<bool>()) return s;
  const auto &r = j.at("reference");
  require(r.is_array() && r.size() == 4, "Reference must have four rows");
  for (int row = 0; row < 4; ++row) {
    require(r.at(row).is_array() && r.at(row).size() == HORIZON, "Reference horizon mismatch");
    for (int k = 0; k < HORIZON; ++k) {
      const double value = r.at(row).at(k).get<double>();
      require(std::isfinite(value), "Nonfinite reference");
      if (row < 2) s.yaw_reference(row, k) = value;
      else s.pitch_reference(row - 2, k) = value;
    }
  }
  const auto &state = j.at("initial_state");
  s.yaw_x0 = {0.0, state.at("yaw_vel").get<double>()};
  s.pitch_x0 = {state.at("pitch").get<double>(), state.at("pitch_vel").get<double>()};
  require(s.yaw_x0.allFinite() && s.pitch_x0.allFinite(), "Nonfinite initial state");
  return s;
}
void close_number(double actual, const json &expected, const std::string &field,
                  double tolerance = 1e-8)
{
  if (expected.is_null()) {
    require(!std::isfinite(actual), "Expected unavailable metric: " + field);
    return;
  }
  const double value = expected.get<double>();
  require(std::isfinite(actual) && std::abs(actual - value) <= tolerance * (1.0 + std::abs(value)),
          "Replay mismatch: " + field);
}
void compare(const MpcDiagnostics &d, const json &expected)
{
  require(to_string(d.status) == expected.at("status"), "MPC status mismatch");
  require(d.solved == expected.at("solved").get<bool>(), "MPC solved mismatch");
  require(d.iterations == expected.at("iterations").get<int>(), "MPC iterations mismatch");
  require(d.library_return_code == expected.at("library_return_code").get<int>(), "MPC return mismatch");
  require(d.library_solved == expected.at("library_solved").get<bool>(), "MPC library solved mismatch");
  close_number(d.primal_residual, expected.at("primal_residual"), "primal");
  close_number(d.dual_residual, expected.at("dual_residual"), "dual");
  close_number(d.input_violation, expected.at("input_violation"), "input violation");
  close_number(d.state_primal_residual, expected.at("state_primal_residual"), "state primal");
  close_number(d.input_primal_residual, expected.at("input_primal_residual"), "input primal");
  close_number(d.state_dual_residual, expected.at("state_dual_residual"), "state dual");
  close_number(d.input_dual_residual, expected.at("input_dual_residual"), "input dual");
  close_number(d.initial_state_error, expected.at("initial_state_error"), "initial state error");
  close_number(d.max_dynamics_error, expected.at("max_dynamics_error"), "dynamics error");
}
void reproduce(const Sample &s)
{
  require(s.record.at("build_id") == DUST_MPC_BUILD_ID, "Solver build mismatch; use sweep for comparisons");
  if (!s.record.at("reference_available").get<bool>()) return;
  for (int axis = 0; axis < 2; ++axis) {
    const std::string name = axis == 0 ? "yaw" : "pitch";
    const auto &expected = s.record.at(name + "_mpc");
    if (!expected.at("attempted").get<bool>()) continue;
    MpcAxis solver(axis == 0 ? s.yaw_config : s.pitch_config);
    const auto d = solver.solve(axis == 0 ? s.yaw_x0 : s.pitch_x0,
                                axis == 0 ? s.yaw_reference : s.pitch_reference);
    compare(d, expected);
    if (s.record.at("control").get<bool>()) {
      const auto &output = s.record.at("output");
      close_number(solver.states()(0, COMMAND_STEP) +
                      (axis == 0 ? s.record.at("yaw_origin").get<double>() : 0.0),
                    output.at("absolute_" + name), "absolute " + name, 1e-6);
      close_number(solver.states()(1, COMMAND_STEP), output.at(name + "_vel"), name + " velocity", 1e-6);
      close_number(solver.inputs()(0, 0) * solver.config().max_acc, output.at(name + "_acc"),
                    name + " acceleration", 1e-6);
    }
  }
}
void self_test()
{
  const auto now = std::chrono::steady_clock::now();
  PlannerDiagnosticEvent event;
  event.plan_id = 1;
  auto trace = std::make_shared<PlanningTrace>();
  event.trace = trace;
  trace->reference_available = true;
  trace->reference.row(0).setConstant(0.1);
  trace->reference.row(2).setConstant(-0.03);
  trace->pitch_config.max_acc = 100;
  auto &p = event.plan;
  p.planned_at = now;
  p.initial_state.timestamp = now;
  MpcAxis yaw(event.trace->yaw_config), pitch(event.trace->pitch_config);
  p.yaw_mpc = yaw.solve({0, 0}, event.trace->reference.topRows<2>());
  p.pitch_mpc = pitch.solve({0, 0}, event.trace->reference.bottomRows<2>());
  require(p.yaw_mpc.solved && p.pitch_mpc.solved, "Self-test solve");
  p.control = true;
  p.status = PlanStatus::ok;
  p.absolute_yaw = yaw.states()(0, COMMAND_STEP);
  p.absolute_pitch = pitch.states()(0, COMMAND_STEP);
  p.yaw_vel = yaw.states()(1, COMMAND_STEP);
  p.pitch_vel = pitch.states()(1, COMMAND_STEP);
  p.yaw_acc = yaw.inputs()(0, 0) * 150;
  p.pitch_acc = pitch.inputs()(0, 0) * 100;
  const auto j = json::parse(planner_trace_json(event, now, "self-test").dump());
  reproduce(parse_sample(j));
  auto bad = j;
  bad["yaw_config"]["a"][0][1] = 0.02;
  bool rejected = false;
  try { parse_sample(bad); } catch (const std::exception &) { rejected = true; }
  require(rejected, "Reject incompatible dynamics");
  bad = j;
  bad["schema_version"] = 2;
  rejected = false;
  try { parse_sample(bad); } catch (const std::exception &) { rejected = true; }
  require(rejected, "Reject unknown schema");
  bad = j;
  bad["reference"][0][0] = nullptr;
  rejected = false;
  try { parse_sample(bad); } catch (const std::exception &) { rejected = true; }
  require(rejected, "Reject nonnumeric reference");
  trace->yaw_config.max_iterations = 1;
  MpcAxis limited(trace->yaw_config);
  p.yaw_mpc = limited.solve({0, 0}, trace->reference.topRows<2>());
  require(p.yaw_mpc.status == MpcStatus::iteration_limit, "Failure sample exhausted iterations");
  p.control = false;
  p.status = PlanStatus::mpc_failed;
  reproduce(parse_sample(json::parse(planner_trace_json(event, now, "failed").dump())));
  std::cout << "PASS planner replay success/failure round-trip, model/schema validation\n";
}
void sweep(const std::vector<Sample> &samples)
{
  std::cout << "rho,max_iter,axis,samples,solved,p95_solve_ms,p99_solve_ms,max_solve_ms,over_2ms,over_10ms\n";
  for (double rho : {0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5}) {
    for (int iterations : {100, 200, 300, 500}) {
      std::vector<double> combined_time(samples.size(), 0.0);
      std::vector<bool> combined_solved(samples.size(), true);
      std::vector<int> attempted_mask(samples.size(), 0);
      for (int axis = 0; axis < 2; ++axis) {
        std::vector<double> elapsed;
        int solved = 0, over2 = 0, over10 = 0;
        for (size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
          const auto &s = samples[sample_index];
          if (!s.record.at("reference_available").get<bool>()) continue;
          const std::string name = axis == 0 ? "yaw" : "pitch";
          if (!s.record.at(name + "_mpc").at("attempted").get<bool>()) continue;
          auto config = axis == 0 ? s.yaw_config : s.pitch_config;
          config.rho = rho;
          config.max_iterations = iterations;
          MpcAxis solver(config);
          const auto d = solver.solve(axis == 0 ? s.yaw_x0 : s.pitch_x0,
                                      axis == 0 ? s.yaw_reference : s.pitch_reference);
          solved += d.solved;
          elapsed.push_back(d.solve_time_ms);
          over2 += d.solve_time_ms > 2.0;
          over10 += d.solve_time_ms > 10.0;
          combined_time[sample_index] += d.solve_time_ms;
          combined_solved[sample_index] = combined_solved[sample_index] && d.solved;
          attempted_mask[sample_index] |= 1 << axis;
        }
        std::sort(elapsed.begin(), elapsed.end());
        std::cout << rho << ',' << iterations << ',' << (axis == 0 ? "yaw" : "pitch") << ','
                  << elapsed.size() << ',' << solved << ',';
        if (elapsed.empty()) std::cout << "nan,nan,nan";
        else std::cout << elapsed[static_cast<size_t>(std::ceil(elapsed.size() * .95)) - 1]
                       << ',' << elapsed[static_cast<size_t>(std::ceil(elapsed.size() * .99)) - 1]
                       << ',' << elapsed.back();
        std::cout << ',' << over2 << ',' << over10 << '\n';
      }
      // Combined timing is the sum of the two solves; reference generation/setup are excluded.
      std::vector<double> combined;
      int combined_successes = 0;
      for (size_t i = 0; i < samples.size(); ++i) {
        if (attempted_mask[i] == 3) {
          combined.push_back(combined_time[i]);
          combined_successes += combined_solved[i];
        }
      }
      if (!combined.empty()) {
        const int over2 = std::count_if(combined.begin(), combined.end(), [](double t) { return t > 2; });
        const int over10 = std::count_if(combined.begin(), combined.end(), [](double t) { return t > 10; });
        std::sort(combined.begin(), combined.end());
        std::cout << rho << ',' << iterations << ",combined," << combined.size() << ',' << combined_successes
                  << ',' << combined[static_cast<size_t>(std::ceil(combined.size() * .95)) - 1]
                  << ',' << combined[static_cast<size_t>(std::ceil(combined.size() * .99)) - 1]
                  << ',' << combined.back() << ',' << over2 << ',' << over10 << '\n';
      }
    }
  }
}
} // namespace

int main(int argc, char **argv)
{
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") { self_test(); return 0; }
    if (argc < 2 || argc > 3) {
      std::cerr << "Usage: planner_replay_test trace.jsonl [--mode=reproduce|--mode=sweep]\n";
      return 2;
    }
    const std::string mode = argc == 3 ? argv[2] : "--mode=reproduce";
    require(mode == "--mode=reproduce" || mode == "--mode=sweep", "Invalid mode argument");
    std::ifstream input(argv[1]);
    require(bool(input), "Cannot open trace");
    std::vector<Sample> samples;
    std::string line;
    while (std::getline(input, line)) {
      require(line.size() <= 1024 * 1024, "Trace line too large");
      if (!line.empty()) samples.push_back(parse_sample(json::parse(line)));
    }
    require(!input.bad() && !samples.empty(), "Empty or unreadable trace");
    if (mode == "--mode=sweep") sweep(samples);
    else {
      size_t skipped = 0;
      for (const auto &s : samples) {
        reproduce(s);
        skipped += !s.record.at("reference_available").get<bool>();
      }
      std::cout << "PASS replay records=" << samples.size() << " upstream_rejections=" << skipped << '\n';
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL replay: " << e.what() << '\n';
    return 1;
  }
}
