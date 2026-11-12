#include <cmath>
#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <unistd.h>
#include <vector>

#include "src/planner_diagnostics.hpp"

using namespace auto_aim;

void require(bool value, const char * message)
{
  if (!value)
    throw std::runtime_error(message);
}

struct RecordedPitch
{
  double time;
  Eigen::Vector2d initial;
  AxisReference reference;
};

double quantile(std::vector<double> values, double fraction)
{
  std::sort(values.begin(), values.end());
  return values.empty() ? NAN
                        : values[static_cast<size_t>(std::ceil(values.size() * fraction)) - 1];
}

int benchmark(const std::string & path)
{
  using auto_aim::runtime::mpc_config_from_json;
  std::ifstream input(path);
  require(bool(input), "Cannot open pitch benchmark trace");
  std::vector<RecordedPitch> records;
  MpcConfig baseline;
  nlohmann::json original_config;
  std::string line;
  while (std::getline(input, line)) {
    const auto j = nlohmann::json::parse(line);
    if (!j.at("reference_available").get<bool>() || !j.at("pitch_mpc").at("attempted").get<bool>())
      continue;
    if (records.empty()) {
      original_config = j.at("pitch_config");
      baseline = mpc_config_from_json(original_config);
    }
    require(j.at("pitch_config") == original_config,
            "Split traces containing multiple cost configurations");
    RecordedPitch sample;
    sample.time = j.at("planned_s").get<double>();
    sample.initial = {j.at("initial_state").at("pitch").get<double>(),
                      j.at("initial_state").at("pitch_vel").get<double>()};
    for (int k = 0; k < HORIZON; ++k)
      for (int r = 0; r < 2; ++r)
        sample.reference(r, k) = j.at("reference").at(r + 2).at(k).get<double>();
    records.push_back(sample);
  }
  require(records.size() > 1, "Benchmark requires actual pitch reference records");
  const double start = records.front().time;
  for (auto & sample : records)
    sample.time -= start;
  struct Choice
  {
    std::string name;
    MpcConfig config;
  };
  std::vector<Choice> choices{{"legacy", baseline}};
  for (double velocity_weight : {.05, .2, 1.}) {
    for (double acceleration_weight : {.3, 1., 3.}) {
      MpcConfig c = baseline;
      c.state_scale = {.1, 1.};
      c.q = {100., velocity_weight};
      c.r = acceleration_weight / 100.;
      choices.push_back(
        {"v" + std::to_string(velocity_weight) + "_a" + std::to_string(acceleration_weight), c});
    }
  }
  std::cout << std::setprecision(10);
  std::cout << "choice,scenario,delay_ms,failures,solve_p99_ms,error_rms_deg,error_p95_deg,"
               "tail_pp_deg,settle_s,peak_acc,acc_delta_rms,saturated_fraction\n";
  for (const auto & choice : choices) {
    std::cerr << "cost benchmark: " << choice.name << '\n';
    // Replayed states are fixed measured states; they test solvability/commands, not tracking improvement.
    MpcAxis replay(choice.config);
    std::vector<double> times;
    double last = 0, delta2 = 0, peak = 0;
    int failures = 0, saturated = 0, success = 0;
    for (const auto & sample : records) {
      const auto d = replay.solve(sample.initial, sample.reference);
      times.push_back(d.solve_time_ms);
      if (!d.solved) {
        ++failures;
        continue;
      }
      const double command = replay.inputs()(0, 0) * choice.config.max_acc;
      if (success)
        delta2 += (command - last) * (command - last);
      last = command;
      peak = std::max(peak, std::abs(command));
      saturated += std::abs(command) > choice.config.max_acc - 1e-6;
      ++success;
    }
    std::cout << choice.name << ",recorded_states,0," << failures << ',' << quantile(times, .99)
              << ",nan,nan,nan,nan," << peak << ',' << std::sqrt(delta2 / std::max(1, success - 1))
              << ',' << double(saturated) / std::max(1, success) << '\n';
    for (const std::string scenario : {"step", "ramp", "sine", "recorded_reference"}) {
      for (int delay_ms : {0, 10, 20}) {
        MpcAxis solver(choice.config);
        const double dt = .002;
        const double duration =
          scenario == "recorded_reference" ? records.back().time : (scenario == "ramp" ? 2. : 4.);
        double theta = scenario == "recorded_reference" ? records.front().initial[0]
                                                        : (scenario == "sine" ? -.03 : 0);
        double omega = scenario == "recorded_reference" ? records.front().initial[1] : 0;
        double filtered_omega = omega, integral = 0;
        struct Command
        {
          double at, theta, omega, alpha;
        };
        std::deque<Command> pending;
        Command applied{0, theta, 0, 0};
        std::vector<double> errors, tail, solve_times;
        size_t sample_index = 0;
        double last_command = 0, delta_sum = 0, peak_command = 0, last_unsettled = -dt;
        int failed = 0, sat = 0, count = 0;
        for (double t = 0; t < duration; t += dt) {
          AxisReference reference;
          if (scenario == "recorded_reference") {
            while (sample_index + 1 < records.size() && records[sample_index + 1].time <= t)
              ++sample_index;
            reference = records[sample_index].reference;
          } else {
            for (int k = 0; k < HORIZON; ++k) {
              const double future = t + k * DT;
              if (scenario == "step") {
                reference(0, k) = .1;
                reference(1, k) = 0;
              } else if (scenario == "ramp") {
                reference(0, k) = .003 + .1 * future;
                reference(1, k) = .1;
              } else {
                const double w = 2 * M_PI * .85;
                reference(0, k) = -.03 + .0232 * std::sin(w * future);
                reference(1, k) = .0232 * w * std::cos(w * future);
              }
            }
          }
          const double error = reference(0, COMMAND_STEP) - theta;
          if (t > .5 || scenario == "step")
            errors.push_back(std::abs(error));
          if (t > duration - 1)
            tail.push_back(error);
          if (std::abs(error) > .0035 || std::abs(omega) > .05)
            last_unsettled = t;
          const auto d = solver.solve({theta, omega}, reference);
          solve_times.push_back(d.solve_time_ms);
          if (d.solved) {
            const double alpha = solver.inputs()(0, 0) * choice.config.max_acc;
            pending.push_back({t + delay_ms / 1000., solver.states()(0, COMMAND_STEP),
                               solver.states()(1, COMMAND_STEP), alpha});
            if (count)
              delta_sum += (alpha - last_command) * (alpha - last_command);
            last_command = alpha;
            peak_command = std::max(peak_command, std::abs(alpha));
            sat += std::abs(alpha) > choice.config.max_acc - 1e-6;
          } else {
            ++failed;
            pending.push_back({t + delay_ms / 1000., theta, 0, 0});
          }
          while (!pending.empty() && pending.front().at <= t + 1e-10) {
            applied = pending.front();
            pending.pop_front();
          }
          // Assumed plant: nominal J, exact gravity cancellation, 100Hz rate LPF,
          // existing Kp=12 / Kp=2,Ki=1 and acceleration FF gain .5. Not hardware identification.
          filtered_omega += dt / (1 / (2 * M_PI * 100) + dt) * (omega - filtered_omega);
          const double speed_error = 12 * (applied.theta - theta) + applied.omega - filtered_omega;
          integral = std::clamp(integral + speed_error * dt, -3., 3.);
          const double physical_torque =
            std::clamp(2 * speed_error + integral + .5 * .212111 * applied.alpha - 1., -6., 6.);
          const double acceleration = (physical_torque + 1.) / .212111;
          theta += omega * dt + .5 * acceleration * dt * dt;
          omega += acceleration * dt;
          ++count;
        }
        double e2 = 0;
        for (double e : errors)
          e2 += e * e;
        const auto range = std::minmax_element(tail.begin(), tail.end());
        const double settle = last_unsettled < duration - dt * 1.5 ? last_unsettled + dt : -1;
        std::cout << choice.name << ',' << scenario << ',' << delay_ms << ',' << failed << ','
                  << quantile(solve_times, .99) << ',' << std::sqrt(e2 / errors.size()) * 180 / M_PI
                  << ',' << quantile(errors, .95) * 180 / M_PI << ','
                  << (*range.second - *range.first) * 180 / M_PI << ',' << settle << ','
                  << peak_command << ',' << std::sqrt(delta_sum / std::max(1, count - 1)) << ','
                  << double(sat) / count << '\n';
      }
    }
  }
  return 0;
}

int main(int argc, char ** argv)
{
  const auto file = std::filesystem::temp_directory_path() /
                    ("dust-mpc-cost-" + std::to_string(getpid()) + ".yaml");
  try {
    if (argc == 3 && std::string(argv[1]) == "--benchmark")
      return benchmark(argv[2]);
    require(argc == 1, "Usage: mpc_cost_test [--benchmark TRACE.jsonl]");
    {
      std::ofstream out(file);
      out << "max_pitch_acc: 10\nmpc_cost_pitch:\n  scales: [0.1, 1, 10]\n"
             "  weights: [1, 1, 0.3]\nmax_yaw_acc: 55\nQ_yaw: [9e9, 1e7]\nR_yaw: [1]\n";
    }
    auto config = MpcConfig::load(file.string(), "pitch");
    require(std::abs(config.q[0] - 100) < 1e-10 && std::abs(config.q[1] - 1) < 1e-10 &&
              std::abs(config.r - .003) < 1e-12,
            "Normalized cost must retain physical units");
    const auto model = mpc_model(config);
    const Eigen::Matrix2d scales = config.state_scale.asDiagonal();
    const Eigen::Matrix2d physical_a{{1, DT}, {0, 1}};
    require((scales * model.a - physical_a * scales).norm() < 1e-12 &&
              (scales * model.b - Eigen::Vector2d(0, MPC_INPUT_SCALE * DT)).norm() < 1e-12,
            "State normalization must preserve the physical dynamics");
    const auto yaw = MpcConfig::load(file.string(), "yaw");
    require(yaw.state_scale == Eigen::Vector2d::Ones() && yaw.q[0] == 9e9 && yaw.r == 1,
            "Legacy yaw configuration must be unchanged");
    const auto encoded = auto_aim::runtime::mpc_config_json(config);
    const auto decoded = auto_aim::runtime::mpc_config_from_json(encoded);
    require(decoded.state_scale == config.state_scale && decoded.q == config.q &&
              decoded.r == config.r,
            "Trace must round-trip normalized state and cost");
    auto legacy = auto_aim::runtime::mpc_config_json(yaw);
    legacy.erase("state_scale");
    require(auto_aim::runtime::mpc_config_from_json(legacy).state_scale == Eigen::Vector2d::Ones(),
            "Old traces use identity state scaling");
    for (double error : {-.1, -.01, .01, .1}) {
      MpcAxis solver(config);
      AxisReference reference = AxisReference::Zero();
      reference.row(0).setConstant(-.04 + error);
      const Eigen::Vector2d initial{-.04, .05};
      const auto diagnostic = solver.solve(initial, reference);
      if (!diagnostic.solved)
        throw std::runtime_error(std::string("Normalized reference failed: error=") +
                                 std::to_string(error) + " status=" + to_string(diagnostic.status));
      require((solver.states().col(0) - initial).norm() < 1e-12,
              "Public states must stay in rad and rad/s");
      for (int k = 0; k < HORIZON - 1; ++k) {
        const double acceleration = solver.inputs()(0, k) * config.max_acc;
        require(std::abs(acceleration) <= 10 + 1e-9, "Physical acceleration bound changed");
        require((solver.states().col(k + 1) - physical_a * solver.states().col(k) -
                 Eigen::Vector2d(0, DT * acceleration))
                    .norm() < 1e-11,
                "Returned trajectory must obey physical dynamics");
      }
      const auto once = solver.inputs().eval();
      require(solver.solve(initial, reference).solved && (solver.inputs() - once).norm() < 1e-12,
              "Physical output scaling must not contaminate the next cold solve");
    }
    for (double bad : {0., -1., std::numeric_limits<double>::infinity()}) {
      auto invalid = config;
      invalid.state_scale[0] = bad;
      bool rejected = false;
      try {
        MpcAxis solver(invalid);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      require(rejected, "Invalid state scales must fail before entering the solver");
    }
    {
      std::ofstream out(file, std::ios::app);
      out << "Q_pitch: [1,1]\nR_pitch: [1]\n";
    }
    bool rejected = false;
    try {
      MpcConfig::load(file.string(), "pitch");
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "Ambiguous normalized and legacy costs must not silently override");
    std::filesystem::remove(file);
    std::cout << "PASS physical cost/model/output units, trace compatibility and cold repeats\n";
    return 0;
  } catch (const std::exception & error) {
    std::filesystem::remove(file);
    std::cerr << "FAIL normalized MPC cost: " << error.what() << '\n';
    return 1;
  }
}
