#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

#include "tools/runtime/yaml_config.hpp"
#include "target_replay.hpp"

using namespace auto_aim;
using namespace std::chrono_literals;

namespace
{
void require(bool value, const char * message)
{
  if (!value)
    throw std::runtime_error(message);
}
struct Config
{
  std::filesystem::path directory;
  std::string path;
  YAML::Node yaml;
  Config()
  {
    char pattern[] = "/tmp/dust-planner-smoothing-XXXXXX";
    const auto created = mkdtemp(pattern);
    if (!created)
      throw std::runtime_error("mkdtemp failed");
    directory = created;
    path = (directory / "planner.yaml").string();
    yaml = YAML::Load(R"(
mpc_mode: reference_smoothing
yaw_offset: 0
pitch_offset: 0
fire_thresh: 0.02
fire_thresh_high_speed: 0.04
decision_speed: 8
launch_delay_time: 0.01
comming_angle: 55
leaving_angle: 20
first_tolerance: 1
second_tolerance: 0.5
judge_distance: 5
auto_fire: true
gap_preaim_enabled: false
max_yaw_acc: 55
max_pitch_acc: 50
mpc_cost_yaw:
  scales: [0.1, 1, 10]
  weights: [15, 0, 0.175]
mpc_cost_pitch:
  scales: [0.1, 1, 10]
  weights: [15, 0, 0.2]
)");
    save();
  }
  void save() const
  {
    std::ofstream out(path);
    out << yaml;
    require(bool(out), "Write test config");
  }
  ~Config()
  {
    std::filesystem::remove_all(directory);
  }
};
PlanningState state_at(std::chrono::steady_clock::time_point t, double yaw = 0, double pitch = 0)
{
  PlanningState state;
  state.valid = state.rates_measured = true;
  state.timestamp = t;
  state.yaw = yaw;
  state.pitch = pitch;
  state.R_gimbal2world = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()))
                           .toRotationMatrix();
  return state;
}
Target target_at(std::chrono::steady_clock::time_point t, double yaw)
{
  Armor armor(0, 1.0F, cv::Rect(0, 0, 20, 10), {{0, 0}, {20, 0}, {20, 10}, {0, 10}});
  armor.xyz_in_world = {3 * std::sin(yaw), -3 * std::cos(yaw), 0};
  armor.ypr_in_world = {yaw - M_PI / 2, 0, 0};
  return Target(armor, t, 0.25, 4, Eigen::VectorXd::Ones(11));
}
void verify(const Plan & plan, const PlanningTrace & trace)
{
  require(plan.control && plan.status == PlanStatus::ok, "Smoothing solve failed");
  require(plan.mpc_mode == MpcMode::reference_smoothing && trace.state_index == SMOOTHING_CENTER &&
            trace.input_index == SMOOTHING_CENTER,
          "Smoothing mode and sample indices");
  require((trace.yaw_x0 - trace.reference.block<2, 1>(0, 0)).norm() == 0 &&
            (trace.pitch_x0 - trace.reference.block<2, 1>(2, 0)).norm() == 0,
          "Solver must start from reference, not measured state");
  require(plan.yaw_vel == 0 && plan.yaw_acc == 0 && plan.pitch_vel == 0 && plan.pitch_acc == 0,
          "Position-only command must carry zero feedforward");
  require(plan.command_at == plan.planned_at && plan.valid_at == plan.planned_at + 10ms,
          "Center command time and sender expiry are distinct");
  for (int axis = 0; axis < 2; ++axis) {
    MpcAxis solver(axis == 0 ? trace.yaw_config : trace.pitch_config);
    const AxisReference reference = axis == 0 ? AxisReference(trace.reference.topRows<2>())
                                              : AxisReference(trace.reference.bottomRows<2>());
    require(solver.solve(axis == 0 ? trace.yaw_x0 : trace.pitch_x0, reference).solved,
            "Independent smoothing solve");
    const double absolute =
      solver.states()(0, SMOOTHING_CENTER) + (axis == 0 ? plan.initial_state.yaw : 0);
    require(std::abs(absolute - (axis == 0 ? plan.absolute_yaw : plan.absolute_pitch)) < 1e-10,
            "Center position must match independent rollout");
    require(std::abs(solver.states()(1, SMOOTHING_CENTER) - plan.mpc_feedforward[2 * axis]) <
                1e-10 &&
              std::abs(solver.inputs()(0, SMOOTHING_CENTER) * solver.config().max_acc -
                       plan.mpc_feedforward[2 * axis + 1]) < 1e-10,
            "Raw optimized feedforward must remain observable");
  }
}

void directional_tracking()
{
  Config config;
  config.yaml["first_tolerance"] = config.yaml["second_tolerance"] = 3.0;
  config.yaml["fire_thresh"] = config.yaml["fire_thresh_high_speed"] = 10.0;
  config.yaml["fire_lag_tolerance_ratio"] = 0.5;
  config.yaml["fire_direction_min_yaw_rate"] = 0.05;
  const auto t = std::chrono::steady_clock::time_point(100s);
  for (const auto * mode : {"measured_state", "reference_smoothing"}) {
    for (double sign : {-1.0, 1.0}) {
      config.yaml["mpc_mode"] = mode;
      config.yaml["fire_directional_tolerance_enabled"] = false;
      config.save();
      auto prediction = target_at(t, 0.12).prediction_state();
      prediction.x[1] = sign * 0.6;
      prediction.x[3] = 0;
      prediction.x[7] = 0;  // Translation has direction even without target spin.
      const auto input = Target::from_prediction_state(prediction);
      Planner old(config.path);
      const auto seed = old.plan_at(input, t, 23, state_at(t));
      require(seed.control, "Directional integration fixture must solve");
      config.yaml["fire_directional_tolerance_enabled"] = true;
      config.save();
      for (double side : {-1.0, 1.0}) {
        const auto feedback = state_at(t, seed.absolute_target_yaw + sign * side * 0.75 * 3 / 57.3,
                                       seed.absolute_target_pitch);
        Planner directional(config.path);
        old.restore_history({-1, true, seed.absolute_target_yaw, t - 2ms});
        directional.restore_history({-1, true, seed.absolute_target_yaw, t - 2ms});
        PlanningTrace old_trace, trace;
        const auto before = old.plan_at(input, t, 23, feedback, &old_trace);
        const auto after = directional.plan_at(input, t, 23, feedback, &trace);
        require(before.control && after.control && before.fire,
                "Original gate must allow both sides in the comparison fixture");
        require(after.settled_window.motion.state == FireDirectionState::moving &&
                  after.fire_window.motion.state == FireDirectionState::moving &&
                  after.settled_window.motion.direction == sign &&
                  after.fire_window.motion.direction == sign,
                "Both sample indices must use actual reference motion, not spin or wire velocity");
        require(after.fire == (side > 0) && after.settled_check.passed == (side > 0),
                "Actual lag rejects while ahead feedback retains permission");
        const bool smoothing = std::string(mode) == "reference_smoothing";
        require(after.settled_reference_index == (smoothing ? SMOOTHING_CENTER : 0) &&
                  after.fire_reference_index == (smoothing ? SMOOTHING_CENTER : COMMAND_STEP),
                "Both MPC modes retain their own settled and command reference indices");
        require(trace.reference == old_trace.reference &&
                  after.absolute_yaw == before.absolute_yaw &&
                  after.absolute_pitch == before.absolute_pitch &&
                  after.mpc_feedforward == before.mpc_feedforward,
                "Directional gating must not change prediction or optimized commands");
        if (smoothing)
          require(after.yaw_vel == 0 && after.pitch_vel == 0,
                  "Motion direction exists with zero wire feedforward");
        target_replay::reproduce(target_replay::serialize({after, trace}));
      }
    }
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    directional_tracking();
    Config config;
    const auto t = std::chrono::steady_clock::time_point(100s);
    const auto target = target_at(t, 0.12);
    auto state = state_at(t);
    Planner planner(config.path);
    PlanningTrace trace;
    const auto plan = planner.plan_at(target, t, 23, state, &trace);
    verify(plan, trace);
    require(plan.yaw > 0.08 && !plan.fire,
            "Static off-axis target needs a position step, not fire");
    planner.restore_history({-1, true, plan.absolute_target_yaw, t - 2ms});
    const auto off_axis = planner.plan_at(target, t, 23, state);
    require(!off_axis.fire, "Smooth reference is not proof of actual alignment");
    const auto sent = plan.relative_to(0.04, -0.01);
    require(std::abs(sent.yaw + 0.04 - plan.absolute_yaw) < 1e-7 &&
              std::abs(sent.pitch - 0.01 - plan.absolute_pitch) < 1e-7,
            "Repeated sends must retain a fixed absolute position");
    state = state_at(t, plan.absolute_target_yaw, plan.absolute_target_pitch);
    planner.restore_history({-1, true, plan.absolute_target_yaw, t - 2ms});
    const auto aligned = planner.plan_at(target, t, 23, state);
    require(aligned.control && aligned.fire, "Actual alignment can pass current fire gates");
    state.valid = false;
    require(!planner.plan_at(target, t, 23, state).control,
            "Invalid feedback still rejects smoothing");
    state = state_at(t);
    state.timestamp = t - 21ms;
    require(!planner.plan_at(target, t, 23, state).control,
            "Stale feedback still rejects smoothing");
    require(!planner.plan_at(std::nullopt, t, 23, state_at(t)).control,
            "No target rejects smoothing");
    config.yaml["auto_fire"] = false;
    config.save();
    Planner no_fire(config.path);
    no_fire.restore_history({-1, true, plan.absolute_target_yaw, t - 2ms});
    require(
      !no_fire
         .plan_at(target, t, 23, state_at(t, plan.absolute_target_yaw, plan.absolute_target_pitch))
         .fire,
      "auto_fire false remains effective");
    config.yaml["auto_fire"] = true;
    config.save();

    const auto record = target_replay::serialize({plan, trace});
    target_replay::reproduce(nlohmann::json::parse(record.dump()));
    if (argc > 1) {
      std::ofstream out(argv[1]);
      out << record.dump() << '\n';
      require(bool(out), "Write smoothing replay fixture");
    }
    config.yaml["fire_directional_tolerance_enabled"] = true;
    config.save();
    for (double omega : {-10.0, -4.0, 4.0, 10.0}) {
      auto moving = target_at(t, 0.12).prediction_state();
      moving.x[1] = 0.35;
      moving.x[3] = 0.1;
      moving.x[7] = omega;
      moving.jumped = true;
      const auto input = Target::from_prediction_state(moving);
      Planner moving_planner(config.path);
      PlanningTrace moving_trace;
      const auto p = moving_planner.plan_at(input, t, 23, state_at(t), &moving_trace);
      verify(p, moving_trace);
      auto measured_config = PlannerConfig::parse(tools::YamlConfig(config.path));
      measured_config.mpc_mode = MpcMode::measured_state;
      Planner measured(measured_config);
      measured.plan_at(input, t, 23, state_at(t));
      PlanningTrace measured_next, smoothing_next;
      measured.plan_at(input, t, 23, state_at(t), &measured_next);
      moving_planner.plan_at(input, t, 23, state_at(t), &smoothing_next);
      require(measured_next.history.lock_id == smoothing_next.history.lock_id,
              "Past/future selection must not overwrite the center decision lock");
      auto center = input;
      center.predict(moving_trace.launch_delay + moving_trace.center_fly_time);
      const auto & tag = moving_trace.reference_tags[SMOOTHING_CENTER];
      require((tag.reference_xyza - center.armor_xyza_list().at(tag.armor_id)).norm() < 1e-9,
              "Center must be current hit decision, not an extra 300ms ahead");
      for (int k = 1; k < HORIZON; ++k)
        require(std::abs(moving_trace.reference(0, k) - moving_trace.reference(0, k - 1)) < M_PI,
                "Yaw unwrap must not introduce full turns");
      target_replay::reproduce(target_replay::serialize({p, moving_trace}));
    }
    // standard3 enables gap preaim. Even perfectly aligned feedback may not turn
    // an unselected boundary reference into fire permission.
    config.yaml["gap_preaim_enabled"] = true;
    config.save();
    int preaim_cases = 0;
    for (int phase = 0; phase < 24; ++phase) {
      auto rotating = target_at(t, 0.12).prediction_state();
      rotating.jumped = true;
      rotating.x[7] = 10.0;
      rotating.x[6] += phase * M_PI / 12;
      const auto input = Target::from_prediction_state(rotating);
      Planner preaim(config.path);
      PlanningTrace preaim_trace;
      const auto first = preaim.plan_at(input, t, 23, state_at(t), &preaim_trace);
      if (!first.control || first.aim_debug.mode != AimMode::preaim)
        continue;
      ++preaim_cases;
      verify(first, preaim_trace);
      preaim.restore_history({-1, true, first.absolute_target_yaw, t - 2ms});
      auto feedback = state_at(t, first.absolute_target_yaw, first.absolute_target_pitch);
      const auto aligned_preaim = preaim.plan_at(input, t, 23, feedback);
      require(aligned_preaim.control && !aligned_preaim.fire,
              "Aligned preaim must not fire without a selected armor");
      target_replay::reproduce(target_replay::serialize({first, preaim_trace}));
    }
    require(preaim_cases > 0, "Exercise the configured gap-preaim path in smoothing mode");
    config.yaml["gap_preaim_enabled"] = false;
    config.save();
    // Current feedback yaw and target yaw can straddle +/-pi without a full-turn command.
    Planner wrapped(config.path);
    PlanningTrace wrapped_trace;
    const auto wrapped_plan =
      wrapped.plan_at(target_at(t, M_PI - 0.02), t, 23, state_at(t, -M_PI + 0.02), &wrapped_trace);
    verify(wrapped_plan, wrapped_trace);
    require(std::abs(wrapped_plan.yaw + 0.04) < 0.01, "Relative position must wrap across pi");

    config.yaml.remove("mpc_mode");
    config.save();
    Planner default_planner(config.path);
    PlanningTrace default_trace;
    const auto default_plan = default_planner.plan_at(target, t, 23, state_at(t), &default_trace);
    config.yaml["mpc_mode"] = "measured_state";
    config.save();
    Planner explicit_planner(config.path);
    PlanningTrace explicit_trace;
    const auto explicit_plan =
      explicit_planner.plan_at(target, t, 23, state_at(t), &explicit_trace);
    require(default_plan.control && explicit_plan.control && default_plan.absolute_yaw == 0 &&
              default_plan.absolute_yaw == explicit_plan.absolute_yaw &&
              default_plan.mpc_feedforward == explicit_plan.mpc_feedforward &&
              default_trace.reference == explicit_trace.reference,
            "Missing mode must preserve the measured-state path");
    config.yaml["mpc_mode"] = "typo";
    config.save();
    bool rejected = false;
    try {
      Planner invalid(config.path);
    } catch (const std::exception &) {
      rejected = true;
    }
    require(rejected, "Invalid mode must fail at configuration load");
    std::cout << "PASS reference smoothing, center timing, position-only send, fire and replay\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL smoothing: " << error.what() << '\n';
    return 1;
  }
}
