#include <algorithm>
#include <cassert>
#include <cmath>

#include "target_replay.hpp"
#include "tools/math/math_tools.hpp"
#include "tools/runtime/yaml_config.hpp"

using namespace auto_aim;
using namespace std::chrono_literals;

namespace
{
const PlannerTime start{100s};

PlanningState feedback(PlannerTime at, double yaw = 0.0, double pitch = 0.0)
{
  PlanningState state;
  state.timestamp = at;
  state.yaw = yaw;
  state.pitch = pitch;
  state.R_gimbal2world = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()))
                           .toRotationMatrix();
  state.valid = state.rates_measured = true;
  return state;
}

TargetPredictionState target(PlannerTime at, double phase, double speed)
{
  TargetPredictionState t;
  t.timestamp = t.observed_at = at;
  t.generation = 1;
  t.name = ArmorName::three;
  t.armor_type = ArmorType::small;
  t.armor_num = 4;
  t.jumped = true;
  t.x << 0., 0.0, -4., 0., 0.01, 0., -M_PI / 2 + phase, speed, .24, .1, .12;
  return t;
}

void confirmations()
{
  AxisStrategyConfig yaw_config{true, 8.0, 4.0, .02, .03, .16};
  AxisStrategyConfig pitch_config{true, 5.0, 2.0, .04, .03, .16};
  AxisStrategyState yaw, pitch;
  const auto observe = [&](PlannerTime at, double speed, bool yaw_switch, bool pitch_switch) {
    assert(yaw.confirm(yaw_config, speed, at, at, 30ms) == yaw_switch);
    assert(pitch.confirm(pitch_config, speed, at, at, 30ms) == pitch_switch);
    if (yaw_switch) {
      yaw.reduced = !yaw.reduced;
      yaw.condition_since = {};
    }
    if (pitch_switch) {
      pitch.reduced = !pitch.reduced;
      pitch.condition_since = {};
    }
  };
  observe(start, 6, false, false);
  observe(start + 20ms, 6, false, false);
  assert(!pitch.confirm(pitch_config, 6, start + 45ms, start + 20ms, 30ms));
  observe(start + 40ms, 6, false, true);
  observe(start + 60ms, 9, false, false);
  observe(start + 80ms, 9, true, false);
  observe(start + 90ms, 6, false, false);  // Both in hysteresis.
  observe(start + 100ms, 3, false, false);
  assert(!yaw.confirm(yaw_config, 3, start + 140ms, start + 100ms, 30ms));
  observe(start + 145ms, 3, false, false);  // Stale observation broke confirmation.
  observe(start + 170ms, 3, false, false);
  observe(start + 175ms, 3, true, false);
  assert(!yaw.reduced && pitch.reduced);
  observe(start + 190ms, 1, false, false);
  observe(start + 220ms, 1, false, true);
}

PlannerConfig config_for(const char * path)
{
  auto config = PlannerConfig::parse(tools::YamlConfig(path));
  config.axis_strategy_enabled = true;
  config.yaw_strategy = {true, 8., 4., .02, .03, .16};
  config.pitch_strategy = {true, 5., 2., .04, .03, .16};
  config.yaw_offset = config.pitch_offset = 0.0;
  config.yaw.max_acc = 150.0;
  config.pitch.max_acc = 100.0;
  // Isolate the reference tests from the solver's robot-specific convergence tuning.
  config.yaw.max_iterations = config.pitch.max_iterations = 2500;

  auto yaml = YAML::Load(config.replay_yaml);
  yaml["planner_axis_strategy"]["enabled"] = true;
  for (const auto axis : {"yaw", "pitch"})
    yaml["planner_axis_strategy"][axis]["enabled"] = true;
  yaml["planner_axis_strategy"]["yaw"]["enter_speed"] = 8.;
  yaml["planner_axis_strategy"]["yaw"]["exit_speed"] = 4.;
  yaml["planner_axis_strategy"]["yaw"]["enter_hold_ms"] = 20;
  yaml["planner_axis_strategy"]["yaw"]["exit_hold_ms"] = 30;
  yaml["planner_axis_strategy"]["pitch"]["enter_speed"] = 5.;
  yaml["planner_axis_strategy"]["pitch"]["exit_speed"] = 2.;
  yaml["planner_axis_strategy"]["pitch"]["enter_hold_ms"] = 40;
  yaml["planner_axis_strategy"]["pitch"]["exit_hold_ms"] = 30;
  yaml["yaw_offset"] = yaml["pitch_offset"] = 0.0;
  yaml["max_yaw_acc"] = config.yaw.max_acc;
  yaml["max_pitch_acc"] = config.pitch.max_acc;
  yaml["mpc_max_iter"] = 2500;
  config.replay_yaml = YAML::Dump(yaml);
  return config;
}

void references(const PlannerConfig & config)
{
  Planner planner(config);
  auto input = target(start, 0.15, 10.0);
  input.x[1] = 0.3;
  input.x[3] = 0.1;
  input.x[5] = 0.02;
  for (const bool yaw : {false, true}) {
    for (const bool pitch : {false, true}) {
      PlannerHistory history;
      auto & axis = history.axis_strategy;
      axis.has_target = true;
      axis.generation = 1;
      axis.yaw.reduced = yaw;
      axis.pitch.reduced = pitch;
      axis.height_group = pitch ? 1 : -1;
      planner.restore_history(history);
      PlanningTrace trace;
      const auto plan =
        planner.plan_at(Target::from_prediction_state(input), start, 23., feedback(start), &trace);
      assert(trace.reference_available);
      assert(plan.control_reference.yaw_mode == (yaw ? YawMode::center : YawMode::plate));
      assert(plan.control_reference.pitch_mode ==
             (pitch ? PitchMode::height_group : PitchMode::plate));
      for (int k = 0; k < HORIZON; ++k) {
        const auto & tag = trace.reference_tags[k];
        if (pitch && tag.armor_id >= 0)
          assert(tag.armor_id % 2 == 1);
        if (yaw)
          assert(trace.reference(1, k) > 0.04);
      }
      if (pitch) {
        const auto before = trace.reference;
        auto reversed = input;
        reversed.x[6] += 1.1;
        reversed.x[7] = -input.x[7];
        planner.restore_history(history);
        planner.plan_at(Target::from_prediction_state(reversed), start, 23., feedback(start),
                        &trace);
        assert(trace.reference_available);
        assert((trace.reference.bottomRows<2>() - before.bottomRows<2>()).norm() < 1e-10);
        if (yaw)
          assert((trace.reference - before).norm() < 1e-10);
      }
    }
  }

  // Each virtual mode keeps its derivatives across real plate changes.
  ReferenceTags tags{};
  Trajectory ref = Trajectory::Zero();
  for (int k = 0; k < HORIZON; ++k) {
    tags[k].yaw_mode = YawMode::center;
    tags[k].pitch_mode = PitchMode::height_group;
    tags[k].yaw_smooth_valid = tags[k].pitch_smooth_valid = true;
    tags[k].height_group = 1;
    tags[k].armor_id = k % 2 ? 1 : 3;
    tags[k].mode = k % 2 ? AimMode::preaim : AimMode::track;
    ref(0, k) = .5 * k * DT;
    ref(2, k) = .2 * k * DT;
  }
  fill_segment_reference_velocity(ref, tags);
  assert((ref.row(1).array() - .5).abs().maxCoeff() < 1e-12);
  assert((ref.row(3).array() - .2).abs().maxCoeff() < 1e-12);
}

void pitch_window_bridges()
{
  constexpr double yaw_origin = M_PI - .01;
  tinyMatrix yaw = tinyMatrix::Zero(2, HORIZON);
  yaw.row(0).setConstant(.02);
  ReferenceTags tags{};
  Trajectory baseline = Trajectory::Zero();
  const auto set_plate = [&](int k, int id, double error, double pitch) {
    auto & tag = tags[k];
    tag.armor_id = id;
    tag.selection_valid = tag.converged = true;
    tag.plate_angles = PlateAimAngles{tools::limit_rad(yaw_origin + yaw(0, k) + error), pitch};
    baseline(2, k) = pitch;
  };
  for (int k = 1; k <= 6; ++k)
    set_plate(k, 0, k == 1 ? .04 : k == 6 ? -.04 : 0.0, .2);
  for (int k = 14; k <= 20; ++k)
    set_plate(k, 1, k == 14 ? -.04 : k == 20 ? .04 : 0.0, .24);

  // A real angle on an invalid candidate may not create a window.
  set_plate(9, 2, 0.0, .5);
  tags[9].selection_valid = false;
  auto reference = baseline;
  PitchWindowTrace trace;
  const auto bridge =
    follow_yaw_windows(start, yaw_origin, yaw, .02, 50.0, tags, reference, std::nullopt, &trace);
  assert(bridge && trace.windows.size() == 2 && trace.gaps.size() == 1);
  assert(trace.windows[0].enter == start + 15ms && trace.windows[0].exit == start + 55ms);
  assert(trace.windows[1].enter == start + 145ms && trace.windows[1].exit == start + 195ms);
  const auto & gap = trace.gaps.front();
  assert(gap.start == trace.windows[0].exit && gap.end == trace.windows[1].enter);
  assert(gap.from_armor_id == 0 && gap.to_armor_id == 1);
  assert(gap.fallback == PitchWindowFallback::none);
  const double expected_peak = .04 * 10 * std::sqrt(3.0) / (3 * .09 * .09);
  assert(std::abs(gap.peak_acceleration - expected_peak) < 1e-10);
  assert(!bridge->curve.within_limits(50.0, 1e6));
  assert(reference.topRows<2>() == baseline.topRows<2>());
  for (int k = 0; k < HORIZON; ++k) {
    if (k < 6 || k >= 15) {
      assert(reference.col(k) == baseline.col(k));
    } else {
      const auto motion = bridge->curve.at(start + k * 10ms);
      assert(reference(2, k) == motion.angle && reference(3, k) == motion.velocity);
    }
  }

  auto fallback = baseline;
  PitchWindowTrace fallback_trace;
  assert(!follow_yaw_windows(start, yaw_origin, yaw, .02, 5.0, tags, fallback, std::nullopt,
                             &fallback_trace));
  assert(fallback == baseline);
  assert(fallback_trace.gaps.front().fallback == PitchWindowFallback::acceleration_limit);

  // After the old window moves outside the horizon, continue from the preceding curve.
  constexpr int elapsed = 8;
  ReferenceTags shifted{};
  std::copy(tags.begin() + elapsed, tags.end(), shifted.begin());
  auto rolling = Trajectory::Zero().eval();
  PitchWindowTrace rolling_trace;
  const auto resumed = follow_yaw_windows(start + elapsed * 10ms, yaw_origin, yaw, .02, 50.0,
                                          shifted, rolling, bridge, &rolling_trace);
  assert(resumed && rolling_trace.windows.size() == 1 && rolling_trace.gaps.size() == 1);
  assert(resumed->curve.start == start + elapsed * 10ms && resumed->curve.end == gap.end);
  for (int k = 0; k + elapsed < 15; ++k) {
    assert(std::abs(rolling(2, k) - reference(2, k + elapsed)) < 1e-12);
    assert(std::abs(rolling(3, k) - reference(3, k + elapsed)) < 1e-12);
  }
}

void transition_and_replay(const PlannerConfig & config)
{
  Planner planner(config);
  auto input = target(start, 0., 11.);
  // Move the center close to the yaw wrap. The planned angle must take the short path.
  input.x[0] = .02;
  input.x[2] = 4.;
  input.x[6] += M_PI;
  auto measured = feedback(start, -M_PI + .01, -.04);
  const auto run = [&](PlannerTime now, double innovation, double new_speed = 11.) {
    const double elapsed = std::chrono::duration<double>(now - start).count();
    auto current = input;
    current.x[0] += innovation;
    current.x[6] += elapsed * new_speed;
    current.x[7] = new_speed;
    current.timestamp = current.observed_at = now;
    measured.timestamp = now;
    target_replay::Result r;
    r.plan = planner.plan_at(Target::from_prediction_state(current), now, 23., measured, &r.trace);
    assert(r.trace.reference_available);
    return r;
  };
  run(start, 0.);
  run(start + 10ms, 0.);
  auto r = run(start + 20ms, 0.);
  assert(r.plan.control_reference.yaw_mode == YawMode::center);
  assert(r.plan.control_reference.yaw_transitioning && !r.plan.fire);
  assert(std::abs(r.trace.reference(0, 0)) < .15);
  r = run(start + 22ms, .02);
  const auto & prior = r.trace.history.axis_strategy.yaw;
  const auto previous = prior.previous_at(start + 22ms);
  assert(std::abs(r.trace.reference(0, 0) + r.plan.initial_state.yaw - previous.angle) < 1e-12);
  assert(std::abs(r.trace.reference(1, 0) - previous.velocity) < 1e-12);
  target_replay::reproduce(nlohmann::json::parse(target_replay::serialize(r).dump()));
  auto last = r;
  for (int ms = 24; ms <= 1200; ms += 2)
    last = run(start + std::chrono::milliseconds(ms), .02);
  assert(last.plan.control_reference.yaw_mode == YawMode::center);
  assert(last.plan.control_reference.pitch_mode == PitchMode::height_group);
  assert(!last.plan.control_reference.yaw_transitioning &&
         !last.plan.control_reference.pitch_transitioning);
  const int group = last.plan.control_reference.height_group;
  assert(group >= 0);

  for (int ms = 1202; ms <= 1232; ms += 2)
    last = run(start + std::chrono::milliseconds(ms), .02, 1.);
  assert(last.plan.control_reference.yaw_mode == YawMode::plate);
  assert(last.plan.control_reference.pitch_mode == PitchMode::plate);
  assert(last.plan.control_reference.yaw_transitioning &&
         last.plan.control_reference.pitch_transitioning);

  input.generation++;
  r = run(start + 1234ms, .02);
  assert(r.plan.control_reference.yaw_transitioning && !r.plan.fire);
  assert(r.plan.control_reference.height_group == -1);
  assert(r.plan.control_reference.yaw_mode == YawMode::plate);
}

void planner_fire_context(PlannerConfig config)
{
  config.yaw_strategy.enabled = config.pitch_strategy.enabled = false;
  Planner planner(config);
  auto input = target(start, 0., 0.);
  const auto aim =
    solve_ballistic_aim(input.armor_at(0, 0.0), 23., Eigen::Matrix3d::Identity(), 0., 0.);
  assert(aim);
  const auto track = [&](PlannerTime at) {
    return planner.plan_at(Target::from_prediction_state(input), at, 23.,
                           feedback(at, aim->yaw, aim->pitch));
  };
  assert(!track(start).fire);
  const auto plan = track(start + 2ms);
  assert(plan.control && plan.fire && plan.fire_context);
  assert(plan.fire_result.armor_id == 0 && plan.fire_result.error_valid);
  const auto relative = plan.relative_to(aim->yaw + .15, aim->pitch);
  assert(relative.fire && !relative.legacy_fire_aligned());
  assert(FireEvaluator::evaluate(*plan.fire_context,
                                 feedback(start + 3ms, aim->yaw + .15, aim->pitch), 23.,
                                 start + 3ms, 2ms)
           .block == FireBlock::angle_outside);
}

void fire()
{
  FireContext context;
  context.target = target(start, 0., 0.);
  context.armor_id = 0;
  context.height_group = 0;
  context.valid_from = start;
  context.valid_until = start + 10ms;
  context.enabled = true;
  context.coming_angle = .7;
  context.leaving_angle = .5;
  context.tolerance = .005;
  const auto aim =
    solve_ballistic_aim(context.target.armor_at(0, 0.0), 23., Eigen::Matrix3d::Identity(), 0., 0.);
  assert(aim);
  auto measured = feedback(start, aim->yaw, aim->pitch);
  const auto check = [&](PlannerTime now = start) {
    return FireEvaluator::evaluate(context, measured, 23., now, 2ms).block;
  };
  assert(check() == FireBlock::none);
  assert(check(start + 9ms) == FireBlock::expired);
  context.transitioning = true;
  assert(check() == FireBlock::transitioning);
  context.transitioning = false;
  context.armor_id = -1;
  assert(check() == FireBlock::no_candidate);
  context.armor_id = 1;
  assert(check() == FireBlock::no_candidate);
  context.armor_id = 2;
  assert(check() == FireBlock::outside_window);
  context.armor_id = 0;
  measured = feedback(start, aim->yaw, aim->pitch + .01);
  assert(check() == FireBlock::angle_outside);
  measured = feedback(start, aim->yaw, aim->pitch);
  measured.pitch_vel = 3.0;
  assert(check() == FireBlock::angle_outside);  // Aligned now, leaves before the next send.

  // A near-side plate at the leading edge expires within the requested hold.
  context.target.x[7] = 12.;
  context.target.x[6] = -M_PI / 2 + .49;
  const auto edge =
    solve_ballistic_aim(context.target.armor_at(0, 0.0), 23., Eigen::Matrix3d::Identity(), 0., 0.);
  assert(edge);
  context.target.x[6] -= edge->fly_time * context.target.x[7];
  context.tolerance = .4;
  measured = feedback(start, aim->yaw, aim->pitch);
  assert(FireEvaluator::evaluate(context, measured, 23., start).allowed());
  assert(check() == FireBlock::outside_window);
}
}  // namespace

int main(int argc, char ** argv)
{
  assert(argc == 2);
  confirmations();
  const auto config = config_for(argv[1]);
  references(config);
  pitch_window_bridges();
  transition_and_replay(config);
  planner_fire_context(config);
  fire();
}
