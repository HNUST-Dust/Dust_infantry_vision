#pragma once

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "src/planner_diagnostics.hpp"

namespace target_replay
{
using Json = nlohmann::json;
using namespace auto_aim;

struct Result
{
  Plan plan;
  PlanningTrace trace;
  double planning_ms = 0.0;
};

// Every record supplies its own pre-plan history. This also supports discontinuous
// captures; A/B compares identical inputs, not a simulated closed-loop vehicle.
inline Result run(const Json & record, std::optional<bool> preaim = std::nullopt,
                  std::optional<bool> pitch_follow_yaw_window = std::nullopt)
{
  if (!record.contains("target_replay") || record.at("target_replay").is_null())
    throw std::invalid_argument("Trace lacks target replay state; recapture, do not infer it");
  const auto & r = record.at("target_replay");
  const int version = r.at("version").get<int>();
  if ((version != 1 && version != 2) || r.at("dt") != DT || r.at("horizon") != HORIZON ||
      r.at("command_step") != COMMAND_STEP)
    throw std::invalid_argument("Target replay model mismatch");
  const auto now = std::chrono::steady_clock::time_point(std::chrono::seconds(100));
  const auto timestamp = [&](const Json & ns) {
    return now + std::chrono::nanoseconds(ns.get<int64_t>());
  };
  const auto optional_time = [&](const Json & ns) {
    return ns.is_null() ? PlannerTime{} : timestamp(ns);
  };
  std::optional<Target> target;
  if (!r.at("target").is_null()) {
    const auto & t = r.at("target");
    TargetPredictionState s;
    if (t.at("state").size() != 11)
      throw std::invalid_argument("Target state dimension");
    for (int i = 0; i < 11; ++i)
      s.x[i] = t.at("state").at(i).at(0).get<double>();
    s.timestamp =
      version >= 2 ? optional_time(t.at("timestamp_ns")) : timestamp(t.at("timestamp_ns"));
    s.name = static_cast<ArmorName>(t.at("name").get<int>());
    s.armor_type = static_cast<ArmorType>(t.at("armor_type").get<int>());
    s.armor_num = t.at("armor_num").get<int>();
    s.jumped = t.at("jumped").get<bool>();
    if (version >= 2) {
      s.generation = t.at("generation").get<uint64_t>();
      s.observed_at = optional_time(t.at("observed_at"));
    }
    target = Target::from_prediction_state(s);
  }
  const auto & received = record.at("received_state");
  PlanningState state;
  state.yaw = received.at("yaw").get<double>();
  state.yaw_vel = received.at("yaw_vel").get<double>();
  state.pitch = received.at("pitch").get<double>();
  state.pitch_vel = received.at("pitch_vel").get<double>();
  state.valid = received.at("valid").get<bool>();
  state.rates_measured = received.at("rates_measured").get<bool>();
  state.timestamp = r.at("feedback_timestamp_missing").get<bool>()
                      ? std::chrono::steady_clock::time_point{}
                      : timestamp(r.at("feedback_timestamp_ns"));
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      state.R_gimbal2world(i, j) = received.at("rotation").at(i).at(j).get<double>();
  auto yaml = YAML::Load(r.at("config_yaml").get<std::string>());
  // Version 1 did not record strategy observations or state.
  if (version == 1)
    yaml.remove("planner_axis_strategy");
  if (preaim)
    yaml["gap_preaim_enabled"] = *preaim;
  if (pitch_follow_yaw_window)
    yaml["planner_axis_strategy"]["pitch_follow_yaw_window"] = *pitch_follow_yaw_window;
  struct Temporary
  {
    std::filesystem::path directory;
    Temporary()
    {
      char pattern[] = "/tmp/dust-target-replay-XXXXXX";
      const auto created = mkdtemp(pattern);
      if (!created)
        throw std::runtime_error("mkdtemp failed");
      directory = created;
    }
    ~Temporary()
    {
      std::filesystem::remove_all(directory);
    }
  } temporary;
  const auto path = temporary.directory / "planner.yaml";
  {
    std::ofstream out(path);
    out << yaml;
    if (!out)
      throw std::runtime_error("Config write failed");
  }
  Planner planner(path.string());
  const auto & h = r.at("history");
  PlannerHistory history{h.at("lock_id").get<int>(), h.at("have_last_command").get<bool>(),
                         h.at("last_target_yaw").get<double>(), timestamp(h.at("last_plan_ns"))};
  if (version >= 2) {
    const auto & axes = h.at("axis_strategy");
    const auto read_axis = [&](const Json & json) -> AxisStrategyState {
      AxisStrategyState state;
      state.reduced = json.at("reduced").get<bool>();
      state.condition_since = optional_time(json.at("condition_since"));
      state.last_observation = optional_time(json.at("last_observation"));
      state.needs_bridge = json.at("needs_bridge").get<bool>();
      state.bridge.start = optional_time(json.at("bridge_start"));
      state.bridge.end = optional_time(json.at("bridge_end"));
      state.bridge.coefficients = json.at("bridge_coefficients").get<std::array<double, 6>>();
      state.reference_at = optional_time(json.at("reference_at"));
      const auto motion = json.at("reference").get<std::array<double, 3>>();
      state.reference = {motion[0], motion[1], motion[2]};
      return state;
    };
    history.axis_strategy = {
      axes.at("has_target").get<bool>(), axes.at("generation").get<uint64_t>(),
      axes.at("height_group").get<int>(), read_axis(axes.at("yaw")), read_axis(axes.at("pitch"))};
  }
  if (h.contains("pitch_window_bridge")) {
    const auto & bridge = h.at("pitch_window_bridge");
    history.pitch_window_bridge =
      PitchWindowBridge{bridge.at("from_armor_id").get<int>(),
                        bridge.at("to_armor_id").get<int>(),
                        {timestamp(bridge.at("start_ns")), timestamp(bridge.at("end_ns")),
                         bridge.at("coefficients").get<std::array<double, 6>>()}};
  }
  planner.restore_history(history);
  Result result;
  const auto begin = std::chrono::steady_clock::now();
  result.plan =
    planner.plan_at(target, now, record.at("bullet_speed").get<double>(), state, &result.trace);
  result.planning_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  if (runtime::mpc_config_json(result.trace.yaw_config) != record.at("yaw_config") ||
      runtime::mpc_config_json(result.trace.pitch_config) != record.at("pitch_config"))
    throw std::invalid_argument("Recorded planner YAML and effective MPC config differ");
  return result;
}

inline Json serialize(const Result & result)
{
  runtime::PlannerDiagnosticEvent event;
  event.plan = result.plan;
  event.trace = std::make_shared<PlanningTrace>(result.trace);
  event.planning_ms = result.planning_ms;
  return runtime::planner_trace_json(event, result.plan.planned_at, "target-replay");
}

inline void reproduce(const Json & record)
{
  const auto regenerated = serialize(run(record));
  if (record.at("build_id") != regenerated.at("build_id") ||
      record.at("target_replay").at("planner_build_id") !=
        regenerated.at("target_replay").at("planner_build_id"))
    throw std::invalid_argument("Target replay build mismatch");
  for (const auto * key :
       {"plan_status", "control", "fire", "reference_available", "reference", "reference_armor_ids",
        "reference_pitch_rate_valid", "reference_modes", "reference_xyza", "reference_fly_time",
        "reference_selection_valid", "reference_converged", "aim", "output",
        "reference_axis_segments", "control_reference", "fire_context"})
    if (record.at(key) != regenerated.at(key))
      throw std::runtime_error(std::string("Target replay differs: ") + key);
  for (const auto * key : {"reference_plate_angles", "pitch_yaw_window"})
    if (record.contains(key) != regenerated.contains(key) ||
        (record.contains(key) && record.at(key) != regenerated.at(key)))
      throw std::runtime_error(std::string("Target replay differs: ") + key);
}
}  // namespace target_replay
