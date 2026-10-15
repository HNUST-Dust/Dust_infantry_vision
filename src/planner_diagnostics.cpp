#include "planner_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

#ifndef DUST_MPC_BUILD_ID
#define DUST_MPC_BUILD_ID "unknown"
#endif

namespace auto_aim::runtime
{
namespace
{
using Json = nlohmann::json;
Json number(double value) { return std::isfinite(value) ? Json(value) : Json(nullptr); }
template <typename Derived>
Json matrix(const Eigen::MatrixBase<Derived> &value)
{
  Json rows = Json::array();
  for (int r = 0; r < value.rows(); ++r) {
    Json row = Json::array();
    for (int c = 0; c < value.cols(); ++c) row.push_back(number(value(r, c)));
    rows.push_back(std::move(row));
  }
  return rows;
}
Json state_json(const PlanningState &state, std::chrono::steady_clock::time_point origin)
{
  return {{"yaw", number(state.yaw)}, {"yaw_vel", number(state.yaw_vel)},
          {"pitch", number(state.pitch)}, {"pitch_vel", number(state.pitch_vel)},
          {"rotation", matrix(state.R_gimbal2world)}, {"valid", state.valid},
          {"rates_measured", state.rates_measured},
          {"timestamp_s", state.timestamp == std::chrono::steady_clock::time_point{} ?
              Json(nullptr) : number(std::chrono::duration<double>(state.timestamp - origin).count())}};
}
} // namespace

PlannerDiagnosticsConfig PlannerDiagnosticsConfig::load(const std::string &path)
{
  PlannerDiagnosticsConfig config;
  const auto yaml = tools::load(path);
  const auto runtime = yaml["runtime"];
  if (!runtime) return config;
  const auto node = runtime["planner_diagnostics"];
  if (!node) return config;
  if (node["trace_enabled"]) config.trace_enabled = node["trace_enabled"].as<bool>();
  const auto mode = node["trace_mode"] ? node["trace_mode"].as<std::string>() : "failures";
  if (mode != "all" && mode != "failures")
    throw std::invalid_argument("planner_diagnostics.trace_mode must be all or failures");
  config.trace_all = mode == "all";
  if (node["trace_directory"]) config.directory = node["trace_directory"].as<std::string>();
  const int limit = node["trace_max_mb"] ? node["trace_max_mb"].as<int>() : 32;
  if (limit <= 0 || limit > 1024 || config.directory.empty())
    throw std::invalid_argument("Invalid planner trace directory or size limit");
  config.max_bytes = uint64_t(limit) * 1024 * 1024;
  return config;
}

Json mpc_diagnostics_json(const MpcDiagnostics &d)
{
  const auto metric = [&](double v) { return d.attempted ? number(v) : Json(nullptr); };
  return {{"attempted", d.attempted}, {"solved", d.solved}, {"status", to_string(d.status)},
          {"status_code", static_cast<int>(d.status)}, {"iterations", d.iterations},
          {"max_iterations", d.max_iterations}, {"library_return_code", d.library_return_code},
          {"library_solved", d.library_solved}, {"primal_residual", metric(d.primal_residual)},
          {"dual_residual", metric(d.dual_residual)},
          {"state_primal_residual", metric(d.state_primal_residual)},
          {"input_primal_residual", metric(d.input_primal_residual)},
          {"state_dual_residual", metric(d.state_dual_residual)},
          {"input_dual_residual", metric(d.input_dual_residual)},
          {"input_violation", metric(d.input_violation)},
          {"initial_state_error", metric(d.initial_state_error)},
          {"max_dynamics_error", metric(d.max_dynamics_error)},
          {"solve_time_ms", metric(d.solve_time_ms)}, {"rho", metric(d.rho)},
          {"input_limit", metric(d.input_limit)},
          {"primal_tolerance", metric(d.primal_tolerance)},
          {"dual_tolerance", metric(d.dual_tolerance)},
          {"nonfinite_result", d.attempted && d.status == MpcStatus::nonfinite}};
}

Json mpc_config_json(const MpcConfig &c)
{
  const double scale = std::max({c.q[0], c.q[1], c.r * c.max_acc * c.max_acc});
  return {{"max_acc", c.max_acc}, {"q", {c.q[0], c.q[1]}}, {"r", c.r},
          {"rho", c.rho}, {"max_iterations", c.max_iterations},
          {"primal_tolerance", c.primal_tolerance}, {"dual_tolerance", c.dual_tolerance},
          {"normalized_q", {c.q[0] / scale, c.q[1] / scale}},
          {"normalized_r", c.r * c.max_acc * c.max_acc / scale},
          {"a", {{1.0, DT}, {0.0, 1.0}}}, {"b", {0.0, c.max_acc * DT}}};
}

MpcConfig mpc_config_from_json(const Json &j)
{
  MpcConfig c;
  c.max_acc = j.at("max_acc").get<double>();
  if (j.at("q").size() != 2) throw std::invalid_argument("MPC q dimension");
  c.q = {j.at("q").at(0).get<double>(), j.at("q").at(1).get<double>()};
  c.r = j.at("r").get<double>();
  c.rho = j.at("rho").get<double>();
  c.max_iterations = j.at("max_iterations").get<int>();
  c.primal_tolerance = j.at("primal_tolerance").get<double>();
  c.dual_tolerance = j.at("dual_tolerance").get<double>();
  MpcAxis validate(c);
  const auto expected = mpc_config_json(c);
  for (const auto *key : {"a", "b", "normalized_q", "normalized_r"})
    if (j.at(key) != expected.at(key)) throw std::invalid_argument("MPC model/scaling mismatch");
  return c;
}

Json planner_trace_json(const PlannerDiagnosticEvent &event,
                        std::chrono::steady_clock::time_point origin, const std::string &session)
{
  if (!event.trace) throw std::invalid_argument("Missing planning trace");
  const auto &t = *event.trace;
  const auto &p = event.plan;
  const auto &a = p.aim_debug;
  Json j = {{"schema_version", 1}, {"units", "rad,rad/s,rad/s2,s"},
            {"solver_mode", "cold"}, {"build_id", DUST_MPC_BUILD_ID}, {"session", session},
            {"plan_id", event.plan_id}, {"frame_sequence", event.source.frame_sequence},
            {"feedback_sequence", event.source.feedback_sequence},
            {"planned_s", number(std::chrono::duration<double>(p.planned_at - origin).count())},
            {"planning_ms", number(event.planning_ms)}, {"feedback_age_ms", number(p.feedback_age_ms)},
            {"plan_status", to_string(p.status)}, {"error_message", p.error_message},
            {"control", p.control}, {"fire", p.fire},
            {"received_state", state_json(t.received_state, origin)},
            {"initial_state", state_json(p.initial_state, origin)},
            {"bullet_speed", number(t.bullet_speed)},
            {"yaw_offset", number(t.yaw_offset)}, {"pitch_offset", number(t.pitch_offset)},
            {"target_available", t.target_available}, {"target_state", t.target_state},
            {"reference_available", t.reference_available},
            {"reference", t.reference_available ? matrix(t.reference) : Json(nullptr)},
            {"yaw_origin", number(p.initial_state.yaw)},
            {"yaw_config", mpc_config_json(t.yaw_config)},
            {"pitch_config", mpc_config_json(t.pitch_config)},
            {"yaw_mpc", mpc_diagnostics_json(p.yaw_mpc)},
            {"pitch_mpc", mpc_diagnostics_json(p.pitch_mpc)},
            {"aim", {{"xyza", matrix(a.xyza)}, {"armor_id", a.armor_id},
                      {"candidate_available", a.candidate_available},
                      {"selection_valid", a.selection_valid},
                      {"ballistic_converged", a.ballistic_converged},
                      {"display_valid", a.display_valid}}},
            {"output", {{"absolute_yaw", number(p.absolute_yaw)},
                        {"absolute_pitch", number(p.absolute_pitch)},
                        {"yaw_vel", number(p.yaw_vel)}, {"pitch_vel", number(p.pitch_vel)},
                        {"yaw_acc", number(p.yaw_acc)}, {"pitch_acc", number(p.pitch_acc)}}}};
  j["frame_s"] = event.source.frame_timestamp == std::chrono::steady_clock::time_point{} ?
      Json(nullptr) : number(std::chrono::duration<double>(event.source.frame_timestamp - origin).count());
  return j;
}

PlannerDiagnostics::PlannerDiagnostics(const PlannerDiagnosticsConfig &config)
  : config_(config), origin_(std::chrono::steady_clock::now()),
    last_summary_(origin_), rate_window_(origin_), trace_enabled_(config.trace_enabled),
    writer_(&PlannerDiagnostics::write_loop, this)
{}
PlannerDiagnostics::~PlannerDiagnostics() { close(); }
void PlannerDiagnostics::close()
{
  queue_.close();
  if (writer_.joinable()) writer_.join();
}

void PlannerDiagnostics::observe(const Plan &plan, bool has_target, double planning_ms,
                                std::chrono::steady_clock::time_point completed,
                                const PlanSource &source, const PlanningTrace *trace)
{
  ++counters_.plans;
  ++counters_.reasons.at(static_cast<size_t>(plan.status));
  counters_.with_target += has_target;
  counters_.attempted += plan.yaw_mpc.attempted || plan.pitch_mpc.attempted;
  counters_.solved += plan.control;
  counters_.mpc_failed += plan.status == PlanStatus::mpc_failed;
  counters_.display_invalid += has_target && !plan.aim_debug.display_valid;
  counters_.over_2ms += has_target && planning_ms > 2.0;
  counters_.expired_at_completion += plan.control && completed > plan.valid_at;
  const MpcDiagnostics *axes[] = {&plan.yaw_mpc, &plan.pitch_mpc};
  for (size_t i = 0; i < 2; ++i) {
    if (axes[i]->attempted) {
      axis_stats_[2 * i].add(axes[i]->iterations);
      axis_stats_[2 * i + 1].add(axes[i]->solve_time_ms);
    }
  }
  const bool failed = has_target && plan.status != PlanStatus::ok;
  counters_.recovered += previous_failed_ && plan.control;
  previous_failed_ = failed;
  if (completed - rate_window_ >= std::chrono::seconds(1)) {
    rate_window_ = completed;
    detail_count_ = trace_count_ = 0;
  }
  const bool changed = plan.status != last_status_ || plan.yaw_mpc.status != last_yaw_ ||
                       plan.pitch_mpc.status != last_pitch_;
  const bool detail = has_target && changed && detail_count_ < 5;
  const bool summary = completed - last_summary_ >= std::chrono::seconds(1);
  const bool capture = trace_enabled() && trace &&
      (config_.trace_all || (failed && trace_count_ < 5));
  last_status_ = plan.status;
  last_yaw_ = plan.yaw_mpc.status;
  last_pitch_ = plan.pitch_mpc.status;
  if (!detail && !summary && !capture) return;
  PlannerDiagnosticEvent event;
  event.plan_id = counters_.plans;
  event.plan = plan;
  event.source = source;
  event.planning_ms = planning_ms;
  event.detail = detail;
  event.summary = summary;
  event.counters = counters_;
  if (capture) { event.trace = std::make_shared<PlanningTrace>(*trace); ++trace_count_; }
  if (summary) {
    for (size_t i = 0; i < event.send_counts.size(); ++i)
      event.send_counts[i] = send_counts_[i].load(std::memory_order_relaxed);
    for (size_t i = 0; i < 2; ++i) {
      event.axis_samples[i] = !axis_stats_[2 * i].empty();
      if (event.axis_samples[i]) {
        event.axis_summaries[2 * i] = axis_stats_[2 * i].summary();
        event.axis_summaries[2 * i + 1] = axis_stats_[2 * i + 1].summary();
      }
    }
  }
  if (detail) ++detail_count_;
  if (summary) last_summary_ = completed;
  if (!queue_.push(event)) ++dropped_;
}

void PlannerDiagnostics::write_loop()
{
  std::ofstream out;
  uint64_t bytes = 0, previous_trace_id = 0;
  const std::string session = std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
                              "-" + std::to_string(getpid());
  if (trace_enabled()) {
    try {
      std::filesystem::create_directories(config_.directory);
      const auto path = std::filesystem::path(config_.directory) / (session + ".jsonl");
      out.open(path);
      if (!out) throw std::runtime_error("Cannot open planner trace");
      tools::logger()->info("[PlannerTrace] path={} limit_bytes={}", path.string(), config_.max_bytes);
    } catch (const std::exception &e) {
      trace_enabled_ = false;
      tools::logger()->warn("[PlannerTrace] disabled: {}", e.what());
    }
  }
  while (const auto event = queue_.wait_pop()) {
    try {
      if (event->detail) {
        tools::logger()->info("[PlannerResult] plan={} reason={} yaw={} iter={} pri={} dua={} pitch={} iter={} pri={} dua={} error={}",
          event->plan_id, to_string(event->plan.status), to_string(event->plan.yaw_mpc.status),
          event->plan.yaw_mpc.iterations, event->plan.yaw_mpc.primal_residual,
          event->plan.yaw_mpc.dual_residual, to_string(event->plan.pitch_mpc.status),
          event->plan.pitch_mpc.iterations, event->plan.pitch_mpc.primal_residual,
          event->plan.pitch_mpc.dual_residual, event->plan.error_message);
      }
      if (event->summary) {
        const auto &c = event->counters;
        Json reasons = Json::object();
        for (size_t i = 0; i < c.reasons.size(); ++i)
          if (c.reasons[i]) reasons[to_string(static_cast<PlanStatus>(i))] = c.reasons[i];
        Json axes = Json::object();
        Json sends = Json::object();
        const char *send_names[] = {"ready", "no_control", "no_plan", "plan_expired",
                                    "feedback_stale", "no_observation", "observation_stale"};
        for (size_t i = 0; i < event->send_counts.size(); ++i)
          sends[send_names[i]] = event->send_counts[i];
        for (size_t i = 0; i < 2; ++i) {
          if (!event->axis_samples[i]) continue;
          const auto &iterations = event->axis_summaries[2 * i];
          const auto &time = event->axis_summaries[2 * i + 1];
          axes[i == 0 ? "yaw" : "pitch"] = {{"samples", iterations.sample_count},
            {"iterations_p95", iterations.p95_ms}, {"iterations_max", iterations.max_ms},
            {"solve_p95_ms", time.p95_ms}, {"solve_max_ms", time.max_ms}};
        }
        tools::logger()->info("[PlannerStatus] {}", Json({{"plans", c.plans},
          {"with_target", c.with_target}, {"attempted", c.attempted}, {"solved", c.solved},
          {"mpc_failed", c.mpc_failed}, {"display_invalid", c.display_invalid},
          {"recovered", c.recovered}, {"over_2ms", c.over_2ms},
          {"expired_at_completion", c.expired_at_completion}, {"reasons", reasons},
          {"axes_last_256_attempts", axes},
          {"send_gate_counts", sends},
          {"diagnostic_dropped", dropped()}, {"trace_written", written()},
          {"trace_enabled", trace_enabled()}}).dump());
      }
      if (event->trace && trace_enabled()) {
        auto j = planner_trace_json(*event, origin_, session);
        j["continuous"] = previous_trace_id != 0 && event->plan_id == previous_trace_id + 1;
        j["diagnostic_dropped"] = dropped();
        const std::string line = j.dump() + '\n';
        if (bytes + line.size() > config_.max_bytes) {
          trace_enabled_ = false;
          tools::logger()->warn("[PlannerTrace] size limit reached; trace disabled");
          continue;
        }
        out << line;
        out.flush();
        if (!out) throw std::runtime_error("Planner trace write failed");
        bytes += line.size();
        previous_trace_id = event->plan_id;
        ++written_;
      }
    } catch (const std::exception &e) {
      trace_enabled_ = false;
      tools::logger()->warn("[PlannerDiagnostics] trace disabled: {}", e.what());
    }
  }
  if (out.is_open()) {
    out.flush();
    if (!out) tools::logger()->warn("[PlannerTrace] final flush failed");
  }
}
} // namespace auto_aim::runtime
