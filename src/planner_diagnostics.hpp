#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <nlohmann/json.hpp>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/latency_stats.hpp"

namespace auto_aim::runtime
{
struct PlannerDiagnosticsConfig {
  bool trace_enabled = false;
  bool trace_all = false;
  std::string directory = "logs/planner_trace";
  uint64_t max_bytes = 32 * 1024 * 1024;
  static PlannerDiagnosticsConfig load(const std::string &path);
};

struct PlanSource {
  uint64_t frame_sequence = 0;
  uint64_t feedback_sequence = 0;
  std::chrono::steady_clock::time_point frame_timestamp{};
};

struct PlannerCounters {
  uint64_t plans = 0, with_target = 0, solved = 0, attempted = 0;
  uint64_t mpc_failed = 0, display_invalid = 0, recovered = 0;
  uint64_t over_2ms = 0, expired_at_completion = 0;
  std::array<uint64_t, static_cast<size_t>(PlanStatus::exception) + 1> reasons{};
};

struct PlannerDiagnosticEvent {
  uint64_t plan_id = 0;
  Plan plan;
  PlanSource source;
  double planning_ms = 0.0;
  bool detail = false, summary = false;
  PlannerCounters counters;
  std::shared_ptr<const PlanningTrace> trace;
  std::array<tools::LatencySummary, 4> axis_summaries{};
  std::array<bool, 2> axis_samples{};
  std::array<uint64_t, 7> send_counts{};
};

nlohmann::json mpc_diagnostics_json(const MpcDiagnostics &diagnostic);
nlohmann::json mpc_config_json(const MpcConfig &config);
MpcConfig mpc_config_from_json(const nlohmann::json &json);
nlohmann::json planner_trace_json(const PlannerDiagnosticEvent &event,
                                  std::chrono::steady_clock::time_point origin,
                                  const std::string &session);

class PlannerDiagnostics
{
public:
  explicit PlannerDiagnostics(const PlannerDiagnosticsConfig &config);
  ~PlannerDiagnostics();
  PlannerDiagnostics(const PlannerDiagnostics &) = delete;
  PlannerDiagnostics &operator=(const PlannerDiagnostics &) = delete;
  bool trace_enabled() const { return trace_enabled_.load(); }
  uint64_t dropped() const { return dropped_.load(); }
  uint64_t written() const { return written_.load(); }
  void observe_send(size_t status) { send_counts_.at(status).fetch_add(1, std::memory_order_relaxed); }
  const PlannerCounters &counters() const { return counters_; } // Planning thread only.
  void observe(const Plan &plan, bool has_target, double planning_ms,
               std::chrono::steady_clock::time_point completed, const PlanSource &source,
               const PlanningTrace *trace);
  void close();

private:
  void write_loop();
  PlannerDiagnosticsConfig config_;
  std::chrono::steady_clock::time_point origin_, last_summary_, rate_window_;
  int detail_count_ = 0, trace_count_ = 0;
  PlanStatus last_status_ = PlanStatus::not_run;
  MpcStatus last_yaw_ = MpcStatus::not_run, last_pitch_ = MpcStatus::not_run;
  bool previous_failed_ = false;
  PlannerCounters counters_;
  std::array<tools::LatencyStats, 4> axis_stats_;
  tools::ThreadSafeQueue<PlannerDiagnosticEvent> queue_{128};
  std::atomic<bool> trace_enabled_{false};
  std::atomic<uint64_t> dropped_{0}, written_{0};
  std::array<std::atomic<uint64_t>, 7> send_counts_{{0, 0, 0, 0, 0, 0, 0}};
  std::thread writer_;
};
} // namespace auto_aim::runtime
