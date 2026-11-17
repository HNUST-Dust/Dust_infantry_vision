#pragma once

#include <chrono>
#include <cstdint>
#include <nlohmann/json_fwd.hpp>
#include <optional>

#include "tools/diagnostics/latency_stats.hpp"
#include "src/diagnostics/send_telemetry.hpp"

namespace io
{
struct GimbalFeedback;
}

namespace auto_aim
{
struct Plan;

namespace multithread
{
struct DetectorDropCounts;
}

namespace runtime
{
struct TelemetryTarget
{
  double z, vz, spin_rate, angle;
};

struct DetectionTiming
{
  double capture_ms, submit_ms, inference_ms, inference_wait_ms, postprocess_ms, delivery_ms;
};

struct PlannerTraceCounts
{
  uint64_t dropped, written;
  bool enabled;
};

// The referenced values are the caller's sampled copies and must live through serialization.
struct TelemetrySnapshot
{
  std::chrono::steady_clock::time_point sampled_at;
  std::chrono::steady_clock::time_point origin;
  int tracker_state;
  std::optional<TelemetryTarget> target;
  const Plan & plan;
  double planning_ms;
  uint64_t plan_sequence;
  int send_status;
  const io::GimbalFeedback & feedback;
  bool fired;
  PlannerTraceCounts trace;
  DetectionTiming timing;
  const multithread::DetectorDropCounts & detector_drops;
  const std::optional<tools::LatencySummary> & latency;
  SendTelemetrySnapshot send;
};

nlohmann::json telemetry_json(const TelemetrySnapshot & snapshot);
}  // namespace runtime
}  // namespace auto_aim
