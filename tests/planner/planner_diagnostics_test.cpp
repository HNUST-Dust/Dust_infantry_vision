#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include "src/planner_diagnostics.hpp"

using namespace auto_aim;
using namespace auto_aim::runtime;
using namespace std::chrono_literals;

namespace
{
void require(bool value, const char * message)
{
  if (!value)
    throw std::runtime_error(message);
}
void reference_metadata_trace()
{
  auto trace = std::make_shared<PlanningTrace>();
  trace->reference_available = true;
  trace->reference_tags[0] = {2, true};
  trace->reference_tags[1] = {3, false};
  PlannerDiagnosticEvent event;
  event.trace = trace;
  const auto origin = std::chrono::steady_clock::now();
  event.plan.planned_at = origin;
  const auto j = planner_trace_json(event, origin, "reference-metadata-test");
  const auto & ids = j.at("reference_armor_ids");
  const auto & valid = j.at("reference_pitch_rate_valid");
  require(ids.size() == HORIZON && valid.size() == HORIZON,
          "Trace reference metadata must cover the complete horizon");
  require(ids.at(0) == 2 && valid.at(0) == true && ids.at(1) == 3 && valid.at(1) == false &&
            ids.at(2) == -1 && valid.at(2) == false,
          "Trace must preserve selected, unusable and unavailable reference identities");
  trace->mpc_mode = MpcMode::reference_smoothing;
  trace->state_index = trace->input_index = SMOOTHING_CENTER;
  trace->yaw_x0 = {0.2, 0.3};
  trace->pitch_x0 = {-0.1, 0.4};
  event.plan.command_at = origin;
  event.plan.mpc_feedforward = {0.5, 1.5, -0.3, -2.0};
  const auto smoothing = planner_trace_json(event, origin, "smoothing-metadata-test");
  const auto & sampling = smoothing.at("mpc_sampling");
  require(sampling.at("mode") == "reference_smoothing" && sampling.at("state_index") == 30 &&
            sampling.at("input_index") == 30 && sampling.at("yaw_x0").at(0).at(0) == 0.2 &&
            sampling.at("pitch_x0").at(1).at(0) == 0.4 &&
            sampling.at("reference_anchor") == "center_hit" &&
            sampling.at("reference_start_s") == -0.3 && sampling.at("command_offset_s") == 0.0 &&
            smoothing.at("optimized_feedforward").at("yaw_acc") == 1.5 &&
            smoothing.at("output").at("yaw_acc") == 0.0,
          "Trace separates solver initial state, time grid and raw optimization from wire fields");
  trace->reference_available = false;
  const auto absent = planner_trace_json(event, origin, "reference-metadata-test");
  require(absent.at("reference_armor_ids").is_null() &&
            absent.at("reference_pitch_rate_valid").is_null(),
          "Unavailable reference metadata must not look like a valid horizon");
  require(absent.at("fire_windows").is_null(), "Absent gate context must be null");
  trace->reference_available = true;
  event.plan.control = true;
  event.plan.settled_reference_index = event.plan.fire_reference_index = SMOOTHING_CENTER;
  event.plan.fire_window = {true, 0.05, 0.5, {FireDirectionState::moving, 1, 0.2}};
  event.plan.settled_window = event.plan.fire_window;
  event.plan.fire_window_check = event.plan.fire_window.evaluate(-0.03, 0);
  event.plan.settled_check = event.plan.fire_window_check;
  const auto gates = planner_trace_json(event, origin, "directional-gate-test").at("fire_windows");
  require(gates.at("settled").at("reference_index") == 30 &&
            gates.at("command").at("direction_state") == "moving" &&
            gates.at("command").at("reference_yaw_rate") == 0.2 &&
            gates.at("command").at("aligned_yaw_error") == -0.03 &&
            gates.at("command").at("applied_yaw_tolerance") == 0.025 &&
            gates.at("command").at("passed") == false,
          "Trace preserves the exact window which rejected lagging feedback");
  event.plan.fire_window.motion = {};
  event.plan.fire_window_check = event.plan.fire_window.evaluate(0, 0);
  const auto unknown =
    planner_trace_json(event, origin, "unknown-direction-test").at("fire_windows").at("command");
  require(unknown.at("reference_yaw_rate").is_null() && unknown.at("aligned_yaw_error").is_null() &&
            unknown.at("passed") == true,
          "Unknown direction is not serialized as a measured zero rate");
}

void completed_snapshot()
{
  tools::ThreadSafeQueue<Plan, true> queue(1);
  Plan first;
  first.status = PlanStatus::ok;
  first.control = true;
  first.aim_debug.display_valid = true;
  first.aim_debug.xyza[0] = 3;
  first.planned_at = std::chrono::steady_clock::now();
  queue.push(first);
  std::mutex mutex;
  std::condition_variable cv;
  bool computing = false, publish = false;
  std::thread producer([&] {
    std::unique_lock<std::mutex> lock(mutex);
    computing = true;
    cv.notify_all();
    cv.wait(lock, [&] { return publish; });
    Plan failed;
    failed.status = PlanStatus::mpc_failed;
    failed.planned_at = first.planned_at + 2ms;
    queue.push(failed);
  });
  {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&] { return computing; });
  }
  const auto during = queue.front();
  const bool intact = during.control && during.aim_debug.display_valid &&
                      during.aim_debug.xyza[0] == 3 && during.planned_at == first.planned_at;
  {
    std::lock_guard<std::mutex> lock(mutex);
    publish = true;
  }
  cv.notify_all();
  producer.join();
  require(intact, "Completed snapshot must survive next computation");
  const auto failed = queue.front();
  require(!failed.control && !failed.fire && !failed.aim_debug.display_valid &&
            failed.status == PlanStatus::mpc_failed,
          "Publish failure atomically");
  require(first.aim_visible_at(first.planned_at + 20ms) &&
            !first.aim_visible_at(first.planned_at + 21ms) &&
            !first.aim_visible_at(first.planned_at - 1ms),
          "Display age gate");
}
}  // namespace

int main()
{
  std::filesystem::path directory;
  try {
    reference_metadata_trace();
    completed_snapshot();
    char pattern[] = "/tmp/dust-planner-diagnostics-XXXXXX";
    const char * path = mkdtemp(pattern);
    require(path, "Temporary directory");
    directory = path;
    const auto minimal_config = directory / "minimal.yaml";
    {
      std::ofstream out(minimal_config);
      out << "unrelated: true\n";
    }
    require(!PlannerDiagnosticsConfig::load(minimal_config.string()).trace_enabled,
            "Missing runtime configuration keeps trace disabled");
    PlannerDiagnosticsConfig config;
    config.trace_enabled = true;
    config.trace_all = true;
    config.directory = (directory / "records").string();
    PlanningTrace trace;
    Plan p;
    p.status = PlanStatus::feedback_stale;
    p.planned_at = std::chrono::steady_clock::now();
    const auto unavailable = mpc_diagnostics_json(p.yaw_mpc);
    require(!unavailable.at("attempted").get<bool>() && unavailable.at("primal_residual").is_null(),
            "Unattempted axis is not zero residual");
    {
      PlannerDiagnostics diagnostics(config);
      diagnostics.observe(p, true, .1, p.planned_at, {}, &trace);
      p.status = PlanStatus::ok;
      p.control = true;
      p.yaw_mpc.attempted = true;
      p.valid_at = p.planned_at + 10ms;
      diagnostics.observe(p, true, .1, p.planned_at + 2ms, {}, &trace);
      diagnostics.close();
      require(diagnostics.written() == 2, "Drain trace on close");
      require(diagnostics.counters().plans == 2 && diagnostics.counters().recovered == 1 &&
                diagnostics.counters().attempted == 1,
              "Exact attempt and recovery counters");
    }
    size_t records = 0;
    for (const auto & entry : std::filesystem::directory_iterator(config.directory)) {
      std::ifstream in(entry.path());
      std::string line;
      while (std::getline(in, line)) {
        const auto j = nlohmann::json::parse(line);
        ++records;
        require(j.at("plan_id").get<size_t>() == records, "Trace plan identity");
        require(j.at("continuous").get<bool>() == (records > 1), "Trace continuity");
      }
    }
    require(records == 2, "Parse stored trace");
    config.directory = (directory / "burst").string();
    {
      PlannerDiagnostics diagnostics(config);
      for (int i = 0; i < 2048; ++i)
        diagnostics.observe(p, true, .1, p.planned_at + i * 2ms, {}, &trace);
      diagnostics.close();
      require(diagnostics.trace_enabled() && diagnostics.written() + diagnostics.dropped() == 2048,
              "Bounded queue drains or accounts for every dropped trace");
      std::cout << "burst written=" << diagnostics.written() << " dropped=" << diagnostics.dropped()
                << '\n';
    }
    config.directory = (directory / "limited").string();
    config.max_bytes = 1;
    {
      PlannerDiagnostics diagnostics(config);
      diagnostics.observe(p, true, .1, p.planned_at, {}, &trace);
      diagnostics.close();
      require(!diagnostics.trace_enabled() && diagnostics.written() == 0, "Trace size bound");
    }
    const auto blocked = directory / "file";
    {
      std::ofstream out(blocked);
      out << "not a directory";
    }
    config.directory = blocked.string();
    {
      PlannerDiagnostics diagnostics(config);
      diagnostics.observe(p, true, .1, p.planned_at, {}, &trace);
      diagnostics.close();
      require(!diagnostics.trace_enabled(), "I/O failure disables capture");
    }
    std::filesystem::remove_all(directory);
    std::cout
      << "PASS planner completed snapshots, trace, counters, queue bound, size/I/O failure\n";
    return 0;
  } catch (const std::exception & e) {
    if (!directory.empty())
      std::filesystem::remove_all(directory);
    std::cerr << "FAIL diagnostics: " << e.what() << '\n';
    return 1;
  }
}
