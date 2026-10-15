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
void require(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
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
  { std::lock_guard<std::mutex> lock(mutex); publish = true; }
  cv.notify_all();
  producer.join();
  require(intact, "Completed snapshot must survive next computation");
  const auto failed = queue.front();
  require(!failed.control && !failed.fire && !failed.aim_debug.display_valid &&
          failed.status == PlanStatus::mpc_failed, "Publish failure atomically");
  require(first.aim_visible_at(first.planned_at + 20ms) &&
          !first.aim_visible_at(first.planned_at + 21ms) &&
          !first.aim_visible_at(first.planned_at - 1ms), "Display age gate");
}
} // namespace

int main()
{
  std::filesystem::path directory;
  try {
    completed_snapshot();
    char pattern[] = "/tmp/dust-planner-diagnostics-XXXXXX";
    const char *path = mkdtemp(pattern);
    require(path, "Temporary directory");
    directory = path;
    const auto minimal_config = directory / "minimal.yaml";
    { std::ofstream out(minimal_config); out << "unrelated: true\n"; }
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
    require(!unavailable.at("attempted").get<bool>() &&
            unavailable.at("primal_residual").is_null(), "Unattempted axis is not zero residual");
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
              diagnostics.counters().attempted == 1, "Exact attempt and recovery counters");
    }
    size_t records = 0;
    for (const auto &entry : std::filesystem::directory_iterator(config.directory)) {
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
      std::cout << "burst written=" << diagnostics.written()
                << " dropped=" << diagnostics.dropped() << '\n';
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
    { std::ofstream out(blocked); out << "not a directory"; }
    config.directory = blocked.string();
    {
      PlannerDiagnostics diagnostics(config);
      diagnostics.observe(p, true, .1, p.planned_at, {}, &trace);
      diagnostics.close();
      require(!diagnostics.trace_enabled(), "I/O failure disables capture");
    }
    std::filesystem::remove_all(directory);
    std::cout << "PASS planner completed snapshots, trace, counters, queue bound, size/I/O failure\n";
    return 0;
  } catch (const std::exception &e) {
    if (!directory.empty()) std::filesystem::remove_all(directory);
    std::cerr << "FAIL diagnostics: " << e.what() << '\n';
    return 1;
  }
}
