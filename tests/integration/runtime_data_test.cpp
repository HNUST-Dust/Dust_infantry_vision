#include <cassert>
#include <cmath>
#include <fstream>
#include <string>

#include "src/app_config.hpp"
#include "src/diagnostics/telemetry.hpp"
#include "tasks/auto_aim/detection/mt_detector.hpp"
#include "tools/runtime/yaml.hpp"

using namespace auto_aim;
using namespace auto_aim::runtime;
using namespace std::chrono_literals;

namespace
{
void check_config(const std::string & config_path, const std::filesystem::path & invalid_path)
{
  const auto config = load_application_config(config_path);
  const auto yaml = tools::load(config_path);
  assert(config.detector.net.model_path == yaml["yolov5_model_path"].as<std::string>());
  assert(config.detector.postprocess.detector.classifier.model_path ==
         yaml["classify_model"].as<std::string>());
  assert(config.runtime.plan_timeout.count() == yaml["runtime"]["plan_timeout_ms"].as<int>());
  assert(config.visualization.headless == yaml["runtime"]["headless"].as<bool>());
  assert(config.planner.yaw.max_acc == yaml["max_yaw_acc"].as<double>());
  assert(config.planner.pitch.max_acc == yaml["max_pitch_acc"].as<double>());
  assert(config.planner.yaw_offset == yaml["yaw_offset"].as<double>() / 57.3);
  assert(config.camera && config.camera->name == yaml["camera_name"].as<std::string>());
  const auto saved = YAML::Load(config.planner.replay_yaml);
  assert(saved["max_pitch_acc"].as<double>() == config.planner.pitch.max_acc);

  {
    std::ofstream output(invalid_path);
    output << "runtime:\n  planner_diagnostics:\n    trace_max_mb: -1\n";
  }
  std::string error_message;
  try {
    load_application_config(invalid_path);
  } catch (const std::invalid_argument & error) {
    error_message = error.what();
  }
  std::filesystem::remove(invalid_path);
  assert(error_message.find(invalid_path.string()) != std::string::npos);
  assert(error_message.find("runtime.planner_diagnostics.trace_max_mb") != std::string::npos);
}

void check_telemetry()
{
  const auto origin = std::chrono::steady_clock::time_point{} + 1s;
  const auto sampled_at = origin + 40ms;
  Plan plan;
  plan.planned_at = sampled_at - 5ms;
  plan.control = true;
  plan.fire = true;
  plan.yaw_acc = 2.5;
  plan.pitch_acc = -3.5;
  plan.aim_debug.mode = AimMode::preaim;
  io::GimbalFeedback feedback;
  feedback.sequence = 19;
  feedback.state.yaw = 0.5;
  feedback.state.pitch_vel = -0.25;
  const multithread::DetectorDropCounts drops{1, 2, 3, 4, 5};
  const std::optional<tools::LatencySummary> latency{{6, 7, 8, 9, 10, 11}};
  TelemetrySnapshot snapshot{sampled_at,
                             origin,
                             2,
                             TelemetryTarget{0.5, -0.25, 8.0, 1.5},
                             plan,
                             1.5,
                             17,
                             4,
                             feedback,
                             true,
                             {7, 8, true},
                             {10, 20, 30, 40, 50, 60},
                             drops,
                             latency};
  const auto data = telemetry_json(snapshot);
  assert(std::abs(data.at("t").get<double>() - 0.04) < 1e-12);
  assert(std::abs(data.at("aim_age_ms").get<double>() - 5.0) < 1e-12);
  assert(data.at("aim_mode") == "preaim");
  assert(data.at("plan_yaw_acc") == 2.5 && data.at("plan_pitch_acc") == -3.5);
  assert(data.at("gimbal_yaw") == 0.5 && data.at("gimbal_pitch_vel") == -0.25);
  assert(data.at("aim_plan_sequence") == 17 && data.at("feedback_sequence") == 19);
  assert(data.at("plan_control").is_boolean() && data.at("plan_control") == true);
  assert(data.at("mode") == 2 && data.at("fire") == 1 && data.at("fired") == 1);
  assert(data.at("send_status_code") == 4 && data.at("send_status") == "feedback_stale");
  assert(data.at("has_target") == 1 && data.at("w") == 8.0 && data.at("angle") == 1.5);
  assert(data.at("target_z") == 0.5 && data.at("target_vz") == -0.25);
  assert(data.at("skipped_empty_frames") == 4 && data.at("skipped_busy_frames") == 3);
  assert(data.at("inference_wait_ms") == 40 && data.at("vision_latency_p95_ms") == 9);
  snapshot.target.reset();
  const auto missing = telemetry_json(snapshot);
  assert(missing.at("has_target") == 0 && missing.at("w") == 0.0);
  assert(!missing.contains("target_z") && !missing.contains("angle"));
}
}  // namespace

int main(int argc, char * argv[])
{
  assert(argc == 3);
  check_config(argv[1], argv[2]);
  check_telemetry();
}
