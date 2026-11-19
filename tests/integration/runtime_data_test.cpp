#include <cassert>
#include <cmath>
#include <fstream>
#include <string>

#include "src/app_config.hpp"
#include "src/diagnostics/telemetry.hpp"
#include "tasks/auto_aim/detection/mt_detector.hpp"
#include "tools/concurrency/thread_safe_queue.hpp"
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
                             {},
                             drops,
                             latency,
                             {}};
  snapshot.send.sequence = 101;
  snapshot.send.plan_id = 42;
  snapshot.send.t = 0.038;
  snapshot.send.mode = 1;
  snapshot.send.target_valid = true;
  snapshot.send.target_yaw = 0.04;
  snapshot.send.target_pitch = -0.02;
  snapshot.send.fire_tolerance = 0.035;
  snapshot.send.fire_block_mask = send_observation_fire_stale | send_angle_outside;
  snapshot.send.mode2_time_s = 0.012;
  snapshot.send.total_time_s = 0.03;
  tools::ThreadSafeQueue<DetectionTiming, true> timing(1);
  timing.push(DetectionTiming{});
  snapshot.timing = timing.front();
  const auto startup = telemetry_json(snapshot);
  assert(startup.at("detection_sequence") == 0 && startup.at("detection_frame_t").is_null());
  const char * fields[] = {"capture_time_ms",   "detection_submit_ms", "inference_time_ms",
                           "inference_wait_ms", "postprocess_time_ms", "result_delivery_ms"};
  for (const auto * field : fields)
    assert(startup.at(field) == 0);

  timing.push({23, origin + 15ms, 10, 20, 30, 40, 50, 60});
  snapshot.timing = timing.front();
  timing.push({24, origin + 25ms, 11, 21, 31, 41, 51, 61});
  const auto data = telemetry_json(snapshot);
  snapshot.timing = timing.front();
  const auto next = telemetry_json(snapshot);
  assert(data.at("detection_sequence") == 23 && next.at("detection_sequence") == 24);
  assert(std::abs(data.at("detection_frame_t").get<double>() - 0.015) < 1e-12);
  assert(std::abs(next.at("detection_frame_t").get<double>() - 0.025) < 1e-12);
  int expected_ms = 10;
  for (const auto * field : fields) {
    assert(data.at(field) == expected_ms && next.at(field) == expected_ms + 1);
    expected_ms += 10;
  }
  assert(std::abs(data.at("t").get<double>() - 0.04) < 1e-12);
  assert(std::abs(data.at("aim_age_ms").get<double>() - 5.0) < 1e-12);
  assert(data.at("aim_mode") == "preaim");
  assert(data.at("plan_yaw_acc") == 2.5 && data.at("plan_pitch_acc") == -3.5);
  assert(data.at("gimbal_yaw") == 0.5 && data.at("gimbal_pitch_vel") == -0.25);
  assert(data.at("aim_plan_sequence") == 17 && data.at("feedback_sequence") == 19);
  assert(data.at("plan_control").is_boolean() && data.at("plan_control") == true);
  assert(data.at("mode") == 2 && data.at("fire") == 1 && data.at("fired") == 1);
  assert(data.at("send_valid") == true && data.at("send_mode") == 1);
  assert(data.at("send_t") == 0.038 && data.at("send_sequence") == 101);
  assert(data.at("send_plan_id") == 42 && data.at("send_plan_id") != data.at("aim_plan_sequence"));
  assert(data.at("send_target_valid") == true);
  assert(data.at("send_target_yaw") == 0.04 && data.at("send_target_pitch") == -0.02);
  assert(data.at("send_fire_tolerance") == 0.035);
  assert(data.at("send_fire_block_mask") == (send_observation_fire_stale | send_angle_outside));
  assert(data.at("send_block_observation_fire_stale") == true);
  assert(data.at("send_block_angle_outside") == true);
  assert(data.at("send_block_plan_expired") == false);
  assert(data.at("send_mode2_time_s") == 0.012 && data.at("send_total_time_s") == 0.03);
  assert(data.at("send_status_code") == 4 && data.at("send_status") == "feedback_stale");
  assert(data.at("has_target") == 1 && data.at("w") == 8.0 && data.at("angle") == 1.5);
  assert(data.at("target_z") == 0.5 && data.at("target_vz") == -0.25);
  assert(data.at("skipped_empty_frames") == 4 && data.at("skipped_busy_frames") == 3);
  assert(data.at("inference_wait_ms") == 40 && data.at("vision_latency_p95_ms") == 9);
  snapshot.target.reset();
  const auto missing = telemetry_json(snapshot);
  assert(missing.at("has_target") == 0 && missing.at("w") == 0.0);
  assert(!missing.contains("target_z") && !missing.contains("angle"));
  snapshot.send = {};
  const auto before_first_send = telemetry_json(snapshot);
  assert(before_first_send.at("send_valid") == false);
  assert(before_first_send.at("send_target_valid") == false);
  assert(before_first_send.at("send_target_yaw").is_null());
  assert(before_first_send.at("send_target_pitch").is_null());
  assert(before_first_send.at("send_fire_tolerance").is_null());
  assert(before_first_send.at("send_total_time_s") == 0.0);

  // Control may be cleared after relative_to(); retain the error for diagnosing that gate.
  snapshot.send.sequence = 102;
  snapshot.send.target_valid = true;
  snapshot.send.target_yaw = 0.2;
  snapshot.send.fire_block_mask = send_plan_expired | send_feedback_stale;
  const auto expired = telemetry_json(snapshot);
  assert(expired.at("send_mode") == 0 && expired.at("send_target_yaw") == 0.2);
  assert(expired.at("send_block_plan_expired") == true);
  assert(expired.at("send_block_feedback_stale") == true);
}

void check_send_durations()
{
  const auto origin = std::chrono::steady_clock::time_point{} + 1s;
  SendTelemetryAccumulator accumulator(origin);
  SendTelemetrySnapshot sample;
  sample.plan_id = 42;
  sample.mode = 0;
  const auto first = accumulator.observe(origin + 100ms, sample);
  assert(first.sequence == 1 && first.plan_id == 42);
  assert(std::abs(first.t - 0.1) < 1e-12);
  assert(first.total_time_s == 0.0 && first.mode2_time_s == 0.0);
  sample.mode = 2;
  accumulator.observe(origin + 110ms, sample);
  sample.mode = 1;
  accumulator.observe(origin + 112ms, sample);
  sample.mode = 2;
  accumulator.observe(origin + 115ms, sample);
  sample.mode = 0;
  const auto last = accumulator.observe(origin + 130ms, sample);
  assert(last.sequence == 5 && last.mode == 0);
  assert(std::abs(last.total_time_s - 0.03) < 1e-12);
  // Includes a 2 ms fire window, even though telemetry only sampled first and last.
  assert(std::abs(last.mode2_time_s - 0.017) < 1e-12);
  assert(
    std::abs((last.mode2_time_s - first.mode2_time_s) / (last.total_time_s - first.total_time_s) -
             17.0 / 30.0) < 1e-12);
}
}  // namespace

int main(int argc, char * argv[])
{
  assert(argc == 3);
  check_config(argv[1], argv[2]);
  check_telemetry();
  check_send_durations();
}
