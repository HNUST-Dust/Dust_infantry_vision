#include "src/auto_aim_runtime.hpp"
#include "src/planner_diagnostics.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <stdexcept>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#ifdef DUST_ENABLE_SIMULATOR
#include "io/talos/talos.hpp"
#endif
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/latency_stats.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/yaml.hpp"
#include "visualization/vision_overlay.hpp"

#ifdef DUST_ENABLE_FOXGLOVE
#include "visualization/foxglove_vision.hpp"
#endif

using namespace std::chrono_literals;

namespace auto_aim
{
namespace runtime
{
namespace
{

// 规划线程与检测主循环之间只传容量一的目标状态快照。
struct TargetUpdate
{
  std::optional<Target> target;
  std::chrono::steady_clock::time_point frame_timestamp;
  std::chrono::steady_clock::time_point last_observed_timestamp;
  uint64_t sequence = 0;
};

struct PlanUpdate
{
  Plan plan;
  std::chrono::steady_clock::time_point generated_at;
  std::chrono::steady_clock::time_point frame_timestamp;
  std::chrono::steady_clock::time_point last_observed_timestamp;
  double planning_ms = 0.0;
  uint64_t sequence = 0;
  float distance_m = -1.0F;
};

void add_latency_metrics(
  nlohmann::json & data, const std::optional<tools::LatencySummary> & latency_summary)
{
  if (!latency_summary) return;

  const auto & summary = *latency_summary;
  data["vision_latency_ms"] = summary.latest_ms;
  data["vision_latency_p50_ms"] = summary.p50_ms;
  data["vision_latency_p95_ms"] = summary.p95_ms;
  data["vision_latency_p99_ms"] = summary.p99_ms;
}

// 运行模式开关来自 config_path 的 runtime 段；缺段或缺键都退回下面的默认值。
// 这四个开关原先在命令行上（--simulate-gimbal/--headless/--verbose-ekf/--foxglove），已移除。
struct RuntimeConfig
{
  std::string io_backend = "hardware";
  bool simulate_gimbal = false;
  bool headless = false;
  bool verbose_ekf = false;
  bool foxglove_enabled = false;
  std::chrono::milliseconds fire_timeout{30};
  std::chrono::milliseconds control_timeout{80};
  std::chrono::milliseconds feedback_timeout{100};
  std::chrono::milliseconds plan_timeout{20};
};

RuntimeConfig load_runtime_config(const std::string & config_path)
{
  RuntimeConfig config;
  const auto node = tools::load(config_path)["runtime"];  // 加载失败时抛 runtime_error
  if (!node) return config;

  if (node["io_backend"]) config.io_backend = node["io_backend"].as<std::string>();
  if (node["simulate_gimbal"]) config.simulate_gimbal = node["simulate_gimbal"].as<bool>();
  if (node["headless"]) config.headless = node["headless"].as<bool>();
  if (node["verbose_ekf"]) config.verbose_ekf = node["verbose_ekf"].as<bool>();
  if (node["foxglove"]) config.foxglove_enabled = node["foxglove"].as<bool>();
  if (node["fire_timeout_ms"])
    config.fire_timeout = std::chrono::milliseconds(node["fire_timeout_ms"].as<int>());
  if (node["control_timeout_ms"])
    config.control_timeout = std::chrono::milliseconds(node["control_timeout_ms"].as<int>());
  if (node["feedback_timeout_ms"])
    config.feedback_timeout = std::chrono::milliseconds(node["feedback_timeout_ms"].as<int>());
  if (node["plan_timeout_ms"])
    config.plan_timeout = std::chrono::milliseconds(node["plan_timeout_ms"].as<int>());
  if (config.io_backend != "hardware" && config.io_backend != "talos")
    throw std::runtime_error("runtime.io_backend must be hardware or talos");
#ifndef DUST_ENABLE_SIMULATOR
  if (config.io_backend == "talos")
    throw std::runtime_error(
      "Talos simulator backend is not compiled in; configure with -DENABLE_SIMULATOR=ON");
#endif
  if (
    config.fire_timeout <= 0ms || config.control_timeout <= 0ms || config.feedback_timeout <= 0ms ||
    config.plan_timeout <= 0ms)
    throw std::runtime_error("runtime safety timeout values must be positive");
  return config;
}

#ifdef DUST_ENABLE_FOXGLOVE
// Foxglove 的运行参数来自 config_path 的 foxglove 段；缺段或缺键都退回下面的默认值，
// 与 visualization::FoxgloveVisionOptions 的默认值一致。启用开关在 runtime 段，不在这里。
struct FoxgloveConfig
{
  std::string host = "127.0.0.1";
  int image_port = 8766;
  int data_port = 0;
  double image_fps = 30.0;
  double image_scale = 0.5;
  int jpeg_quality = 80;
  bool yield_cpu = true;
};

FoxgloveConfig load_foxglove_config(const std::string & config_path)
{
  FoxgloveConfig config;
  const auto node = tools::load(config_path)["foxglove"];  // 加载失败时抛 runtime_error
  if (!node) return config;

  if (node["host"]) config.host = node["host"].as<std::string>();
  if (node["port"]) config.image_port = node["port"].as<int>();
  if (node["data_port"]) config.data_port = node["data_port"].as<int>();
  if (node["fps"]) config.image_fps = node["fps"].as<double>();
  if (node["scale"]) config.image_scale = node["scale"].as<double>();
  if (node["jpeg_quality"]) config.jpeg_quality = node["jpeg_quality"].as<int>();
  if (node["sched"]) {
    const auto sched = node["sched"].as<std::string>();
    if (sched != "auto" && sched != "off")
      throw std::runtime_error("foxglove.sched must be auto or off, but is: " + sched);
    config.yield_cpu = sched == "auto";
  }
  return config;
}
#endif

}  // namespace

int run(const RuntimeOptions & options)
{
  tools::Exiter exiter;

  // 运行模式开关先于任何硬件对象读取：配置文件缺失、runtime 段类型不合法都在这里退出
  RuntimeConfig runtime_config;
  PlannerDiagnosticsConfig diagnostics_config;
  try {
    runtime_config = load_runtime_config(options.config_path);
    diagnostics_config = PlannerDiagnosticsConfig::load(options.config_path);
  } catch (const std::exception & e) {
    tools::logger()->error("[Runtime] {}", e.what());
    return 2;
  }

  const bool headless = runtime_config.headless;

#ifdef DUST_ENABLE_FOXGLOVE
  std::unique_ptr<visualization::FoxgloveVision> foxglove;
  if (runtime_config.foxglove_enabled) {
    FoxgloveConfig foxglove_config;
    try {
      foxglove_config = load_foxglove_config(options.config_path);
    } catch (const std::exception & e) {
      tools::logger()->error("[Foxglove] {}", e.what());
      return 2;
    }
    if (
      foxglove_config.image_port < 1 || foxglove_config.image_port > 65535 ||
      foxglove_config.data_port < 0 || foxglove_config.data_port > 65535 ||
      foxglove_config.image_fps <= 0 || foxglove_config.image_scale <= 0 ||
      foxglove_config.image_scale > 1.0 || foxglove_config.jpeg_quality < 1 ||
      foxglove_config.jpeg_quality > 100) {
      tools::logger()->error(
        "[Foxglove] Invalid port, fps, scale, or JPEG quality in the foxglove section");
      return 2;
    }
    visualization::FoxgloveVisionOptions foxglove_options;
    foxglove_options.host = foxglove_config.host;
    foxglove_options.image_port = static_cast<uint16_t>(foxglove_config.image_port);
    foxglove_options.data_port = static_cast<uint16_t>(foxglove_config.data_port);
    foxglove_options.image_fps = foxglove_config.image_fps;
    foxglove_options.image_scale = foxglove_config.image_scale;
    foxglove_options.jpeg_quality = foxglove_config.jpeg_quality;
    foxglove_options.yield_cpu = foxglove_config.yield_cpu;
    try {
      foxglove = std::make_unique<visualization::FoxgloveVision>(foxglove_options);
      tools::logger()->info(
        "[Foxglove] image ws://{}:{}, telemetry ws://{}:{}", foxglove_options.host,
        foxglove->image_port(), foxglove_options.host, foxglove->data_port());
    } catch (const std::exception & e) {
      // 构造失败只记日志，视觉主链路继续运行
      tools::logger()->error("[Foxglove] disabled: {}", e.what());
    }
  }
#else
  if (runtime_config.foxglove_enabled)
    tools::logger()->error(
      "this binary was built without Foxglove; configure with -DENABLE_FOXGLOVE_VISION=ON");
#endif

  const bool talos_backend = runtime_config.io_backend == "talos";
  io::Gimbal gimbal(options.config_path, runtime_config.simulate_gimbal || talos_backend);
  std::unique_ptr<io::Camera> camera;
#ifdef DUST_ENABLE_SIMULATOR
  std::unique_ptr<io::Talos> talos;
#endif
  try {
#ifdef DUST_ENABLE_SIMULATOR
    if (talos_backend)
      talos = std::make_unique<io::Talos>();
    else
#endif
      camera = std::make_unique<io::Camera>(options.config_path);
  } catch (const std::exception & e) {
    tools::logger()->error("[Runtime] {}", e.what());
    return 2;
  }

  multithread::MultiThreadDetector detector(options.config_path, !headless);
  Solver solver(options.config_path);
  Tracker tracker(options.config_path, solver);
  Planner planner(options.config_path);

  // Both queues are capacity-one snapshots: a producer always replaces the old value.
  // The initial entries keep consumers non-blocking from the first cycle onward.
  const auto epoch = std::chrono::steady_clock::time_point{};
  tools::ThreadSafeQueue<TargetUpdate, true> target_queue(1);
  target_queue.push({std::nullopt, epoch, epoch, 0});
  tools::ThreadSafeQueue<PlanUpdate, true> plan_queue(1);
  plan_queue.push({Plan{}, epoch, epoch, epoch, 0.0, 0, -1.0F});

  // 计划线程只读这个原子状态码；直接调 Tracker::state() 会和主循环的 track() 抢同一把锁
  std::atomic<int> tracker_state_code{static_cast<int>(TrackerState::lost)};
  std::atomic<bool> quit = false;
  std::atomic<double> latest_capture_ms{0.0};
  std::atomic<double> latest_submit_ms{0.0};
  std::atomic<double> latest_inference_ms{0.0};
  std::atomic<double> latest_inference_wait_ms{0.0};
  std::atomic<double> latest_postprocess_ms{0.0};
  std::atomic<double> latest_delivery_ms{0.0};
  std::atomic<int> latest_send_status{1};
  PlannerDiagnostics planner_diagnostics(diagnostics_config);

  auto plan_thread = std::thread([&]() {
    constexpr auto period = 2ms;
    auto next = std::chrono::steady_clock::now();
    tools::LatencyStats planning_stats;
    auto last_planning_log = next;
    bool unsupported_frame_reported = false;
    while (!quit) {
      const auto update = target_queue.front();
      const auto started = std::chrono::steady_clock::now();
      Plan plan;
      std::optional<PlanningTrace> trace;
      if (planner_diagnostics.trace_enabled()) trace.emplace();
      PlanSource source;
      source.frame_sequence = update.sequence;
      source.frame_timestamp = update.frame_timestamp;
      try {
        const auto feedback = gimbal.feedback();
        source.feedback_sequence = feedback.sequence;
        const auto control_at = std::chrono::steady_clock::now();
        PlanningState state;
        state.yaw = feedback.state.yaw;
        state.yaw_vel = feedback.state.yaw_vel;
        state.pitch = feedback.state.pitch;
        state.pitch_vel = feedback.state.pitch_vel;
        state.timestamp = feedback.received_at;
        state.R_gimbal2world = solver.R_gimbal2world(feedback.q);
        // The reported rates are defined in the raw ZXY joint frame. A different
        // IMU axis calibration needs a rate mapping, not just rotated angles.
        const bool rate_frame_supported =
          (state.R_gimbal2world - feedback.q.toRotationMatrix()).norm() < 1e-6;
        if (!rate_frame_supported && !unsupported_frame_reported)
          tools::logger()->error(
            "[Planner] IMU axis calibration needs a matching joint-rate mapping; control disabled");
        unsupported_frame_reported = !rate_frame_supported;
        state.valid = feedback.fresh_at(control_at, runtime_config.feedback_timeout) &&
          rate_frame_supported;
        state.rates_measured = feedback.rates_measured;
        plan = planner.plan_at(update.target, control_at, feedback.state.bullet_speed, state,
                               trace ? &*trace : nullptr);
      } catch (const std::exception &e) {
        plan = Plan{};
        plan.planned_at = started;
        plan.status = PlanStatus::exception;
        plan.error_message = e.what();
      }
      const auto generated_at = std::chrono::steady_clock::now();
      const double planning_ms =
        std::chrono::duration<double, std::milli>(generated_at - started).count();
      planning_stats.add(planning_ms);
      {
        float distance_m = -1.0F;
        if (update.target) {
          const auto x = update.target->ekf_x();
          distance_m = static_cast<float>(std::hypot(x[0], x[2]));
        }
        plan_queue.push({
          plan, generated_at, update.frame_timestamp, update.last_observed_timestamp, planning_ms,
          update.sequence, distance_m});
      }
      planner_diagnostics.observe(plan, update.target.has_value(), planning_ms, generated_at,
                                  source, trace ? &*trace : nullptr);

      if (tools::delta_time(generated_at, last_planning_log) >= 1.0 && !planning_stats.empty()) {
        const auto summary = planning_stats.summary();
        tools::logger()->info(
          "[PlannerTiming] samples: {}, latest: {:.3f} ms, p50: {:.3f} ms, p95: {:.3f} ms, "
          "p99: {:.3f} ms, max: {:.3f} ms",
          summary.sample_count, summary.latest_ms, summary.p50_ms, summary.p95_ms, summary.p99_ms,
          summary.max_ms);
        last_planning_log = generated_at;
      }

      next += period;
      const auto now = std::chrono::steady_clock::now();
      if (next <= now)
        next = now + period;  // drop missed planning slots; never replay a backlog
      else
        std::this_thread::sleep_until(next);
    }
  });

  auto send_thread = std::thread([&]() {
    constexpr auto period = 2ms;
    auto next = std::chrono::steady_clock::now();
    while (!quit) {
      const auto update = plan_queue.front();
      const auto feedback = gimbal.feedback();
      const auto now = std::chrono::steady_clock::now();
      auto plan = update.plan.relative_to(feedback.state.yaw, feedback.state.pitch);
      const bool plan_fresh =
        update.generated_at != epoch && now >= update.generated_at &&
        now - update.generated_at <= runtime_config.plan_timeout && now <= update.plan.valid_at;
      const bool feedback_fresh =
        feedback.fresh_at(now, runtime_config.feedback_timeout);
      const bool observation_known = update.last_observed_timestamp != epoch;
      const auto observation_age = observation_known ? now - update.last_observed_timestamp : now - epoch;
      // 0=ready, 1=no_control, 2=no_plan, 3=plan_expired, 4=feedback_stale,
      // 5=no_observation, 6=observation_stale. The gating below is unchanged.
      const int send_status = update.generated_at == epoch ? 2 : !update.plan.control ? 1 :
          !plan_fresh ? 3 : !feedback_fresh ? 4 : !observation_known ? 5 :
          observation_age > runtime_config.control_timeout ? 6 : 0;
      latest_send_status.store(send_status);
      planner_diagnostics.observe_send(send_status);

      if (!plan_fresh || !feedback_fresh || !observation_known ||
          observation_age > runtime_config.control_timeout) {
        plan = Plan{};  // startup, no target, stale plan/pose, or stale real observation
      } else if (observation_age > runtime_config.fire_timeout) {
        plan.fire = false;
      }

      // The gimbal must interpret a continuous mode=2 stream as a fire permission,
      // not as one trigger event per received frame.
#ifdef DUST_ENABLE_SIMULATOR
      if (talos_backend) {
        const float distance_m = plan.control ? update.distance_m : -1.0F;
        const double absolute_yaw = plan.control ? plan.absolute_yaw : feedback.state.yaw;
        const double absolute_pitch = plan.control ? plan.absolute_pitch : feedback.state.pitch;
        // Talos consumes an absolute solver target; the hardware planner command is relative.
        const float pitch_deg = static_cast<float>(
          -(90.0 + absolute_pitch * 57.29577951308232));
        const int result = talos->send(
          static_cast<float>(absolute_yaw * 57.29577951308232), pitch_deg, distance_m,
          plan.control && plan.fire);
        if (result != 0) tools::logger()->error("[Talos] command write failed: {}", result);
      } else {
#endif
        gimbal.send(
          plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch,
          plan.pitch_vel, plan.pitch_acc);
#ifdef DUST_ENABLE_SIMULATOR
      }
#endif

      next += period;
      if (next <= now)
        next = now + period;  // skip missed send slots; do not burst to catch up
      else
        std::this_thread::sleep_until(next);
    }
  });

  auto telemetry_thread = std::thread([&]() {
    constexpr auto period = 20ms;
    auto next = std::chrono::steady_clock::now();
    const auto t0 = next;
    uint16_t last_bullet_count = 0;
    uint64_t last_latency_sequence = 0;
    auto last_latency_log = t0;
    tools::LatencyStats latency_stats;
    std::optional<tools::LatencySummary> latency_summary;
    tools::Plotter plotter;

    while (!quit) {
      const auto target_update = target_queue.front();
      const auto plan_update = plan_queue.front();
      const auto feedback = gimbal.feedback();
      const auto & gs = feedback.state;
      const auto sampled_at = std::chrono::steady_clock::now();
      if (target_update.sequence != 0 && target_update.sequence != last_latency_sequence) {
        latency_stats.add(1e3 * tools::delta_time(sampled_at, target_update.frame_timestamp));
        last_latency_sequence = target_update.sequence;
        latency_summary = latency_stats.summary();
      }

      nlohmann::json data;
      data["t"] = tools::delta_time(sampled_at, t0);
      data["tracker_state"] = tracker_state_code.load();
      data["has_target"] = target_update.target.has_value() ? 1 : 0;
      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;
      data["target_yaw"] = plan_update.plan.target_yaw;
      data["target_pitch"] = plan_update.plan.target_pitch;
      data["plan_yaw"] = plan_update.plan.yaw;
      data["plan_yaw_vel"] = plan_update.plan.yaw_vel;
      data["plan_yaw_acc"] = plan_update.plan.yaw_acc;
      data["plan_pitch"] = plan_update.plan.pitch;
      data["plan_pitch_vel"] = plan_update.plan.pitch_vel;
      data["plan_pitch_acc"] = plan_update.plan.pitch_acc;
      data["plan_time_ms"] = plan_update.planning_ms;
      data["plan_status"] = to_string(plan_update.plan.status);
      data["plan_status_code"] = static_cast<int>(plan_update.plan.status);
      data["aim_selection_valid"] = plan_update.plan.aim_debug.selection_valid;
      data["aim_ballistic_converged"] = plan_update.plan.aim_debug.ballistic_converged;
      data["aim_display_valid"] = plan_update.plan.aim_visible_at(sampled_at);
      data["aim_plan_sequence"] = plan_update.sequence;
      data["aim_age_ms"] = 1e3 * tools::delta_time(sampled_at, plan_update.plan.planned_at);
      data["mpc_yaw"] = mpc_diagnostics_json(plan_update.plan.yaw_mpc);
      data["mpc_pitch"] = mpc_diagnostics_json(plan_update.plan.pitch_mpc);
      const int send_status = latest_send_status.load();
      data["send_status_code"] = send_status;
      static constexpr const char *send_names[] = {"ready", "no_control", "no_plan",
          "plan_expired", "feedback_stale", "no_observation", "observation_stale"};
      data["send_status"] = send_names[send_status];
      data["planner_diagnostic_dropped"] = planner_diagnostics.dropped();
      data["planner_trace_written"] = planner_diagnostics.written();
      data["planner_trace_enabled"] = planner_diagnostics.trace_enabled();
      data["feedback_sequence"] = feedback.sequence;
      data["feedback_valid"] = feedback.valid;
      data["feedback_rates_measured"] = feedback.rates_measured;
      data["mpc_feedback_age_ms"] = plan_update.plan.feedback_age_ms;
      data["mpc_state_valid"] = plan_update.plan.initial_state.valid;
      data["mpc_initial_yaw"] = plan_update.plan.initial_state.yaw;
      data["mpc_initial_yaw_vel"] = plan_update.plan.initial_state.yaw_vel;
      data["mpc_initial_pitch"] = plan_update.plan.initial_state.pitch;
      data["mpc_initial_pitch_vel"] = plan_update.plan.initial_state.pitch_vel;
      data["plan_absolute_yaw"] = plan_update.plan.absolute_yaw;
      data["plan_absolute_pitch"] = plan_update.plan.absolute_pitch;
      data["mpc_yaw_solved"] = plan_update.plan.yaw_mpc.solved;
      data["mpc_pitch_solved"] = plan_update.plan.pitch_mpc.solved;
      data["mpc_yaw_iterations"] = plan_update.plan.yaw_mpc.iterations;
      data["mpc_pitch_iterations"] = plan_update.plan.pitch_mpc.iterations;
      data["mpc_yaw_primal_residual"] = plan_update.plan.yaw_mpc.primal_residual;
      data["mpc_pitch_primal_residual"] = plan_update.plan.pitch_mpc.primal_residual;
      data["mpc_yaw_dual_residual"] = plan_update.plan.yaw_mpc.dual_residual;
      data["mpc_pitch_dual_residual"] = plan_update.plan.pitch_mpc.dual_residual;
      data["mpc_yaw_input_violation"] = plan_update.plan.yaw_mpc.input_violation;
      data["mpc_pitch_input_violation"] = plan_update.plan.pitch_mpc.input_violation;
      data["capture_time_ms"] = latest_capture_ms.load();
      data["detection_submit_ms"] = latest_submit_ms.load();
      data["inference_time_ms"] = latest_inference_ms.load();
      data["inference_wait_ms"] = latest_inference_wait_ms.load();
      data["postprocess_time_ms"] = latest_postprocess_ms.load();
      data["result_delivery_ms"] = latest_delivery_ms.load();
      const auto detector_drops = detector.drop_counts();
      data["dropped_overwritten_frames"] = detector_drops.overwritten;
      data["dropped_result_capacity_frames"] = detector_drops.result_capacity;
      data["skipped_empty_frames"] = detector_drops.skipped_empty;
      data["skipped_busy_frames"] = detector_drops.skipped_busy;
      data["skipped_error_frames"] = detector_drops.skipped_error;
      data["fire"] = plan_update.plan.fire ? 1 : 0;
      data["fired"] = gs.bullet_count > last_bullet_count ? 1 : 0;
      last_bullet_count = gs.bullet_count;
      if (target_update.target.has_value()) {
        const auto x = target_update.target->ekf_x();
        data["target_z"] = x[4];
        data["target_vz"] = x[5];
        data["w"] = x[7];
        data["angle"] = x[6];
      } else {
        data["w"] = 0.0;
      }
      data["mode"] = plan_update.plan.mode();
      add_latency_metrics(data, latency_summary);
      plotter.plot(data);
#ifdef DUST_ENABLE_FOXGLOVE
      if (foxglove) {
        foxglove->publish_telemetry(std::move(data), sampled_at);
        foxglove->publish_status(
          {{"tracker_state_name", to_string(static_cast<TrackerState>(tracker_state_code.load()))},
           {"has_target", target_update.target.has_value()}},
          sampled_at);
      }
#endif
      if (latency_summary && tools::delta_time(sampled_at, last_latency_log) >= 1.0) {
        const auto & summary = *latency_summary;
        tools::logger()->info(
          "[VisionLatency] samples: {}, latest: {:.2f} ms, p50: {:.2f} ms, p95: {:.2f} ms, "
          "p99: {:.2f} ms, max: {:.2f} ms",
          summary.sample_count, summary.latest_ms, summary.p50_ms, summary.p95_ms, summary.p99_ms,
          summary.max_ms);
        last_latency_log = sampled_at;
      }

      next += period;
      const auto now = std::chrono::steady_clock::now();
      if (next <= now)
        next = now + period;
      else
        std::this_thread::sleep_until(next);
    }
  });

  TrackerState last_state = TrackerState::lost;
  auto last_detector_timing_log = std::chrono::steady_clock::now();
  auto capture_thread = std::thread([&] {
    bool orientation_was_valid = false;
#ifdef DUST_ENABLE_SIMULATOR
    bool talos_intrinsics_applied = false;
#endif
    while (!quit) {
      cv::Mat image;
      std::chrono::steady_clock::time_point timestamp;
      std::optional<Eigen::Quaterniond> talos_orientation;
      const auto capture_started = std::chrono::steady_clock::now();
#ifdef DUST_ENABLE_SIMULATOR
      if (talos_backend) {
        io::TalosFrame frame;
        const int result = talos->read(frame);
        if (result == 0) {
          std::this_thread::sleep_for(1ms);
          continue;
        }
        if (result < 0) {
          tools::logger()->error("[Talos] frame read failed: {}", result);
          quit = true;
          break;
        }
        image = std::move(frame.image_bgr);
        timestamp = frame.timestamp;
        talos_orientation = frame.gimbal_to_world;
        if (!talos_intrinsics_applied) {
          solver.set_camera_intrinsics(
            frame.camera.fx, frame.camera.fy, frame.camera.cx, frame.camera.cy);
          talos_intrinsics_applied = true;
        }
        gimbal.set_external_feedback(frame.gimbal_to_world, timestamp);
      } else {
#endif
        camera->read(image, timestamp);
#ifdef DUST_ENABLE_SIMULATOR
      }
#endif
      const double capture_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - capture_started)
                                  .count();
      if (image.empty()) break;
      const auto orientation = talos_orientation
                                 ? std::optional<tools::QuaternionSample>(tools::QuaternionSample{
                                     *talos_orientation, timestamp, timestamp})
                                 : gimbal.orientation_at(timestamp);
      const bool was_valid = orientation_was_valid;
      orientation_was_valid = orientation.has_value();
      if (!orientation) {
        if (was_valid) tools::logger()->warn("[Gimbal] No synchronized pose; skipping images");
        continue;
      }
      const auto & q = orientation->q;
      const auto rois = tracker.focus_rois(image.size(), timestamp, solver.R_gimbal2world(q));
      detector.submit(image, timestamp, rois.net, rois.light, q, capture_ms);
    }
  });

  while (!quit) {
    // SIGINT 通过 Exiter 的全局标志进来，这里统一转成 quit，让所有线程走同一条退出路径
    if (exiter.exit()) quit = true;

#ifdef DUST_ENABLE_FOXGLOVE
    // 必须放在 wait_pop_for 之前：跳帧时的 continue 会跳过这里。
    // 本地窗口需要源图；仅远程观看时只在真有客户端订阅图像时才让检测器保留源图。
    detector.set_keep_source(!headless || (foxglove && foxglove->image_requested()));
#else
    detector.set_keep_source(!headless);
#endif

    auto detection = detector.wait_pop_for(50ms);
    if (!detection || !detection->q) continue;
    latest_capture_ms.store(detection->capture_ms);
    latest_submit_ms.store(detection->submit_ms);
    latest_inference_ms.store(detection->inference_ms);
    latest_inference_wait_ms.store(detection->inference_wait_ms);
    latest_postprocess_ms.store(detection->postprocess_ms);
    latest_delivery_ms.store(detection->delivery_ms);
    if (tools::delta_time(std::chrono::steady_clock::now(), last_detector_timing_log) >= 1.0) {
      const auto drops = detector.drop_counts();
      tools::logger()->info(
        "[DetectorTiming] capture: {:.2f} ms, submit: {:.2f} ms, inference: {:.2f} ms "
        "(wait: {:.2f} ms, postprocess: {:.2f} ms), result delivery: {:.2f} ms; dropped overwritten/result-capacity/busy/empty/error: {}/{}/{}/{}/{}",
        detection->capture_ms, detection->submit_ms, detection->inference_ms,
        detection->inference_wait_ms, detection->postprocess_ms, detection->delivery_ms, drops.overwritten, drops.result_capacity, drops.skipped_busy,
        drops.skipped_empty, drops.skipped_error);
      last_detector_timing_log = std::chrono::steady_clock::now();
    }
    const auto & q = *detection->q;
    solver.set_R_gimbal2world(q);
    cv::Mat img_det = std::move(detection->source);
    auto armors = std::move(detection->armors);
    const auto t = detection->timestamp;

    auto targets = tracker.track(armors, t);

    const auto current_state = tracker.state_enum();
    if (current_state != last_state) {
      tools::logger()->info("[Tracker] State: {} -> {}", to_string(last_state), to_string(current_state));
      last_state = current_state;
    }
    tracker_state_code.store(static_cast<int>(current_state));

    target_queue.push(
      {targets.empty() ? std::nullopt : std::optional<Target>(targets.front()), t,
       tracker.last_observed_timestamp(), detection->sequence});

    if (!targets.empty() && runtime_config.verbose_ekf) {
      // 在终端上显示 EKF 状态信息；默认关闭，165 fps 下会把日志刷满
      auto ekf_x = targets.front().ekf_x();
      tools::logger()->debug(
        "[EKF] x={:.4f} vx={:.4f} y={:.4f} vy={:.4f} z={:.4f} vz={:.4f} a={:.4f} w={:.4f} r={:.4f} "
        "l={:.4f} h={:.4f}",
        ekf_x[0], ekf_x[1], ekf_x[2], ekf_x[3], ekf_x[4], ekf_x[5], ekf_x[6], ekf_x[7], ekf_x[8],
        ekf_x[9], ekf_x[10]);
    }

#ifdef DUST_ENABLE_FOXGLOVE
    const bool publish_frame =
      foxglove && foxglove->image_requested() && detection->inferred && !img_det.empty();
#else
    const bool publish_frame = false;
#endif

    // 本地窗口与 Foxglove 图像通道共用同一份叠加绘制
    if ((!headless || publish_frame) && detection->inferred && !img_det.empty()) {
      visualization::VisionOverlayInput overlay{
        armors, detection->net_roi, detection->light_roi, to_string(current_state)};
      if (!targets.empty()) {
        overlay.target = &targets.front();
        const auto completed_plan = plan_queue.front();
        overlay.aim_xyza = completed_plan.plan.aim_debug.xyza;
        overlay.aim_valid = completed_plan.plan.aim_visible_at(std::chrono::steady_clock::now());
      }
      visualization::draw_vision_overlay(img_det, overlay, solver);

#ifdef DUST_ENABLE_FOXGLOVE
      // 在缩放之前发布，通道内部再按 image_scale 缩放
      if (publish_frame) foxglove->publish_image(img_det, t);
#endif

      if (!headless) {
        cv::resize(img_det, img_det, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
        cv::imshow(options.window_name, img_det);
        // 与 SIGINT 走同一条退出路径：置位后由循环条件收尾
        if (cv::waitKey(1) == 'q') quit = true;
      }
    }
  }

  if (camera) camera->stop();
  detector.close();
  if (capture_thread.joinable()) capture_thread.join();
  // 排空时必须复用检测帧里携带的姿态，不能重新消费云台队列
  while (auto detection = detector.wait_pop()) {
    auto armors = std::move(detection->armors);
    if (!detection->q) continue;
    solver.set_R_gimbal2world(*detection->q);
    auto targets = tracker.track(armors, detection->timestamp);
    target_queue.push(
      {targets.empty() ? std::nullopt : std::optional<Target>(targets.front()),
       detection->timestamp, tracker.last_observed_timestamp(), detection->sequence});
  }
  detector.join();
  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  if (send_thread.joinable()) send_thread.join();
  if (telemetry_thread.joinable()) telemetry_thread.join();
#ifdef DUST_ENABLE_SIMULATOR
  if (talos_backend)
    talos->send(0.0F, -90.0F, -1.0F, false);
  else
#endif
    gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}

}  // namespace runtime
}  // namespace auto_aim
