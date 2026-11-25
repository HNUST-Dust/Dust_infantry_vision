#include "src/auto_aim_runtime.hpp"
#include "src/app_config.hpp"
#include "src/diagnostics/telemetry.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
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
#include "tasks/auto_aim/detection/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/tracking/tracker.hpp"
#include "tools/runtime/exiter.hpp"
#include "tools/diagnostics/latency_stats.hpp"
#include "tools/diagnostics/logger.hpp"
#include "tools/math/math_tools.hpp"
#include "tools/diagnostics/plotter.hpp"
#include "tools/concurrency/thread_safe_queue.hpp"

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
  uint64_t plan_id = 0;
};

}  // namespace

int run(const RuntimeOptions & options)
{
  tools::Exiter exiter;

  ApplicationConfig config;
  try {
    config = load_application_config(options.config_path);
  } catch (const std::exception & e) {
    tools::logger()->error("[Runtime] {}", e.what());
    return 2;
  }

  const auto & runtime_config = config.runtime;
  const bool headless = config.visualization.headless;
  config.visualization.window_name = options.window_name;
  visualization::RuntimeVisualizer visualizer(config.visualization);
#ifdef DUST_ENABLE_SIMULATOR
  const bool talos_backend = runtime_config.talos_backend;
#endif
  io::Gimbal gimbal(config.gimbal);
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
      camera = std::make_unique<io::Camera>(*config.camera);
  } catch (const std::exception & e) {
    tools::logger()->error("[Runtime] {}", e.what());
    return 2;
  }

  multithread::MultiThreadDetector detector(config.detector, !headless);
  Solver solver(config.solver);
  Tracker tracker(config.tracker, solver);
  Planner planner(config.planner);

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
  tools::ThreadSafeQueue<DetectionTiming, true> detection_timing(1);
  detection_timing.push(DetectionTiming{});
  std::atomic<int> latest_send_status{1};
  PlannerDiagnostics planner_diagnostics(config.diagnostics);
  const auto telemetry_origin = std::chrono::steady_clock::now();
  tools::ThreadSafeQueue<SendTelemetrySnapshot, true> send_queue(1);
  send_queue.push(SendTelemetrySnapshot{});

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
      if (planner_diagnostics.trace_enabled())
        trace.emplace();
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
        state.valid =
          feedback.fresh_at(control_at, runtime_config.feedback_timeout) && rate_frame_supported;
        state.rates_measured = feedback.rates_measured;
        plan = planner.plan_at(update.target, control_at, feedback.state.bullet_speed, state,
                               trace ? &*trace : nullptr);
      } catch (const std::exception & e) {
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
        plan_queue.push({plan, generated_at, update.frame_timestamp, update.last_observed_timestamp,
                         planning_ms, update.sequence, distance_m,
                         // observe() below assigns this same planning-cycle ID to the trace.
                         planner_diagnostics.counters().plans + 1});
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
    SendTelemetryAccumulator send_accumulator(telemetry_origin);
    while (!quit) {
      const auto update = plan_queue.front();
      const auto feedback = gimbal.feedback();
      const auto now = std::chrono::steady_clock::now();
      auto plan = update.plan.relative_to(feedback.state.yaw, feedback.state.pitch);
      const bool plan_fresh = update.generated_at != epoch && now >= update.generated_at &&
                              now - update.generated_at <= runtime_config.plan_timeout &&
                              now <= update.plan.valid_at;
      const bool feedback_fresh = feedback.fresh_at(now, runtime_config.feedback_timeout);
      const bool observation_known =
        update.last_observed_timestamp != epoch && update.last_observed_timestamp <= now;
      const auto observation_age =
        observation_known ? now - update.last_observed_timestamp : now - epoch;
      // 0=ready, 1=no_control, 2=no_plan, 3=plan_expired, 4=feedback_stale,
      // 5=no_observation, 6=observation_stale. The gating below is unchanged.
      const int send_status = update.generated_at == epoch                       ? 2
                              : !update.plan.control                             ? 1
                              : !plan_fresh                                      ? 3
                              : !feedback_fresh                                  ? 4
                              : !observation_known                               ? 5
                              : observation_age > runtime_config.control_timeout ? 6
                                                                                 : 0;
      latest_send_status.store(send_status);
      planner_diagnostics.observe_send(send_status);

      SendTelemetrySnapshot send_sample;
      send_sample.plan_id = update.plan_id;
      send_sample.target_valid = update.plan.control && plan.control;
      send_sample.target_yaw = plan.target_yaw;
      send_sample.target_pitch = plan.target_pitch;
      send_sample.fire_tolerance = update.plan.fire_tolerance;
      send_sample.fire_window = plan.fire_window;
      send_sample.fire_window_check = plan.fire_window_check;
      auto & blocked = send_sample.fire_block_mask;
      if (!plan.control)
        blocked |= send_no_control;
      if (update.plan.control && !update.plan.fire)
        blocked |= send_planner_no_fire;
      if (send_sample.target_valid && !plan.fire_window_check.passed)
        blocked |= send_angle_outside;
      if (update.generated_at == epoch)
        blocked |= send_no_plan;
      else if (!plan_fresh)
        blocked |= send_plan_expired;
      if (!feedback_fresh)
        blocked |= send_feedback_stale;
      if (!observation_known)
        blocked |= send_no_observation;
      else {
        if (observation_age > runtime_config.control_timeout)
          blocked |= send_observation_control_stale;
        if (observation_age > runtime_config.fire_timeout)
          blocked |= send_observation_fire_stale;
      }

      if (!plan_fresh || !feedback_fresh || !observation_known ||
          observation_age > runtime_config.control_timeout) {
        plan = Plan{};  // startup, no target, stale plan/pose, or stale real observation
      } else if (observation_age > runtime_config.fire_timeout) {
        plan.fire = false;
      }

      // mode=2 carries host permission. MCU trigger timing depends on the firmware.
      const auto sent_at = std::chrono::steady_clock::now();
#ifdef DUST_ENABLE_SIMULATOR
      if (talos_backend) {
        const float distance_m = plan.control ? update.distance_m : -1.0F;
        const double absolute_yaw = plan.control ? plan.absolute_yaw : feedback.state.yaw;
        const double absolute_pitch = plan.control ? plan.absolute_pitch : feedback.state.pitch;
        // Talos consumes an absolute solver target; the hardware planner command is relative.
        const float pitch_deg = static_cast<float>(-(90.0 + absolute_pitch * 57.29577951308232));
        const int result = talos->send(static_cast<float>(absolute_yaw * 57.29577951308232),
                                       pitch_deg, distance_m, plan.control && plan.fire);
        if (result != 0)
          tools::logger()->error("[Talos] command write failed: {}", result);
      } else {
#endif
        gimbal.send(plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch,
                    plan.pitch_vel, plan.pitch_acc);
#ifdef DUST_ENABLE_SIMULATOR
      }
#endif

      send_sample.mode = plan.mode();
      send_queue.push(send_accumulator.observe(sent_at, send_sample));

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
    const auto t0 = telemetry_origin;
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
      const auto timing = detection_timing.front();
      const auto sampled_at = std::chrono::steady_clock::now();
      if (target_update.sequence != 0 && target_update.sequence != last_latency_sequence) {
        latency_stats.add(1e3 * tools::delta_time(sampled_at, target_update.frame_timestamp));
        last_latency_sequence = target_update.sequence;
        latency_summary = latency_stats.summary();
      }

      const bool fired = gs.bullet_count < last_bullet_count;
      last_bullet_count = gs.bullet_count;
      std::optional<TelemetryTarget> target;
      if (target_update.target) {
        const auto x = target_update.target->ekf_x();
        target = TelemetryTarget{x[4], x[5], x[7], x[6]};
      }
      const auto detector_drops = detector.drop_counts();
      const TelemetrySnapshot snapshot{sampled_at,
                                       t0,
                                       tracker_state_code.load(),
                                       target,
                                       plan_update.plan,
                                       plan_update.planning_ms,
                                       plan_update.sequence,
                                       latest_send_status.load(),
                                       feedback,
                                       fired,
                                       {planner_diagnostics.dropped(),
                                        planner_diagnostics.written(),
                                        planner_diagnostics.trace_enabled()},
                                       timing,
                                       detector_drops,
                                       latency_summary,
                                       send_queue.front()};
      auto data = telemetry_json(snapshot);
      plotter.plot(data);
      visualizer.publish_telemetry(std::move(data),
                                   to_string(static_cast<TrackerState>(tracker_state_code.load())),
                                   target.has_value(), sampled_at);
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
          solver.set_camera_intrinsics(frame.camera.fx, frame.camera.fy, frame.camera.cx,
                                       frame.camera.cy);
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
      if (image.empty())
        break;
      const auto orientation =
        talos_orientation ? std::optional<tools::QuaternionSample>(
                              tools::QuaternionSample{*talos_orientation, timestamp, timestamp})
                          : gimbal.orientation_at(timestamp);
      const bool was_valid = orientation_was_valid;
      orientation_was_valid = orientation.has_value();
      if (!orientation) {
        if (was_valid)
          tools::logger()->warn("[Gimbal] No synchronized pose; skipping images");
        continue;
      }
      const auto & q = orientation->q;
      const auto rois = tracker.focus_rois(image.size(), timestamp, solver.R_gimbal2world(q));
      detector.submit(image, timestamp, rois.net, rois.light, q, capture_ms);
    }
  });

  while (!quit) {
    // SIGINT 通过 Exiter 的全局标志进来，这里统一转成 quit，让所有线程走同一条退出路径
    if (exiter.exit())
      quit = true;

    // Refresh subscriptions before waiting, including on cycles that skip a frame.
    detector.set_keep_source(visualizer.needs_source());

    auto detection = detector.wait_pop_for(50ms);
    if (!detection || !detection->q)
      continue;
    detection_timing.push({detection->sequence, detection->timestamp, detection->capture_ms,
                           detection->submit_ms, detection->inference_ms,
                           detection->inference_wait_ms, detection->postprocess_ms,
                           detection->delivery_ms});
    if (tools::delta_time(std::chrono::steady_clock::now(), last_detector_timing_log) >= 1.0) {
      const auto drops = detector.drop_counts();
      tools::logger()->info(
        "[DetectorTiming] capture: {:.2f} ms, submit: {:.2f} ms, inference: {:.2f} ms "
        "(wait: {:.2f} ms, postprocess: {:.2f} ms), result delivery: {:.2f} ms; dropped "
        "overwritten/result-capacity/busy/empty/error: {}/{}/{}/{}/{}",
        detection->capture_ms, detection->submit_ms, detection->inference_ms,
        detection->inference_wait_ms, detection->postprocess_ms, detection->delivery_ms,
        drops.overwritten, drops.result_capacity, drops.skipped_busy, drops.skipped_empty,
        drops.skipped_error);
      last_detector_timing_log = std::chrono::steady_clock::now();
    }
    const auto & q = *detection->q;
    solver.set_R_gimbal2world(q);
    cv::Mat img_det = std::move(detection->source);
    auto armors = std::move(detection->armors);
    const auto t = detection->timestamp;

    tracker.set_enemy_color(gimbal.state().enemy_color);
    auto targets = tracker.track(armors, t);

    const auto current_state = tracker.state_enum();
    if (current_state != last_state) {
      tools::logger()->info("[Tracker] State: {} -> {}", to_string(last_state),
                            to_string(current_state));
      last_state = current_state;
    }
    tracker_state_code.store(static_cast<int>(current_state));

    target_queue.push({targets.empty() ? std::nullopt : std::optional<Target>(targets.front()), t,
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

    const auto output = visualizer.frame_output(detection->inferred, img_det);
    if (output) {
      visualization::VisionOverlayInput overlay{armors, detection->net_roi, detection->light_roi,
                                                to_string(current_state)};
      if (!targets.empty()) {
        overlay.target = &targets.front();
        const auto completed_plan = plan_queue.front();
        overlay.aim_xyza = completed_plan.plan.aim_debug.xyza;
        overlay.aim_valid = completed_plan.plan.aim_visible_at(std::chrono::steady_clock::now());
      }
      if (visualizer.draw_frame(img_det, overlay, solver, output, t))
        quit = true;
    }
  }

  if (camera)
    camera->stop();
  detector.close();
  if (capture_thread.joinable())
    capture_thread.join();
  // 排空时必须复用检测帧里携带的姿态，不能重新消费云台队列
  while (auto detection = detector.wait_pop()) {
    auto armors = std::move(detection->armors);
    if (!detection->q)
      continue;
    solver.set_R_gimbal2world(*detection->q);
    tracker.set_enemy_color(gimbal.state().enemy_color);
    auto targets = tracker.track(armors, detection->timestamp);
    target_queue.push({targets.empty() ? std::nullopt : std::optional<Target>(targets.front()),
                       detection->timestamp, tracker.last_observed_timestamp(),
                       detection->sequence});
  }
  detector.join();
  quit = true;
  if (plan_thread.joinable())
    plan_thread.join();
  if (send_thread.joinable())
    send_thread.join();
  if (telemetry_thread.joinable())
    telemetry_thread.join();
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
