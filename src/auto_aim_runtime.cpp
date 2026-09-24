#include "src/auto_aim_runtime.hpp"

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
#include "visualization/vision_overlay.hpp"

#ifdef DUST_ENABLE_FOXGLOVE
#include "tools/yaml.hpp"
#include "visualization/foxglove_vision.hpp"
#endif

using namespace std::chrono_literals;

namespace auto_aim
{
namespace runtime
{
namespace
{

// 规划线程与检测主循环之间只传目标本身，姿态有效性随目标一起走
struct TargetUpdate
{
  std::optional<Target> target;
  std::chrono::steady_clock::time_point frame_timestamp;
  uint64_t sequence = 0;
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

#ifdef DUST_ENABLE_FOXGLOVE
// Foxglove 的运行参数来自 config_path 的 foxglove 段；缺段或缺键都退回下面的默认值，
// 与 visualization::FoxgloveVisionOptions 的默认值一致。启用开关在命令行，不在这里。
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
  tools::Plotter plotter;

  const bool headless = options.headless;

#ifdef DUST_ENABLE_FOXGLOVE
  std::unique_ptr<visualization::FoxgloveVision> foxglove;
  if (options.foxglove_enabled) {
    FoxgloveConfig config;
    try {
      config = load_foxglove_config(options.config_path);
    } catch (const std::exception & e) {
      tools::logger()->error("[Foxglove] {}", e.what());
      return 2;
    }
    if (
      config.image_port < 1 || config.image_port > 65535 || config.data_port < 0 ||
      config.data_port > 65535 || config.image_fps <= 0 || config.image_scale <= 0 ||
      config.image_scale > 1.0 || config.jpeg_quality < 1 || config.jpeg_quality > 100) {
      tools::logger()->error(
        "[Foxglove] Invalid port, fps, scale, or JPEG quality in the foxglove section");
      return 2;
    }
    visualization::FoxgloveVisionOptions foxglove_options;
    foxglove_options.host = config.host;
    foxglove_options.image_port = static_cast<uint16_t>(config.image_port);
    foxglove_options.data_port = static_cast<uint16_t>(config.data_port);
    foxglove_options.image_fps = config.image_fps;
    foxglove_options.image_scale = config.image_scale;
    foxglove_options.jpeg_quality = config.jpeg_quality;
    foxglove_options.yield_cpu = config.yield_cpu;
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
  if (options.foxglove_enabled)
    tools::logger()->error(
      "this binary was built without Foxglove; configure with -DENABLE_FOXGLOVE_VISION=ON");
#endif

  io::Gimbal gimbal(options.config_path, options.simulate_gimbal);
  io::Camera camera(options.config_path);

  multithread::MultiThreadDetector detector(options.config_path, !headless);
  Solver solver(options.config_path);
  Tracker tracker(options.config_path, solver);
  Planner planner(options.config_path);

  tools::ThreadSafeQueue<TargetUpdate, true> target_queue(1);
  target_queue.push({std::nullopt, std::chrono::steady_clock::now(), 0});

  // 计划线程只读这个原子状态码；直接调 Tracker::state() 会和主循环的 track() 抢同一把锁
  std::atomic<int> tracker_state_code{static_cast<int>(TrackerState::lost)};
  std::atomic<bool> orientation_valid{false};
  std::atomic<bool> quit = false;

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;
    uint64_t last_latency_sequence = 0;
    auto last_latency_log = t0;
    tools::LatencyStats latency_stats;
    std::optional<tools::LatencySummary> latency_summary;

    while (!quit) {
      const auto update = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(
        orientation_valid.load() ? update.target : std::nullopt, gs.bullet_speed,
        solver.R_gimbal2world());

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);
      const auto sent_at = std::chrono::steady_clock::now();
      if (update.sequence != 0 && update.sequence != last_latency_sequence) {
        latency_stats.add(1e3 * tools::delta_time(sent_at, update.frame_timestamp));
        last_latency_sequence = update.sequence;
        latency_summary = latency_stats.summary();
      }

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      // Tracker 状态
      data["tracker_state"] = tracker_state_code.load();
      data["has_target"] = update.target.has_value() ? 1 : 0;

      data["gimbal_yaw"] = gs.yaw;  // radians
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;  // 向上为负 (radians)
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      // plan.yaw 已是云台系绝对目标角（Planner 内部 limit_rad(x + yaw0)），不要叠加 gs.yaw
      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (update.target.has_value()) {
        data["target_z"] = update.target->ekf_x()[4];   // z
        data["target_vz"] = update.target->ekf_x()[5];  // vz
      }

      if (update.target.has_value()) {
        data["w"] = update.target->ekf_x()[7];
        data["angle"] = update.target->ekf_x()[6];  // EKF 角度 a
      } else {
        data["w"] = 0.0;
      }

      data["mode"] = plan.mode();  // 0=IDLE, 1=AUTO_AIM, 2=FIRE
      add_latency_metrics(data, latency_summary);
      plotter.plot(data);
#ifdef DUST_ENABLE_FOXGLOVE
      if (foxglove) {
        const auto sampled_at = std::chrono::steady_clock::now();
        // 按值传参 + move：上一条语句的 plotter.plot(data) 已经消费过 data，之后不再使用。
        foxglove->publish_telemetry(std::move(data), sampled_at);
        foxglove->publish_status(
          {{"tracker_state_name", to_string(static_cast<TrackerState>(tracker_state_code.load()))},
           {"has_target", update.target.has_value()}},
          sampled_at);
      }
#endif

      if (latency_summary && tools::delta_time(sent_at, last_latency_log) >= 1.0) {
        const auto & summary = *latency_summary;
        tools::logger()->info(
          "[VisionLatency] samples: {}, latest: {:.2f} ms, p50: {:.2f} ms, p95: {:.2f} ms, "
          "p99: {:.2f} ms, max: {:.2f} ms",
          summary.sample_count, summary.latest_ms, summary.p50_ms, summary.p95_ms, summary.p99_ms,
          summary.max_ms);
        last_latency_log = sent_at;
      }

      std::this_thread::sleep_for(10ms);
    }
  });

  TrackerState last_state = TrackerState::lost;
  auto capture_thread = std::thread([&] {
    while (!quit) {
      cv::Mat image;
      std::chrono::steady_clock::time_point timestamp;
      camera.read(image, timestamp);
      if (image.empty()) break;
      const auto orientation = gimbal.orientation_at(timestamp);
      const bool was_valid = orientation_valid.exchange(orientation.has_value());
      if (!orientation) {
        if (was_valid) tools::logger()->warn("[Gimbal] No synchronized pose; skipping images");
        continue;
      }
      const auto & q = orientation->q;
      const auto rois = tracker.dynamic_roi_enabled()
                          ? tracker.focus_rois(image.size(), timestamp, solver.R_gimbal2world(q))
                          : FocusRois{cv::Rect(0, 0, image.cols, image.rows), std::nullopt};
      detector.submit(image, timestamp, rois.net, rois.light, q);
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
       detection->sequence});

    if (!targets.empty() && options.verbose_ekf) {
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
        overlay.aim_xyza = planner.debug_xyza();
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

  camera.stop();
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
       detection->timestamp, detection->sequence});
  }
  detector.join();
  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}

}  // namespace runtime
}  // namespace auto_aim
