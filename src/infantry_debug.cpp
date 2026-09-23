#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/command_line.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/latency_stats.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

// 叠加层与 Foxglove 无关，本地窗口在所有构建下都用它
#include "visualization/vision_overlay.hpp"

#ifdef DUST_ENABLE_FOXGLOVE
#include "visualization/foxglove_vision.hpp"
#endif

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }"
  "{simulate-gimbal | false                  | 使用虚拟云台姿态和串口输出 }"
  "{headless        | false                  | 关闭检测可视化和窗口事件 }"
  "{foxglove        | false                  | 启用Foxglove WebSocket图像与遥测推送 }"
  "{foxglove-host   | 127.0.0.1              | Foxglove监听地址 }"
  "{foxglove-port   | 8766                   | 图像端口，单端口模式下同时承载遥测 }"
  "{foxglove-data-port | 0                   | 非0时遥测走独立端口，与图像互不抢占带宽 }"
  "{foxglove-fps    | 30                     | 图像发布帧率上限 }"
  "{foxglove-scale  | 0.5                    | 图像发布缩放系数(0,1] }"
  "{foxglove-sched  | auto                   | 发布线程降级让出CPU给推理(auto/off) }"
  "{jpeg-quality    | 80                     | JPEG质量(1-100) }";

namespace
{

struct TargetUpdate
{
  std::optional<auto_aim::Target> target;
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

// 计划线程只读这个原子状态码；直接调 Tracker::state() 会和主循环的 track() 抢同一把锁
const char * tracker_state_name(int code)
{
  switch (code) {
    case 1:
      return "detecting";
    case 2:
      return "tracking";
    case 3:
      return "temp_lost";
    case 4:
      return "switching";
    default:
      return "lost";
  }
}

}  // namespace

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  const bool headless = cli.get<bool>("headless");

  // 可选的 Foxglove 图像与遥测推送；构造失败只记日志，视觉主链路继续运行
#ifdef DUST_ENABLE_FOXGLOVE
  std::unique_ptr<visualization::FoxgloveVision> foxglove;
  if (cli.get<bool>("foxglove")) {
    // OpenCV 只解析 --key=value；写成 --key value 会静默取到默认值（端口会退回 8766，
    // host 会变成字符串 "true" 让 server 根本起不来），这里直接报错。
    for (const char * flag :
         {"foxglove-host", "foxglove-port", "foxglove-data-port", "foxglove-fps", "foxglove-scale",
          "foxglove-sched", "jpeg-quality"}) {
      if (tools::cli_value_flag_misused(cli.get<std::string>(flag))) {
        tools::logger()->error("[Foxglove] use --{}=<value>; space-separated values are not parsed", flag);
        return 2;
      }
    }

    const int image_port = cli.get<int>("foxglove-port");
    const int data_port = cli.get<int>("foxglove-data-port");
    const auto sched = cli.get<std::string>("foxglove-sched");
    if (sched != "auto" && sched != "off") {
      tools::logger()->error("[Foxglove] --foxglove-sched must be auto or off");
      return 2;
    }
    visualization::FoxgloveVisionOptions options;
    options.host = cli.get<std::string>("foxglove-host");
    options.image_port = static_cast<uint16_t>(image_port);
    options.data_port = static_cast<uint16_t>(data_port);
    options.image_fps = cli.get<double>("foxglove-fps");
    options.image_scale = cli.get<double>("foxglove-scale");
    options.jpeg_quality = cli.get<int>("jpeg-quality");
    options.yield_cpu = sched == "auto";
    if (
      image_port < 1 || image_port > 65535 || data_port < 0 || data_port > 65535 ||
      options.image_fps <= 0 || options.image_scale <= 0 || options.image_scale > 1.0 ||
      options.jpeg_quality < 1 || options.jpeg_quality > 100) {
      tools::logger()->error("[Foxglove] Invalid port, fps, scale, or JPEG quality");
      return 2;
    }
    try {
      foxglove = std::make_unique<visualization::FoxgloveVision>(options);
      tools::logger()->info(
        "[Foxglove] image ws://{}:{}, telemetry ws://{}:{}", options.host, foxglove->image_port(),
        options.host, foxglove->data_port());
    } catch (const std::exception & e) {
      tools::logger()->error("[Foxglove] disabled: {}", e.what());
    }
  }
#else
  if (cli.get<bool>("foxglove"))
    tools::logger()->error(
      "infantry_debug was built without Foxglove; configure with -DENABLE_FOXGLOVE_VISION=ON");
#endif

  io::Gimbal gimbal(config_path, cli.get<bool>("simulate-gimbal"));
  io::Camera camera(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path, !headless);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<TargetUpdate, true> target_queue(1);
  target_queue.push({std::nullopt, std::chrono::steady_clock::now(), 0});

  // 用于线程间共享 Tracker 状态
  std::atomic<int> tracker_state_code{0};  // 0=lost, 1=detecting, 2=tracking, 3=temp_lost, 4=switching

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
        orientation_valid.load() ? update.target : std::nullopt,
        gs.bullet_speed, solver.R_gimbal2world());

      io::VisionToGimbal vtg;
      vtg.mode = plan.control ? (plan.fire ? 2 : 1) : 0;
      vtg.yaw = plan.yaw;
      vtg.yaw_vel = plan.yaw_vel;
      vtg.yaw_acc = plan.yaw_acc;
      vtg.pitch = plan.pitch;
      vtg.pitch_vel = plan.pitch_vel;
      vtg.pitch_acc = plan.pitch_acc;
      gimbal.send(vtg);
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
      data["gimbal_pitch"] = gs.pitch;      // 向上为负 (radians)
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw + gs.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["mode"] = vtg.mode;


      if (update.target.has_value()) {
        data["target_z"] = update.target->ekf_x()[4];   //z
        data["target_vz"] = update.target->ekf_x()[5];  //vz
      }

      if (update.target.has_value()) {
        data["w"] = update.target->ekf_x()[7];
        data["angle"] = update.target->ekf_x()[6];  // EKF 角度 a
      } else {
        data["w"] = 0.0;
      }

      add_latency_metrics(data, latency_summary);
      plotter.plot(data);
#ifdef DUST_ENABLE_FOXGLOVE
      if (foxglove) {
        const auto sampled_at = std::chrono::steady_clock::now();
        // 按值传参 + move：上一条语句的 plotter.plot(data) 已经消费过 data，之后不再使用。
        foxglove->publish_telemetry(std::move(data), sampled_at);
        foxglove->publish_status(
          {{"tracker_state_name", tracker_state_name(tracker_state_code.load())},
           {"has_target", update.target.has_value()}},
          sampled_at);
      }
#endif

      if (
        latency_summary &&
        tools::delta_time(sent_at, last_latency_log) >= 1.0) {
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

  std::string last_state = "lost";
  auto detect_thread = std::thread([&]() {
    while (!exiter.exit()) {
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
                          : auto_aim::FocusRois{cv::Rect(0, 0, image.cols, image.rows), std::nullopt};
      detector.submit(image, timestamp, rois.net, rois.light, q);
    }
  });

  while (!exiter.exit()) {
#ifdef DUST_ENABLE_FOXGLOVE
    // 必须放在 wait_pop_for 之前：跳帧时的 continue 会跳过这里。
    // 本地窗口需要源图；仅远程观看时只在真有 Foxglove 客户端订阅图像时才让检测器保留源图。
    detector.set_keep_source(!headless || (foxglove && foxglove->image_requested()));
#endif
    auto detection = detector.wait_pop_for(50ms);
    if (!detection || !detection->q) continue;
    cv::Mat img_det = std::move(detection->source);
    auto armors = std::move(detection->armors);
    const auto t_det = detection->timestamp;

    const auto & q = *detection->q;
    solver.set_R_gimbal2world(q);
    auto targets = tracker.track(armors, t_det);
    
    // 调试信息：Tracker 状态变化
    auto current_state = tracker.state();
    if (current_state != last_state) {
      tools::logger()->info("[Tracker] State: {} -> {}", last_state, current_state);
      last_state = current_state;
    }
    
    // 更新状态码供 plotter 使用
    if (current_state == "lost") tracker_state_code = 0;
    else if (current_state == "detecting") tracker_state_code = 1;
    else if (current_state == "tracking") tracker_state_code = 2;
    else if (current_state == "temp_lost") tracker_state_code = 3;
    else if (current_state == "switching") tracker_state_code = 4;
    
    target_queue.push(
      {targets.empty() ? std::nullopt : std::optional<auto_aim::Target>(targets.front()), t_det,
        detection->sequence});

    if (!targets.empty()) {
      // 在终端上显示 EKF 状态信息
      auto ekf_x = targets.front().ekf_x();
      tools::logger()->info(
        "[EKF] x={:.4f} vx={:.4f} y={:.4f} vy={:.4f} z={:.4f} vz={:.4f} a={:.4f} w={:.4f} r={:.4f} l={:.4f} h={:.4f}",
        ekf_x[0], ekf_x[1], ekf_x[2], ekf_x[3], ekf_x[4], ekf_x[5], ekf_x[6], ekf_x[7],
        ekf_x[8], ekf_x[9], ekf_x[10]);
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
        armors, detection->net_roi, detection->light_roi, current_state};
      if (!targets.empty()) {
        overlay.target = &targets.front();
        overlay.aim_xyza = planner.debug_xyza();
      }
      visualization::draw_vision_overlay(img_det, overlay, solver);

#ifdef DUST_ENABLE_FOXGLOVE
      // 在缩放之前发布，通道内部再按 image_scale 缩放
      if (publish_frame) foxglove->publish_image(img_det, t_det);
#endif

      if (!headless) {
        cv::resize(img_det, img_det, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
        cv::imshow("reprojection", img_det);
        if (cv::waitKey(1) == 'q') break;
      }
    }
  }

  camera.stop();
  detector.close();
  if (detect_thread.joinable()) detect_thread.join();
  while (auto detection = detector.wait_pop()) {
    auto armors = std::move(detection->armors);
    if (!detection->q) continue;
    solver.set_R_gimbal2world(*detection->q);
    auto targets = tracker.track(armors, detection->timestamp);
    target_queue.push(
      {targets.empty() ? std::nullopt : std::optional<auto_aim::Target>(targets.front()),
        detection->timestamp, detection->sequence});
  }
  detector.join();
  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
