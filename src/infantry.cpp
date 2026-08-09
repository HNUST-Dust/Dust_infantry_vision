#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/latency_stats.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }"
  "{simulate-gimbal | false                  | 使用虚拟云台姿态和串口输出 }";

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

  io::Gimbal gimbal(config_path, cli.get<bool>("simulate-gimbal"));
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<TargetUpdate, true> target_queue(1);
  target_queue.push({std::nullopt, std::chrono::steady_clock::now(), 0});

  // 用于线程间共享 Tracker 状态
  std::atomic<int> tracker_state_code{0};  // 0=lost, 1=detecting, 2=tracking, 3=temp_lost, 4=switching

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
      auto plan = planner.plan(update.target, gs.bullet_speed, solver.R_gimbal2world());

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
      data["gimbal_pitch"] = gs.pitch;      // 向上为负 (radians)
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

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

      // 计算并添加 mode 信息
      int mode_value = plan.control ? (plan.fire ? 2 : 1) : 0;  // 0=IDLE, 1=AUTO_AIM, 2=FIRE
      data["mode"] = mode_value;
      add_latency_metrics(data, latency_summary);
      plotter.plot(data);

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

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto t0 = std::chrono::steady_clock::now();
  std::string last_state = "lost";
  uint64_t target_sequence = 0;

  while (!exiter.exit()) {
    camera.read(img, t);
    // cv::flip(img,img,-1);
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    
    const auto roi_override = tracker.dynamic_roi_enabled()
                                ? std::optional<cv::Rect>(tracker.focus_roi(
                                    img.size(), t, solver.R_gimbal2world(q)))
                                : std::nullopt;
    auto t_yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, -1, roi_override);
    auto t_yolo_end = std::chrono::steady_clock::now();
    auto yolo_inference_time = tools::delta_time(t_yolo_end, t_yolo_start);
    
    auto targets = tracker.track(armors, t);
    
    // 调试信息：Tracker 状态变化
    auto current_state = tracker.state();
    if (current_state != last_state) {
      tools::logger()->trace("[Tracker] State: {} -> {}", last_state, current_state);
      last_state = current_state;
    }
    
    // 更新状态码供 plotter 使用
    if (current_state == "lost") tracker_state_code = 0;
    else if (current_state == "detecting") tracker_state_code = 1;
    else if (current_state == "tracking") tracker_state_code = 2;
    else if (current_state == "temp_lost") tracker_state_code = 3;
    else if (current_state == "switching") tracker_state_code = 4;
    
    target_queue.push(
      {targets.empty() ? std::nullopt : std::optional<auto_aim::Target>(targets.front()), t,
        ++target_sequence});

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza();
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
      cv::line(img, cv::Point(700, 540), cv::Point(740, 540), cv::Scalar(255, 255, 255));
      cv::line(img, cv::Point(720, 520), cv::Point(720, 560), cv::Scalar(255, 255, 255));

      
      // 在终端上显示 EKF 状态信息
      auto ekf_x = target.ekf_x();
      tools::logger()->info(
        "[EKF] x={:.4f} vx={:.4f} y={:.4f} vy={:.4f} z={:.4f} vz={:.4f} a={:.4f} w={:.4f} r={:.4f} l={:.4f} h={:.4f}",
        ekf_x[0], ekf_x[1], ekf_x[2], ekf_x[3], ekf_x[4], ekf_x[5], 
        ekf_x[6], ekf_x[7], ekf_x[8], ekf_x[9], ekf_x[10]);
    }
    
    // // 在图像上显示 Tracker 状态
    // cv::Scalar state_color = (current_state == "tracking") ? cv::Scalar(0, 255, 0) : 
    //                          (current_state == "detecting") ? cv::Scalar(0, 255, 255) :
    //                          (current_state == "temp_lost") ? cv::Scalar(0, 165, 255) :
    //                          cv::Scalar(0, 0, 255);  // lost = red
    // cv::putText(img, fmt::format("State: {}", current_state), 
    //             {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2);

    // cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    // cv::imshow("reprojection", img);
    // auto key = cv::waitKey(1);
    // if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
