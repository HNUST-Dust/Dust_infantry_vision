#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

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

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  // 用于线程间共享 Tracker 状态
  std::atomic<int> tracker_state_code{0};  // 0=lost, 1=detecting, 2=tracking, 3=temp_lost, 4=switching

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed, solver.R_gimbal2world());

      io::VisionToGimbal vtg;
      vtg.mode = plan.control ? (plan.fire ? 2 : 1) : 0;
      vtg.yaw = plan.yaw;
      vtg.yaw_vel = plan.yaw_vel;
      vtg.yaw_acc = plan.yaw_acc;
      vtg.pitch = plan.pitch;
      vtg.pitch_vel = plan.pitch_vel;
      vtg.pitch_acc = plan.pitch_acc;
      gimbal.send(vtg); 

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      
      // Tracker 状态
      data["tracker_state"] = tracker_state_code.load();
      data["has_target"] = target.has_value() ? 1 : 0;

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


      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
        data["angle"] = target->ekf_x()[6];  // EKF 角度 a
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  std::string last_state = "lost";

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    while (!exiter.exit()) {
      camera.read(img, t);
      detector.push(img, t);
    }
  });

  while (!exiter.exit()) {
    auto [img_det, armors, t_det] = detector.debug_pop();

    auto q = gimbal.q(t_det);
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
    
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img_det, image_points, {0, 255, 0});

      }
      Eigen::Vector4d aim_xyza = planner.debug_xyza();
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img_det, image_points, {0, 0, 255});
      
      // 在终端上显示 EKF 状态信息
      auto ekf_x = target.ekf_x();
      tools::logger()->info(
        "[EKF] x={:.4f} vx={:.4f} y={:.4f} vy={:.4f} z={:.4f} vz={:.4f} a={:.4f} w={:.4f} r={:.4f} l={:.4f} h={:.4f}",
        ekf_x[0], ekf_x[1], ekf_x[2], ekf_x[3], ekf_x[4], ekf_x[5], 
        ekf_x[6], ekf_x[7], ekf_x[8], ekf_x[9], ekf_x[10]);
    }
    
    // 在图像上显示 Tracker 状态
    cv::Scalar state_color = (current_state == "tracking") ? cv::Scalar(0, 255, 0) : 
                             (current_state == "detecting") ? cv::Scalar(0, 255, 255) :
                             (current_state == "temp_lost") ? cv::Scalar(0, 165, 255) :
                             cv::Scalar(0, 0, 255);  // lost = red
    cv::putText(img_det, fmt::format("State: {}", current_state), 
                {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2);
    cv::line(img_det, cv::Point(700, 540), cv::Point(740, 540), cv::Scalar(255, 255, 255));
    cv::line(img_det, cv::Point(720, 520), cv::Point(720, 560), cv::Scalar(255, 255, 255));

    cv::resize(img_det, img_det, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img_det);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  if (detect_thread.joinable()) detect_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
