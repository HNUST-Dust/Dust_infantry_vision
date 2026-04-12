#include <fmt/core.h>

#include <chrono>
#include <exception>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
    
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    try {
      camera.read(img, t);
    } catch (const std::exception & e) {
      // 相机读取失败时，创建空图像并继续
      img = cv::Mat::zeros(480, 640, CV_8UC3);
      t = std::chrono::steady_clock::now();
      tools::logger()->debug("Camera read failed, using empty frame");
    }
    
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

  auto command = aimer.aim(targets, t, cboard.bullet_speed, solver.R_gimbal2world());

    cboard.send(command);

    // 发送调试数据
    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    data["mode"] = static_cast<int>(mode);
  data["gimbal_yaw"] = ypr[0];
  data["gimbal_pitch"] = -ypr[1];  // 向上为负
    data["gimbal_roll"] = ypr[2];
    data["bullet_speed"] = cboard.bullet_speed;
    data["armors_detected"] = armors.size();
    data["targets_tracked"] = targets.size();

    // if (!targets.empty()) {
    //   data["target_distance"] = targets.front().distance;
    //   data["target_yaw"] = targets.front().yaw;
    //   data["target_pitch"] = targets.front().pitch;
    // }
    data["command_yaw"] = command.yaw;
    data["command_pitch"] = command.pitch;
    data["command_shoot"] = command.shoot;

    plotter.plot(data);
  }

  return 0;
}