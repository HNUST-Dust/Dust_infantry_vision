
#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/core/eigen.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/calibration.yaml | 位置参数，yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

// 世界坐标到像素坐标的转换

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto display = cli.has("display");
  auto yaml = YAML::LoadFile(config_path);
  auto height = yaml["height"].as<double>();
  auto grid_num = yaml["grid_num"].as<int>();
  auto grid_size = yaml["grid_size"].as<double>();
  auto delay = yaml["delay"].as<int>();
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  std::vector<cv::Point3f> points;
  for (int x = 0; x < grid_num; x++) {
    for (int y = 0; y < grid_num; y++) {
      points.emplace_back(x * grid_size, y * grid_size - grid_num * grid_size / 2, -height);
      points.emplace_back(-x * grid_size, y * grid_size - grid_num * grid_size / 2, -height);
    }
  }
  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) break;
    const auto orientation = gimbal.orientation_at(t - 1ms * delay);
    if (!orientation) {
      if (display) {
        tools::draw_text(img, "Pose unavailable", {40, 40}, {0, 0, 255});
        cv::imshow("result", img);
        if (cv::waitKey(1) == 'q') break;
      }
      continue;
    }
    solver.set_R_gimbal2world(orientation->q);
    cv::Mat result = img.clone();
    std::vector<cv::Point2f> projectedPoints = solver.world2pixel(points);
    for (const auto & point : projectedPoints) {
      if (point.x >= 0 && point.x < result.cols && point.y >= 0 && point.y < result.rows) {
        cv::circle(result, point, 3, cv::Scalar(255, 255, 255), -1);
      }
    }
    Eigen::Vector3d euler = solver.R_gimbal2world().eulerAngles(2, 1, 0) * 180.0 / M_PI;
    tools::draw_text(result, fmt::format("yaw   {:.2f}", euler[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(result, fmt::format("pitch {:.2f}", euler[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(result, fmt::format("roll  {:.2f}", euler[2]), {40, 120}, {0, 0, 255});
    if (!display) continue;
    cv::imshow("result", result);
    if (cv::waitKey(1) == 'q') break;
  }
}
