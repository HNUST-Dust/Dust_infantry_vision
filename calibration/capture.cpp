#include <fmt/core.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

#ifdef DUST_ENABLE_FOXGLOVE
#include "calibration/foxglove_calibration.hpp"
#endif

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{@config-path   | configs/calibration.yaml | 位置参数，yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }"
  "{headless       |                          | 不创建OpenCV窗口 }"
  "{foxglove      |                          | 启用Foxglove WebSocket }"
  "{foxglove-host | 127.0.0.1                | Foxglove监听地址 }"
  "{foxglove-port | 8765                     | Foxglove监听端口 }"
  "{foxglove-fps  | 10                       | Foxglove预览帧率 }"
  "{jpeg-quality  | 80                       | JPEG质量(1-100) }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

int existing_sample_count(const std::string & output_folder)
{
  std::set<int> image_indices;
  std::set<int> quaternion_indices;
  for (const auto & entry : std::filesystem::directory_iterator(output_folder)) {
    if (!entry.is_regular_file()) continue;
    auto extension = entry.path().extension().string();
    if (extension != ".jpg" && extension != ".txt") continue;

    auto stem = entry.path().stem().string();
    if (stem.empty() ||
        !std::all_of(stem.begin(), stem.end(), [](unsigned char c) { return std::isdigit(c); }))
      continue;
    auto index = std::stoi(stem);
    if (index <= 0) continue;
    (extension == ".jpg" ? image_indices : quaternion_indices).insert(index);
  }

  if (image_indices != quaternion_indices)
    throw std::runtime_error("Output folder contains unmatched JPG/TXT calibration samples");

  int expected = 1;
  for (auto index : image_indices) {
    if (index != expected)
      throw std::runtime_error("Calibration sample numbering must be continuous from 1");
    expected++;
  }
  return expected - 1;
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder, bool headless,
  bool enable_foxglove, const std::string & foxglove_host, int foxglove_port,
  double foxglove_fps, int jpeg_quality)
{
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = existing_sample_count(output_folder);
  std::string last_event = "ready";

#ifdef DUST_ENABLE_FOXGLOVE
  std::unique_ptr<calibration::FoxgloveCalibration> foxglove;
  if (enable_foxglove) {
    foxglove = std::make_unique<calibration::FoxgloveCalibration>(calibration::FoxgloveOptions{
      foxglove_host, static_cast<uint16_t>(foxglove_port), foxglove_fps, jpeg_quality});
    tools::logger()->info(
      "[Calibration] Foxglove listening at ws://{}:{}", foxglove_host, foxglove_port);
  }
#else
  (void)foxglove_host;
  (void)foxglove_port;
  (void)foxglove_fps;
  (void)jpeg_quality;
  if (enable_foxglove)
    throw std::runtime_error(
      "capture was built without Foxglove; configure with -DENABLE_FOXGLOVE_CALIBRATION=ON");
#endif

  tools::logger()->info("[Calibration] Continuing after {} existing samples", count);
  while (true) {
    camera.read(img, timestamp);
    // cv::flip(img,img,-1);
    // 使用云台回传的四元数，并按时间戳做 slerp 插值，与图像对齐
    Eigen::Quaterniond q = gimbal.q(timestamp);

    // 在图像上显示欧拉角，用来判断imuabs系的xyz正方向，同时判断imu是否存在零漂
    auto img_with_ypr = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, cv::Size(10, 7), centers_2d);  // 默认是对称圆点图案
    cv::drawChessboardCorners(img_with_ypr, cv::Size(10, 7), centers_2d, success);  // 显示识别结果
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

    bool save_requested = false;
    if (!headless) {
      // 按“s”保存图片和对应四元数，按“q”退出程序
      cv::imshow("Press s to save, q to quit", img_with_ypr);
      auto key = cv::waitKey(1);
      if (key == 'q') break;
      save_requested = key == 's';
    }

#ifdef DUST_ENABLE_FOXGLOVE
    if (foxglove) {
      if (foxglove->quit_requested()) break;
      save_requested = foxglove->take_save_request() || save_requested;
    }
#endif

    if (save_requested) {
      if (!success) {
        last_event = "save_rejected_grid_not_detected";
        tools::logger()->warn("[Calibration] Save rejected: circle grid is not detected");
      } else {
        count++;
        auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
        auto q_path = fmt::format("{}/{}.txt", output_folder, count);
        if (!cv::imwrite(img_path, img))
          throw std::runtime_error(fmt::format("Failed to write {}", img_path));
        write_q(q_path, q);
        last_event = fmt::format("saved_{}", count);
        tools::logger()->info("[{}] Saved in {}", count, output_folder);
      }
    }

#ifdef DUST_ENABLE_FOXGLOVE
    if (foxglove) foxglove->publish(img_with_ypr, success, zyx, count, last_event);
#endif
  }

  // 离开该作用域时，camera和gimbal会自动关闭
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  auto output_folder = cli.get<std::string>("output-folder");
  auto headless = cli.has("headless");
  auto enable_foxglove = cli.has("foxglove");
  auto foxglove_host = cli.get<std::string>("foxglove-host");
  auto foxglove_port = cli.get<int>("foxglove-port");
  auto foxglove_fps = cli.get<double>("foxglove-fps");
  auto jpeg_quality = cli.get<int>("jpeg-quality");
  if (!cli.check()) {
    cli.printErrors();
    return 2;
  }
  if (headless && !enable_foxglove) {
    tools::logger()->error("--headless requires --foxglove so the process remains controllable");
    return 2;
  }
  if (foxglove_port < 1 || foxglove_port > 65535 || foxglove_fps <= 0 ||
      jpeg_quality < 1 || jpeg_quality > 100) {
    tools::logger()->error("Invalid Foxglove port, fps, or JPEG quality");
    return 2;
  }

  // 新建输出文件夹
  std::filesystem::create_directories(output_folder);

  tools::logger()->info("默认标定板尺寸为10列7行");
  // 主循环，保存图片和对应四元数
  capture_loop(
    config_path, output_folder, headless, enable_foxglove, foxglove_host, foxglove_port,
    foxglove_fps, jpeg_quality);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
