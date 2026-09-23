#include <fmt/core.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <memory>
#include <limits>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/command_line.hpp"
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
  "{headless       | false                    | 不创建OpenCV窗口 }"
  "{foxglove      | false                    | 启用Foxglove WebSocket }"
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
  if (!q_file) throw std::runtime_error(fmt::format("Failed to write {}", q_path));
}

void write_timing(
  const std::string & path, std::chrono::steady_clock::time_point image_timestamp,
  const tools::QuaternionSample & orientation)
{
  auto ns = [](auto t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
  };
  nlohmann::json data{
    {"clock", "host_steady_clock"},
    {"image_timestamp_ns", ns(image_timestamp)},
    {"orientation_before_ns", ns(orientation.before)},
    {"orientation_after_ns", ns(orientation.after)}};
  std::ofstream file(path);
  file << data.dump(2) << '\n';
  file.close();
  if (!file) throw std::runtime_error(fmt::format("Failed to write {}", path));
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
    if (img.empty()) break;
    // cv::flip(img,img,-1);
    // 使用云台回传的四元数，并按时间戳做 slerp 插值，与图像对齐
    const auto orientation = gimbal.orientation_at(timestamp);

    // 在图像上显示欧拉角，用来判断imuabs系的xyz正方向，同时判断imu是否存在零漂
    auto img_with_ypr = img.clone();
    Eigen::Vector3d zyx = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    if (orientation) {
      zyx = tools::eulers(orientation->q, 2, 1, 0) * 180.0 / M_PI;  // degree
      tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
      tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
      tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});
    } else {
      tools::draw_text(img_with_ypr, "Pose unavailable - cannot save", {40, 40}, {0, 0, 255});
    }

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
      if (!orientation) {
        last_event = "save_rejected_pose_unavailable";
        tools::logger()->warn("[Calibration] Save rejected: no synchronized gimbal pose");
      } else if (!success) {
        last_event = "save_rejected_grid_not_detected";
        tools::logger()->warn("[Calibration] Save rejected: circle grid is not detected");
      } else {
        count++;
        auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
        auto q_path = fmt::format("{}/{}.txt", output_folder, count);
        if (!cv::imwrite(img_path, img))
          throw std::runtime_error(fmt::format("Failed to write {}", img_path));
        write_q(q_path, orientation->q);
        write_timing(fmt::format("{}/{}.json", output_folder, count), timestamp, *orientation);
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
  // OpenCV 只解析 --key=value：写成 --key value 时该开关取到字符串 "true"，而后面那个值还会被
  // 当成位置参数吃掉 config-path，接着按错误路径加载 YAML。因此必须在任何强类型读取之前报错退出。
  // 必须用 get<std::string>：has() 的语义是“值非空”，默认值非空时恒为 true，无法用来自查。
  for (const char * flag :
       {"output-folder", "foxglove-host", "foxglove-port", "foxglove-fps", "jpeg-quality"}) {
    if (tools::cli_value_flag_misused(cli.get<std::string>(flag))) {
      tools::logger()->error(
        "[cli] --{} was parsed as the literal string \"true\"; use --{}=<value>", flag, flag);
      return 2;
    }
  }

  auto output_folder = cli.get<std::string>("output-folder");
  // 注意这两项与 keys 里的默认值强耦合：has() 判断的是“值非空”，默认值一改成 false 就会恒为 true，
  // 所以必须用 get<bool>；反过来若保留空默认值，get<bool> 会报 Missing parameter 让 check() 失败。
  auto headless = cli.get<bool>("headless");
  auto enable_foxglove = cli.get<bool>("foxglove");
  auto foxglove_host = cli.get<std::string>("foxglove-host");
  auto foxglove_port = cli.get<int>("foxglove-port");
  auto foxglove_fps = cli.get<double>("foxglove-fps");
  auto jpeg_quality = cli.get<int>("jpeg-quality");
  if (!cli.check()) {
    cli.printErrors();
    return 2;
  }
  if (headless && !enable_foxglove) {
    tools::logger()->error("--headless requires --foxglove=true so the process remains controllable");
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
  // 包 try/catch：Foxglove 构造失败（例如 host 绑不上）以及采集循环内的校验/写盘异常
  // 都改为记一条明确日志并返回 2，而不是未捕获异常直接 terminate。
  try {
    capture_loop(
      config_path, output_folder, headless, enable_foxglove, foxglove_host, foxglove_port,
      foxglove_fps, jpeg_quality);
  } catch (const std::exception & e) {
    tools::logger()->error("[Calibration] {}", e.what());
    return 2;
  }

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
