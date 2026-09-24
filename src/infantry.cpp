#include <memory>
#include <string>

#include "tools/logger.hpp"
#include "src/auto_aim_runtime.hpp"

#ifdef DUST_ENABLE_FOXGLOVE
#include <opencv2/opencv.hpp>
#endif

// 自瞄唯一入口：默认开本地窗口；--headless=true 关闭检测可视化与窗口事件，适合无桌面部署。
// Foxglove 图像与遥测推送用 --foxglove 打开；监听地址、端口、帧率、缩放、发布线程调度与
// JPEG 质量不在命令行上，来自配置文件（configs/standard3.yaml）的 foxglove 段。
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }"
  "{simulate-gimbal | false                  | 使用虚拟云台姿态和串口输出 }"
  "{headless        | false                  | 关闭检测可视化和窗口事件 }"
  "{foxglove        | false                  | 启用Foxglove图像与遥测推送（端口/帧率等见 YAML 的 foxglove 段） }"
  "{verbose-ekf     | false                  | 每帧输出 EKF 11 维状态（默认关闭，165fps 下会刷满日志） }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // Foxglove 的监听与编码参数已移入配置文件的 foxglove 段。CommandLineParser 对未声明的键
  // 静默忽略（--config-path=foo.yaml 就是这样被吃掉的），所以这里显式拦一次：沿用旧写法时
  // 直接报错退出，而不是让人以为参数已经生效。
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    for (const char * flag :
         {"--foxglove-host", "--foxglove-port", "--foxglove-data-port", "--foxglove-fps",
          "--foxglove-scale", "--foxglove-sched", "--jpeg-quality"}) {
      if (arg == flag || arg.rfind(std::string(flag) + "=", 0) == 0) {
        tools::logger()->error(
          "[CLI] {} was removed; set it in the YAML foxglove section instead", flag);
        return 2;
      }
    }
  }

  auto_aim::runtime::RuntimeOptions options;
  options.config_path = config_path;
  options.simulate_gimbal = cli.get<bool>("simulate-gimbal");
  options.headless = cli.get<bool>("headless");
  options.verbose_ekf = cli.get<bool>("verbose-ekf");

#ifdef DUST_ENABLE_FOXGLOVE
  options.foxglove_enabled = cli.get<bool>("foxglove");
#else
  if (cli.get<bool>("foxglove"))
    tools::logger()->error(
      "infantry was built without Foxglove; configure with -DENABLE_FOXGLOVE_VISION=ON");
#endif

  return auto_aim::runtime::run(options);
}
