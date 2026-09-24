#include <memory>
#include <string>

#include "tools/command_line.hpp"
#include "tools/logger.hpp"
#include "src/auto_aim_runtime.hpp"

#ifdef DUST_ENABLE_FOXGLOVE
#include <opencv2/opencv.hpp>
#endif

// 自瞄唯一入口：默认开本地窗口；--headless=true 关闭检测可视化与窗口事件，适合无桌面部署。
// Foxglove 图像发布默认 30 fps，图像与遥测推送用 --foxglove 打开。
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
  "{jpeg-quality    | 80                     | JPEG质量(1-100) }"
  "{verbose-ekf     | false                  | 每帧输出 EKF 11 维状态（默认关闭，165fps 下会刷满日志） }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // OpenCV 只解析 --key=value；写成 --key value 会静默取到默认值，且后面那个值会被当成
  // 位置参数顶掉 @config-path。这段守卫必须在任何 get<int>/get<double> 之前执行，且不能
  // 放进 #ifdef —— 非 Foxglove 构建下这些开关同样会被写错。
  // 只列带值的开关：布尔开关裸写与 --key=true 取到的原始值都是 "true"，无法据此区分，
  // 列进来反而会把 --headless / --simulate-gimbal 的正常写法判成误用并退出。
  for (const char * flag :
       {"foxglove-host", "foxglove-port", "foxglove-data-port", "foxglove-fps", "foxglove-scale",
        "foxglove-sched", "jpeg-quality"}) {
    if (tools::cli_value_flag_misused(cli.get<std::string>(flag))) {
      tools::logger()->error("[CLI] use --{}=<value>; space-separated values are not parsed", flag);
      return 2;
    }
  }

  const auto sched = cli.get<std::string>("foxglove-sched");
  if (sched != "auto" && sched != "off") {
    tools::logger()->error("[Foxglove] --foxglove-sched must be auto or off");
    return 2;
  }

  auto_aim::runtime::RuntimeOptions options;
  options.config_path = config_path;
  options.simulate_gimbal = cli.get<bool>("simulate-gimbal");
  options.headless = cli.get<bool>("headless");
  options.verbose_ekf = cli.get<bool>("verbose-ekf");
  options.foxglove_host = cli.get<std::string>("foxglove-host");
  options.foxglove_port = cli.get<int>("foxglove-port");
  options.foxglove_data_port = cli.get<int>("foxglove-data-port");
  options.foxglove_fps = cli.get<double>("foxglove-fps");
  options.foxglove_scale = cli.get<double>("foxglove-scale");
  options.foxglove_jpeg_quality = cli.get<int>("jpeg-quality");
  options.foxglove_yield_cpu = sched == "auto";

#ifdef DUST_ENABLE_FOXGLOVE
  options.foxglove_enabled = cli.get<bool>("foxglove");
#else
  if (cli.get<bool>("foxglove"))
    tools::logger()->error(
      "infantry was built without Foxglove; configure with -DENABLE_FOXGLOVE_VISION=ON");
#endif

  return auto_aim::runtime::run(options);
}
