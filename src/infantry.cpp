#include <string>
#include <utility>

#include <opencv2/opencv.hpp>

#include "tools/logger.hpp"
#include "src/auto_aim_runtime.hpp"

// 自瞄唯一入口：只解析配置文件路径。运行模式开关（虚拟云台、本地窗口、EKF 日志、Foxglove）
// 统一放在配置文件的 runtime 段（见 configs/standard3.yaml），命令行不再有对应开关。
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

// 已移入配置文件的命令行开关：{开关, 现在所在的 YAML 段}。
// CommandLineParser 对未声明的键静默忽略（--config-path=foo.yaml 就是这样被吃掉的），所以这里显式
// 拦一次：沿用旧写法时直接报错退出，而不是让人以为参数已经生效。--key 与 --key=value 两种拼法都查。
const std::pair<const char *, const char *> removed_flags[] = {
  {"--foxglove-host", "foxglove"},      {"--foxglove-port", "foxglove"},
  {"--foxglove-data-port", "foxglove"}, {"--foxglove-fps", "foxglove"},
  {"--foxglove-scale", "foxglove"},     {"--foxglove-sched", "foxglove"},
  {"--jpeg-quality", "foxglove"},       {"--simulate-gimbal", "runtime"},
  {"--headless", "runtime"},            {"--verbose-ekf", "runtime"},
  {"--foxglove", "runtime"},
};

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    for (const auto & [flag, section] : removed_flags) {
      if (arg == flag || arg.rfind(std::string(flag) + "=", 0) == 0) {
        tools::logger()->error(
          "[CLI] {} was removed; set it in the YAML {} section instead", flag, section);
        return 2;
      }
    }
  }

  auto_aim::runtime::RuntimeOptions options;
  options.config_path = config_path;

  return auto_aim::runtime::run(options);
}
