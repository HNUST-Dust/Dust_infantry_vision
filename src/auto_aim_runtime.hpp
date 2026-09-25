#ifndef AUTO_AIM__RUNTIME_HPP
#define AUTO_AIM__RUNTIME_HPP

#include <string>

namespace auto_aim
{
namespace runtime
{

// 入口（infantry）只负责解析配置文件路径、填充本结构，然后调用 run()。
// 运行模式开关（虚拟云台、本地窗口、EKF 日志、Foxglove）不在命令行上：run() 从 config_path 的
// runtime 段读取（见 configs/standard3.yaml），缺段或缺键时用内置默认值。
struct RuntimeOptions
{
  std::string config_path;

  // 仅在未关闭本地窗口（runtime.headless 为 false）时使用
  std::string window_name = "reprojection";
};

// 运行自瞄主链路，返回进程退出码。
// 内部负责：云台/相机/检测器/求解器/跟踪器/规划器的构造、三个线程的编排与退出顺序。
// 姿态随帧携带、退出排空必须复用同一姿态等约束都在这里实现一次。
int run(const RuntimeOptions & options);

}  // namespace runtime

}  // namespace auto_aim

#endif  // AUTO_AIM__RUNTIME_HPP
