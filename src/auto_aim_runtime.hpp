#ifndef AUTO_AIM__RUNTIME_HPP
#define AUTO_AIM__RUNTIME_HPP

#include <string>

namespace auto_aim
{
namespace runtime
{

// 入口（infantry）只负责解析命令行、填充本结构，然后调用 run()：
// 窗口开关、Foxglove 参数与 EKF 日志等运行期差异都由本结构表达。
struct RuntimeOptions
{
  std::string config_path;

  // 虚拟云台：不依赖真机，姿态与串口都走模拟
  bool simulate_gimbal = false;

  // 是否关闭本地窗口与按键事件（--headless）。默认与入口一致：开本地窗口
  bool headless = false;

  // Foxglove 的启用开关由入口解析后写入；监听地址、端口、帧率、缩放、发布线程调度与 JPEG
  // 质量来自 config_path 的 foxglove 段（见 configs/standard3.yaml），不在命令行上。
  bool foxglove_enabled = false;

  // 每帧输出 EKF 11 维状态。默认关闭：165 fps 下它会把 logs/ 写满
  bool verbose_ekf = false;

  // 仅在 !headless 时使用
  std::string window_name = "reprojection";
};

// 运行自瞄主链路，返回进程退出码。
// 内部负责：云台/相机/检测器/求解器/跟踪器/规划器的构造、三个线程的编排与退出顺序。
// 姿态随帧携带、退出排空必须复用同一姿态等约束都在这里实现一次。
int run(const RuntimeOptions & options);

}  // namespace runtime

}  // namespace auto_aim

#endif  // AUTO_AIM__RUNTIME_HPP
