#ifndef VISUALIZATION__FOXGLOVE_VISION_HPP
#define VISUALIZATION__FOXGLOVE_VISION_HPP

#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace visualization
{
struct FoxgloveVisionOptions
{
  std::string host{"127.0.0.1"};
  // image_port 在单端口模式下同时承载遥测；data_port 非 0 且与 image_port 不同才拆成两端口
  uint16_t image_port{8766};
  uint16_t data_port{0};
  double image_fps{10.0};
  double image_scale{0.5};
  int jpeg_quality{80};
  std::size_t image_backlog{2};
  std::size_t data_backlog{4096};
  // 让内部两个发布线程主动降级（图像线程 SCHED_IDLE、数据线程 nice 19），把 CPU 让给 OpenVINO 推理。
  // 设置失败（例如没有 CAP_SYS_NICE）不影响功能，实际生效的策略回显在 /vision/status。
  // 注意 SDK 自己创建的 WebSocket I/O 线程不受此开关影响。
  bool yield_cpu{true};
};

// 把单调的 steady_clock 时间点映射成 Foxglove 使用的 epoch 纳秒。
// 偏移在进程内首次调用时采样一次并保持不变，因此图像与遥测的时间轴始终一致。
uint64_t to_epoch_ns(std::chrono::steady_clock::time_point time);

// 把自瞄主链的图像与遥测推送到 Foxglove WebSocket。
// 内部有两个发布线程：图像线程做缩放与 JPEG 编码，数据线程做遥测/状态的序列化与发送。
// 两个待发队列都有界且丢最旧，慢客户端只会让画面或遥测变稀疏，不会让时延无限增长。
// 不注册任何 service：远程客户端只能观看，不能保存、退出或控制机器人。
class FoxgloveVision
{
public:
  explicit FoxgloveVision(FoxgloveVisionOptions options);
  ~FoxgloveVision();

  FoxgloveVision(const FoxgloveVision &) = delete;
  FoxgloveVision & operator=(const FoxgloveVision &) = delete;

  // 是否已有客户端订阅；无人订阅时上层可跳过绘制，发布接口本身也会直接返回
  bool image_requested() const;
  bool data_requested() const;

  // 发布一帧已标注图像：只做限流、订阅判断和入队，缩放与 JPEG 编码在内部发布线程完成，
  // 因此既不占用检测主循环，也不会被慢客户端阻塞。只能从同一个线程连续调用。
  void publish_image(const cv::Mat & image, std::chrono::steady_clock::time_point capture_time);

  // 发布遥测，字段与 tools::Plotter 发送的 JSON 完全一致，不做限流。可从任意线程调用。
  // 调用线程只做订阅判断、取时间戳和入队，dump() 与 SDK 调用由内部数据线程完成，因此 100 Hz 的
  // 规划线程不再承担序列化开销。队列 FIFO，只在满时丢最旧，丢包数见 /vision/status 的
  // dropped_messages。data 按值接收，调用方可 std::move 传参以避免深拷贝。
  void publish_telemetry(nlohmann::json data, std::chrono::steady_clock::time_point sample_time);

  // 发布低频状态（模块内限流到 1 Hz），fields 会与模块自身的统计在调用线程合并后入队，
  // dump() 与 SDK 调用同样由数据线程完成。1 Hz 节流状态只由调用线程读写，因此本函数仍只能
  // 从同一个线程调用（publish_telemetry 无此限制）；应与 publish_telemetry 从同一线程调用，
  // 以保证两者在队列中的先后顺序稳定。
  void publish_status(
    nlohmann::json fields, std::chrono::steady_clock::time_point sample_time);

  uint16_t image_port() const;
  uint16_t data_port() const;  // 单端口模式返回 image_port

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace visualization

#endif  // VISUALIZATION__FOXGLOVE_VISION_HPP
