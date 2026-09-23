#include "foxglove_vision.hpp"

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

// 发布线程的调度降级用；本模块本来就只构建在 Linux 上（相机、串口驱动同理）。
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>

#ifndef SCHED_IDLE
#define SCHED_IDLE 5  // 少数 libc 只在打开 _GNU_SOURCE 时暴露；g++ 默认已定义
#endif

namespace visualization
{
namespace
{
constexpr std::string_view kImageTopic = "/vision/image/compressed";
constexpr std::string_view kTelemetryTopic = "/vision/telemetry";
constexpr std::string_view kStatusTopic = "/vision/status";

std::vector<std::byte> bytes(std::string_view value)
{
  auto begin = reinterpret_cast<const std::byte *>(value.data());
  return {begin, begin + value.size()};
}

// 数据线程待发队列的深度：约 2.5 s @100 Hz，够吸收突发，又不让时延无限增长。
constexpr std::size_t kDataQueueDepth = 256;
// 耗时指标的指数滑动平均系数。
constexpr double kEmaAlpha = 0.1;

const char * sched_policy_name(int policy)
{
  switch (policy) {
    case SCHED_OTHER:
      return "SCHED_OTHER";
    case SCHED_FIFO:
      return "SCHED_FIFO";
    case SCHED_RR:
      return "SCHED_RR";
    case SCHED_BATCH:
      return "SCHED_BATCH";
    case SCHED_IDLE:
      return "SCHED_IDLE";
    default:
      return "unknown";
  }
}

// 只映射本模块可能遇到的 errno：strerror() 返回静态缓冲区，多线程下不适合直接使用。
const char * sched_errno_name(int error)
{
  switch (error) {
    case 0:
      return "";
    case EPERM:
      return "EPERM";
    case EACCES:
      return "EACCES";
    case EINVAL:
      return "EINVAL";
    case ESRCH:
      return "ESRCH";
    default:
      return "error";
  }
}

// 遥测的 jsonschema：列出 data 中可能出现的全部数值字段，让 Foxglove 的 Plot 面板能直接
// 按字段画曲线。新增遥测字段时必须同步这里；additionalProperties 保持 true，
// 这样多出来的字段不会导致解析失败。
foxglove::Schema telemetry_schema()
{
  static const auto schema = bytes(R"({
    "type": "object",
    "properties": {
      "t": {"type": "number"},
      "tracker_state": {"type": "number"},
      "has_target": {"type": "number"},
      "mode": {"type": "number"},
      "fire": {"type": "number"},
      "fired": {"type": "number"},
      "gimbal_yaw": {"type": "number"},
      "gimbal_yaw_vel": {"type": "number"},
      "gimbal_pitch": {"type": "number"},
      "gimbal_pitch_vel": {"type": "number"},
      "target_yaw": {"type": "number"},
      "target_pitch": {"type": "number"},
      "plan_yaw": {"type": "number"},
      "plan_yaw_vel": {"type": "number"},
      "plan_yaw_acc": {"type": "number"},
      "plan_pitch": {"type": "number"},
      "plan_pitch_vel": {"type": "number"},
      "plan_pitch_acc": {"type": "number"},
      "target_z": {"type": "number"},
      "target_vz": {"type": "number"},
      "w": {"type": "number"},
      "angle": {"type": "number"},
      "vision_latency_ms": {"type": "number"},
      "vision_latency_p50_ms": {"type": "number"},
      "vision_latency_p95_ms": {"type": "number"},
      "vision_latency_p99_ms": {"type": "number"}
    },
    "additionalProperties": true
  })");
  return {"VisionTelemetry", "jsonschema", schema.data(), schema.size()};
}

std::chrono::nanoseconds clock_offset()
{
  static const auto offset = std::chrono::system_clock::now().time_since_epoch() -
                             std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(offset);
}
}  // namespace

uint64_t to_epoch_ns(std::chrono::steady_clock::time_point time)
{
  const auto epoch = time.time_since_epoch() + clock_offset();
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count());
}

class FoxgloveVision::Impl
{
public:
  explicit Impl(FoxgloveVisionOptions options) : options_(std::move(options))
  {
    split_ = options_.data_port != 0 && options_.data_port != options_.image_port;

    foxglove::setLogLevel(foxglove::LogLevel::Info);

    image_context_ = foxglove::Context::create();
    image_server_ = create_server(
      "dust-vision-image", image_context_, options_.image_port,
      split_ ? options_.image_backlog : options_.data_backlog);

    // 单端口模式：遥测与图像共用同一个 context 与服务器。
    // 双端口模式：遥测另起一个 context 与服务器，两条连接各自独立积压，
    // 图像塞满积压也不会挤占遥测的带宽和时延。
    data_context_ = image_context_;
    if (split_) {
      data_context_ = foxglove::Context::create();
      data_server_ = create_server(
        "dust-vision-data", data_context_, options_.data_port, options_.data_backlog);
    }

    auto image_result =
      foxglove::messages::CompressedImageChannel::create(kImageTopic, image_context_);
    if (!image_result.has_value())
      throw std::runtime_error(
        std::string("Failed to create Foxglove image channel: ") +
        foxglove::strerror(image_result.error()));
    image_channel_ = std::make_unique<foxglove::messages::CompressedImageChannel>(
      std::move(image_result.value()));

    auto telemetry_result =
      foxglove::RawChannel::create(kTelemetryTopic, "json", telemetry_schema(), data_context_);
    if (!telemetry_result.has_value())
      throw std::runtime_error(
        std::string("Failed to create Foxglove telemetry channel: ") +
        foxglove::strerror(telemetry_result.error()));
    telemetry_channel_ =
      std::make_unique<foxglove::RawChannel>(std::move(telemetry_result.value()));

    auto status_result = foxglove::RawChannel::create(kStatusTopic, "json", std::nullopt, data_context_);
    if (!status_result.has_value())
      throw std::runtime_error(
        std::string("Failed to create Foxglove status channel: ") +
        foxglove::strerror(status_result.error()));
    status_channel_ = std::make_unique<foxglove::RawChannel>(std::move(status_result.value()));

    // 两个发布线程都放在最后创建：上面任何一步抛异常时都还没有线程存在，析构可以直接返回。
    worker_ = std::thread(&Impl::publish_loop, this);
    data_worker_ = std::thread(&Impl::publish_data_loop, this);
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      stop_ = true;
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      data_stop_ = true;
    }
    frame_ready_.notify_all();
    data_ready_.notify_all();

    // 先等两个发布线程退出，再停 server，保证没有线程在使用 channel/server。
    if (worker_.joinable()) worker_.join();
    if (data_worker_.joinable()) data_worker_.join();

    if (data_server_) (void)data_server_->stop();
    if (image_server_) (void)image_server_->stop();
  }

  bool image_requested() const { return image_channel_->hasSinks(); }

  bool data_requested() const { return telemetry_channel_->hasSinks(); }

  void publish_image(const cv::Mat & image, std::chrono::steady_clock::time_point capture_time)
  {
    if (image.empty()) return;

    const auto now = std::chrono::steady_clock::now();
    if (now < next_image_publish_) return;    // 过密的帧直接丢弃，保持画面时延
    if (!image_channel_->hasSinks()) return;  // 无人订阅：不缩放、不编码、不入队
    next_image_publish_ =
      now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(1.0 / options_.image_fps));

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (pending_frame_) dropped_images_.fetch_add(1);
      pending_frame_ = PendingFrame{image, to_epoch_ns(capture_time)};
    }
    frame_ready_.notify_one();
  }

  // 100 Hz 的规划线程只做订阅判断、取时间戳和入队，序列化与 SDK 调用交给数据线程。
  void publish_telemetry(nlohmann::json data, std::chrono::steady_clock::time_point t)
  {
    if (!telemetry_channel_->hasSinks()) return;  // 无人订阅：不序列化、不入队
    enqueue(PendingMessage::Kind::Telemetry, std::move(data), to_epoch_ns(t));
  }

  void publish_status(nlohmann::json fields, std::chrono::steady_clock::time_point sample_time)
  {
    if (!status_channel_->hasSinks()) return;
    const auto now = std::chrono::steady_clock::now();
    if (now < next_status_publish_) return;
    next_status_publish_ = now + std::chrono::seconds(1);

    const auto image_clients = static_cast<uint64_t>(image_server_->clientCount());
    const auto data_clients =
      split_ ? static_cast<uint64_t>(data_server_->clientCount()) : uint64_t{0};

    fields["split_mode"] = split_;
    fields["image_port"] = options_.image_port;
    fields["data_port"] = data_port();
    fields["image_clients"] = image_clients;
    fields["data_clients"] = data_clients;
    fields["connected_clients"] = image_clients + data_clients;
    fields["published_images"] = published_images_.load();
    fields["dropped_images"] = dropped_images_.load();
    fields["last_jpeg_bytes"] = last_jpeg_bytes_.load();
    fields["server_time_ns"] = to_epoch_ns(sample_time);

    // 下面这些量由内部发布线程写入，这里在调用线程读取，用来验证「Foxglove 有没有拖慢推理」：
    // encode_ms_* 是缩放 + JPEG 编码的耗时，image_sched/data_sched 是两个线程实际生效的调度策略，
    // dropped_messages 是数据队列满时丢掉的条数（不为 0 说明数据线程已经很吃力）。
    fields["sched_mode"] = options_.yield_cpu ? "auto" : "off";
    fields["image_sched"] = sched_echo(image_sched_);
    fields["data_sched"] = sched_echo(data_sched_);
    fields["encode_ms_last"] = encode_ms_last_.load();
    fields["encode_ms_ema"] = encode_ms_ema_.load();
    fields["published_telemetry"] = published_telemetry_.load();
    fields["dropped_messages"] = dropped_messages_.load();
    fields["queued_messages"] = queued_messages();

    enqueue(PendingMessage::Kind::Status, std::move(fields), to_epoch_ns(sample_time));
  }

  uint16_t image_port() const { return image_server_->port(); }

  uint16_t data_port() const { return split_ ? data_server_->port() : image_server_->port(); }

private:
  struct PendingFrame
  {
    cv::Mat image;
    uint64_t log_time_ns = 0;
  };

  struct PendingMessage
  {
    enum class Kind : uint8_t { Telemetry, Status };

    Kind kind = Kind::Telemetry;
    nlohmann::json payload;    // 不在这里序列化：dump() 在数据线程上做
    uint64_t log_time_ns = 0;  // 调用线程入队时算好，线上时间戳与改动前逐字节一致
  };

  // 发布线程实际生效的调度参数，供 /vision/status 回显。
  struct SchedReport
  {
    std::atomic<int> policy{-1};
    std::atomic<int> nice{0};
    std::atomic<int> error{0};
  };

  // 让发布线程把 CPU 让给 OpenVINO 推理：本模块的推送是给人看的，推理才是实时路径。
  // 只影响调用线程（sched_setscheduler/setpriority 的 pid 0 就是自己），
  // 所以放在 worker 函数开头调用。设置失败不致命，回显到 /vision/status 即可。
  void apply_scheduling(bool allow_idle, SchedReport & report)
  {
    if (options_.yield_cpu) {
      // 图像线程：帧本来就会丢最旧，允许降到最低优先级。
      // 数据线程不允许：它必须能及时送出遥测，所以只降 nice，不降到 SCHED_IDLE。
      if (allow_idle) {
        sched_param param{};
        param.sched_priority = 0;
        if (::sched_setscheduler(0, SCHED_IDLE, &param) != 0) report.error.store(errno);
      }
      // 提高 nice 值（降低优先级）不需要任何特权，是 SCHED_IDLE 不可用时的兜底。
      if (::setpriority(PRIO_PROCESS, 0, 19) != 0) report.error.store(errno);
    }

    errno = 0;  // getpriority 返回 -1 既可能是失败也可能是合法的 nice = -1，必须先清 errno
    const int nice = ::getpriority(PRIO_PROCESS, 0);
    report.nice.store(errno == 0 ? nice : -1000);
    report.policy.store(::sched_getscheduler(0));
  }

  static void name_this_thread(const char * name)
  {
    (void)pthread_setname_np(pthread_self(), name);  // 最多 15 字节
  }

  static nlohmann::json sched_echo(const SchedReport & report)
  {
    const int error = report.error.load();
    nlohmann::json echo;
    echo["policy"] = sched_policy_name(report.policy.load());
    echo["nice"] = report.nice.load();
    echo["errno"] = error;
    echo["err"] = sched_errno_name(error);
    return echo;
  }

  // 只在单个线程上调用，因此用普通 bool 记录 EMA 是否已初始化。
  static void update_ema(std::atomic<double> & ema, bool & valid, double sample)
  {
    const double previous = valid ? ema.load() : sample;
    ema.store(previous + kEmaAlpha * (sample - previous));
    valid = true;
  }

  std::size_t queued_messages()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return data_queue_.size();
  }

  // 永不阻塞调用线程：满时丢最旧，与图像路径同为「最新的数据最重要」。
  void enqueue(PendingMessage::Kind kind, nlohmann::json payload, uint64_t log_time_ns)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (data_queue_.size() >= kDataQueueDepth) {
        data_queue_.pop_front();
        dropped_messages_.fetch_add(1);
      }
      data_queue_.push_back(PendingMessage{kind, std::move(payload), log_time_ns});
    }
    data_ready_.notify_one();
  }

  // 遥测与状态的 dump() 与 SDK 调用都在这里完成，不占用 100 Hz 的规划线程。
  // 单线程串行发送，因此队列的 FIFO 顺序就是线上的先后顺序。
  void publish_data_loop()
  {
    name_this_thread("foxglove-data");
    apply_scheduling(/*allow_idle=*/false, data_sched_);

    while (true) {
      PendingMessage message;
      {
        std::unique_lock<std::mutex> lock(data_mutex_);
        data_ready_.wait(lock, [this] { return data_stop_ || !data_queue_.empty(); });
        if (data_stop_) break;  // 退出时不排空：与图像路径一致，退出就是要快
        message = std::move(data_queue_.front());
        data_queue_.pop_front();
      }
      log_message(message);
    }
  }

  void log_message(const PendingMessage & message)
  {
    const bool is_status = message.kind == PendingMessage::Kind::Status;
    foxglove::RawChannel & channel = is_status ? *status_channel_ : *telemetry_channel_;
    if (!channel.hasSinks()) return;  // 排队期间客户端可能已经断开

    const auto payload = message.payload.dump();
    (void)channel.log(
      reinterpret_cast<const std::byte *>(payload.data()), payload.size(), message.log_time_ns);
    if (!is_status) published_telemetry_.fetch_add(1);
  }

  std::unique_ptr<foxglove::WebSocketServer> create_server(
    const std::string & name, const foxglove::Context & context, uint16_t port,
    std::size_t backlog)
  {
    foxglove::WebSocketServerOptions server_options;
    server_options.name = name;
    server_options.host = options_.host;
    server_options.port = port;
    server_options.context = context;
    server_options.capabilities = foxglove::WebSocketServerCapabilities::None;
    server_options.supported_encodings = {"json"};
    server_options.message_backlog_size = backlog;

    auto server_result = foxglove::WebSocketServer::create(std::move(server_options));
    if (!server_result.has_value())
      throw std::runtime_error(
        std::string("Failed to start Foxglove server: ") +
        foxglove::strerror(server_result.error()));
    return std::make_unique<foxglove::WebSocketServer>(std::move(server_result.value()));
  }

  // 缩放与 JPEG 编码都在这里完成，不占用调用 publish_image 的检测主循环。
  void publish_loop()
  {
    name_this_thread("foxglove-image");
    // 图像线程可以降到最低优先级：帧本来就会丢最旧，推理比画面重要。
    apply_scheduling(/*allow_idle=*/true, image_sched_);

    while (true) {
      PendingFrame frame;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_ready_.wait(lock, [this] { return stop_ || pending_frame_.has_value(); });
        if (stop_) break;
        frame = std::move(*pending_frame_);
        pending_frame_.reset();
      }
      encode_and_log(frame);
    }
  }

  void encode_and_log(const PendingFrame & frame)
  {
    if (frame.image.empty() || frame.log_time_ns == 0) return;
    if (!image_channel_->hasSinks()) return;  // 排队期间客户端可能已经断开

    // 只统计真正吃 CPU 的缩放 + 编码，供 /vision/status 判断这台机器上编码到底占多少。
    const auto encode_started = std::chrono::steady_clock::now();
    cv::Mat to_encode = frame.image;
    if (options_.image_scale < 1.0) {
      cv::Mat scaled;
      cv::resize(
        frame.image, scaled, {}, options_.image_scale, options_.image_scale, cv::INTER_AREA);
      to_encode = scaled;
    }

    std::vector<uint8_t> jpeg;
    if (!cv::imencode(
          ".jpg", to_encode, jpeg, {cv::IMWRITE_JPEG_QUALITY, options_.jpeg_quality}))
      return;

    const double encode_ms = 0.001 * static_cast<double>(
                                        std::chrono::duration_cast<std::chrono::microseconds>(
                                          std::chrono::steady_clock::now() - encode_started)
                                          .count());
    encode_ms_last_.store(encode_ms);
    update_ema(encode_ms_ema_, encode_ema_valid_, encode_ms);

    foxglove::messages::CompressedImage message;
    message.timestamp = foxglove::messages::Timestamp{
      static_cast<uint32_t>(frame.log_time_ns / 1000000000ULL),
      static_cast<uint32_t>(frame.log_time_ns % 1000000000ULL)};
    message.frame_id = "camera";
    message.format = "jpeg";
    message.data.resize(jpeg.size());
    std::memcpy(message.data.data(), jpeg.data(), jpeg.size());

    last_jpeg_bytes_.store(jpeg.size());
    published_images_.fetch_add(1);
    (void)image_channel_->log(message, frame.log_time_ns);
  }

  FoxgloveVisionOptions options_;
  bool split_ = false;

  foxglove::Context image_context_;
  foxglove::Context data_context_;
  std::unique_ptr<foxglove::WebSocketServer> image_server_;
  std::unique_ptr<foxglove::WebSocketServer> data_server_;
  std::unique_ptr<foxglove::messages::CompressedImageChannel> image_channel_;
  std::unique_ptr<foxglove::RawChannel> telemetry_channel_;
  std::unique_ptr<foxglove::RawChannel> status_channel_;

  std::mutex frame_mutex_;
  std::condition_variable frame_ready_;
  std::optional<PendingFrame> pending_frame_;
  bool stop_ = false;
  std::thread worker_;

  // 数据线程：遥测与状态共用一个有界队列，由单个线程串行 dump + log，因此 FIFO 不会乱序。
  std::mutex data_mutex_;
  std::condition_variable data_ready_;
  std::deque<PendingMessage> data_queue_;
  bool data_stop_ = false;
  std::thread data_worker_;

  std::chrono::steady_clock::time_point next_image_publish_{};
  std::chrono::steady_clock::time_point next_status_publish_{};
  std::atomic<uint64_t> published_images_{0};
  std::atomic<uint64_t> dropped_images_{0};
  std::atomic<uint64_t> last_jpeg_bytes_{0};

  // 发布线程写、状态线程读的观测值（只用于 /vision/status 的自我诊断）
  std::atomic<double> encode_ms_last_{0.0};
  std::atomic<double> encode_ms_ema_{0.0};
  bool encode_ema_valid_ = false;  // 只属于图像线程
  std::atomic<uint64_t> published_telemetry_{0};
  std::atomic<uint64_t> dropped_messages_{0};
  SchedReport image_sched_;
  SchedReport data_sched_;
};

FoxgloveVision::FoxgloveVision(FoxgloveVisionOptions options)
: impl_(std::make_unique<Impl>(std::move(options)))
{
}

FoxgloveVision::~FoxgloveVision() = default;

bool FoxgloveVision::image_requested() const { return impl_->image_requested(); }

bool FoxgloveVision::data_requested() const { return impl_->data_requested(); }

void FoxgloveVision::publish_image(
  const cv::Mat & image, std::chrono::steady_clock::time_point capture_time)
{
  impl_->publish_image(image, capture_time);
}

void FoxgloveVision::publish_telemetry(
  nlohmann::json data, std::chrono::steady_clock::time_point sample_time)
{
  impl_->publish_telemetry(std::move(data), sample_time);
}

void FoxgloveVision::publish_status(
  nlohmann::json fields, std::chrono::steady_clock::time_point sample_time)
{
  impl_->publish_status(std::move(fields), sample_time);
}

uint16_t FoxgloveVision::image_port() const { return impl_->image_port(); }

uint16_t FoxgloveVision::data_port() const { return impl_->data_port(); }

}  // namespace visualization
