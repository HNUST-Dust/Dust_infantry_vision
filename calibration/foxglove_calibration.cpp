#include "foxglove_calibration.hpp"

#include <foxglove/channel.hpp>
#include <foxglove/error.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/service.hpp>
#include <foxglove/websocket.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace calibration
{
namespace
{
std::vector<std::byte> bytes(std::string_view value)
{
  auto begin = reinterpret_cast<const std::byte *>(value.data());
  return {begin, begin + value.size()};
}

void respond_json(foxglove::ServiceResponder && responder, const nlohmann::json & value)
{
  auto payload = bytes(value.dump());
  std::move(responder).respondOk(payload);
}

foxglove::ServiceSchema trigger_schema()
{
  static const auto request_schema = bytes(R"({
    "type": "object",
    "properties": {},
    "additionalProperties": false
  })");
  static const auto response_schema = bytes(R"({
    "type": "object",
    "properties": {
      "success": {"type": "boolean"},
      "message": {"type": "string"}
    },
    "required": ["success", "message"],
    "additionalProperties": false
  })");

  foxglove::ServiceMessageSchema request{
    "json",
    foxglove::Schema{
      "CalibrationTriggerRequest", "jsonschema", request_schema.data(), request_schema.size()}};
  foxglove::ServiceMessageSchema response{
    "json",
    foxglove::Schema{
      "CalibrationTriggerResponse", "jsonschema", response_schema.data(), response_schema.size()}};
  return {"CalibrationTrigger", request, response};
}

uint64_t system_time_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}
}  // namespace

class FoxgloveCalibration::Impl
{
public:
  explicit Impl(FoxgloveOptions options) : options_(std::move(options))
  {
    foxglove::setLogLevel(foxglove::LogLevel::Info);

    foxglove::WebSocketServerOptions server_options;
    server_options.name = "dust-handeye-calibration";
    server_options.host = options_.host;
    server_options.port = options_.port;
    server_options.capabilities = foxglove::WebSocketServerCapabilities::Services;
    server_options.supported_encodings = {"json"};
    server_options.message_backlog_size = 4;

    auto server_result = foxglove::WebSocketServer::create(std::move(server_options));
    if (!server_result.has_value()) {
      throw std::runtime_error(
        std::string("Failed to start Foxglove server: ") +
        foxglove::strerror(server_result.error()));
    }
    server_ = std::make_unique<foxglove::WebSocketServer>(std::move(server_result.value()));

    auto image_result =
      foxglove::messages::CompressedImageChannel::create("/calibration/image/compressed");
    if (!image_result.has_value()) {
      throw std::runtime_error(
        std::string("Failed to create Foxglove image channel: ") +
        foxglove::strerror(image_result.error()));
    }
    image_channel_ = std::make_unique<foxglove::messages::CompressedImageChannel>(
      std::move(image_result.value()));

    auto status_result = foxglove::RawChannel::create("/calibration/status", "json");
    if (!status_result.has_value()) {
      throw std::runtime_error(
        std::string("Failed to create Foxglove status channel: ") +
        foxglove::strerror(status_result.error()));
    }
    status_channel_ =
      std::make_unique<foxglove::RawChannel>(std::move(status_result.value()));

    register_services();
  }

  ~Impl()
  {
    if (server_) (void)server_->stop();
  }

  bool take_save_request() { return save_requested_.exchange(false); }

  bool quit_requested() const { return quit_requested_.load(); }

  void publish(
    const cv::Mat & preview, bool grid_detected, const Eigen::Vector3d & zyx_degree,
    int saved_count, const std::string & last_event)
  {
    auto now = std::chrono::steady_clock::now();
    if (now < next_publish_) return;
    next_publish_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(1.0 / options_.fps));

    const auto timestamp_ns = system_time_ns();
    size_t jpeg_size = 0;
    if (image_channel_->hasSinks()) {
      std::vector<uint8_t> jpeg;
      if (cv::imencode(
            ".jpg", preview, jpeg, {cv::IMWRITE_JPEG_QUALITY, options_.jpeg_quality})) {
        foxglove::messages::CompressedImage message;
        message.timestamp = foxglove::messages::Timestamp{
          static_cast<uint32_t>(timestamp_ns / 1000000000ULL),
          static_cast<uint32_t>(timestamp_ns % 1000000000ULL)};
        message.frame_id = "camera";
        message.format = "jpeg";
        message.data.resize(jpeg.size());
        std::memcpy(message.data.data(), jpeg.data(), jpeg.size());
        jpeg_size = jpeg.size();
        (void)image_channel_->log(message, timestamp_ns);
      }
    }

    if (status_channel_->hasSinks()) {
      nlohmann::json status{
        {"grid_detected", grid_detected},
        {"save_pending", save_requested_.load()},
        {"saved_count", saved_count},
        {"last_event", last_event},
        {"yaw_degree", zyx_degree[0]},
        {"pitch_degree", zyx_degree[1]},
        {"roll_degree", zyx_degree[2]},
        {"jpeg_bytes", jpeg_size},
        {"connected_clients", server_->clientCount()},
        {"server_time_ns", timestamp_ns}};
      auto payload = status.dump();
      (void)status_channel_->log(
        reinterpret_cast<const std::byte *>(payload.data()), payload.size(), timestamp_ns);
    }
  }

  uint16_t port() const { return server_->port(); }

private:
  void register_services()
  {
    save_handler_ = [this](
                      const foxglove::ServiceRequest &, foxglove::ServiceResponder && responder) {
      bool already_pending = save_requested_.exchange(true);
      respond_json(
        std::move(responder),
        {{"success", !already_pending},
         {"message", already_pending ? "save already pending" : "save requested"}});
    };
    quit_handler_ = [this](
                      const foxglove::ServiceRequest &, foxglove::ServiceResponder && responder) {
      quit_requested_.store(true);
      respond_json(std::move(responder), {{"success", true}, {"message", "quit requested"}});
    };

    auto schema = trigger_schema();
    auto save_service =
      foxglove::Service::create("/calibration/save", schema, save_handler_);
    if (!save_service.has_value()) {
      throw std::runtime_error(
        std::string("Failed to create Foxglove save service: ") +
        foxglove::strerror(save_service.error()));
    }
    auto error = server_->addService(std::move(save_service.value()));
    if (error != foxglove::FoxgloveError::Ok) {
      throw std::runtime_error(
        std::string("Failed to register Foxglove save service: ") + foxglove::strerror(error));
    }

    auto quit_service =
      foxglove::Service::create("/calibration/quit", schema, quit_handler_);
    if (!quit_service.has_value()) {
      throw std::runtime_error(
        std::string("Failed to create Foxglove quit service: ") +
        foxglove::strerror(quit_service.error()));
    }
    error = server_->addService(std::move(quit_service.value()));
    if (error != foxglove::FoxgloveError::Ok) {
      throw std::runtime_error(
        std::string("Failed to register Foxglove quit service: ") + foxglove::strerror(error));
    }
  }

  FoxgloveOptions options_;
  std::atomic_bool save_requested_{false};
  std::atomic_bool quit_requested_{false};
  std::chrono::steady_clock::time_point next_publish_{};
  std::unique_ptr<foxglove::WebSocketServer> server_;
  std::unique_ptr<foxglove::messages::CompressedImageChannel> image_channel_;
  std::unique_ptr<foxglove::RawChannel> status_channel_;
  foxglove::ServiceHandler save_handler_;
  foxglove::ServiceHandler quit_handler_;
};

FoxgloveCalibration::FoxgloveCalibration(FoxgloveOptions options)
: impl_(std::make_unique<Impl>(std::move(options)))
{
}

FoxgloveCalibration::~FoxgloveCalibration() = default;

bool FoxgloveCalibration::take_save_request() { return impl_->take_save_request(); }

bool FoxgloveCalibration::quit_requested() const { return impl_->quit_requested(); }

void FoxgloveCalibration::publish(
  const cv::Mat & preview, bool grid_detected, const Eigen::Vector3d & zyx_degree,
  int saved_count, const std::string & last_event)
{
  impl_->publish(preview, grid_detected, zyx_degree, saved_count, last_event);
}
}  // namespace calibration
