#include "runtime_visualizer.hpp"

#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>

#include "tools/diagnostics/logger.hpp"
#include "tools/runtime/yaml_config.hpp"

namespace visualization
{
RuntimeVisualizationConfig RuntimeVisualizationConfig::parse(const tools::YamlConfig & yaml)
{
  RuntimeVisualizationConfig config;
  const auto runtime = yaml.section("runtime");
  config.headless = runtime.get("headless", false);
  config.foxglove_enabled = runtime.get("foxglove", false);
#ifdef DUST_ENABLE_FOXGLOVE
  if (config.foxglove_enabled) {
    const auto node = yaml.section("foxglove");
    auto & options = config.foxglove;
    options.host = node.get("host", options.host);
    const auto image_port = node.get("port", static_cast<int>(options.image_port));
    const auto data_port = node.get("data_port", static_cast<int>(options.data_port));
    options.image_fps = node.get("fps", options.image_fps);
    options.image_scale = node.get("scale", options.image_scale);
    options.jpeg_quality = node.get("jpeg_quality", options.jpeg_quality);
    const auto sched = node.get<std::string>("sched", "auto");
    if (image_port < 1 || image_port > 65535)
      node.fail("port", "must be between 1 and 65535");
    if (data_port < 0 || data_port > 65535)
      node.fail("data_port", "must be between 0 and 65535");
    if (!std::isfinite(options.image_fps) || options.image_fps <= 0)
      node.fail("fps", "must be finite and positive");
    if (!std::isfinite(options.image_scale) || options.image_scale <= 0 || options.image_scale > 1)
      node.fail("scale", "must be in (0, 1]");
    if (options.jpeg_quality < 1 || options.jpeg_quality > 100)
      node.fail("jpeg_quality", "must be between 1 and 100");
    if (sched != "auto" && sched != "off")
      node.fail("sched", "must be auto or off");
    options.image_port = static_cast<uint16_t>(image_port);
    options.data_port = static_cast<uint16_t>(data_port);
    options.yield_cpu = sched == "auto";
  }
#endif
  return config;
}

RuntimeVisualizer::RuntimeVisualizer(const RuntimeVisualizationConfig & config)
    : headless_(config.headless), window_name_(config.window_name)
{
  if (!config.foxglove_enabled)
    return;
#ifdef DUST_ENABLE_FOXGLOVE
  const auto & options = config.foxglove;
  try {
    foxglove_ = std::make_unique<FoxgloveVision>(options);
    tools::logger()->info("[Foxglove] image ws://{}:{}, telemetry ws://{}:{}", options.host,
                          foxglove_->image_port(), options.host, foxglove_->data_port());
  } catch (const std::exception & e) {
    tools::logger()->error("[Foxglove] disabled: {}", e.what());
  }
#else
  tools::logger()->error(
    "this binary was built without Foxglove; configure with -DENABLE_FOXGLOVE_VISION=ON");
#endif
}

RuntimeVisualizer::~RuntimeVisualizer() = default;

bool RuntimeVisualizer::remote_image_requested() const
{
#ifdef DUST_ENABLE_FOXGLOVE
  return foxglove_ && foxglove_->image_requested();
#else
  return false;
#endif
}

bool RuntimeVisualizer::needs_source() const
{
  return !headless_ || remote_image_requested();
}

RuntimeVisualizer::FrameOutput RuntimeVisualizer::frame_output(bool inferred,
                                                               const cv::Mat & image) const
{
  if (!inferred || image.empty())
    return {false, false};
  return {!headless_, remote_image_requested()};
}

bool RuntimeVisualizer::draw_frame(
  cv::Mat & image, const VisionOverlayInput & overlay, const auto_aim::Solver & solver,
  FrameOutput output, [[maybe_unused]] std::chrono::steady_clock::time_point captured_at)
{
  draw_vision_overlay(image, overlay, solver);
#ifdef DUST_ENABLE_FOXGLOVE
  if (output.remote)
    foxglove_->publish_image(image, captured_at);
#endif
  if (!output.local)
    return false;
  cv::resize(image, image, {}, 0.5, 0.5);
  cv::imshow(window_name_, image);
  return cv::waitKey(1) == 'q';
}

void RuntimeVisualizer::publish_telemetry(
  [[maybe_unused]] nlohmann::json data, [[maybe_unused]] const char * tracker_state,
  [[maybe_unused]] bool has_target,
  [[maybe_unused]] std::chrono::steady_clock::time_point sampled_at)
{
#ifdef DUST_ENABLE_FOXGLOVE
  if (foxglove_) {
    foxglove_->publish_telemetry(std::move(data), sampled_at);
    foxglove_->publish_status({{"tracker_state_name", tracker_state}, {"has_target", has_target}},
                              sampled_at);
  }
#endif
}
}  // namespace visualization
