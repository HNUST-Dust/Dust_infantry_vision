#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include "yolos/yolov5.hpp"

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
: net_detector_([&config_path] {
    auto yaml = YAML::LoadFile(config_path);
    NetDetector::Config config;
    config.device = yaml["device"].as<std::string>();
    config.infer_request_buffer_num =
      yaml["infer_request_buffer_num"] ? yaml["infer_request_buffer_num"].as<int>() : 2;
    const auto roi = yaml["roi"];
    config.use_roi = yaml["use_roi"].as<bool>();
    config.roi = {roi["x"].as<int>(), roi["y"].as<int>(), roi["width"].as<int>(),
      roi["height"].as<int>()};
    config.model_path = yaml["yolov5_model_path"].as<std::string>();
    config.input_width = 640;
    config.input_height = 640;
    return config;
  }())
{
  yolo_ = std::make_unique<YOLOV5>(config_path, debug);
}

std::list<Armor> YOLO::detect(
  const cv::Mat & img, int frame_count, std::optional<cv::Rect> roi_override,
  std::optional<cv::Rect> light_roi)
{
  // Synchronous convenience path for offline/test use: submit and immediately wait.
  // MultiThreadDetector is the production async pipeline.
  if (img.empty()) {
    return {};
  }
  auto ticket = net_detector_.start(img, roi_override);
  return postprocess(ticket, frame_count, light_roi);
}

NetDetector::TicketPtr YOLO::try_start_async(
  const cv::Mat & img, std::optional<cv::Rect> roi_override)
{
  return net_detector_.try_start_async(img, true, roi_override);
}

std::list<Armor> YOLO::postprocess(
  const NetDetector::TicketPtr & ticket, int frame_count, std::optional<cv::Rect> light_roi)
{
  auto result = net_detector_.wait(ticket);
  return yolo_->postprocess(result, frame_count, light_roi);
}

cv::Mat YOLO::source(const NetDetector::TicketPtr & ticket) const
{
  return net_detector_.wait(ticket).source;
}

std::size_t YOLO::request_capacity() const
{
  return net_detector_.request_capacity();
}

}  // namespace auto_aim
