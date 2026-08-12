#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include "awakening_detector.hpp"
#include "yolos/yolov5.hpp"

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
: net_detector_([&config_path] {
    auto yaml = YAML::LoadFile(config_path);
    const auto detector_name = yaml["detector_name"] ? yaml["detector_name"].as<std::string>() : "yolo";
    NetDetector::Config config;
    config.device = yaml["device"].as<std::string>();
    config.infer_request_buffer_num =
      yaml["infer_request_buffer_num"] ? yaml["infer_request_buffer_num"].as<int>() : 2;
    const auto roi = yaml["roi"];
    config.use_roi = yaml["use_roi"].as<bool>();
    config.roi = {roi["x"].as<int>(), roi["y"].as<int>(), roi["width"].as<int>(),
      roi["height"].as<int>()};
    if (detector_name == "awakening_tup") {
      const auto awakening = yaml["awakening_detector"];
      config.model_path = awakening["model_path"].as<std::string>();
      config.input_width = 416;
      config.input_height = 416;
      config.model_color_format = NetDetector::ColorFormat::bgr;
      config.normalize = false;
      config.center_letterbox = true;
      return config;
    }
    config.model_path = yaml["yolov5_model_path"].as<std::string>();
    config.input_width = 640;
    config.input_height = 640;
    return config;
  }())
{
  auto yaml = YAML::LoadFile(config_path);
  const auto detector_name = yaml["detector_name"] ? yaml["detector_name"].as<std::string>() : "yolo";

  if (detector_name == "awakening_tup") {
    yolo_ = std::make_unique<AwakeningArmorDetector>(config_path, debug);
  } else if (detector_name != "yolo") {
    throw std::runtime_error("Unknown detector name: " + detector_name + "!");
  } else {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  }
}

std::list<Armor> YOLO::detect(
  const cv::Mat & img, int frame_count, std::optional<cv::Rect> roi_override,
  std::optional<cv::Rect> light_roi)
{
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
