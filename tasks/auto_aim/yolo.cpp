#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include "yolos/yolo11.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov8.hpp"

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
: net_detector_([&config_path] {
    auto yaml = YAML::LoadFile(config_path);
    const auto yolo_name = yaml["yolo_name"].as<std::string>();
    const int input_size = yolo_name == "yolov8" ? 416 : 640;
    const auto roi = yaml["roi"];
    return NetDetector::Config {
      yaml[yolo_name + "_model_path"].as<std::string>(), yaml["device"].as<std::string>(), input_size,
      input_size, yaml["infer_request_buffer_num"] ? yaml["infer_request_buffer_num"].as<int>() : 2,
      yaml["use_roi"].as<bool>(),
      {roi["x"].as<int>(), roi["y"].as<int>(), roi["width"].as<int>(), roi["height"].as<int>()}};
  }())
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  if (yolo_name == "yolov8") {
    yolo_ = std::make_unique<YOLOV8>(config_path, debug);
  }

  else if (yolo_name == "yolo11") {
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
  }

  else if (yolo_name == "yolov5") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  }

  else {
    throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
  }
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  if (img.empty()) {
    return {};
  }
  auto ticket = net_detector_.start(img);
  return postprocess(ticket, frame_count);
}

NetDetector::TicketPtr YOLO::try_start_async(const cv::Mat & img)
{
  return net_detector_.try_start_async(img, true);
}

std::list<Armor> YOLO::postprocess(const NetDetector::TicketPtr & ticket, int frame_count)
{
  auto result = net_detector_.wait(ticket);
  return yolo_->postprocess(result, frame_count);
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
