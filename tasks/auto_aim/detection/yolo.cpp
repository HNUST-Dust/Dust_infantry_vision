#include "yolo.hpp"

#include "tools/runtime/yaml_config.hpp"

#include "yolov5.hpp"

namespace auto_aim
{
YoloConfig YoloConfig::parse(const tools::YamlConfig & yaml)
{
  YoloConfig config;
  auto & net = config.net;
  net.backend = yaml.get<std::string>("inference_backend", "openvino");
  net.device = yaml.get<std::string>("device", "CPU");
  net.cuda_device = yaml.get<int>("cuda_device", 0);
  net.infer_request_buffer_num = yaml.get<int>("infer_request_buffer_num", 2);
  net.performance_mode = yaml.get<std::string>("performance_mode", "LATENCY");
  net.model_path = yaml.input_path("yolov5_model_path");
  net.input_width = 640;
  net.input_height = 640;
  if (net.infer_request_buffer_num <= 0)
    yaml.fail("infer_request_buffer_num", "must be positive");
  if (net.backend != "openvino" && net.backend != "tensorrt")
    yaml.fail("inference_backend", "must be openvino or tensorrt");
#ifndef DUST_ENABLE_TENSORRT
  if (net.backend == "tensorrt")
    yaml.fail("inference_backend", "tensorrt requires -DENABLE_TENSORRT=ON");
#endif
  if (net.backend == "openvino" && net.performance_mode != "LATENCY" &&
      net.performance_mode != "THROUGHPUT")
    yaml.fail("performance_mode", "must be LATENCY or THROUGHPUT");
  config.postprocess = YoloV5Config::parse(yaml);
  return config;
}

YOLO::YOLO(const std::string & config_path, bool debug)
    : YOLO(YoloConfig::parse(tools::YamlConfig(config_path)), debug)
{
}

YOLO::YOLO(const YoloConfig & config, bool debug)
    : net_detector_(config.net), yolo_(std::make_unique<YOLOV5>(config.postprocess, debug))
{
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count,
                              std::optional<cv::Rect> roi_override,
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

NetDetector::TicketPtr
YOLO::try_start_async(const cv::Mat & img, std::optional<cv::Rect> roi_override, bool clone_source)
{
  return net_detector_.try_start_async(img, clone_source, roi_override);
}

std::list<Armor> YOLO::postprocess(const NetDetector::TicketPtr & ticket, int frame_count,
                                   std::optional<cv::Rect> light_roi, double * wait_ms)
{
  auto result = net_detector_.wait(ticket);
  if (wait_ms)
    *wait_ms = result.wait_ms;
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
