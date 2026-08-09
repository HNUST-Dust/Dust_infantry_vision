#include "mt_detector.hpp"

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug)
{
  queue_ = std::make_unique<tools::ThreadSafeQueue<Pending>>(
    yolo_.request_capacity(), [] { tools::logger()->debug("[MultiThreadDetector] queue is full!"); });

  tools::logger()->info(
    "[MultiThreadDetector] initialized with {} asynchronous requests", yolo_.request_capacity());
}

bool MultiThreadDetector::push(
  cv::Mat img, std::chrono::steady_clock::time_point t, std::optional<cv::Rect> roi_override)
{
  // 延迟优先：队列已满时直接丢弃新帧，不要启动推理浪费 GPU 时间
  if (queue_->full()) {
    tools::logger()->debug("[MultiThreadDetector] queue is full, drop frame!");
    return false;
  }

  auto ticket = yolo_.try_start_async(img, roi_override);
  if (!ticket) {
    tools::logger()->debug("[MultiThreadDetector] request pool is full, drop frame!");
    return false;
  }
  if (!queue_->push({std::move(ticket), t})) {
    return false;
  }
  return true;
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  auto pending = queue_->pop();
  auto armors = yolo_.postprocess(pending.ticket, 0);
  return {std::move(armors), pending.timestamp};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  auto pending = queue_->pop();
  auto img = yolo_.source(pending.ticket);
  auto armors = yolo_.postprocess(pending.ticket, 0);
  return {img, std::move(armors), pending.timestamp};
}

}  // namespace multithread

}  // namespace auto_aim
