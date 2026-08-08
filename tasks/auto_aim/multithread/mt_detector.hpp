#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <tuple>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
namespace multithread
{

class MultiThreadDetector
{
public:
  MultiThreadDetector(const std::string & config_path, bool debug = false);

  /// 返回 false 表示队列已满、该帧被丢弃
  bool push(cv::Mat img, std::chrono::steady_clock::time_point t);

  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop();  //暂时不支持yolov8

  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();

  bool empty() { return queue_->empty(); }

private:
  struct Pending
  {
    NetDetector::TicketPtr ticket;
    std::chrono::steady_clock::time_point timestamp;
  };

  YOLO yolo_;
  std::unique_ptr<tools::ThreadSafeQueue<Pending>> queue_;
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP
