#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <list>
#include <mutex>
#include <optional>
#include <opencv2/core.hpp>
#include <thread>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/ordered_delivery.hpp"

namespace auto_aim
{
namespace multithread
{

enum class SubmitStatus
{
  accepted,
  skipped_busy,
  skipped_empty,
  skipped_error,
  closed
};

struct SubmitResult
{
  SubmitStatus status;
  uint64_t sequence;
};

struct Detection
{
  uint64_t sequence = 0;
  std::chrono::steady_clock::time_point timestamp;
  std::list<Armor> armors;
  cv::Rect net_roi;
  std::optional<cv::Rect> light_roi;
  bool inferred = false;
  cv::Mat source;
};

class MultiThreadDetector
{
public:
  explicit MultiThreadDetector(const std::string & config_path, bool keep_source = false);
  ~MultiThreadDetector();

  MultiThreadDetector(const MultiThreadDetector &) = delete;
  MultiThreadDetector & operator=(const MultiThreadDetector &) = delete;

  SubmitResult submit(
    cv::Mat image, std::chrono::steady_clock::time_point timestamp, const cv::Rect & net_roi,
    std::optional<cv::Rect> light_roi = std::nullopt);

  std::optional<Detection> wait_pop();
  std::optional<Detection> wait_pop_for(std::chrono::milliseconds timeout);
  void close();
  void join();

  std::size_t request_capacity() const;

private:
  struct Pending
  {
    uint64_t sequence;
    NetDetector::TicketPtr ticket;
    std::chrono::steady_clock::time_point timestamp;
    cv::Rect net_roi;
    std::optional<cv::Rect> light_roi;
  };

  Detection skipped_detection(
    uint64_t sequence, std::chrono::steady_clock::time_point timestamp, const cv::Rect & net_roi,
    std::optional<cv::Rect> light_roi) const;
  void worker_loop();

  YOLO yolo_;
  const bool keep_source_;
  tools::OrderedDelivery<Detection> delivery_;
  std::atomic<bool> accepting_{true};
  std::atomic<uint64_t> next_sequence_{1};
  std::mutex pending_mutex_;
  std::condition_variable pending_ready_;
  std::deque<Pending> pending_;
  bool pending_closed_ = false;
  std::thread worker_;
};

}  // namespace multithread
}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP
