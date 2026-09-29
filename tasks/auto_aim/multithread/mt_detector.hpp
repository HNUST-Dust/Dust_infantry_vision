#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <Eigen/Geometry>

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

struct DetectorDropCounts
{
  uint64_t overwritten = 0;
  uint64_t result_capacity = 0;
  uint64_t skipped_busy = 0;
  uint64_t skipped_empty = 0;
  uint64_t skipped_error = 0;
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
  // Pose sampled at capture time; retained for inferred and skipped frames alike.
  std::optional<Eigen::Quaterniond> q;
  double capture_ms = 0.0;
  double submit_ms = 0.0;
  double inference_ms = 0.0;
  double delivery_ms = 0.0;
  std::chrono::steady_clock::time_point ready_at;
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
    std::optional<cv::Rect> light_roi = std::nullopt,
    std::optional<Eigen::Quaterniond> q = std::nullopt,
    double capture_ms = 0.0);

  std::optional<Detection> wait_pop();
  std::optional<Detection> wait_pop_for(std::chrono::milliseconds timeout);
  void close();
  void join();

  // 源图保留可随订阅状态动态开关：无人观看时不必让整帧随检测结果传递
  void set_keep_source(bool keep_source) { keep_source_.store(keep_source); }
  bool keep_source() const { return keep_source_.load(); }

  std::size_t request_capacity() const;
  DetectorDropCounts drop_counts() const;

private:
  struct CapturedFrame
  {
    cv::Mat image;
    std::chrono::steady_clock::time_point timestamp;
    cv::Rect net_roi;
    std::optional<cv::Rect> light_roi;
    std::optional<Eigen::Quaterniond> q;
    double capture_ms = 0.0;
    double submit_ms = 0.0;
  };

  struct Pending
  {
    uint64_t sequence;
    NetDetector::TicketPtr ticket;
    std::chrono::steady_clock::time_point timestamp;
    cv::Rect net_roi;
    std::optional<cv::Rect> light_roi;
    std::optional<Eigen::Quaterniond> q;
    double capture_ms = 0.0;
    double submit_ms = 0.0;
    std::chrono::steady_clock::time_point inference_started;
  };

  Detection skipped_detection(
    uint64_t sequence, std::chrono::steady_clock::time_point timestamp, const cv::Rect & net_roi,
    std::optional<cv::Rect> light_roi, std::optional<Eigen::Quaterniond> q,
    double capture_ms, double submit_ms) const;
  void dispatch_loop();
  void worker_loop();
  std::optional<Detection> pop_delivery(std::optional<tools::OrderedDelivery<Detection>::Event> event);

  YOLO yolo_;
  std::atomic<bool> keep_source_;
  tools::OrderedDelivery<Detection> delivery_;
  std::atomic<bool> accepting_{true};
  uint64_t next_sequence_ = 1;
  std::mutex frame_mutex_;
  std::condition_variable frame_ready_;
  std::optional<CapturedFrame> latest_frame_;
  bool frame_closed_ = false;
  std::mutex pending_mutex_;
  std::condition_variable pending_ready_;
  std::deque<Pending> pending_;
  bool pending_closed_ = false;
  std::atomic<uint64_t> dropped_overwritten_{0};
  std::atomic<uint64_t> dropped_result_capacity_{0};
  std::atomic<uint64_t> skipped_busy_{0};
  std::atomic<uint64_t> skipped_empty_{0};
  std::atomic<uint64_t> skipped_error_{0};
  std::thread worker_;
  std::thread dispatcher_;
};

}  // namespace multithread
}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP
