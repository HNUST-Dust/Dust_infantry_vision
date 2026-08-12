#include "mt_detector.hpp"

#include <exception>
#include <utility>

#include "tools/logger.hpp"

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool keep_source)
: yolo_(config_path, false), keep_source_(keep_source), delivery_(2 * yolo_.request_capacity(), 1),
  worker_(&MultiThreadDetector::worker_loop, this)
{
  tools::logger()->info(
    "[MultiThreadDetector] initialized with {} asynchronous requests and {} event capacity",
    yolo_.request_capacity(), 2 * yolo_.request_capacity());
}

MultiThreadDetector::~MultiThreadDetector()
{
  close();
  join();
}

SubmitResult MultiThreadDetector::submit(
  cv::Mat image, std::chrono::steady_clock::time_point timestamp, const cv::Rect & net_roi,
  std::optional<cv::Rect> light_roi)
{
  if (!accepting_.load()) return {SubmitStatus::closed, 0};

  const uint64_t sequence = next_sequence_.fetch_add(1);
  if (!delivery_.wait_reserve(sequence)) return {SubmitStatus::closed, 0};

  if (image.empty()) {
    delivery_.skip(sequence, skipped_detection(sequence, timestamp, net_roi, light_roi));
    return {SubmitStatus::skipped_empty, sequence};
  }

  try {
    auto ticket = yolo_.try_start_async(image, net_roi);
    if (!ticket) {
      delivery_.skip(sequence, skipped_detection(sequence, timestamp, net_roi, light_roi));
      return {SubmitStatus::skipped_busy, sequence};
    }

    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      if (pending_closed_) {
        delivery_.skip(sequence, skipped_detection(sequence, timestamp, net_roi, light_roi));
        return {SubmitStatus::closed, sequence};
      }
      pending_.push_back({sequence, std::move(ticket), timestamp, net_roi, light_roi});
    }
    pending_ready_.notify_one();
    return {SubmitStatus::accepted, sequence};
  } catch (const std::exception & e) {
    tools::logger()->warn("[MultiThreadDetector] submit failed: {}", e.what());
  } catch (...) {
    tools::logger()->warn("[MultiThreadDetector] submit failed with unknown exception");
  }

  delivery_.skip(sequence, skipped_detection(sequence, timestamp, net_roi, light_roi));
  return {SubmitStatus::skipped_error, sequence};
}

std::optional<Detection> MultiThreadDetector::wait_pop()
{
  auto event = delivery_.wait_pop();
  if (!event) return std::nullopt;
  return std::move(event->value);
}

std::optional<Detection> MultiThreadDetector::wait_pop_for(std::chrono::milliseconds timeout)
{
  auto event = delivery_.wait_pop_for(timeout);
  if (!event) return std::nullopt;
  return std::move(event->value);
}

void MultiThreadDetector::close()
{
  if (!accepting_.exchange(false)) return;
  delivery_.close();
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_closed_ = true;
  }
  pending_ready_.notify_all();
}

void MultiThreadDetector::join()
{
  if (worker_.joinable()) worker_.join();
}

std::size_t MultiThreadDetector::request_capacity() const
{
  return yolo_.request_capacity();
}

Detection MultiThreadDetector::skipped_detection(
  uint64_t sequence, std::chrono::steady_clock::time_point timestamp, const cv::Rect & net_roi,
  std::optional<cv::Rect> light_roi) const
{
  return {sequence, timestamp, {}, net_roi, light_roi, false, {}};
}

void MultiThreadDetector::worker_loop()
{
  while (true) {
    Pending pending;
    {
      std::unique_lock<std::mutex> lock(pending_mutex_);
      pending_ready_.wait(lock, [this] { return pending_closed_ || !pending_.empty(); });
      if (pending_.empty()) {
        if (pending_closed_) break;
        continue;
      }
      pending = std::move(pending_.front());
      pending_.pop_front();
    }

    Detection detection{
      pending.sequence, pending.timestamp, {}, pending.net_roi, pending.light_roi, true, {}};
    try {
      detection.armors = yolo_.postprocess(pending.ticket, -1, pending.light_roi);
      if (keep_source_) detection.source = yolo_.source(pending.ticket);
    } catch (const std::exception & e) {
      detection.inferred = false;
      tools::logger()->warn(
        "[MultiThreadDetector] postprocess failed for sequence {}: {}", pending.sequence,
        e.what());
    } catch (...) {
      detection.inferred = false;
      tools::logger()->warn(
        "[MultiThreadDetector] postprocess failed for sequence {}", pending.sequence);
    }
    delivery_.complete(pending.sequence, std::move(detection));
  }
}

}  // namespace multithread
}  // namespace auto_aim
