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
  worker_(&MultiThreadDetector::worker_loop, this),
  dispatcher_(&MultiThreadDetector::dispatch_loop, this)
{
  tools::logger()->info(
    "[MultiThreadDetector] initialized with {} asynchronous requests, event capacity {}, and a "
    "latest-frame cache",
    yolo_.request_capacity(), 2 * yolo_.request_capacity());
}

MultiThreadDetector::~MultiThreadDetector()
{
  close();
  join();
}

SubmitResult MultiThreadDetector::submit(
  cv::Mat image, std::chrono::steady_clock::time_point timestamp, const cv::Rect & net_roi,
  std::optional<cv::Rect> light_roi, std::optional<Eigen::Quaterniond> q, double capture_ms)
{
  if (!accepting_.load()) return {SubmitStatus::closed, 0};

  const auto started = std::chrono::steady_clock::now();
  CapturedFrame frame{
    std::move(image), timestamp, net_roi, light_roi, std::move(q), capture_ms};
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (frame_closed_) return {SubmitStatus::closed, 0};
    frame.submit_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - started)
                        .count();
    if (latest_frame_) dropped_overwritten_.fetch_add(1);
    latest_frame_ = std::move(frame);
  }
  frame_ready_.notify_one();
  return {SubmitStatus::accepted, 0};
}

std::optional<Detection> MultiThreadDetector::wait_pop()
{
  return pop_delivery(delivery_.wait_pop());
}

std::optional<Detection> MultiThreadDetector::wait_pop_for(std::chrono::milliseconds timeout)
{
  return pop_delivery(delivery_.wait_pop_for(timeout));
}

std::optional<Detection> MultiThreadDetector::pop_delivery(
  std::optional<tools::OrderedDelivery<Detection>::Event> event)
{
  if (!event) return std::nullopt;
  auto detection = std::move(event->value);
  detection.delivery_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - detection.ready_at)
                            .count();
  return detection;
}

void MultiThreadDetector::close()
{
  if (!accepting_.exchange(false)) return;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame_closed_ = true;
    latest_frame_.reset();
  }
  delivery_.close();
  frame_ready_.notify_all();
}

void MultiThreadDetector::join()
{
  if (dispatcher_.joinable()) dispatcher_.join();
  if (worker_.joinable()) worker_.join();
}

std::size_t MultiThreadDetector::request_capacity() const
{
  return yolo_.request_capacity();
}

DetectorDropCounts MultiThreadDetector::drop_counts() const
{
  return {dropped_overwritten_.load(), dropped_result_capacity_.load(), skipped_busy_.load(),
    skipped_empty_.load(), skipped_error_.load()};
}

Detection MultiThreadDetector::skipped_detection(
  uint64_t sequence, std::chrono::steady_clock::time_point timestamp, const cv::Rect & net_roi,
  std::optional<cv::Rect> light_roi, std::optional<Eigen::Quaterniond> q, double capture_ms,
  double submit_ms) const
{
  Detection detection;
  detection.sequence = sequence;
  detection.timestamp = timestamp;
  detection.net_roi = net_roi;
  detection.light_roi = light_roi;
  detection.q = std::move(q);
  detection.capture_ms = capture_ms;
  detection.submit_ms = submit_ms;
  detection.ready_at = std::chrono::steady_clock::now();
  return detection;
}

void MultiThreadDetector::dispatch_loop()
{
  while (true) {
    CapturedFrame frame;
    {
      std::unique_lock<std::mutex> lock(frame_mutex_);
      frame_ready_.wait(lock, [this] { return frame_closed_ || latest_frame_.has_value(); });
      if (frame_closed_) break;
      frame = std::move(*latest_frame_);
      latest_frame_.reset();
    }

    const uint64_t sequence = next_sequence_;
    if (!delivery_.reserve(sequence)) {
      if (!delivery_.closed()) dropped_result_capacity_.fetch_add(1);
      continue;
    }

    if (frame.image.empty()) {
      skipped_empty_.fetch_add(1);
      delivery_.cancel(sequence);
      continue;
    }

    try {
      const auto inference_started = std::chrono::steady_clock::now();
      auto ticket = yolo_.try_start_async(frame.image, frame.net_roi, false);
      if (!ticket) {
        skipped_busy_.fetch_add(1);
        delivery_.cancel(sequence);
        continue;
      }

      ++next_sequence_;

      Pending pending{};
      pending.sequence = sequence;
      pending.ticket = std::move(ticket);
      pending.timestamp = frame.timestamp;
      pending.net_roi = frame.net_roi;
      pending.light_roi = frame.light_roi;
      pending.q = frame.q;
      pending.capture_ms = frame.capture_ms;
      pending.submit_ms = frame.submit_ms;
      pending.inference_started = inference_started;
      {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_.push_back(std::move(pending));
      }
      pending_ready_.notify_one();
    } catch (const std::exception & e) {
      skipped_error_.fetch_add(1);
      tools::logger()->warn("[MultiThreadDetector] submit failed: {}", e.what());
      ++next_sequence_;
      delivery_.skip(sequence, skipped_detection(
                                 sequence, frame.timestamp, frame.net_roi, frame.light_roi, frame.q,
                                 frame.capture_ms, frame.submit_ms));
    } catch (...) {
      skipped_error_.fetch_add(1);
      tools::logger()->warn("[MultiThreadDetector] submit failed with unknown exception");
      ++next_sequence_;
      delivery_.skip(sequence, skipped_detection(
                                 sequence, frame.timestamp, frame.net_roi, frame.light_roi, frame.q,
                                 frame.capture_ms, frame.submit_ms));
    }
  }

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_closed_ = true;
  }
  pending_ready_.notify_all();
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

    Detection detection;
    detection.sequence = pending.sequence;
    detection.timestamp = pending.timestamp;
    detection.net_roi = pending.net_roi;
    detection.light_roi = pending.light_roi;
    detection.inferred = true;
    detection.q = pending.q;
    detection.capture_ms = pending.capture_ms;
    detection.submit_ms = pending.submit_ms;
    try {
      const auto postprocess_started = std::chrono::steady_clock::now();
      detection.armors = yolo_.postprocess(
        pending.ticket, -1, pending.light_roi, &detection.inference_wait_ms);
      detection.postprocess_ms = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - postprocess_started)
                                   .count() -
                               detection.inference_wait_ms;
      if (keep_source_.load()) detection.source = yolo_.source(pending.ticket);
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
    detection.inference_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - pending.inference_started)
                               .count();
    detection.ready_at = std::chrono::steady_clock::now();
    delivery_.complete(pending.sequence, std::move(detection));
  }
}

}  // namespace multithread
}  // namespace auto_aim
