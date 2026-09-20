#ifndef TOOLS__QUATERNION_BUFFER_HPP
#define TOOLS__QUATERNION_BUFFER_HPP

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <iterator>
#include <mutex>
#include <optional>
#include <stdexcept>

namespace tools
{
struct QuaternionSample
{
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point before;
  std::chrono::steady_clock::time_point after;
};

// A bounded, non-destructive history in the host steady-clock time domain.
class QuaternionBuffer
{
public:
  using Clock = std::chrono::steady_clock;

  explicit QuaternionBuffer(
    std::size_t capacity, std::chrono::milliseconds max_gap = std::chrono::milliseconds(20))
  : capacity_(capacity), max_gap_(max_gap)
  {
    if (capacity < 2 || max_gap <= std::chrono::milliseconds::zero())
      throw std::invalid_argument("QuaternionBuffer requires capacity >= 2 and max_gap > 0");
  }

  bool push(const Eigen::Quaterniond & q, Clock::time_point timestamp)
  {
    const auto norm = q.norm();
    if (!q.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-6) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_ || (!samples_.empty() && timestamp <= samples_.back().timestamp)) return false;
    samples_.push_back({q.normalized(), timestamp});
    if (samples_.size() > capacity_) samples_.pop_front();
    ready_.notify_all();
    return true;
  }

  bool wait_first(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait_for(lock, timeout, [this] { return closed_ || !samples_.empty(); });
    return !closed_ && !samples_.empty();
  }

  // Never extrapolate, clamp to an old pose, or interpolate across a telemetry gap.
  std::optional<QuaternionSample> at(
    Clock::time_point timestamp,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20))
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool available = ready_.wait_for(lock, timeout, [this, timestamp] {
      return closed_ || (!samples_.empty() && samples_.back().timestamp >= timestamp);
    });
    if (!available || closed_ || samples_.empty() || timestamp < samples_.front().timestamp)
      return std::nullopt;

    const auto upper = std::lower_bound(
      samples_.begin(), samples_.end(), timestamp,
      [](const Entry & entry, Clock::time_point t) { return entry.timestamp < t; });
    if (upper->timestamp == timestamp) return QuaternionSample{upper->q, timestamp, timestamp};

    const auto lower = std::prev(upper);
    const auto span = upper->timestamp - lower->timestamp;
    if (span > max_gap_) return std::nullopt;
    const double k = std::chrono::duration<double>(timestamp - lower->timestamp).count() /
                     std::chrono::duration<double>(span).count();
    return QuaternionSample{
      lower->q.slerp(k, upper->q).normalized(), lower->timestamp, upper->timestamp};
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
  }

  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    ready_.notify_all();
  }

private:
  struct Entry
  {
    Eigen::Quaterniond q;
    Clock::time_point timestamp;
  };

  const std::size_t capacity_;
  const std::chrono::milliseconds max_gap_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Entry> samples_;
  bool closed_ = false;
};
}  // namespace tools

#endif  // TOOLS__QUATERNION_BUFFER_HPP
