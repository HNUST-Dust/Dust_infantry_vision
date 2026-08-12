#ifndef TOOLS__ORDERED_DELIVERY_HPP
#define TOOLS__ORDERED_DELIVERY_HPP

#include <condition_variable>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace tools
{

template <typename T>
class OrderedDelivery
{
public:
  struct Event
  {
    uint64_t sequence;
    T value;
  };

  explicit OrderedDelivery(std::size_t capacity, uint64_t first_sequence = 0)
  : capacity_(capacity), next_reservation_sequence_(first_sequence),
    next_delivery_sequence_(first_sequence)
  {
    if (capacity_ == 0) throw std::invalid_argument("OrderedDelivery capacity must be positive");
  }

  bool reserve(uint64_t sequence)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return reserve_locked(sequence);
  }

  bool wait_reserve(uint64_t sequence)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    space_available_.wait(lock, [this] { return closed_ || events_.size() < capacity_; });
    return reserve_locked(sequence);
  }

  bool complete(uint64_t sequence, T value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = events_.find(sequence);
    if (it == events_.end() || it->second.has_value()) return false;
    it->second.emplace(std::move(value));
    ready_.notify_all();
    return true;
  }

  bool skip(uint64_t sequence, T value)
  {
    return complete(sequence, std::move(value));
  }

  std::optional<Event> wait_pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait(lock, [this] {
      const auto it = events_.find(next_delivery_sequence_);
      return (it != events_.end() && it->second.has_value()) || (closed_ && events_.empty());
    });

    return pop_ready_locked();
  }

  template <typename Rep, typename Period>
  std::optional<Event> wait_pop_for(const std::chrono::duration<Rep, Period> & timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool ready = ready_.wait_for(lock, timeout, [this] {
      const auto it = events_.find(next_delivery_sequence_);
      return (it != events_.end() && it->second.has_value()) || (closed_ && events_.empty());
    });
    if (!ready) return std::nullopt;
    return pop_ready_locked();
  }

  void close()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    ready_.notify_all();
    space_available_.notify_all();
  }

  bool closed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_.size();
  }

private:
  bool reserve_locked(uint64_t sequence)
  {
    if (closed_ || events_.size() >= capacity_ || sequence != next_reservation_sequence_) {
      return false;
    }
    events_.emplace(sequence, std::nullopt);
    ++next_reservation_sequence_;
    return true;
  }

  std::optional<Event> pop_ready_locked()
  {
    const auto it = events_.find(next_delivery_sequence_);
    if (it == events_.end()) return std::nullopt;
    Event event{it->first, std::move(*it->second)};
    events_.erase(it);
    ++next_delivery_sequence_;
    space_available_.notify_all();
    return event;
  }

  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::condition_variable space_available_;
  std::map<uint64_t, std::optional<T>> events_;
  uint64_t next_reservation_sequence_ = 0;
  uint64_t next_delivery_sequence_ = 0;
  bool closed_ = false;
};

}  // namespace tools

#endif  // TOOLS__ORDERED_DELIVERY_HPP
