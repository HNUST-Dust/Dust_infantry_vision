#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <stdexcept>

namespace tools
{
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
    if (max_size_ == 0) throw std::invalid_argument("ThreadSafeQueue max_size must be positive");
  }

  bool push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (closed_) return false;

    if (queue_.size() >= max_size_) {
      if (PopWhenFull) {
        queue_.pop();
      } else {
        lock.unlock();
        full_handler_();
        return false;
      }
    }

    queue_.push(value);
    not_empty_condition_.notify_all();
    return true;
  }

  bool pop(T & value)
  {
    auto result = wait_pop();
    if (!result) return false;
    value = std::move(*result);
    return true;
  }

  T pop()
  {
    auto result = wait_pop();
    if (!result) throw std::runtime_error("ThreadSafeQueue is closed");
    return std::move(*result);
  }

  T front()
  {
    auto result = wait_front();
    if (!result) throw std::runtime_error("ThreadSafeQueue is closed");
    return std::move(*result);
  }

  std::optional<T> wait_pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_condition_.wait(lock, [this] { return closed_ || !queue_.empty(); });
    if (queue_.empty()) return std::nullopt;

    T value = std::move(queue_.front());
    queue_.pop();
    return value;
  }

  template <typename Rep, typename Period>
  std::optional<T> wait_pop_for(const std::chrono::duration<Rep, Period> & timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!not_empty_condition_.wait_for(
          lock, timeout, [this] { return closed_ || !queue_.empty(); }))
      return std::nullopt;
    if (queue_.empty()) return std::nullopt;

    T value = std::move(queue_.front());
    queue_.pop();
    return value;
  }

  std::optional<T> wait_front()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_condition_.wait(lock, [this] { return closed_ || !queue_.empty(); });
    if (queue_.empty()) return std::nullopt;
    return queue_.front();
  }

  template <typename Rep, typename Period>
  std::optional<T> wait_front_for(const std::chrono::duration<Rep, Period> & timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!not_empty_condition_.wait_for(
          lock, timeout, [this] { return closed_ || !queue_.empty(); }))
      return std::nullopt;
    if (queue_.empty()) return std::nullopt;
    return queue_.front();
  }

  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();
  }

  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  bool full()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.size() >= max_size_;
  }

  bool try_pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      return false;
    }

    value = queue_.front();
    queue_.pop();
    return true;
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
    not_empty_condition_.notify_all();
  }

  void close()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    not_empty_condition_.notify_all();
  }

  bool closed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

private:
  std::queue<T> queue_;
  size_t max_size_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
  bool closed_ = false;
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP
