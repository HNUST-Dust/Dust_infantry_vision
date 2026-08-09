#ifndef TOOLS__LATENCY_STATS_HPP
#define TOOLS__LATENCY_STATS_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <vector>

namespace tools
{

struct LatencySummary
{
  std::size_t sample_count;
  double latest_ms;
  double p50_ms;
  double p95_ms;
  double p99_ms;
  double max_ms;
};

class LatencyStats
{
public:
  explicit LatencyStats(std::size_t window_size = 256)
  : window_size_(window_size)
  {
    if (window_size_ == 0) throw std::invalid_argument("LatencyStats window_size must be positive");
  }

  void add(double latency_ms)
  {
    samples_.push_back(latency_ms);
    if (samples_.size() > window_size_) samples_.pop_front();
  }

  bool empty() const { return samples_.empty(); }

  LatencySummary summary() const
  {
    if (samples_.empty()) throw std::runtime_error("LatencyStats has no samples");

    std::vector<double> sorted(samples_.begin(), samples_.end());
    std::sort(sorted.begin(), sorted.end());
    return {
      sorted.size(),
      samples_.back(),
      percentile(sorted, 0.50),
      percentile(sorted, 0.95),
      percentile(sorted, 0.99),
      sorted.back(),
    };
  }

private:
  static double percentile(const std::vector<double> & sorted, double quantile)
  {
    const auto index =
      static_cast<std::size_t>(std::ceil(quantile * sorted.size()) - 1.0);
    return sorted[index];
  }

  std::size_t window_size_;
  std::deque<double> samples_;
};

}  // namespace tools

#endif  // TOOLS__LATENCY_STATS_HPP
