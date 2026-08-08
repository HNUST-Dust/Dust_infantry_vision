#ifndef AUTO_AIM__NET_DETECTOR_HPP
#define AUTO_AIM__NET_DETECTOR_HPP

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

namespace auto_aim
{

// Owns OpenVINO preprocessing and the reusable request/input-buffer pool.
class NetDetector
{
public:
  struct Config
  {
    std::string model_path;
    std::string device;
    int input_width;
    int input_height;
    int infer_request_buffer_num = 2;
    bool use_roi = false;
    cv::Rect roi;
  };

  struct Result
  {
    cv::Mat source;
    cv::Mat output;
    double scale = 1.0;
    cv::Rect roi;
    bool has_roi = false;
  };

private:
  struct Impl;

public:
  class Ticket
  {
  public:
    ~Ticket();

    Ticket(const Ticket &) = delete;
    Ticket & operator=(const Ticket &) = delete;

  private:
    friend class NetDetector;

    Ticket(std::shared_ptr<Impl> impl, std::size_t slot_index, cv::Mat source, double scale,
      cv::Rect roi, bool has_roi);

    void release() noexcept;

    std::shared_ptr<Impl> impl_;
    std::size_t slot_index_ = 0;
    cv::Mat source_;
    double scale_ = 1.0;
    cv::Rect roi_;
    bool has_roi_ = false;
    bool completed_ = false;
    bool released_ = false;
  };

  using TicketPtr = std::shared_ptr<Ticket>;

  explicit NetDetector(Config config);

  // Returns no ticket when every request is in flight. Suitable for a low-latency capture loop.
  TicketPtr try_start_async(const cv::Mat & image, bool clone_source);

  // Waits for a reusable request when synchronous callers need a result.
  TicketPtr start(const cv::Mat & image);

  Result wait(const TicketPtr & ticket) const;

  std::size_t request_capacity() const;

private:
  TicketPtr start_impl(const cv::Mat & image, bool clone_source, bool wait_for_slot);

  std::shared_ptr<Impl> impl_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__NET_DETECTOR_HPP
