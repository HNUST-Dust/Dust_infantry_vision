#pragma once

#include <chrono>
#include <opencv2/core/mat.hpp>

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  virtual void stop() = 0;
};
}  // namespace io
