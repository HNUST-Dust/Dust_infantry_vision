#ifndef CALIBRATION__FOXGLOVE_CALIBRATION_HPP
#define CALIBRATION__FOXGLOVE_CALIBRATION_HPP

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace calibration
{
struct FoxgloveOptions
{
  std::string host{"127.0.0.1"};
  uint16_t port{8765};
  double fps{10.0};
  int jpeg_quality{80};
};

class FoxgloveCalibration
{
public:
  explicit FoxgloveCalibration(FoxgloveOptions options);
  ~FoxgloveCalibration();

  FoxgloveCalibration(const FoxgloveCalibration &) = delete;
  FoxgloveCalibration & operator=(const FoxgloveCalibration &) = delete;

  bool take_save_request();
  bool quit_requested() const;

  void publish(
    const cv::Mat & preview, bool grid_detected, const Eigen::Vector3d & zyx_degree,
    int saved_count, const std::string & last_event);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace calibration

#endif  // CALIBRATION__FOXGLOVE_CALIBRATION_HPP
