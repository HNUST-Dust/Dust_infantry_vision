#ifndef AUTO_AIM__DYNAMIC_ROI_HPP
#define AUTO_AIM__DYNAMIC_ROI_HPP

#include <optional>
#include <opencv2/core.hpp>
#include <vector>

namespace auto_aim
{

struct DynamicRoiConfig
{
  double expand_ratio = 1.4;
  double base_expand_ratio = 3.0;
  double net_ratio = 1.0;
  double lost_time = 0.5;
};

struct FocusRois
{
  cv::Rect net;
  std::optional<cv::Rect> light;
};

FocusRois make_focus_rois(
  const cv::Size & image_size, const std::vector<cv::Point2f> & projected_points, bool is_base,
  double seconds_since_observed, const DynamicRoiConfig & config);

}  // namespace auto_aim

#endif  // AUTO_AIM__DYNAMIC_ROI_HPP
