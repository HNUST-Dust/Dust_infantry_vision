#include "dynamic_roi.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_aim
{
namespace
{

cv::Rect centered_rect(const cv::Point2d & center, double width, double height)
{
  const int x = static_cast<int>(std::floor(center.x - width / 2.0));
  const int y = static_cast<int>(std::floor(center.y - height / 2.0));
  const int right = static_cast<int>(std::ceil(center.x + width / 2.0));
  const int bottom = static_cast<int>(std::ceil(center.y + height / 2.0));
  return {x, y, std::max(1, right - x), std::max(1, bottom - y)};
}

cv::Rect recover_to_full(const cv::Rect & rect, const cv::Rect & full, double recovery)
{
  if (recovery >= 1.0) return full;
  const auto interpolate = [recovery](double from, double to) {
    return from + (to - from) * recovery;
  };
  const int left = static_cast<int>(std::floor(interpolate(rect.x, full.x)));
  const int top = static_cast<int>(std::floor(interpolate(rect.y, full.y)));
  const int right = static_cast<int>(std::ceil(interpolate(rect.br().x, full.br().x)));
  const int bottom = static_cast<int>(std::ceil(interpolate(rect.br().y, full.br().y)));
  return cv::Rect(left, top, right - left, bottom - top) & full;
}

}  // namespace

FocusRois make_focus_rois(
  const cv::Size & image_size, const std::vector<cv::Point2f> & projected_points, bool is_base,
  double seconds_since_observed, const DynamicRoiConfig & config)
{
  const cv::Rect full(0, 0, image_size.width, image_size.height);
  if (image_size.width <= 0 || image_size.height <= 0) return {full, std::nullopt};

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & point : projected_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
    min_x = std::min(min_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_x = std::max(max_x, static_cast<double>(point.x));
    max_y = std::max(max_y, static_cast<double>(point.y));
  }
  if (!std::isfinite(min_x) || max_x <= min_x || max_y <= min_y) {
    return {full, std::nullopt};
  }

  const cv::Point2d center((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);
  const double raw_width = max_x - min_x;
  const double raw_height = max_y - min_y;
  const auto light = centered_rect(center, raw_width * 1.6, raw_height * 1.6) & full;

  const double expand = is_base ? config.base_expand_ratio : config.expand_ratio;
  double net_width = std::max(1.0, raw_width * expand);
  double net_height = std::max(1.0, raw_height * expand);
  const double net_ratio = config.net_ratio > 0.0 ? config.net_ratio : 1.0;
  if (net_width / net_height < net_ratio) {
    net_width = net_height * net_ratio;
  } else {
    net_height = net_width / net_ratio;
  }

  auto net = centered_rect(center, net_width, net_height) & full;
  if (net.empty()) return {full, std::nullopt};
  const double recovery = config.lost_time <= 0.0
                            ? 1.0
                            : std::clamp(seconds_since_observed / config.lost_time, 0.0, 1.0);
  net = recover_to_full(net, full, recovery);
  return {net.empty() ? full : net, light.empty() ? std::nullopt : std::optional<cv::Rect>(light)};
}

}  // namespace auto_aim
