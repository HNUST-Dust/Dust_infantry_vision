#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <limits>

#include "tasks/auto_aim/dynamic_roi.hpp"

namespace
{

void assert_rect(const cv::Rect & actual, const cv::Rect & expected)
{
  assert(actual == expected);
}

}  // namespace

int main()
{
  const cv::Size image_size(400, 300);
  const std::vector<cv::Point2f> points{{100, 100}, {200, 100}, {200, 150}, {100, 150}};
  auto_aim::DynamicRoiConfig config{2.0, 3.0, 1.0, 0.5};

  const auto normal = auto_aim::make_focus_rois(image_size, points, false, 0.0, config);
  assert_rect(normal.net, {50, 25, 200, 200});
  assert(normal.light);
  assert_rect(*normal.light, {70, 85, 160, 80});

  const auto base = auto_aim::make_focus_rois(image_size, points, true, 0.0, config);
  assert_rect(base.net, {0, 0, 300, 275});

  config.net_ratio = 2.0;
  const auto wide = auto_aim::make_focus_rois(image_size, points, false, 0.0, config);
  assert_rect(wide.net, {50, 75, 200, 100});

  const std::vector<cv::Point2f> edge{{-20, -10}, {20, -10}, {20, 10}, {-20, 10}};
  const auto clipped = auto_aim::make_focus_rois(image_size, edge, false, 0.0, config);
  assert(clipped.net.x == 0 && clipped.net.y == 0);
  assert(clipped.net.width > 0 && clipped.net.height > 0);
  assert(clipped.light && clipped.light->x == 0 && clipped.light->y == 0);

  const std::vector<cv::Point2f> invalid{{std::numeric_limits<float>::quiet_NaN(), 20}};
  const auto fallback = auto_aim::make_focus_rois(image_size, invalid, false, 0.0, config);
  assert_rect(fallback.net, {0, 0, 400, 300});
  assert(!fallback.light);

  const auto recovered = auto_aim::make_focus_rois(image_size, points, false, 0.5, config);
  assert_rect(recovered.net, {0, 0, 400, 300});
  assert(recovered.light);

  return 0;
}
