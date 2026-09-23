#include "vision_overlay.hpp"

#include <fmt/core.h>

#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "tools/img_tools.hpp"

namespace visualization
{
namespace
{
cv::Scalar state_color(const std::string & state)
{
  if (state == "tracking") return {0, 255, 0};
  if (state == "detecting") return {0, 255, 255};
  if (state == "temp_lost") return {0, 165, 255};
  return {0, 0, 255};  // lost / switching
}
}  // namespace

void draw_vision_overlay(
  cv::Mat & image, const VisionOverlayInput & input, const auto_aim::Solver & solver)
{
  cv::rectangle(image, input.net_roi, {0, 255, 0}, 2);
  if (input.light_roi) cv::rectangle(image, *input.light_roi, {0, 255, 255}, 2);

  for (const auto & armor : input.armors) tools::draw_points(image, armor.points, {255, 255, 0});

  if (input.target) {
    const auto & target = *input.target;
    for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
      auto image_points =
        solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
      tools::draw_points(image, image_points, {0, 255, 0});
    }
    auto aim_points = solver.reproject_armor(
      input.aim_xyza.head(3), input.aim_xyza[3], target.armor_type, target.name);
    tools::draw_points(image, aim_points, {0, 0, 255});
  }

  cv::putText(
    image, fmt::format("State: {}", input.tracker_state), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
    state_color(input.tracker_state), 2);
  cv::line(image, cv::Point(700, 540), cv::Point(740, 540), cv::Scalar(255, 255, 255));
  cv::line(image, cv::Point(720, 520), cv::Point(720, 560), cv::Scalar(255, 255, 255));
}

}  // namespace visualization
