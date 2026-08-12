#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/detector.hpp"

int main(int argc, char * argv[])
{
  assert(argc >= 2);

  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(image, {120, 90, 6, 50}, cv::Scalar(255, 255, 255), cv::FILLED);
  cv::rectangle(image, {180, 90, 6, 50}, cv::Scalar(255, 255, 255), cv::FILLED);

  std::vector<cv::Point2f> points{{123, 90}, {183, 90}, {183, 139}, {123, 139}};
  auto_aim::Armor armor(0, 1.0F, cv::boundingRect(points), points);
  auto_aim::Detector detector(argv[1], false);

  assert(detector.detect(armor, image, cv::Rect(100, 70, 110, 100)));
  for (const auto & point : armor.points) {
    assert(point.x > 110.0F && point.x < 195.0F);
    assert(point.y > 80.0F && point.y < 150.0F);
  }
  assert(armor.points[0].x < armor.points[1].x);
  assert(armor.points[3].x < armor.points[2].x);

  return 0;
}
