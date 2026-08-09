#include <cassert>
#include <cmath>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/yolo.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: awakening_detector_test <config.yaml> <video>\n";
    return 1;
  }

  cv::VideoCapture video(argv[2]);
  if (!video.isOpened()) return 1;
  auto_aim::YOLO detector(argv[1], false);

  int frames = 0;
  std::size_t detections = 0;
  cv::Mat image;
  while (frames < 30 && video.read(image)) {
    const auto armors = detector.detect(image);
    detections += armors.size();
    for (const auto & armor : armors) {
      assert(armor.points.size() == 4);
      for (const auto & point : armor.points) {
        assert(std::isfinite(point.x) && std::isfinite(point.y));
        assert(point.x >= -1.0F && point.x <= image.cols + 1.0F);
        assert(point.y >= -1.0F && point.y <= image.rows + 1.0F);
      }
      assert(armor.confidence >= 0.2);
    }
    ++frames;
  }
  assert(frames > 0);
  std::cout << "awakening detector frames=" << frames << " detections=" << detections << '\n';
  return 0;
}
