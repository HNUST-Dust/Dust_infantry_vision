#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <list>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "armor.hpp"

namespace auto_aim
{
class YOLOBase
{
public:
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

inline std::vector<YOLO> create_yolo11s(
  const std::string & config_path, int numebr, bool debug)
{
  std::vector<YOLO> yolo11s;
  for (int i = 0; i < numebr; i++) {
    yolo11s.push_back(YOLO(config_path, debug));
  }
  return yolo11s;
}

inline std::vector<YOLO> create_yolov8s(
  const std::string & config_path, int numebr, bool debug)
{
  std::vector<YOLO> yolov8s;
  for (int i = 0; i < numebr; i++) {
    yolov8s.push_back(YOLO(config_path, debug));
  }
  return yolov8s;
}

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
