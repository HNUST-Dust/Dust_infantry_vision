#ifndef AUTO_AIM__YOLOV8_HPP
#define AUTO_AIM__YOLOV8_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/classifier.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{

class YOLOV8 : public YOLOBase
{
public:
  YOLOV8(const std::string & config_path, bool debug);

  std::list<Armor> postprocess(NetDetector::Result & result, int frame_count) override;

private:
  Classifier classifier_;
  Detector detector_;

  std::string save_path_, debug_path_;
  bool debug_;

  const int class_num_ = 2;
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  double min_confidence_, binary_threshold_;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;
  ArmorType get_type(const Armor & armor);
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count,
    const cv::Point2f & offset, bool has_roi, const cv::Rect & roi);

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count,
    bool has_roi, const cv::Rect & roi) const;
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
};

}  // namespace auto_aim

#endif  // TOOLS__YOLOV8_HPP
