#ifndef AUTO_AIM__AWAKENING_DETECTOR_HPP
#define AUTO_AIM__AWAKENING_DETECTOR_HPP

#include <list>
#include <opencv2/dnn.hpp>
#include <string>

#include "armor.hpp"
#include "detector.hpp"
#include "yolo.hpp"

namespace auto_aim
{

// Adapter for awakening's opt-1208-001 TUP output layout.
class AwakeningArmorDetector : public YOLOBase
{
public:
  AwakeningArmorDetector(const std::string & config_path, bool debug);

  std::list<Armor> postprocess(NetDetector::Result & result, int frame_count) override;

private:
  double confidence_threshold_ = 0.2;
  double nms_threshold_ = 0.35;
  bool color_classifier_enabled_ = true;
  bool number_classifier_enabled_ = false;
  bool corner_refinement_enabled_ = false;
  cv::dnn::Net number_classifier_;
  std::unique_ptr<Detector> traditional_detector_;

  void classify_color(const cv::Mat & image, Armor & armor) const;
  void classify_number(const cv::Mat & image, Armor & armor);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AWAKENING_DETECTOR_HPP
