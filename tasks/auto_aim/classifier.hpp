#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <opencv2/dnn.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim
{
class Classifier
{
public:
  explicit Classifier(const std::string & config_path);
  ~Classifier();

  void classify(Armor & armor);

private:
  cv::dnn::Net net_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CLASSIFIER_HPP
