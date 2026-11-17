#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <opencv2/dnn.hpp>
#include <string>

#include "tasks/auto_aim/model/armor.hpp"

namespace tools
{
class YamlConfig;
}

namespace auto_aim
{
struct ClassifierConfig
{
  std::string model_path;
  static ClassifierConfig parse(const tools::YamlConfig & yaml);
};

class Classifier
{
public:
  explicit Classifier(const std::string & config_path);
  explicit Classifier(const ClassifierConfig & config);
  ~Classifier();

  void classify(Armor & armor);

private:
  cv::dnn::Net net_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CLASSIFIER_HPP
