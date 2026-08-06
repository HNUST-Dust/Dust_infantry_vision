#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <memory>
#include <opencv2/dnn.hpp>
#include <string>

#include "armor.hpp"

namespace ov
{
class Core;
class CompiledModel;
}  // namespace ov

namespace auto_aim
{
class Classifier
{
public:
  explicit Classifier(const std::string & config_path);
  ~Classifier();

  void classify(Armor & armor);

  void ovclassify(Armor & armor);

private:
  cv::dnn::Net net_;
  std::unique_ptr<ov::Core> core_;
  std::unique_ptr<ov::CompiledModel> compiled_model_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CLASSIFIER_HPP
