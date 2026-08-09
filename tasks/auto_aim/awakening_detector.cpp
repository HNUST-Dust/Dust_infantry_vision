#include "awakening_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace
{
constexpr int k_tup_input_size = 416;
constexpr int k_tup_output_columns = 21;

struct GridAndStride
{
  int x;
  int y;
  int stride;
};

const std::vector<GridAndStride> & tup_grids()
{
  static const std::vector<GridAndStride> grids = [] {
    std::vector<GridAndStride> result;
    for (const int stride : {8, 16, 32}) {
      for (int y = 0; y < k_tup_input_size / stride; ++y) {
        for (int x = 0; x < k_tup_input_size / stride; ++x) {
          result.push_back({x, y, stride});
        }
      }
    }
    return result;
  }();
  return grids;
}

ArmorName armor_name_from_tup(int id)
{
  static constexpr std::array<ArmorName, 8> names{
    ArmorName::sentry, ArmorName::one, ArmorName::two, ArmorName::three,
    ArmorName::four, ArmorName::five, ArmorName::outpost, ArmorName::base};
  return id >= 0 && id < static_cast<int>(names.size()) ? names[id] : ArmorName::not_armor;
}

Color color_from_tup(int id)
{
  static constexpr std::array<Color, 4> colors{Color::blue, Color::red, Color::purple,
                                                 Color::extinguish};
  return id >= 0 && id < static_cast<int>(colors.size()) ? colors[id] : Color::extinguish;
}

double intersection_over_union(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const auto intersection = a & b;
  const auto union_area = a.area() + b.area() - intersection.area();
  return union_area > 0.0 ? intersection.area() / union_area : 0.0;
}

cv::Rect clip_rect(const cv::Rect & rect, const cv::Size & size)
{
  return rect & cv::Rect(0, 0, size.width, size.height);
}

}  // namespace

AwakeningArmorDetector::AwakeningArmorDetector(const std::string & config_path, bool /* debug */)
{
  const auto yaml = YAML::LoadFile(config_path);
  const auto config = yaml["awakening_detector"];
  if (!config) {
    throw std::runtime_error("Missing awakening_detector configuration");
  }
  confidence_threshold_ = config["confidence_threshold"].as<double>();
  nms_threshold_ = config["nms_threshold"].as<double>();
  color_classifier_enabled_ = config["color_classifier_enabled"].as<bool>();
  number_classifier_enabled_ = config["number_classifier_enabled"].as<bool>();
  corner_refinement_enabled_ = config["corner_refinement_enabled"].as<bool>();
  if (corner_refinement_enabled_) {
    traditional_detector_ = std::make_unique<Detector>(config_path, false);
  }
  if (number_classifier_enabled_) {
    number_classifier_ = cv::dnn::readNetFromONNX(config["number_classifier_model_path"].as<std::string>());
    if (number_classifier_.empty()) {
      throw std::runtime_error("Unable to load awakening number classifier");
    }
  }
}

std::list<Armor> AwakeningArmorDetector::postprocess(NetDetector::Result & result, int /* frame_count */)
{
  if (result.output.empty() || result.output.cols != k_tup_output_columns) {
    return {};
  }

  std::vector<Armor> candidates;
  const auto & grids = tup_grids();
  const int rows = std::min(result.output.rows, static_cast<int>(grids.size()));
  candidates.reserve(rows);
  for (int row = 0; row < rows; ++row) {
    const float * values = result.output.ptr<float>(row);
    const float confidence = values[8];
    if (!std::isfinite(confidence) || confidence < confidence_threshold_) continue;

    const auto color_it = std::max_element(values + 9, values + 13);
    const auto class_it = std::max_element(values + 13, values + 21);
    const auto & grid = grids[row];
    const auto decode = [&](int index) {
      return cv::Point2f(
        (values[index] - result.padding.x) / result.scale + result.roi.x,
        (values[index + 1] - result.padding.y) / result.scale + result.roi.y);
    };

    const auto left_top = decode(0);
    const auto left_bottom = decode(2);
    const auto right_bottom = decode(4);
    const auto right_top = decode(6);
    const auto apply_grid = [&](cv::Point2f point) {
      return cv::Point2f(
        point.x + grid.x * grid.stride / result.scale,
        point.y + grid.y * grid.stride / result.scale);
    };
    std::vector<cv::Point2f> points{
      apply_grid(left_top), apply_grid(right_top), apply_grid(right_bottom), apply_grid(left_bottom)};
    const auto box = clip_rect(cv::boundingRect(points), result.source.size());
    if (box.width <= 0 || box.height <= 0) continue;

    Armor armor(0, confidence, box, points);
    armor.class_id = static_cast<int>(class_it - (values + 13));
    armor.confidence = confidence;
    armor.color = color_from_tup(static_cast<int>(color_it - (values + 9)));
    armor.name = armor_name_from_tup(armor.class_id);
    armor.type = (armor.name == ArmorName::one || armor.name == ArmorName::base) ? ArmorType::big
                                                                                    : ArmorType::small;
    armor.points = std::move(points);
    armor.box = box;
    armor.center = (armor.points[0] + armor.points[1] + armor.points[2] + armor.points[3]) / 4.0F;
    armor.center_norm = {armor.center.x / result.source.cols, armor.center.y / result.source.rows};
    if (traditional_detector_) {
      traditional_detector_->detect(armor, result.source);
      armor.center =
        (armor.points[0] + armor.points[1] + armor.points[2] + armor.points[3]) / 4.0F;
      armor.box = clip_rect(cv::boundingRect(armor.points), result.source.size());
      armor.center_norm = {armor.center.x / result.source.cols, armor.center.y / result.source.rows};
    }
    candidates.push_back(std::move(armor));
  }

  std::sort(candidates.begin(), candidates.end(), [](const Armor & a, const Armor & b) {
    return a.confidence > b.confidence;
  });
  std::list<Armor> armors;
  for (auto & candidate : candidates) {
    bool overlaps = false;
    for (const auto & kept : armors) {
      if (intersection_over_union(candidate.box, kept.box) > nms_threshold_) {
        overlaps = true;
        break;
      }
    }
    if (overlaps) continue;
    if (color_classifier_enabled_) classify_color(result.source, candidate);
    if (number_classifier_enabled_) classify_number(result.source, candidate);
    armors.push_back(std::move(candidate));
  }
  return armors;
}

void AwakeningArmorDetector::classify_color(const cv::Mat & image, Armor & armor) const
{
  if (image.empty() || armor.points.size() != 4) return;
  const auto classify_light = [&](const cv::Point2f & top, const cv::Point2f & bottom) {
    const auto center = (top + bottom) * 0.5F;
    const int side = std::max(2, static_cast<int>(cv::norm(top - bottom) / 3.0));
    const auto roi = clip_rect(cv::Rect(
      static_cast<int>(center.x) - side / 2, static_cast<int>(center.y) - side / 2, side, side),
      image.size());
    if (roi.empty()) return Color::extinguish;
    const auto mean = cv::mean(image(roi));
    if (mean[2] - mean[0] > 20.0) return Color::red;
    if (mean[0] - mean[2] > 20.0) return Color::blue;
    return Color::extinguish;
  };
  const auto left = classify_light(armor.points[0], armor.points[3]);
  const auto right = classify_light(armor.points[1], armor.points[2]);
  if (left == right && left != Color::extinguish) armor.color = left;
  if (left != Color::extinguish && right == Color::extinguish) armor.color = left;
  if (right != Color::extinguish && left == Color::extinguish) armor.color = right;
}

void AwakeningArmorDetector::classify_number(const cv::Mat & image, Armor & armor)
{
  if (image.empty() || armor.points.size() != 4) return;
  const std::array<cv::Point2f, 4> source{armor.points[3], armor.points[0], armor.points[1], armor.points[2]};
  const std::array<cv::Point2f, 4> destination{
    cv::Point2f(0, 19), cv::Point2f(0, 8), cv::Point2f(31, 8), cv::Point2f(31, 19)};
  const auto transform = cv::getPerspectiveTransform(source.data(), destination.data());
  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, cv::Size(32, 28));
  cv::cvtColor(warped, warped, cv::COLOR_BGR2GRAY);
  cv::threshold(warped, warped, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  const auto input = warped(cv::Rect(6, 0, 20, 28));
  number_classifier_.setInput(cv::dnn::blobFromImage(input, 1.0 / 255.0));
  const auto output = number_classifier_.forward().reshape(1, 1);
  cv::Point index;
  cv::minMaxLoc(output, nullptr, nullptr, nullptr, &index);
  armor.name = armor_name_from_tup(index.x);
  armor.type = (armor.name == ArmorName::one || armor.name == ArmorName::base) ? ArmorType::big
                                                                                  : ArmorType::small;
}

}  // namespace auto_aim
