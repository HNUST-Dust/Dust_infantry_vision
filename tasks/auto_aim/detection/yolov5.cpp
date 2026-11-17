#include "yolov5.hpp"

#include <chrono>
#include <fmt/chrono.h>
#include "tools/runtime/yaml_config.hpp"

#include <filesystem>
#include <algorithm>
#include <stdexcept>

#include "tools/image/img_tools.hpp"
#include "tools/diagnostics/logger.hpp"

namespace auto_aim
{
YoloV5Config YoloV5Config::parse(const tools::YamlConfig & yaml)
{
  YoloV5Config config;
  config.detector = DetectorConfig::parse(yaml);
  config.use_traditional = yaml.get<bool>("use_traditional");
  if (yaml.has("yolov5_color_order")) {
    const auto colors = yaml.get<std::vector<std::string>>("yolov5_color_order");
    if (colors.size() != config.color_map.size()) {
      yaml.fail("yolov5_color_order", "yolov5_color_order requires four colors");
    }
    std::array<bool, 4> seen{};
    for (std::size_t i = 0; i < colors.size(); ++i) {
      const auto name = colors[i] == "gray" ? "extinguish" : colors[i];
      const auto found = std::find(COLORS.begin(), COLORS.end(), name);
      if (found == COLORS.end())
        yaml.fail("yolov5_color_order", "Unknown YOLOV5 color: " + colors[i]);
      const auto index = static_cast<std::size_t>(found - COLORS.begin());
      if (seen[index])
        yaml.fail("yolov5_color_order", "yolov5_color_order contains duplicate colors");
      seen[index] = true;
      config.color_map[i] = static_cast<Color>(index);
    }
  }

  return config;
}

YOLOV5::YOLOV5(const std::string & config_path, bool debug)
    : YOLOV5(YoloV5Config::parse(tools::YamlConfig(config_path)), debug)
{
}

YOLOV5::YOLOV5(const YoloV5Config & config, bool debug)
    : debug_(debug), detector_(config.detector, false)
{
  binary_threshold_ = config.detector.threshold;
  min_confidence_ = config.detector.min_confidence;
  use_traditional_ = config.use_traditional;
  color_map_ = config.color_map;
  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);
}

std::list<Armor> YOLOV5::parse(double scale, cv::Mat & output, const cv::Mat & bgr_img,
                               int frame_count, const cv::Point2f & offset, bool has_roi,
                               const cv::Rect & roi, std::optional<cv::Rect> light_roi)
{
  if (output.type() != CV_32F || output.cols != 22) {
    throw std::runtime_error("YOLOV5 requires FP32 output with 22 columns");
  }
  // for each row: xywh + classess
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);

    if (score < score_threshold_)
      continue;

    std::vector<cv::Point2f> armor_key_points;

    //颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     //color
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  //num
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;

    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (int i = 1; i < armor_key_points.size(); i++) {
      if (armor_key_points[i].x < min_x)
        min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x)
        max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y)
        min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y)
        max_y = armor_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (has_roi) {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i],
                          offset);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
    armors.back().color = color_map_[color_ids[i]];
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    // 使用传统方法二次矫正角点
    if (use_traditional_)
      detector_.detect(*it, bgr_img, light_roi);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_)
    draw_detections(bgr_img, armors, frame_count, has_roi, roi);

  return armors;
}

bool YOLOV5::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于神经网络的迭代
  // if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool YOLOV5::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  // 保存异常的图案，用于神经网络的迭代
  // if (!name_ok) save(armor);

  return name_ok;
}

cv::Point2f YOLOV5::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5::draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count,
                             bool has_roi, const cv::Rect & roi) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format("{:.2f} {} {} {}", armor.confidence, COLORS[armor.color],
                            ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (has_roi) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
  cv::imshow("detection", detection);
}

void YOLOV5::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, tmp_img_);
}

double YOLOV5::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::list<Armor> YOLOV5::postprocess(NetDetector::Result & result, int frame_count,
                                     std::optional<cv::Rect> light_roi)
{
  auto output = result.output;
  return parse(result.scale, output, result.source, frame_count,
               cv::Point2f(result.roi.x, result.roi.y), result.has_roi, result.roi, light_roi);
}

}  // namespace auto_aim
