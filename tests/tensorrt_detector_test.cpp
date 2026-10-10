#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_aim/net_detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolos/yolov5.hpp"

namespace
{
using json = nlohmann::json;
using Net = auto_aim::NetDetector;

void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

template <typename Function>
void require_invalid(Function function)
{
  try {
    function();
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error("Expected invalid_argument");
}

double sigmoid(double x)
{
  return x >= 0 ? 1.0 / (1.0 + std::exp(-x)) : std::exp(x) / (1.0 + std::exp(x));
}

int argmax(const cv::Mat & row, int first, int last)
{
  cv::Point index;
  cv::minMaxLoc(row.colRange(first, last), nullptr, nullptr, nullptr, &index);
  return index.x;
}

std::string write_config(YAML::Node yaml, const std::filesystem::path & path, bool traditional)
{
  yaml["use_traditional"] = traditional;
  std::ofstream file(path);
  file << yaml;
  require(file.good(), "Cannot write test configuration");
  return path.string();
}

void pool_test(Net & net, const cv::Mat & image)
{
  require(!net.start(cv::Mat()), "Empty image should not acquire a slot");
  require_invalid([&] { net.start(cv::Mat(10, 10, CV_8UC1)); });
  auto first = net.try_start_async(image, true);
  auto second = net.try_start_async(image, true);
  require(first && second && !net.try_start_async(image, true), "Pool capacity mismatch");
  bool bad_roi = false;
  try {
    net.try_start_async(image, true, cv::Rect(-1, 0, image.cols, image.rows));
  } catch (const std::runtime_error &) {
    bad_roi = true;
  }
  require(bad_roi, "Invalid ROI must be checked even when the pool is full");
  auto result = net.wait(first);
  require(result.output.rows == 25200 && result.output.cols == 22, "Unexpected 0526 output shape");
  auto saved = result.output.clone();
  auto blocked = std::async(std::launch::async, [&] { return net.start(image); });
  require(blocked.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout,
    "Synchronous start did not wait for a slot");
  first.reset();
  auto third = blocked.get();
  second.reset();  // Dropping an unwaited ticket must drain its stream before reuse.
  third.reset();
  auto fourth = net.start(cv::Mat(image.size(), CV_8UC3, cv::Scalar(0, 0, 0)));
  net.wait(fourth);
  require(cv::norm(saved, result.output, cv::NORM_INF) == 0, "Result changed after slot reuse");
  fourth.reset();
  require_invalid([&] { net.wait(nullptr); });

  const cv::Rect roi(image.cols / 4, image.rows / 4, image.cols / 2, image.rows / 2);
  auto cropped = net.start(image, roi);
  auto crop_result = net.wait(cropped);
  auto direct = net.start(image(roi));
  auto direct_result = net.wait(direct);
  require(crop_result.has_roi && crop_result.roi == roi, "ROI metadata was lost");
  require(cv::norm(crop_result.output, direct_result.output, cv::NORM_INF) == 0,
    "ROI preprocessing differs from a direct crop");
}

void parser_test(auto_aim::YOLOV5 & parser)
{
  Net::Result result;
  result.source = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  result.roi = {10, 20, 400, 400};
  result.has_roi = true;
  result.output = cv::Mat(1, 22, CV_32F, cv::Scalar(-10));
  const float points[] = {100, 100, 100, 150, 300, 150, 300, 100};
  std::copy(std::begin(points), std::end(points), result.output.ptr<float>());
  result.output.at<float>(0, 8) = 10;
  result.output.at<float>(0, 12) = 10;  // Purple.
  result.output.at<float>(0, 21) = 10;  // Bb: big base armor.
  auto armors = parser.postprocess(result, -1);
  require(armors.size() == 1, "Bb was discarded by the V5 parser");
  const auto & armor = armors.front();
  require(armor.color == auto_aim::purple && armor.name == auto_aim::base &&
    armor.type == auto_aim::big, "Incorrect purple/Bb mapping");
  require(armor.points[0] == cv::Point2f(110, 120) &&
    armor.points[1] == cv::Point2f(310, 120) &&
    armor.points[2] == cv::Point2f(310, 170) &&
    armor.points[3] == cv::Point2f(110, 170), "Incorrect corner order or ROI offset");
}

json compare_raw(const cv::Mat & reference, const cv::Mat & actual)
{
  require(reference.size() == actual.size() && actual.type() == CV_32F, "Output shape/type mismatch");
  require(cv::checkRange(reference) && cv::checkRange(actual), "Nonfinite network output");
  double confidence_delta = 0, keypoint_delta = 0;
  int candidates = 0, color_mismatches = 0, class_mismatches = 0;
  for (int row = 0; row < reference.rows; ++row) {
    const auto a = reference.row(row), b = actual.row(row);
    if (sigmoid(a.at<float>(8)) < 0.7) continue;
    ++candidates;
    confidence_delta = std::max(confidence_delta,
      std::abs(sigmoid(a.at<float>(8)) - sigmoid(b.at<float>(8))));
    keypoint_delta = std::max(keypoint_delta, cv::norm(a.colRange(0, 8), b.colRange(0, 8), cv::NORM_INF));
    color_mismatches += argmax(a, 9, 13) != argmax(b, 9, 13);
    class_mismatches += argmax(a, 13, 22) != argmax(b, 13, 22);
  }
  return {{"rmse", cv::norm(reference, actual, cv::NORM_L2) / std::sqrt(reference.total())},
    {"max_abs", cv::norm(reference, actual, cv::NORM_INF)}, {"candidates", candidates},
    {"candidate_confidence_max_abs", confidence_delta}, {"candidate_keypoint_max_px", keypoint_delta},
    {"candidate_color_mismatches", color_mismatches}, {"candidate_class_mismatches", class_mismatches}};
}

json compare_armors(std::list<auto_aim::Armor> reference, std::list<auto_aim::Armor> actual,
  auto_aim::Solver & solver)
{
  json report = {{"reference_count", reference.size()}, {"tensorrt_count", actual.size()},
    {"matched", json::array()}, {"passed", reference.size() == actual.size()}};
  for (auto & armor : reference) {
    auto nearest = actual.end();
    double distance = 20.0;
    for (auto it = actual.begin(); it != actual.end(); ++it) {
      const double candidate_distance = cv::norm(armor.center - it->center);
      if (candidate_distance < distance) {
        distance = candidate_distance;
        nearest = it;
      }
    }
    if (nearest == actual.end()) {
      report["passed"] = false;
      continue;
    }
    solver.solve(armor);
    solver.solve(*nearest);
    double corner_delta = 0;
    for (int i = 0; i < 4; ++i) {
      corner_delta = std::max(corner_delta, cv::norm(armor.points[i] - nearest->points[i]));
    }
    const double confidence_delta = std::abs(armor.confidence - nearest->confidence);
    const double translation_delta = (armor.t_armor2camera - nearest->t_armor2camera).norm();
    const double rotation_delta = std::acos(std::clamp(
      ((armor.R_armor2camera.transpose() * nearest->R_armor2camera).trace() - 1.0) / 2.0, -1.0, 1.0));
    const double optimized_yaw_delta = std::abs(std::remainder(
      armor.ypr_in_world[0] - nearest->ypr_in_world[0], 2.0 * CV_PI));
    const double aim_delta = (armor.ypd_in_world.head<2>() - nearest->ypd_in_world.head<2>()).norm();
    const bool labels_match = armor.color == nearest->color && armor.name == nearest->name &&
      armor.type == nearest->type;
    const bool passed = labels_match && confidence_delta < 0.02 && corner_delta < 3.0 &&
      std::isfinite(translation_delta) && std::isfinite(rotation_delta) &&
      translation_delta < std::max(0.03, armor.t_armor2camera.norm() * 0.05) && rotation_delta < 0.15 &&
      std::isfinite(optimized_yaw_delta) && optimized_yaw_delta < 0.15 &&
      std::isfinite(aim_delta) && aim_delta < 0.01;
    report["matched"].push_back({{"color", auto_aim::COLORS[armor.color]},
      {"class", auto_aim::ARMOR_NAMES[armor.name]}, {"labels_match", labels_match},
      {"confidence_max_abs", confidence_delta}, {"corner_max_px", corner_delta},
      {"pnp_translation_m", translation_delta}, {"pnp_rotation_rad", rotation_delta},
      {"optimized_yaw_rad", optimized_yaw_delta}, {"aim_angles_rad", aim_delta}, {"passed", passed}});
    if (!passed) report["passed"] = false;
    actual.erase(nearest);
  }
  return report;
}

json color_evidence(const cv::Mat & output, const std::list<auto_aim::Armor> & legacy, double scale)
{
  json evidence = json::array();
  for (const auto & armor : legacy) {
    int nearest = -1;
    double distance = 20;
    for (int row = 0; row < output.rows; ++row) {
      if (sigmoid(output.at<float>(row, 8)) < 0.8) continue;
      cv::Point2f center;
      for (int i = 0; i < 4; ++i) {
        center += cv::Point2f(output.at<float>(row, i * 2), output.at<float>(row, i * 2 + 1));
      }
      center *= 1.0 / (4 * scale);
      const auto candidate = cv::norm(center - armor.center);
      if (candidate < distance) {
        distance = candidate;
        nearest = row;
      }
    }
    if (nearest >= 0) evidence.push_back({{"dust_color", auto_aim::COLORS[armor.color]},
      {"0526_color_index", argmax(output.row(nearest), 9, 13)}, {"center_distance_px", distance}});
  }
  return evidence;
}
}  // namespace

int main(int argc, char * argv[])
{
  if (argc < 5) {
    std::cerr << "Usage: tensorrt_detector_test <config.yaml> <0526.onnx> <engine> <image-or-video> "
              << "[samples=30] [stride=20] [output-dir=build-ninja/tensorrt-validation]\n";
    return 1;
  }
  try {
    const int samples = argc > 5 ? std::stoi(argv[5]) : 30;
    const int stride = argc > 6 ? std::stoi(argv[6]) : 20;
    require(samples > 0 && stride > 0, "Samples and stride must be positive");
    const std::filesystem::path output_dir = argc > 7 ? argv[7] : "build-ninja/tensorrt-validation";
    std::filesystem::create_directories(output_dir);
    auto yaml = YAML::LoadFile(argv[1]);
    const auto plain_config = write_config(YAML::Clone(yaml), output_dir / "plain.yaml", false);
    const auto corrected_config = write_config(YAML::Clone(yaml), output_dir / "corrected.yaml", true);
    auto legacy_yaml = YAML::Clone(yaml);
    legacy_yaml.remove("yolov5_color_order");
    const auto legacy_config = write_config(legacy_yaml, output_dir / "legacy.yaml", false);
    auto_aim::YOLOV5 plain(plain_config, false), corrected(corrected_config, false), legacy_parser(legacy_config, false);
    auto_aim::Solver solver(argv[1]);
    parser_test(plain);

    Net::Config config;
    config.model_path = argv[2];
    config.device = "CPU";
    config.input_width = config.input_height = 640;
    config.infer_request_buffer_num = 2;
    Net reference(config);
    config.model_path = argv[3];
    config.backend = "tensorrt";
    Net actual(config);
    config.model_path = "assets/yolov5.xml";
    config.backend = "openvino";
    Net legacy(config);

    cv::Mat still = cv::imread(argv[4]);
    cv::VideoCapture video;
    if (still.empty()) require(video.open(argv[4]), "Cannot open image/video");
    json report = {{"onnx", argv[2]}, {"engine", argv[3]}, {"frames", json::array()},
      {"tolerances", {{"confidence", 0.02}, {"corners_px", 3}, {"pnp_translation_m", 0.03},
        {"pnp_translation_relative", 0.05}, {"pnp_rotation_rad", 0.15},
        {"optimized_yaw_rad", 0.15}, {"aim_angles_rad", 0.01}}}, {"passed", true}};
    int detections = 0;
    for (int frame = 0, sample = 0; sample < samples; ++frame) {
      cv::Mat image;
      if (!still.empty()) {
        image = still;
        if (sample > 0) break;
      } else if (!video.read(image)) break;
      if (frame % stride != 0) continue;
      if (sample == 0) {
        pool_test(actual, image);
        auto foreign = reference.start(image);
        require_invalid([&] { actual.wait(foreign); });
        auto orphan_config = config;
        orphan_config.model_path = argv[3];
        orphan_config.backend = "tensorrt";
        auto orphan_net = std::make_unique<Net>(orphan_config);
        auto orphan = orphan_net->start(image);
        orphan_net.reset();
        orphan.reset();
      }
      auto a_ticket = reference.start(image), b_ticket = actual.start(image);
      auto a = reference.wait(a_ticket), b = actual.wait(b_ticket);
      auto a_plain = plain.postprocess(a, frame), b_plain = plain.postprocess(b, frame);
      detections += static_cast<int>(a_plain.size());
      const auto legacy_ticket = legacy.start(image);
      auto legacy_result = legacy.wait(legacy_ticket);
      const auto legacy_armors = legacy_parser.postprocess(legacy_result, frame);
      json row = {{"frame", frame}, {"raw", compare_raw(a.output, b.output)},
        {"network", compare_armors(a_plain, b_plain, solver)},
        {"traditional", compare_armors(corrected.postprocess(a, frame), corrected.postprocess(b, frame), solver)},
        {"color_evidence", color_evidence(a.output, legacy_armors, a.scale)}};
      const cv::Rect roi(image.cols / 4, image.rows / 4, image.cols / 2, image.rows / 2);
      a_ticket.reset();
      b_ticket.reset();
      a_ticket = reference.start(image, roi);
      b_ticket = actual.start(image, roi);
      auto ar = reference.wait(a_ticket), br = actual.wait(b_ticket);
      row["roi"] = compare_armors(plain.postprocess(ar, frame), plain.postprocess(br, frame), solver);
      if (!row["network"]["passed"].get<bool>() || !row["traditional"]["passed"].get<bool>() ||
          !row["roi"]["passed"].get<bool>()) report["passed"] = false;
      if (!a_plain.empty() && sample % 5 == 0) {
        auto annotated = image.clone();
        for (const auto & armor : b_plain) {
          for (int i = 0; i < 4; ++i) cv::line(annotated, armor.points[i], armor.points[(i + 1) % 4], {0, 255, 0}, 2);
          cv::putText(annotated, auto_aim::COLORS[armor.color] + " " + auto_aim::ARMOR_NAMES[armor.name],
            armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
        }
        cv::imwrite((output_dir / ("frame-" + std::to_string(frame) + ".png")).string(), annotated);
      }
      report["frames"].push_back(row);
      std::cout << "frame " << frame << ": OV=" << a_plain.size() << " TRT=" << b_plain.size()
                << " raw RMSE=" << row["raw"]["rmse"] << " colors=" << row["color_evidence"].dump() << '\n';
      ++sample;
    }
    require(!report["frames"].empty(), "No frames were compared");
    report["reference_detections"] = detections;
    report["pool_and_parser_passed"] = true;
    if (detections == 0) report["passed"] = false;
    std::ofstream report_file(output_dir / "report.json");
    report_file << report.dump(2) << '\n';
    require(report_file.good(), "Cannot write comparison report");
    std::cout << "Report: " << output_dir / "report.json" << ", passed=" << report["passed"] << '\n';
    return report["passed"].get<bool>() ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
