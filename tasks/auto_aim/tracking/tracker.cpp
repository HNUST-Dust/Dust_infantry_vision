#include "tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "tools/runtime/yaml_config.hpp"

#include <tuple>

#include "tasks/omniperception/detection.hpp"
#include "tools/diagnostics/logger.hpp"
#include "tools/math/math_tools.hpp"

namespace auto_aim
{
TrackerConfig TrackerConfig::parse(const tools::YamlConfig & yaml)
{
  TrackerConfig config;
  config.enemy_color = yaml.get<std::string>("enemy_color") == "red" ? Color::red : Color::blue;
  config.min_detect_count = yaml.get<int>("min_detect_count");
  config.max_temp_lost_count = yaml.get<int>("max_temp_lost_count");
  config.outpost_max_temp_lost_count = yaml.get<int>("outpost_max_temp_lost_count");

  const auto association = yaml.section("armor_association");
  auto & a = config.armor_association;
  a.enabled = association.get("enabled", a.enabled);
  a.center_gate_px = association.get("center_gate_px", a.center_gate_px);
  a.corner_gate_px = association.get("corner_gate_px", a.corner_gate_px);
  a.angle_gate_rad = association.get("angle_gate_rad", a.angle_gate_rad);
  a.perimeter_ratio_gate = association.get("perimeter_ratio_gate", a.perimeter_ratio_gate);
  const auto image = association.section("image_observation");
  a.image_observation_enabled = image.get("enabled", a.image_observation_enabled);
  a.image_point_sigma_px = image.get("point_sigma_px", a.image_point_sigma_px);
  const auto pnp = association.section("pnp_observation");
  a.pnp_point_sigma_px = pnp.get("point_sigma_px", a.pnp_point_sigma_px);
  a.pnp_angle_prior_rad = pnp.get("angle_prior_rad", a.pnp_angle_prior_rad);

  if (yaml.has("dynamic_roi")) {
    const auto roi = yaml.section("dynamic_roi");
    config.dynamic_roi_enabled = roi.get<bool>("enabled");
    config.dynamic_roi.expand_ratio = roi.get<double>("expand_ratio");
    config.dynamic_roi.base_expand_ratio = roi.get<double>("base_expand_ratio");
    config.dynamic_roi.lost_time = roi.get<double>("lost_time");
    config.dynamic_roi.net_ratio = roi.get("net_ratio", 1.0);
  }
  return config;
}

Tracker::Tracker(const std::string & config_path, Solver & solver)
    : Tracker(TrackerConfig::parse(tools::YamlConfig(config_path)), solver)
{
}

Tracker::Tracker(const TrackerConfig & config, Solver & solver)
    : solver_(solver), enemy_color_(config.enemy_color), min_detect_count_(config.min_detect_count),
      max_temp_lost_count_(config.max_temp_lost_count), detect_count_(0), temp_lost_count_(0),
      outpost_max_temp_lost_count_(config.outpost_max_temp_lost_count),
      normal_temp_lost_count_(config.max_temp_lost_count), state_(TrackerState::lost),
      pre_state_(TrackerState::lost), last_timestamp_(std::chrono::steady_clock::now()),
      last_observed_timestamp_(), omni_target_priority_(ArmorPriority::fifth),
      dynamic_roi_enabled_(config.dynamic_roi_enabled), dynamic_roi_config_(config.dynamic_roi),
      armor_association_config_(config.armor_association)
{
}

const char * to_string(TrackerState state)
{
  switch (state) {
  case TrackerState::detecting:
    return "detecting";
  case TrackerState::tracking:
    return "tracking";
  case TrackerState::temp_lost:
    return "temp_lost";
  case TrackerState::switching:
    return "switching";
  default:
    return "lost";
  }
}

TrackerState tracker_state_from(const std::string & name)
{
  if (name == "detecting")
    return TrackerState::detecting;
  if (name == "tracking")
    return TrackerState::tracking;
  if (name == "temp_lost")
    return TrackerState::temp_lost;
  if (name == "switching")
    return TrackerState::switching;
  return TrackerState::lost;
}

std::string Tracker::state() const
{
  return to_string(state_enum());
}

TrackerState Tracker::state_enum() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::chrono::steady_clock::time_point Tracker::last_observed_timestamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_observed_timestamp_;
}

FocusRois Tracker::focus_rois(const cv::Size & image_size, std::chrono::steady_clock::time_point t,
                              const Eigen::Matrix3d & R_gimbal2world) const
{
  const cv::Rect full_frame(0, 0, image_size.width, image_size.height);
  if (!dynamic_roi_enabled_ || image_size.width <= 0 || image_size.height <= 0) {
    return {full_frame, std::nullopt};
  }

  Target target;
  std::chrono::steady_clock::time_point last_observed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == TrackerState::lost ||
        last_observed_timestamp_ == std::chrono::steady_clock::time_point{}) {
      return {full_frame, std::nullopt};
    }
    target = target_;
    last_observed = last_observed_timestamp_;
  }

  target.predict(t);
  std::vector<cv::Point2f> points;
  for (const auto & xyza : target.armor_xyza_list()) {
    const auto projected = solver_.reproject_armor(xyza.head(3), xyza[3], target.armor_type,
                                                   target.name, R_gimbal2world);
    for (const auto & point : projected) {
      if (std::isfinite(point.x) && std::isfinite(point.y))
        points.push_back(point);
    }
  }
  const auto elapsed = std::chrono::duration<double>(t - last_observed).count();
  return make_focus_rois(image_size, points, target.name == ArmorName::base, std::max(0.0, elapsed),
                         dynamic_roi_config_);
}

std::list<Target> Tracker::track(std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
                                 bool use_enemy_color)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_update_real_observation_ = false;
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != TrackerState::lost && dt > 0.3) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = TrackerState::lost;
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  armors.remove_if([this](const auto_aim::Armor & a) {
    return a.name == ArmorName::outpost &&
           solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
             solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == TrackerState::lost) {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  if (last_update_real_observation_) {
    last_observed_timestamp_ = t;
    target_.observed_at_ = t;
  }

  state_machine(found);

  // 发散检测
  if (state_ != TrackerState::lost && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = TrackerState::lost;
    return {};
  }

  // 收敛效果检测：
  if (std::accumulate(target_.ekf().recent_nis_failures.begin(),
                      target_.ekf().recent_nis_failures.end(),
                      0) >= (0.8 * target_.ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = TrackerState::lost;
    return {};
  }

  if (state_ == TrackerState::lost)
    return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>>
Tracker::track(const std::vector<omniperception::DetectionResult> & detection_queue,
               std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
               bool use_enemy_color)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_update_real_observation_ = false;
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != TrackerState::lost && dt > 0.3) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = TrackerState::lost;
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == TrackerState::lost) {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == TrackerState::tracking && !armors.empty() &&
           armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == TrackerState::tracking && !temp_target.armors.empty() &&
           temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = TrackerState::switching;
    switch_target = omniperception::DetectionResult{temp_target.armors, t, temp_target.delta_yaw,
                                                    temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == TrackerState::switching) {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == TrackerState::detecting && pre_state_ == TrackerState::switching) {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  if (last_update_real_observation_) {
    last_observed_timestamp_ = t;
    target_.observed_at_ = t;
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != TrackerState::lost && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = TrackerState::lost;
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == TrackerState::lost)
    return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == TrackerState::lost) {
    if (!found)
      return;

    state_ = TrackerState::detecting;
    detect_count_ = 1;
  }

  else if (state_ == TrackerState::detecting) {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_)
        state_ = TrackerState::tracking;
    } else {
      detect_count_ = 0;
      state_ = TrackerState::lost;
    }
  }

  else if (state_ == TrackerState::tracking) {
    if (found)
      return;

    temp_lost_count_ = 1;
    state_ = TrackerState::temp_lost;
  }

  else if (state_ == TrackerState::switching) {
    if (found) {
      state_ = TrackerState::detecting;
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200)
        state_ = TrackerState::lost;
    }
  }

  else if (state_ == TrackerState::temp_lost) {
    if (found) {
      state_ = TrackerState::tracking;
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_)
        state_ = TrackerState::lost;
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty())
    return false;

  auto & armor = armors.front();
  solver_.solve(armor);

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 0.5, 0.5, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 1}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
    target_.set_initial_omega(0.5);  // 前哨站典型转速，避免从0开始收敛
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  target_.generation_ = ++target_generation_;
  last_update_real_observation_ = true;
  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  int found_count = 0;
  double min_x = 1e10;  // 画面最左侧
  for (const auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type)
      continue;
    found_count++;
    min_x = armor.center.x < min_x ? armor.center.x : min_x;
  }

  if (found_count == 0) {
    // 前哨站已收敛：短时间检测间隙用 EKF 预测顶着，避免 temp_lost 抖动
    if (target_.name == ArmorName::outpost && target_.convergened() &&
        target_.virtual_update_count() < 10) {
      int virtual_update_count = target_.record_virtual_update();
      tools::logger()->debug("[Target] outpost gap frame {}, predict only", virtual_update_count);
      return true;
    }
    return false;
  }

  if (!armor_association_config_.enabled) {
    bool updated = false;
    for (auto & armor : armors) {
      if (armor.name != target_.name || armor.type != target_.armor_type)
        continue;
      solver_.solve(armor);
      // 该分支（armor_association.enabled=false）保持只走 PnP 观测，
      // 但同样使用 PnP 路径自己的噪声，而不是被 uv 观测的开关连带关掉。
      target_.update(armor, -1, &solver_, 0.0, armor_association_config_.pnp_point_sigma_px,
                     armor_association_config_.pnp_angle_prior_rad);
      updated = true;
    }
    last_update_real_observation_ = updated;
    return updated;
  }

  struct Match
  {
    Armor * armor;
    int predicted_id;
    double cost;
  };
  constexpr double k_invalid_cost = std::numeric_limits<double>::infinity();
  const auto quad_cost = [&](const std::vector<cv::Point2f> & predicted,
                             const std::vector<cv::Point2f> & measured) {
    if (predicted.size() != 4 || measured.size() != 4)
      return k_invalid_cost;

    cv::Point2f predicted_center(0, 0), measured_center(0, 0);
    double predicted_perimeter = 0;
    double measured_perimeter = 0;
    double corner_error = 0;
    double angle_error = 0;
    for (int i = 0; i < 4; ++i) {
      const auto & p = predicted[i];
      const auto & m = measured[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(m.x) || !std::isfinite(m.y))
        return k_invalid_cost;
      predicted_center += p;
      measured_center += m;
      corner_error += cv::norm(p - m);
      const int next = (i + 1) % 4;
      const auto predicted_edge = predicted[next] - p;
      const auto measured_edge = measured[next] - m;
      predicted_perimeter += cv::norm(predicted_edge);
      measured_perimeter += cv::norm(measured_edge);
      angle_error += std::abs(tools::limit_rad(std::atan2(predicted_edge.y, predicted_edge.x) -
                                               std::atan2(measured_edge.y, measured_edge.x)));
    }
    if (predicted_perimeter <= std::numeric_limits<double>::epsilon())
      return k_invalid_cost;

    predicted_center *= 0.25F;
    measured_center *= 0.25F;
    corner_error *= 0.25;
    const double center_error = cv::norm(predicted_center - measured_center);
    const double perimeter_ratio_error =
      std::abs(predicted_perimeter - measured_perimeter) / predicted_perimeter;
    if (center_error > armor_association_config_.center_gate_px ||
        corner_error > armor_association_config_.corner_gate_px ||
        angle_error > armor_association_config_.angle_gate_rad ||
        perimeter_ratio_error > armor_association_config_.perimeter_ratio_gate)
      return k_invalid_cost;

    return center_error / armor_association_config_.center_gate_px +
           corner_error / armor_association_config_.corner_gate_px +
           angle_error / armor_association_config_.angle_gate_rad +
           perimeter_ratio_error / armor_association_config_.perimeter_ratio_gate;
  };

  std::vector<Armor *> candidates;
  for (auto & armor : armors) {
    if (armor.name == target_.name && armor.type == target_.armor_type) {
      candidates.push_back(&armor);
    }
  }

  const auto predicted_armors = target_.armor_xyza_list();
  std::vector<std::vector<double>> costs(
    candidates.size(), std::vector<double>(predicted_armors.size(), k_invalid_cost));
  for (std::size_t obs = 0; obs < candidates.size(); ++obs) {
    for (std::size_t id = 0; id < predicted_armors.size(); ++id) {
      const auto & xyza = predicted_armors[id];
      const auto projected =
        solver_.reproject_armor(xyza.head(3), xyza[3], target_.armor_type, target_.name);
      costs[obs][id] = quad_cost(projected, candidates[obs]->points);
    }
  }

  std::vector<bool> used_observation(candidates.size(), false);
  std::vector<bool> used_prediction(predicted_armors.size(), false);
  std::vector<Match> matches;
  while (true) {
    Match best{nullptr, -1, k_invalid_cost};
    for (std::size_t obs = 0; obs < candidates.size(); ++obs) {
      if (used_observation[obs])
        continue;
      for (std::size_t id = 0; id < predicted_armors.size(); ++id) {
        if (!used_prediction[id] && costs[obs][id] < best.cost) {
          best = {candidates[obs], static_cast<int>(id), costs[obs][id]};
        }
      }
    }
    if (!best.armor)
      break;
    const auto obs = static_cast<std::size_t>(
      std::find(candidates.begin(), candidates.end(), best.armor) - candidates.begin());
    used_observation[obs] = true;
    used_prediction[best.predicted_id] = true;
    matches.push_back(best);
  }

  for (const auto & match : matches) {
    solver_.solve(*match.armor);
    // 两条观测路径的噪声互不复用：image_point_sigma_px 只门控 8 维像素观测，
    // PnP 路径的传播 R 由 pnp_point_sigma_px 决定，关掉 uv 观测不再连带停用传播 R。
    target_.update(*match.armor, match.predicted_id, &solver_,
                   armor_association_config_.image_observation_enabled
                     ? armor_association_config_.image_point_sigma_px
                     : 0.0,
                   armor_association_config_.pnp_point_sigma_px,
                   armor_association_config_.pnp_angle_prior_rad);
  }
  last_update_real_observation_ = !matches.empty();
  return last_update_real_observation_;
}

}  // namespace auto_aim
