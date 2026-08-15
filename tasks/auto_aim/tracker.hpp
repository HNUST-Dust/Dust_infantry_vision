#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include "armor.hpp"
#include "dynamic_roi.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception
{
struct DetectionResult;
}

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;

  bool dynamic_roi_enabled() const;

  FocusRois focus_rois(
    const cv::Size & image_size, std::chrono::steady_clock::time_point t,
    const Eigen::Matrix3d & R_gimbal2world) const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

private:
  struct ArmorAssociationConfig
  {
    bool enabled = true;
    double center_gate_px = 100.0;
    double corner_gate_px = 140.0;
    double angle_gate_rad = 0.8;
    double perimeter_ratio_gate = 0.6;
    bool image_observation_enabled = true;
    double image_point_sigma_px = 8.0;
  };

  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  std::chrono::steady_clock::time_point last_observed_timestamp_;
  ArmorPriority omni_target_priority_;
  bool dynamic_roi_enabled_ = false;
  DynamicRoiConfig dynamic_roi_config_;
  ArmorAssociationConfig armor_association_config_;
  mutable std::mutex mutex_;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
