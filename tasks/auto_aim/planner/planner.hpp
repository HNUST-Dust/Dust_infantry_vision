#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 30;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  Planner(const std::string & config_path);

  Plan plan(Target target, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world = Eigen::Matrix3d::Identity());
  Plan plan(std::optional<Target> target, double bullet_speed,
            const Eigen::Matrix3d & R_gimbal2world = Eigen::Matrix3d::Identity());

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  double fire_thresh_high_speed_; 
  double fire_thresh_low_speed_;


  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed,
                                  const Eigen::Matrix3d & R_gimbal2world = Eigen::Matrix3d::Identity());
  
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed,
                            const Eigen::Matrix3d & R_gimbal2world = Eigen::Matrix3d::Identity());
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP