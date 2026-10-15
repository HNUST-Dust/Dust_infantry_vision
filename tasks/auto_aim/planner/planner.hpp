#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <array>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "mpc_axis.hpp"

namespace auto_aim
{
using Trajectory = Eigen::Matrix<double, 4, HORIZON>;

enum class PlanStatus {
  not_run, ok, no_target, empty_armor, feedback_invalid, timestamp_missing,
  feedback_future, feedback_stale, state_nonfinite, rotation_invalid,
  state_frame_mismatch, pitch_singular, propagated_state_invalid,
  trajectory_unavailable, mpc_failed, exception
};
const char *to_string(PlanStatus status);

struct PlanningState {
  double yaw = 0.0;
  double yaw_vel = 0.0;
  double pitch = 0.0;
  double pitch_vel = 0.0;
  Eigen::Matrix3d R_gimbal2world = Eigen::Matrix3d::Identity();
  std::chrono::steady_clock::time_point timestamp{};
  bool valid = false;
  bool rates_measured = false;
};

struct AimDebugSnapshot {
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
  int armor_id = -1;
  bool candidate_available = false;
  bool selection_valid = false;
  bool ballistic_converged = false;
  bool display_valid = false;
};

struct Plan {
  PlanStatus status = PlanStatus::not_run;
  std::string error_message;
  AimDebugSnapshot aim_debug;
  bool control = false;
  bool fire = false;
  // Relative joint increments at the feedback used for this representation.
  float target_yaw = 0;
  float target_pitch = 0;
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float pitch_acc = 0;
  double absolute_yaw = 0.0;
  double absolute_pitch = 0.0;
  double absolute_target_yaw = 0.0;
  double absolute_target_pitch = 0.0;
  double fire_tolerance = 0.0;
  double feedback_age_ms = 0.0;
  PlanningState initial_state;
  MpcDiagnostics yaw_mpc;
  MpcDiagnostics pitch_mpc;
  std::chrono::steady_clock::time_point planned_at{};
  std::chrono::steady_clock::time_point valid_at{};

  int mode() const { return control ? (fire ? 2 : 1) : 0; }
  Plan relative_to(double measured_yaw, double measured_pitch) const;
  bool aim_visible_at(std::chrono::steady_clock::time_point now) const;
};

struct PlanningTrace {
  bool reference_available = false;
  Trajectory reference = Trajectory::Zero();
  PlanningState received_state;
  MpcConfig yaw_config;
  MpcConfig pitch_config;
  double bullet_speed = 0.0;
  double yaw_offset = 0.0;
  double pitch_offset = 0.0;
  bool target_available = false;
  std::array<double, 11> target_state{};
};

struct AimPoint {
  bool valid = false;
  int id = 0;
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
};

class Planner
{
public:
  explicit Planner(const std::string &config_path);
  ~Planner() = default;
  Planner(const Planner &) = delete;
  Planner &operator=(const Planner &) = delete;

  Plan plan(std::optional<Target> target, double bullet_speed, const PlanningState &state,
            PlanningTrace *trace = nullptr);
  Plan plan_at(std::optional<Target> target, std::chrono::steady_clock::time_point now,
               double bullet_speed, const PlanningState &state, PlanningTrace *trace = nullptr);

private:
  double yaw_offset_;
  double pitch_offset_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  double fire_thresh_high_speed_, fire_thresh_low_speed_;
  double comming_angle_, leaving_angle_;
  double first_tolerance_, second_tolerance_, judge_distance_;
  bool auto_fire_;
  double max_state_age_ = 0.02;
  MpcAxis yaw_solver_;
  MpcAxis pitch_solver_;
  int lock_id_ = -1;
  bool have_last_command_ = false;
  double last_target_yaw_ = 0.0;
  std::chrono::steady_clock::time_point last_plan_at_{};

  struct AimDecision {
    bool valid = false;
    bool converged = false;
    bool command_valid = false;
    int id = 0;
    int lock = -1;
    double fly_time = 0.0;
  };
  struct AimSolution {
    double fly_time = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
  };

  int nearest_armor_id(const Target &target) const;
  AimPoint choose_aim_point(const Target &target, int &lock_id) const;
  std::optional<AimSolution> solve_xyza(const Eigen::Vector4d &xyza, double bullet_speed,
                                        const Eigen::Matrix3d &R_gimbal2world) const;
  bool resolve_aim_time(Target base, double bullet_speed, const Eigen::Matrix3d &R_gimbal2world,
                        int initial_lock, AimDecision &decision) const;
  bool get_trajectory(const Target &launch_target, double yaw_origin, double bullet_speed,
                      const Eigen::Matrix3d &R_gimbal2world, Trajectory &out, AimDecision &decision,
                      Eigen::Vector4d &aim_xyza) const;
};
} // namespace auto_aim

#endif
