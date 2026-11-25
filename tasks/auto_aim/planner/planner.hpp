#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <array>
#include <optional>

#include "tasks/auto_aim/tracking/target.hpp"
#include "mpc_axis.hpp"
#include "reference.hpp"

namespace tools
{
class YamlConfig;
}

namespace auto_aim
{
constexpr int SMOOTHING_CENTER = HORIZON / 2;
enum class MpcMode
{
  measured_state,
  reference_smoothing
};
const char * to_string(MpcMode mode);

enum class PlanStatus
{
  not_run,
  ok,
  no_target,
  empty_armor,
  feedback_invalid,
  timestamp_missing,
  feedback_future,
  feedback_stale,
  state_nonfinite,
  rotation_invalid,
  state_frame_mismatch,
  pitch_singular,
  propagated_state_invalid,
  trajectory_unavailable,
  mpc_failed,
  exception
};
const char * to_string(PlanStatus status);

struct PlanningState
{
  double yaw = 0.0;
  double yaw_vel = 0.0;
  double pitch = 0.0;
  double pitch_vel = 0.0;
  Eigen::Matrix3d R_gimbal2world = Eigen::Matrix3d::Identity();
  std::chrono::steady_clock::time_point timestamp{};
  bool valid = false;
  bool rates_measured = false;
};

struct AimDebugSnapshot
{
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
  int armor_id = -1;
  AimMode mode = AimMode::fallback;
  bool candidate_available = false;
  bool selection_valid = false;
  bool ballistic_converged = false;
  bool display_valid = false;
};

struct FireWindowCheck
{
  bool passed = false;
  std::optional<double> aligned_yaw_error;
  double applied_yaw_tolerance = 0.0;
  std::optional<double> normalized_error_squared;
};

struct FireWindow
{
  bool enabled = false;
  double base_tolerance = 0.0;
  double lag_ratio = 0.5;
  FireYawMotion motion;
  // Errors are measured minus target; positive aligned error means ahead.
  FireWindowCheck evaluate(double yaw_error, double pitch_error) const;
};

struct Plan
{
  MpcMode mpc_mode = MpcMode::measured_state;
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
  int settled_reference_index = 0;
  int fire_reference_index = COMMAND_STEP;
  FireWindow settled_window, fire_window;
  FireWindowCheck settled_check, fire_window_check;
  double feedback_age_ms = 0.0;
  PlanningState initial_state;
  MpcDiagnostics yaw_mpc;
  MpcDiagnostics pitch_mpc;
  // Raw optimized yaw velocity/acceleration, pitch velocity/acceleration.
  // Position-only mode sends zeros in the four command feedforward fields above.
  Eigen::Vector4d mpc_feedforward = Eigen::Vector4d::Zero();
  std::chrono::steady_clock::time_point planned_at{};
  std::chrono::steady_clock::time_point command_at{};
  // Sender expiry, independent of the command sampling time.
  std::chrono::steady_clock::time_point valid_at{};

  int mode() const
  {
    return control ? (fire ? 2 : 1) : 0;
  }
  Plan relative_to(double measured_yaw, double measured_pitch) const;
  bool aim_visible_at(std::chrono::steady_clock::time_point now) const;
};

struct PlannerHistory
{
  int lock_id = -1;
  bool have_last_command = false;
  double last_target_yaw = 0.0;
  std::chrono::steady_clock::time_point last_plan_at{};
};

struct PlanningTrace
{
  MpcMode mpc_mode = MpcMode::measured_state;
  int state_index = COMMAND_STEP;
  int input_index = 0;
  Eigen::Vector2d yaw_x0 = Eigen::Vector2d::Zero();
  Eigen::Vector2d pitch_x0 = Eigen::Vector2d::Zero();
  double launch_delay = 0.0;
  // Smoothing columns share a hit-time anchor; measured columns are launch-time samples.
  double center_fly_time = 0.0;
  bool reference_available = false;
  Trajectory reference = Trajectory::Zero();
  ReferenceTags reference_tags{};
  PlanningState received_state;
  MpcConfig yaw_config;
  MpcConfig pitch_config;
  double bullet_speed = 0.0;
  double yaw_offset = 0.0;
  double pitch_offset = 0.0;
  // Captured before target propagation/selection. YAML contains planner keys only.
  std::string planner_config_yaml;
  PlannerHistory history;
  std::optional<TargetPredictionState> target_prediction;
  bool target_available = false;
  std::array<double, 11> target_state{};
};

struct AimPoint
{
  bool valid = false;
  int id = 0;
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
};

struct PlannerConfig
{
  MpcMode mpc_mode = MpcMode::measured_state;
  MpcConfig yaw, pitch;
  std::string replay_yaml;
  double yaw_offset = 0.0;
  double pitch_offset = 0.0;
  double fire_thresh = 0.0;
  double fire_thresh_high_speed = 0.0;
  double decision_speed = 0.0;
  double high_speed_delay_time = 0.0;
  double low_speed_delay_time = 0.0;
  double comming_angle = 0.0;
  double leaving_angle = 0.0;
  double first_tolerance = 0.0;
  double second_tolerance = 0.0;
  double first_tolerance_high_speed = 0.0;
  double second_tolerance_high_speed = 0.0;
  bool fire_directional_tolerance_enabled = false;
  double fire_lag_tolerance_ratio = 0.5;
  double fire_direction_min_yaw_rate = 0.05;
  double judge_distance = 0.0;
  bool auto_fire = false;
  bool gap_preaim_enabled = false;
  double max_state_age = 0.02;

  static PlannerConfig parse(const tools::YamlConfig & yaml);
};

class Planner
{
public:
  explicit Planner(const std::string & config_path);
  explicit Planner(const PlannerConfig & config);
  ~Planner() = default;
  Planner(const Planner &) = delete;
  Planner & operator=(const Planner &) = delete;

  Plan plan(std::optional<Target> target, double bullet_speed, const PlanningState & state,
            PlanningTrace * trace = nullptr);
  Plan plan_at(std::optional<Target> target, std::chrono::steady_clock::time_point now,
               double bullet_speed, const PlanningState & state, PlanningTrace * trace = nullptr);

  // Offline single-sample replay restores the recorded selection/fire history explicitly.
  void restore_history(const PlannerHistory & history);

private:
  MpcMode mpc_mode_ = MpcMode::measured_state;
  std::string planner_config_yaml_;
  double yaw_offset_;
  double pitch_offset_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  double fire_thresh_, fire_thresh_high_speed_;
  double comming_angle_, leaving_angle_;
  double first_tolerance_, second_tolerance_, judge_distance_;
  double first_tolerance_high_speed_, second_tolerance_high_speed_;
  bool fire_directional_tolerance_enabled_ = false;
  double fire_lag_tolerance_ratio_ = 0.5;
  double fire_direction_min_yaw_rate_ = 0.05;
  bool auto_fire_;
  bool gap_preaim_enabled_ = false;
  double max_state_age_ = 0.02;
  MpcAxis yaw_solver_;
  MpcAxis pitch_solver_;
  int lock_id_ = -1;
  bool have_last_command_ = false;
  double last_target_yaw_ = 0.0;
  std::chrono::steady_clock::time_point last_plan_at_{};

  struct AimDecision
  {
    bool valid = false;
    bool converged = false;
    bool command_valid = false;
    int id = 0;
    int lock = -1;
    double fly_time = 0.0;
    AimMode mode = AimMode::fallback;
    Eigen::Vector4d reference_xyza = Eigen::Vector4d::Zero();
  };
  struct AimSolution
  {
    double fly_time = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
  };

  int nearest_armor_id(const Target & target) const;
  AimPoint choose_aim_point(const Target & target, int & lock_id) const;
  AimDecision choose_reference(const Target & target, int & lock, int previous_id) const;
  std::optional<AimSolution> solve_xyza(const Eigen::Vector4d & xyza, double bullet_speed,
                                        const Eigen::Matrix3d & R_gimbal2world) const;
  bool resolve_aim_time(Target base, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world,
                        int initial_lock, AimDecision & decision) const;
  bool get_trajectory(const Target & launch_target, double yaw_origin, double bullet_speed,
                      const Eigen::Matrix3d & R_gimbal2world, Trajectory & out,
                      ReferenceTags & tags, AimDecision & decision,
                      Eigen::Vector4d & aim_xyza) const;
  bool get_smoothing_trajectory(const Target & launch_target, double yaw_origin,
                                double bullet_speed, const Eigen::Matrix3d & rotation,
                                Trajectory & out, ReferenceTags & tags, AimDecision & decision,
                                Eigen::Vector4d & aim_xyza) const;
};
}  // namespace auto_aim

#endif
