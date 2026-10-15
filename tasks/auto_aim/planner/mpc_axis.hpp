#pragma once

#include <Eigen/Dense>
#include <limits>
#include <string>

#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HORIZON = 60;
constexpr int COMMAND_STEP = 1;
using AxisReference = Eigen::Matrix<double, 2, HORIZON>;

enum class MpcStatus {
  not_run, solved, x0_rejected, iteration_limit, solver_error, nonfinite,
  initial_state_mismatch, dynamics_mismatch, input_violation
};
const char *to_string(MpcStatus status);

struct MpcConfig {
  double max_acc = 150.0;
  Eigen::Vector2d q{9e9, 0.0};
  double r = 1.0;
  double rho = 0.05;
  int max_iterations = 300;
  double primal_tolerance = 1e-3;
  double dual_tolerance = 1e-3;
  static MpcConfig load(const std::string &path, const std::string &axis);
};

struct MpcDiagnostics {
  bool attempted = false;
  bool solved = false;
  MpcStatus status = MpcStatus::not_run;
  int library_return_code = -1;
  bool library_solved = false;
  int iterations = 0;
  int max_iterations = 0;
  double primal_residual = std::numeric_limits<double>::quiet_NaN();
  double dual_residual = std::numeric_limits<double>::quiet_NaN();
  double state_primal_residual = std::numeric_limits<double>::quiet_NaN();
  double input_primal_residual = std::numeric_limits<double>::quiet_NaN();
  double state_dual_residual = std::numeric_limits<double>::quiet_NaN();
  double input_dual_residual = std::numeric_limits<double>::quiet_NaN();
  double input_violation = std::numeric_limits<double>::quiet_NaN();
  double initial_state_error = std::numeric_limits<double>::quiet_NaN();
  double max_dynamics_error = std::numeric_limits<double>::quiet_NaN();
  double solve_time_ms = 0.0;
  double rho = 0.0;
  double input_limit = 0.0;
  double primal_tolerance = 0.0;
  double dual_tolerance = 0.0;
};

class MpcAxis
{
public:
  explicit MpcAxis(const MpcConfig &config);
  ~MpcAxis();
  MpcAxis(const MpcAxis &) = delete;
  MpcAxis &operator=(const MpcAxis &) = delete;
  MpcDiagnostics solve(const Eigen::Vector2d &x0, const AxisReference &reference);
  const tinyMatrix &states() const { return solver_->work->x; }
  const tinyMatrix &inputs() const { return solver_->work->u; }
  const MpcConfig &config() const { return config_; }

private:
  MpcConfig config_;
  TinySolver *solver_ = nullptr;
};
} // namespace auto_aim
