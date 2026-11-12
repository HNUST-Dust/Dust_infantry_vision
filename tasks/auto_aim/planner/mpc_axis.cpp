#include "mpc_axis.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "tools/runtime/yaml.hpp"

namespace auto_aim
{
const char * to_string(MpcStatus status)
{
  switch (status) {
  case MpcStatus::not_run:
    return "not_run";
  case MpcStatus::solved:
    return "solved";
  case MpcStatus::x0_rejected:
    return "x0_rejected";
  case MpcStatus::iteration_limit:
    return "iteration_limit";
  case MpcStatus::solver_error:
    return "solver_error";
  case MpcStatus::nonfinite:
    return "nonfinite";
  case MpcStatus::initial_state_mismatch:
    return "initial_state_mismatch";
  case MpcStatus::dynamics_mismatch:
    return "dynamics_mismatch";
  case MpcStatus::input_violation:
    return "input_violation";
  }
  return "unknown";
}

MpcConfig MpcConfig::load(const std::string & path, const std::string & axis)
{
  const auto yaml = tools::load(path);
  MpcConfig config;
  config.max_acc = tools::read<double>(yaml, "max_" + axis + "_acc");
  if (const auto cost = yaml["mpc_cost_" + axis]) {
    if (yaml["Q_" + axis] || yaml["R_" + axis])
      throw std::invalid_argument("Use either normalized or legacy MPC costs for " + axis);
    const auto scales = tools::read<std::vector<double>>(cost, "scales");
    const auto weights = tools::read<std::vector<double>>(cost, "weights");
    if (scales.size() != 3 || weights.size() != 3)
      throw std::invalid_argument("MPC cost scales/weights need position, velocity, acceleration");
    for (int i = 0; i < 3; ++i)
      if (!std::isfinite(scales[i]) || scales[i] <= 0 || !std::isfinite(weights[i]) ||
          weights[i] < 0 || (i != 1 && weights[i] == 0))
        throw std::invalid_argument("Invalid normalized MPC cost for " + axis);
    config.state_scale = {scales[0], scales[1]};
    config.q = {weights[0] / (scales[0] * scales[0]), weights[1] / (scales[1] * scales[1])};
    config.r = weights[2] / (scales[2] * scales[2]);
  } else {
    const auto q = tools::read<std::vector<double>>(yaml, "Q_" + axis);
    const auto r = tools::read<std::vector<double>>(yaml, "R_" + axis);
    if (q.size() != 2 || r.size() != 1)
      throw std::invalid_argument("Invalid MPC weight dimensions for " + axis);
    config.q = {q[0], q[1]};
    config.r = r[0];
  }
  if (yaml["mpc_rho"])
    config.rho = yaml["mpc_rho"].as<double>();
  if (yaml["mpc_max_iter"])
    config.max_iterations = yaml["mpc_max_iter"].as<int>();
  return config;
}

MpcModel mpc_model(const MpcConfig & config)
{
  if (!config.q.allFinite() || config.q[0] <= 0 || config.q[1] < 0 || !std::isfinite(config.r) ||
      config.r <= 0 || !std::isfinite(config.max_acc) || config.max_acc <= 0 ||
      !std::isfinite(config.rho) || config.rho <= 0 || config.max_iterations <= 0 ||
      !std::isfinite(config.primal_tolerance) || config.primal_tolerance <= 0 ||
      !std::isfinite(config.dual_tolerance) || config.dual_tolerance <= 0 ||
      !config.state_scale.allFinite() || (config.state_scale.array() <= 0).any())
    throw std::invalid_argument("Invalid MPC configuration");
  const Eigen::Vector2d scaled_q = config.q.cwiseProduct(config.state_scale.cwiseAbs2());
  const double scaled_r = config.r * MPC_INPUT_SCALE * MPC_INPUT_SCALE;
  const double scale = std::max({scaled_q[0], scaled_q[1], scaled_r});
  if (!scaled_q.allFinite() || !std::isfinite(scale) || scale <= 0)
    throw std::invalid_argument("MPC cost scale overflow");
  MpcModel model;
  model.a << 1, DT * config.state_scale[1] / config.state_scale[0], 0, 1;
  model.b = {0.0, MPC_INPUT_SCALE * DT / config.state_scale[1]};
  model.q = scaled_q / scale;
  model.r = scaled_r / scale;
  if (!model.a.allFinite() || !model.b.allFinite() || !model.q.allFinite() ||
      !std::isfinite(model.r) || model.q[0] <= 0 || model.r <= 0)
    throw std::invalid_argument("Invalid normalized MPC model");
  return model;
}

MpcAxis::MpcAxis(const MpcConfig & config) : config_(config)
{
  const auto model = mpc_model(config);
  if (tiny_setup(&solver_, model.a, model.b, Eigen::Vector2d::Zero(), model.q.asDiagonal(),
                 Eigen::Matrix<double, 1, 1>::Constant(model.r), config.rho, 2, 1, HORIZON, 0) != 0)
    throw std::runtime_error("TinyMPC setup failed");
  tiny_set_bound_constraints(
    solver_, Eigen::MatrixXd::Constant(2, HORIZON, -1e17),
    Eigen::MatrixXd::Constant(2, HORIZON, 1e17),
    Eigen::MatrixXd::Constant(1, HORIZON - 1, -config.max_acc / MPC_INPUT_SCALE),
    Eigen::MatrixXd::Constant(1, HORIZON - 1, config.max_acc / MPC_INPUT_SCALE));
  solver_->settings->abs_pri_tol = config.primal_tolerance;
  solver_->settings->abs_dua_tol = config.dual_tolerance;
  solver_->settings->max_iter = config.max_iterations;
}

MpcAxis::~MpcAxis()
{
  tiny_cleanup(solver_);
}

MpcDiagnostics MpcAxis::solve(const Eigen::Vector2d & x0, const AxisReference & reference)
{
  const auto started = std::chrono::steady_clock::now();
  MpcDiagnostics diagnostic;
  diagnostic.attempted = true;
  diagnostic.max_iterations = config_.max_iterations;
  diagnostic.rho = config_.rho;
  diagnostic.input_limit = config_.max_acc;
  diagnostic.primal_tolerance = config_.primal_tolerance;
  diagnostic.dual_tolerance = config_.dual_tolerance;
  const auto finish = [&](MpcStatus status) {
    diagnostic.status = status;
    diagnostic.solved = status == MpcStatus::solved;
    diagnostic.solve_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    return diagnostic;
  };
  if (!x0.allFinite() || !reference.allFinite())
    return finish(MpcStatus::nonfinite);
  auto & work = *solver_->work;
  // Keep the production cold start; replay and planning share this exact path.
  work.x.setZero();
  work.u.setZero();
  work.v.setZero();
  work.vnew.setZero();
  work.z.setZero();
  work.znew.setZero();
  work.g.setZero();
  work.y.setZero();
  work.q.setZero();
  work.r.setZero();
  work.p.setZero();
  work.d.setZero();
  work.Xref = reference;
  for (int row = 0; row < 2; ++row)
    work.Xref.row(row) /= config_.state_scale[row];
  const Eigen::Vector2d normalized_x0 = x0.cwiseQuotient(config_.state_scale);
  if (!normalized_x0.allFinite() || !work.Xref.allFinite())
    return finish(MpcStatus::nonfinite);
  if (tiny_set_x0(solver_, normalized_x0) != 0)
    return finish(MpcStatus::x0_rejected);
  const int status = tiny_solve(solver_);
  diagnostic.library_return_code = status;
  diagnostic.library_solved = solver_->solution->solved;
  diagnostic.iterations = work.iter;
  diagnostic.state_primal_residual = work.primal_residual_state;
  diagnostic.input_primal_residual = work.primal_residual_input;
  diagnostic.state_dual_residual = work.dual_residual_state;
  diagnostic.input_dual_residual = work.dual_residual_input;
  diagnostic.primal_residual = std::max(work.primal_residual_state, work.primal_residual_input);
  diagnostic.dual_residual = std::max(work.dual_residual_state, work.dual_residual_input);
  diagnostic.input_violation =
    std::max(0.0, work.u.cwiseAbs().maxCoeff() * MPC_INPUT_SCALE - config_.max_acc);
  diagnostic.initial_state_error =
    (work.x.col(0) - normalized_x0).cwiseProduct(config_.state_scale).norm();
  diagnostic.max_dynamics_error = 0.0;
  for (int k = 0; k < HORIZON - 1; ++k)
    diagnostic.max_dynamics_error =
      std::max(diagnostic.max_dynamics_error,
               (work.x.col(k + 1) - work.Adyn * work.x.col(k) - work.Bdyn * work.u.col(k))
                 .cwiseProduct(config_.state_scale)
                 .norm());
  if (!work.x.allFinite() || !work.u.allFinite())
    return finish(MpcStatus::nonfinite);
  if (status != 0 || !diagnostic.library_solved)
    return finish(work.iter >= config_.max_iterations ? MpcStatus::iteration_limit
                                                      : MpcStatus::solver_error);
  if (!(diagnostic.initial_state_error < 1e-9))
    return finish(MpcStatus::initial_state_mismatch);
  if (!(diagnostic.max_dynamics_error < 1e-8))
    return finish(MpcStatus::dynamics_mismatch);
  if (!(diagnostic.input_violation <= MPC_INPUT_SCALE * config_.primal_tolerance))
    return finish(MpcStatus::input_violation);
  work.u = solver_->solution->u;
  work.x.col(0) = normalized_x0;
  for (int k = 0; k < HORIZON - 1; ++k)
    work.x.col(k + 1) = work.Adyn * work.x.col(k) + work.Bdyn * work.u.col(k);
  // Roll out in solver units first, then adapt to the existing command interface.
  work.u *= MPC_INPUT_SCALE / config_.max_acc;
  for (int row = 0; row < 2; ++row)
    work.x.row(row) *= config_.state_scale[row];
  return finish(MpcStatus::solved);
}
}  // namespace auto_aim
