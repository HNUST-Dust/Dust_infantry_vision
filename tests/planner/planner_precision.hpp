#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "src/planner_diagnostics.hpp"

// Offline replay support only. No production solver settings are changed here.
namespace planner_precision
{
using auto_aim::AxisReference;
using auto_aim::MpcAxis;
using auto_aim::MpcConfig;
using nlohmann::json;

template <typename Derived> json matrix(const Eigen::MatrixBase<Derived> & value)
{
  json result = json::array();
  for (int row = 0; row < value.rows(); ++row) {
    json cells = json::array();
    for (int col = 0; col < value.cols(); ++col)
      cells.push_back(value(row, col));
    result.push_back(std::move(cells));
  }
  return result;
}

inline json evaluate(const MpcConfig & config, const Eigen::Vector2d & x0,
                     const AxisReference & reference, const std::string & name)
{
  MpcAxis solver(config);
  const auto diagnostic = solver.solve(x0, reference);
  json result = {{"variant", name},
                 {"config", auto_aim::runtime::mpc_config_json(config)},
                 {"diagnostic", auto_aim::runtime::mpc_diagnostics_json(diagnostic)},
                 {"normalized_inputs", nullptr}};
  if (diagnostic.solved)
    result["normalized_inputs"] =
      matrix(solver.inputs() * config.max_acc / auto_aim::MPC_INPUT_SCALE);
  return result;
}

inline json axis(const MpcConfig & config, const Eigen::Vector2d & x0,
                 const AxisReference & reference, const std::string & name)
{
  using namespace auto_aim;
  const auto model = mpc_model(config);
  const auto & q = model.q;
  const auto & a = model.a;
  const auto & b = model.b;
  const double r = model.r;
  TinyCache cache;
  if (tiny_precompute_and_set_cache(&cache, a, b, Eigen::Vector2d::Zero(), q.asDiagonal(),
                                    Eigen::Matrix<double, 1, 1>::Constant(r), 2, 1, config.rho,
                                    0) != 0)
    throw std::runtime_error("Cannot construct precision reference problem");

  // The terminal backward-pass Hessian is Pinf, but its linear term includes
  // -rho*(v-g). At consensus this subtracts rho*I from the original terminal
  // Hessian. The reference linear term remains -Pinf*xref (not -(Pinf-rho I)*xref).
  const Eigen::Matrix2d terminal = cache.Pinf - config.rho * Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d q_augmented =
    q.asDiagonal().toDenseMatrix() + config.rho * Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d riccati = q_augmented + a.transpose() * cache.Pinf * (a - b * cache.Kinf);
  const double cache_residual = (riccati - cache.Pinf).cwiseAbs().maxCoeff();

  json candidates = json::array();
  candidates.push_back(evaluate(config, x0, reference, "baseline"));
  for (double tolerance : {1e-4, 1e-5}) {
    for (int iterations : {300, 1000}) {
      MpcConfig candidate = config;
      candidate.primal_tolerance = tolerance;
      candidate.dual_tolerance = tolerance;
      candidate.max_iterations = iterations;
      const std::string tag = std::string(tolerance == 1e-4 ? "tol1e-4" : "tol1e-5") + "_iter" +
                              std::to_string(iterations);
      candidates.push_back(evaluate(candidate, x0, reference, tag));
    }
  }
  return {{"axis", name},
          {"state_scale", {config.state_scale[0], config.state_scale[1]}},
          {"x0", {x0[0], x0[1]}},
          {"reference", matrix(reference)},
          {"a", matrix(a)},
          {"b", matrix(b)},
          {"q", {q[0], q[1]}},
          {"r", r},
          {"terminal_hessian", matrix(terminal)},
          {"terminal_reference_weight", matrix(cache.Pinf)},
          {"riccati_cache_residual", cache_residual},
          {"input_scale", MPC_INPUT_SCALE},
          {"normalized_input_limit", config.max_acc / MPC_INPUT_SCALE},
          {"candidates", std::move(candidates)}};
}
}  // namespace planner_precision
