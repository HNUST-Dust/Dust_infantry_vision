#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  Eigen::VectorXd x_prior = x;
  Eigen::MatrixXd P_prior = P;
  Eigen::VectorXd innovation = z_subtract(z, h(x_prior));
  Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
  if (!innovation.allFinite() || !S.allFinite()) {
    data["nis_fail"] = 1.0;
    data["nees_fail"] = 0.0;
    return x;
  }
  Eigen::LDLT<Eigen::MatrixXd> s_solver(S);
  if (s_solver.info() != Eigen::Success) {
    data["nis_fail"] = 1.0;
    data["nees_fail"] = 0.0;
    return x;
  }

  Eigen::MatrixXd S_inverse =
    s_solver.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
  if (!S_inverse.allFinite()) {
    data["nis_fail"] = 1.0;
    data["nees_fail"] = 0.0;
    return x;
  }
  Eigen::MatrixXd K = P_prior * H.transpose() * S_inverse;
  if (!K.allFinite()) {
    data["nis_fail"] = 1.0;
    data["nees_fail"] = 0.0;
    return x;
  }

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P_prior * (I - K * H).transpose() + K * R * K.transpose();

  x = x_add(x_prior, K * innovation);
  if (!x.allFinite() || !P.allFinite()) {
    data["nis_fail"] = 1.0;
    data["nees_fail"] = 0.0;
    return x;
  }

  /// 卡方检验
  Eigen::VectorXd residual = innovation;
  Eigen::VectorXd nis_solution = s_solver.solve(residual);
  if (!nis_solution.allFinite()) {
    data["nis_fail"] = 1.0;
    data["nees_fail"] = 0.0;
    return x;
  }
  double nis = residual.dot(nis_solution);
  Eigen::LDLT<Eigen::MatrixXd> p_solver(P);
  double nees = 0.0;
  if (p_solver.info() == Eigen::Success) {
    auto state_delta = x - x_prior;
    Eigen::VectorXd nees_solution = p_solver.solve(state_delta);
    if (nees_solution.allFinite()) nees = state_delta.dot(nees_solution);
  }

  // 卡方检验阈值（自由度 4 或 8，取置信水平 95%）
  const double nis_threshold = residual.size() == 8 ? 15.5073 : 9.4877;
  constexpr double nees_threshold = 19.675;

  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual.size() > 0 ? residual[0] : 0.0;
  data["residual_pitch"] = residual.size() > 1 ? residual[1] : 0.0;
  data["residual_distance"] = residual.size() > 2 ? residual[2] : 0.0;
  data["residual_angle"] = residual.size() > 3 ? residual[3] : 0.0;
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools
