#include "planner.hpp"

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_high_speed_ = tools::read<double>(yaml, "fire_thresh_high_speed");
  fire_thresh_low_speed_ = tools::read<double>(yaml, "fire_thresh_low_speed");
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");
  // 装甲板选择策略（原 Aimer）
  comming_angle_ = tools::read<double>(yaml, "comming_angle") / 57.3;
  leaving_angle_ = tools::read<double>(yaml, "leaving_angle") / 57.3;
  // 开火判定（原 Shooter）
  first_tolerance_ = tools::read<double>(yaml, "first_tolerance") / 57.3;
  second_tolerance_ = tools::read<double>(yaml, "second_tolerance") / 57.3;
  judge_distance_ = tools::read<double>(yaml, "judge_distance");
  auto_fire_ = tools::read<bool>(yaml, "auto_fire");

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Planner::~Planner()
{
  tiny_cleanup(yaw_solver_);
  tiny_cleanup(pitch_solver_);
}

Eigen::Vector4d Planner::debug_xyza() const
{
  std::lock_guard<std::mutex> lock(debug_mutex_);
  return debug_xyza_;
}

bool Planner::debug_aim_valid() const
{
  std::lock_guard<std::mutex> lock(debug_mutex_);
  return debug_aim_valid_;
}

int Planner::nearest_armor_id(const Target & target) const
{
  const auto armor_xyza_list = target.armor_xyza_list();
  int best = -1;
  double min_dist = 1e10;
  for (int i = 0; i < static_cast<int>(armor_xyza_list.size()); i++) {
    const double dist = armor_xyza_list[i].head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      best = i;
    }
  }
  return best;
}

AimPoint Planner::choose_aim_point(const Target & target, int & lock_id) const
{
  const Eigen::VectorXd ekf_x = target.ekf_x();
  const std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  const int armor_num = static_cast<int>(armor_xyza_list.size());
  // 默认构造的 Target 一块装甲板都没有；原实现在这里会越界访问 armor_xyza_list[0]
  if (armor_num == 0) return AimPoint{};

  // 装甲板未发生过跳变时，只有当前装甲板的位置已知
  if (!target.jumped) return {true, 0, armor_xyza_list[0]};

  // 整车旋转中心的球坐标 yaw
  const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // delta_angle 为 0 时，该装甲板中心和整车中心的连线在世界系 xy 平面过原点
  std::vector<double> delta_angle_list(armor_num);
  for (int i = 0; i < armor_num; i++)
    delta_angle_list[i] = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);

  // 不考虑小陀螺：|w| <= 2 rad/s 走下面的锁定分支，转得更快才走 coming/leaving 分支。
  // 索引别写错 —— 状态排列是 x vx y vy z vz a w r l h，w 在 [7]，r 在 [8]。
  // 原 Aimer 这里误用了 ekf_x[8]（旋转半径 r，恒在 0.05-0.5），条件恒真，使该分支对非前哨站
  // 目标成了死代码；2026-10-03 合并进 Planner 时按原样移植，随后修正为 ekf_x[7]。
  if (std::abs(ekf_x[7]) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      // 每个规划周期最多被调用 70 次，这里只留 debug 级别
      tools::logger()->debug("[Planner] Empty id list!");
      return {false, 0, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈 45 度的装甲板之间来回切换
    if (id_list.size() > 1) {
      const int id0 = id_list[0], id1 = id_list[1];
      // 未处于锁定模式时，选择 delta_angle 绝对值较小的装甲板，进入锁定模式
      if (lock_id != id0 && lock_id != id1)
        lock_id = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;
      return {true, lock_id, armor_xyza_list[lock_id]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id = -1;
    return {true, id_list[0], armor_xyza_list[id_list[0]]};
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 小陀螺时一侧的装甲板不断出现、另一侧不断消失，前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle)
      return {true, i, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle)
      return {true, i, armor_xyza_list[i]};
  }

  return {false, 0, armor_xyza_list[0]};
}

std::optional<Planner::AimSolution> Planner::solve_xyza(
  const Eigen::Vector4d & xyza, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world) const
{
  const Eigen::Vector3d xyz = xyza.head<3>();
  // 重力沿世界系 z 轴，弹道的水平距离和高度必须使用同一世界参考系。
  const double dist = xyz.head<2>().norm();
  const tools::Trajectory bullet_traj(bullet_speed, dist, xyz.z());
  if (bullet_traj.unsolvable) return std::nullopt;

  // 先在世界系求出包含重力补偿的发射方向，再变换到当前云台系。
  // 云台的关节轴为 Z(yaw)-X(pitch)-Y(roll)，其默认瞄准方向是 -Y：
  //   dir_g = Rz(yaw) Rx(pitch) (-Y)
  // 因此 pitch 必须从变换后的方向分解，不能直接把世界系弹道仰角当作 X 轴角度。
  const Eigen::Vector2d horizontal = xyz.head<2>();
  const double cos_pitch = std::cos(bullet_traj.pitch);
  const Eigen::Vector3d ballistic_dir_world{
    horizontal.x() / dist * cos_pitch,
    horizontal.y() / dist * cos_pitch,
    std::sin(bullet_traj.pitch)};
  const Eigen::Vector3d ballistic_dir_gimbal = R_gimbal2world.transpose() * ballistic_dir_world;

  // 云台 -Y 为零 yaw，绕 X 轴正方向抬头为负（与控制帧约定一致）。
  const double azim = std::atan2(ballistic_dir_gimbal.x(), -ballistic_dir_gimbal.y());
  const double pitch = std::atan2(
    -ballistic_dir_gimbal.z(),
    std::hypot(ballistic_dir_gimbal.x(), ballistic_dir_gimbal.y()));

  AimSolution solution;
  solution.fly_time = bullet_traj.fly_time;
  solution.yaw = tools::limit_rad(azim + yaw_offset_);
  solution.pitch = tools::limit_rad(pitch - pitch_offset_);
  return solution;
}

bool Planner::resolve_aim_time(
  Target base, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world, AimDecision & decision)
{
  // lock_id_ 每次规划只写回一次；迭代内部用局部副本，60 步前瞻不会污染持久状态
  int lock = lock_id_;
  int last_id = -1;
  bool selected = false;
  double used = 0.0;
  double evaluated_time = 0.0;
  bool converged = false;

  {
    const AimPoint p = choose_aim_point(base, lock);
    const auto armor_xyza_list = base.armor_xyza_list();
    const int id = p.valid ? p.id : nearest_armor_id(base);
    if (id < 0 || id >= static_cast<int>(armor_xyza_list.size())) return false;

    const auto solution = solve_xyza(armor_xyza_list[id], bullet_speed, R_gimbal2world);
    if (!solution) return false;

    used = solution->fly_time;
    evaluated_time = used;
    last_id = id;
    selected = p.valid;
  }

  // 迭代求解飞行时间：每次都从 base 副本重新预测，误差不累积，只有标量 fly_time 在收敛
  for (int iter = 0; iter < 10; iter++) {
    const double current_time = used;
    Target predicted = base;
    predicted.predict(current_time);  // 绝对时刻 T1 + current_time

    const AimPoint p = choose_aim_point(predicted, lock);
    const auto armor_xyza_list = predicted.armor_xyza_list();
    const int id = p.valid ? p.id : (last_id >= 0 ? last_id : nearest_armor_id(predicted));
    if (id < 0 || id >= static_cast<int>(armor_xyza_list.size())) break;

    const auto solution = solve_xyza(armor_xyza_list[id], bullet_speed, R_gimbal2world);
    // 弹道不可解：保留上一轮的结果，不让一次失败毁掉整次规划
    if (!solution) break;

    last_id = id;
    selected = p.valid;
    evaluated_time = current_time;
    if (std::abs(solution->fly_time - current_time) < 0.001) {
      converged = true;
      used = current_time;
      break;
    }
    used = solution->fly_time;
  }

  if (last_id < 0) return false;

  // On exhaustion or a failed iteration, keep the last internally consistent candidate for
  // tracking, but do not publish its lock as a settled decision or allow it to fire.
  if (!converged) used = evaluated_time;
  if (converged) lock_id_ = lock;
  decision.valid = selected;
  decision.converged = converged;
  decision.id = last_id;
  decision.lock = lock;
  decision.fly_time = used;
  return true;
}

bool Planner::get_trajectory(
  const Target & hit_target, double yaw0, double bullet_speed,
  const Eigen::Matrix3d & R_gimbal2world, int decision_lock, int decision_id, Trajectory & out)
{
  // 列索引 i 对应时刻 T_hit + (i - HALF_HORIZON) * DT，第 HALF_HORIZON 列就是命中列。
  //
  // 旧实现把目标从命中时刻倒退 310ms，再让局部锁从过去重新演进：单候选区会把锁清成 -1，
  // 到了双候选区就可能选到另一块板，使第 HALF_HORIZON 列偏离 resolve_aim_time 的决策
  // （3m / r=0.25m / w=±1.5rad/s 下实测约 7°）。现在以命中列的决策板为锚点，
  // 前向（未来）与后向（过去）各持一份锁副本演进，中心板 ID 与参考方向由构造保证一致。
  std::vector<Eigen::Matrix<double, 2, 1>> columns(HORIZON + 2);
  auto column = [&](int i) -> Eigen::Matrix<double, 2, 1> & { return columns[i + 1]; };
  bool any_solved = false;

  // 命中列：直接用决策板与它的弹道解，不重新过一遍策略
  {
    const auto armor_xyza_list = hit_target.armor_xyza_list();
    if (decision_id < 0 || decision_id >= static_cast<int>(armor_xyza_list.size())) return false;
    const auto solution = solve_xyza(armor_xyza_list[decision_id], bullet_speed, R_gimbal2world);
    if (!solution) return false;
    column(HALF_HORIZON) << solution->yaw, solution->pitch;
    any_solved = true;
  }

  // 单列求解：过一遍击打策略再解弹道。lock / last_id / fallback 由调用方持有，
  // 前向与后向各持一份，互不干扰。策略给不出结果时沿用上一列的装甲板，无装甲板或弹道
  // 不可解时沿用上一列的角度，都不让参考轨迹断掉。
  auto solve_column = [&](
                        const Target & target, int & lock, int & last_id,
                        Eigen::Matrix<double, 2, 1> & fallback) -> Eigen::Matrix<double, 2, 1> {
    const auto armor_xyza_list = target.armor_xyza_list();
    if (armor_xyza_list.empty()) return fallback;

    const AimPoint p = choose_aim_point(target, lock);
    int id = p.valid ? p.id : last_id;
    if (id < 0 || id >= static_cast<int>(armor_xyza_list.size())) id = nearest_armor_id(target);
    if (id < 0 || id >= static_cast<int>(armor_xyza_list.size())) return fallback;

    const auto solution = solve_xyza(armor_xyza_list[id], bullet_speed, R_gimbal2world);
    if (!solution) return fallback;

    last_id = id;
    any_solved = true;
    fallback << solution->yaw, solution->pitch;
    return fallback;
  };

  // 命中列之后：从命中列连续前向预测
  {
    Target target = hit_target;
    int lock = decision_lock;  // 锁副本，只在本次构造内演进，不回写 lock_id_
    int last_id = decision_id;
    auto fallback = column(HALF_HORIZON);
    for (int i = HALF_HORIZON + 1; i <= HORIZON; i++) {
      target.predict(DT);
      column(i) = solve_column(target, lock, last_id, fallback);
    }
  }

  // 命中列之前：每一列都从命中列单独倒退，不做连续负向预测 —— dt < 0 时过程噪声项
  // Q 的符号会翻转，连续倒退会把协方差越推越歪；旧实现也只做一次 -310ms。
  {
    int lock = decision_lock;  // 与上一步独立的一份锁副本
    int last_id = decision_id;
    auto fallback = column(HALF_HORIZON);
    for (int i = HALF_HORIZON - 1; i >= -1; i--) {
      Target target = hit_target;
      target.predict(-DT * (HALF_HORIZON - i));
      column(i) = solve_column(target, lock, last_id, fallback);
    }
  }

  // 中心差分求角速度，两端（i=0 与 i=HORIZON-1）用列 -1 与列 HORIZON 的单侧差分
  for (int i = 0; i < HORIZON; i++) {
    const auto & prev = column(i - 1);
    const auto & next = column(i + 1);
    const double yaw_vel = tools::limit_rad(next(0) - prev(0)) / (2 * DT);
    const double pitch_vel = (next(1) - prev(1)) / (2 * DT);
    out.col(i) << tools::limit_rad(column(i)(0) - yaw0), yaw_vel, column(i)(1), pitch_vel;
  }

  return any_solved;
}

Plan Planner::plan(
  Target target, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world,
  std::optional<double> gimbal_yaw)
{
  {
    std::lock_guard<std::mutex> guard(debug_mutex_);
    debug_aim_valid_ = false;
  }

  const Eigen::VectorXd ekf_x = target.ekf_x();
  const double fire_thresh =
    std::abs(ekf_x[7]) > decision_speed_ ? fire_thresh_high_speed_ : fire_thresh_low_speed_;

  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) bullet_speed = 23;

  if (target.armor_xyza_list().empty()) return Plan{};

  // 1. 迭代收敛飞行时间，并定下决策时刻要打的那块装甲板
  AimDecision decision;
  if (!resolve_aim_time(target, bullet_speed, R_gimbal2world, decision)) return Plan{};

  Target aim_target = target;
  aim_target.predict(decision.fly_time);  // 推进到 T1 + fly_time，即子弹到达时刻

  const auto armor_xyza_list = aim_target.armor_xyza_list();
  if (decision.id < 0 || decision.id >= static_cast<int>(armor_xyza_list.size())) return Plan{};
  const Eigen::Vector4d aim_xyza = armor_xyza_list[decision.id];

  const auto aim_solution = solve_xyza(aim_xyza, bullet_speed, R_gimbal2world);
  if (!aim_solution) return Plan{};

  // 2. 参考轨迹：以命中列的决策板为锚点向两侧构造
  const double yaw0 = aim_solution->yaw;
  Trajectory traj;
  if (!get_trajectory(
        aim_target, yaw0, bullet_speed, R_gimbal2world, decision.lock, decision.id, traj))
    return Plan{};

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);
  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  // 5. 开火 = "参考可达" AND ("总开关 + 指令未跳变 + 决策有效 + 云台已到位")
  const auto shoot_offset = 2;
  const bool track_ok = std::hypot(
                          traj(0, HALF_HORIZON + shoot_offset) -
                            yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset),
                          traj(2, HALF_HORIZON + shoot_offset) -
                            pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset)) < fire_thresh;

  const double tolerance =
    std::hypot(ekf_x[0], ekf_x[2]) > judge_distance_ ? second_tolerance_ : first_tolerance_;

  // 这几项与云台反馈无关，缺反馈也必须照样生效：总开关、已有上一条指令、这一拍真的选中了
  // 装甲板、指令没有跳变。角度差用 limit_rad 做环绕，避免 179°/-179° 被判成 358°。
  bool shoot_ok =
    auto_fire_ && have_last_command_ && decision.valid && decision.converged &&
    std::abs(tools::limit_rad(last_target_yaw_ - plan.target_yaw)) < tolerance * 2;

  // "云台已到位"是唯一依赖反馈的一项，没有反馈时跳过（回放/单测），不影响上面几项。
  // 下位机按「目标 = 当前云台 yaw + 下发 yaw」解释控制帧，下发值是相对云台系的增量，
  // 所以要比较的是"云台从上一条指令生成时刻到现在实际转过的角度"与"上一条指令增量"，
  // 而不是把反馈的绝对 yaw 与相对命令直接相减（那样同向也会被判成相差很大）。
  if (shoot_ok && gimbal_yaw && last_plan_gimbal_yaw_) {
    const double moved = tools::limit_rad(*gimbal_yaw - *last_plan_gimbal_yaw_);
    shoot_ok = std::abs(tools::limit_rad(moved - last_plan_yaw_)) < tolerance;
  }

  plan.fire = track_ok && shoot_ok;

  if (plan.control) {
    last_plan_yaw_ = plan.yaw;
    last_target_yaw_ = plan.target_yaw;
    // 与 last_plan_yaw_ 配套：记录生成这条指令时的云台绝对 yaw。没传反馈就清空，
    // 免得下一拍拿过期角度做到位判定。
    last_plan_gimbal_yaw_ = gimbal_yaw;
    have_last_command_ = true;
  }

  {
    std::lock_guard<std::mutex> guard(debug_mutex_);
    debug_xyza_ = aim_xyza;
    debug_aim_valid_ = decision.valid;
  }

  return plan;
}

void Planner::record_command_gimbal_yaw(std::optional<double> gimbal_yaw)
{
  // The production runtime samples once more after the expensive planning step so this baseline
  // corresponds to the command's generation time rather than the start of planning.
  last_plan_gimbal_yaw_ = gimbal_yaw;
}

Plan Planner::plan(
  std::optional<Target> target, double bullet_speed, const Eigen::Matrix3d & R_gimbal2world,
  std::optional<double> gimbal_yaw)
{
  return plan_at(target, std::chrono::steady_clock::now(), bullet_speed, R_gimbal2world, gimbal_yaw);
}

Plan Planner::plan_at(
  std::optional<Target> target, std::chrono::steady_clock::time_point now, double bullet_speed,
  const Eigen::Matrix3d & R_gimbal2world, std::optional<double> gimbal_yaw)
{
  if (!target.has_value()) {
    std::lock_guard<std::mutex> guard(debug_mutex_);
    debug_aim_valid_ = false;
    return Plan{};
  }

  const double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
  target->predict(now + std::chrono::microseconds(static_cast<int>(delay_time * 1e6)));

  return plan(*target, bullet_speed, R_gimbal2world, gimbal_yaw);
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;
}

}  // namespace auto_aim
