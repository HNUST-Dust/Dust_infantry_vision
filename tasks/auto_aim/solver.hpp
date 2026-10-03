#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <mutex>
#include <optional>
#include <vector>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  Eigen::Matrix3d R_gimbal2world(const Eigen::Quaterniond & q) const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(Armor & armor) const;

  // 给定装甲板在相机系下的位姿，投影 4 个角点。
  // reproject_armor() 的位姿只能表达成 (world_xyz, yaw)+固定 pitch，表达不了扰动需要的 tilt/roll。
  std::vector<cv::Point2f> project_armor_camera(
    const Eigen::Matrix3d & R_armor2camera, const Eigen::Vector3d & t_armor2camera,
    ArmorType type) const;

  // 按像素噪声一阶传播 + 位姿先验，得到 ypda 观测 [yaw, pitch, distance, armor_yaw] 的 4x4 协方差。
  // 像素噪声决定良态方向；平面靶法向（第 4 维）不可观测，由 angle_prior_rad 先验主导。
  // 失败（角点数不对、非有限、信息矩阵不可逆）返回 std::nullopt，调用方应回退到默认噪声。
  std::optional<Eigen::Matrix4d> ypda_measurement_covariance(
    const Armor & armor, double point_sigma_px, double angle_prior_rad) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name,
    const Eigen::Matrix3d & R_gimbal2world) const;

  double oupost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;
  mutable std::mutex rotation_mutex_;

  void optimize_yaw(Armor & armor, const Eigen::Matrix3d & R_gimbal2world) const;

  double armor_reprojection_error(
    const Armor & armor, double yaw, const double & inclined,
    const Eigen::Matrix3d & R_gimbal2world) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP
