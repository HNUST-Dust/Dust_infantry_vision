#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{
struct Trajectory
{
  bool unsolvable = true;
  double fly_time = 0.0;
  double pitch = 0.0;  // 抬头为正

  // 不考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP
