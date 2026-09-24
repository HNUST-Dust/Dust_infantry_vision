#ifndef VISUALIZATION__VISION_OVERLAY_HPP
#define VISUALIZATION__VISION_OVERLAY_HPP

#include <Eigen/Core>
#include <list>
#include <opencv2/core.hpp>
#include <optional>
#include <string>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"

namespace visualization
{
// 一帧的可视化输入，全部是只读引用，所有权仍归调用方；请保证它们比本次绘制活得久。
struct VisionOverlayInput
{
  const std::list<auto_aim::Armor> & armors;
  const cv::Rect & net_roi;
  const std::optional<cv::Rect> & light_roi;
  const std::string & tracker_state;
  const auto_aim::Target * target = nullptr;            // nullptr 表示本帧没有目标
  Eigen::Vector4d aim_xyza = Eigen::Vector4d::Zero();   // 仅 target 非空时使用
};

// 绘制网络 ROI、灯条 ROI、装甲板角点、重投影装甲板、重投影瞄准点、Tracker 状态与中心准星。
// 与本地窗口的叠加内容一一对应；不含缩放、窗口、按键与日志。
void draw_vision_overlay(
  cv::Mat & image, const VisionOverlayInput & input, const auto_aim::Solver & solver);

}  // namespace visualization

#endif  // VISUALIZATION__VISION_OVERLAY_HPP
