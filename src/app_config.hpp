#pragma once

#include <chrono>
#include <filesystem>
#include <optional>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "src/planner_diagnostics.hpp"
#include "src/visualization/runtime_visualizer.hpp"
#include "tasks/auto_aim/detection/yolov5.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/tracking/tracker.hpp"

namespace auto_aim::runtime
{
struct RuntimeConfig
{
  bool talos_backend = false;
  bool simulate_gimbal = false;
  bool verbose_ekf = false;
  std::chrono::milliseconds fire_timeout{30};
  std::chrono::milliseconds control_timeout{80};
  std::chrono::milliseconds feedback_timeout{100};
  std::chrono::milliseconds plan_timeout{20};
};

struct ApplicationConfig
{
  RuntimeConfig runtime;
  visualization::RuntimeVisualizationConfig visualization;
  PlannerDiagnosticsConfig diagnostics;
  YoloConfig detector;
  SolverConfig solver;
  TrackerConfig tracker;
  PlannerConfig planner;
  std::optional<io::CameraConfig> camera;
  io::GimbalConfig gimbal;
};

ApplicationConfig load_application_config(const std::filesystem::path & path);
}  // namespace auto_aim::runtime
