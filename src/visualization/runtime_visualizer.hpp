#pragma once

#include <memory>

#include "visualization/foxglove_vision.hpp"
#include "visualization/vision_overlay.hpp"

namespace tools
{
class YamlConfig;
}

namespace visualization
{
struct RuntimeVisualizationConfig
{
  bool headless = false;
  bool foxglove_enabled = false;
  std::string window_name = "reprojection";
  FoxgloveVisionOptions foxglove;

  RuntimeVisualizationConfig()
  {
    foxglove.image_fps = 30.0;
  }
  static RuntimeVisualizationConfig parse(const tools::YamlConfig & yaml);
};

class RuntimeVisualizer
{
public:
  explicit RuntimeVisualizer(const RuntimeVisualizationConfig & config);
  ~RuntimeVisualizer();

  struct FrameOutput
  {
    bool local;
    bool remote;

    explicit operator bool() const
    {
      return local || remote;
    }
  };

  bool needs_source() const;
  FrameOutput frame_output(bool inferred, const cv::Mat & image) const;

  // Draws in place. Input references remain valid through this call. The window displays
  // the resized frame and Foxglove retains its own cv::Mat header for queued images.
  // Returns true when the local window receives the quit key.
  bool draw_frame(cv::Mat & image, const VisionOverlayInput & overlay,
                  const auto_aim::Solver & solver, FrameOutput output,
                  std::chrono::steady_clock::time_point captured_at);
  void publish_telemetry(nlohmann::json data, const char * tracker_state, bool has_target,
                         std::chrono::steady_clock::time_point sampled_at);

private:
  bool headless_;
  std::string window_name_;
#ifdef DUST_ENABLE_FOXGLOVE
  std::unique_ptr<FoxgloveVision> foxglove_;
#endif

  bool remote_image_requested() const;
};
}  // namespace visualization
