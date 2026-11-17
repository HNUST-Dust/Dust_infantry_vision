#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <memory>
#include <string>

#include "io/camera_base.hpp"

namespace tools
{
class YamlConfig;
}

namespace io
{
struct CameraConfig
{
  std::string name;
  std::string vid_pid;
  double exposure_ms = 0.0;
  double gamma = 0.0;
  double gain = 0.0;
  double frame_rate = 165.0;

  static CameraConfig parse(const tools::YamlConfig & yaml);
};

class Camera
{
public:
  Camera(const std::string & config_path);
  explicit Camera(const CameraConfig & config);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  void stop();

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP
