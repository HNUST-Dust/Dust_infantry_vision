#include "camera.hpp"

#include <stdexcept>

#ifdef DUST_ENABLE_HIKROBOT
#include "hikrobot/hikrobot.hpp"
#endif
#ifdef DUST_ENABLE_MINDVISION
#include "mindvision/mindvision.hpp"
#endif
#include "tools/runtime/yaml_config.hpp"

namespace io
{
CameraConfig CameraConfig::parse(const tools::YamlConfig & yaml)
{
  CameraConfig config;
  config.name = yaml.get<std::string>("camera_name");
  config.exposure_ms = yaml.get<double>("exposure_ms");
  if (config.name == "mindvision") {
#ifndef DUST_ENABLE_MINDVISION
    yaml.fail("camera_name", "mindvision requires -DENABLE_MINDVISION=ON");
#endif
    config.gamma = yaml.get<double>("gamma");
  } else if (config.name == "hikrobot") {
#ifndef DUST_ENABLE_HIKROBOT
    yaml.fail("camera_name", "hikrobot requires -DENABLE_HIKROBOT=ON");
#endif
    config.gain = yaml.get<double>("gain");
    config.frame_rate = yaml.get("frame_rate", config.frame_rate);
  } else {
    yaml.fail("camera_name", "unknown camera: " + config.name);
  }
  config.vid_pid = yaml.get<std::string>("vid_pid");
  return config;
}

Camera::Camera(const std::string & config_path)
    : Camera(CameraConfig::parse(tools::YamlConfig(config_path)))
{
}

Camera::Camera(const CameraConfig & config)
{
#ifdef DUST_ENABLE_MINDVISION
  if (config.name == "mindvision") {
    camera_ = std::make_unique<MindVision>(config.exposure_ms, config.gamma, config.vid_pid);
    return;
  }
#endif
#ifdef DUST_ENABLE_HIKROBOT
  if (config.name == "hikrobot") {
    camera_ = std::make_unique<HikRobot>(config.exposure_ms, config.gain, config.frame_rate,
                                         config.vid_pid);
    return;
  }
#endif
  throw std::invalid_argument("Unavailable camera_name: " + config.name);
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

void Camera::stop()
{
  camera_->stop();
}

}  // namespace io
