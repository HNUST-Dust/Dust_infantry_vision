#include "app_config.hpp"

#include "tools/runtime/yaml_config.hpp"

namespace auto_aim::runtime
{
namespace
{
RuntimeConfig parse_runtime(const tools::YamlConfig & yaml)
{
  RuntimeConfig config;
  const auto runtime = yaml.section("runtime");
  const auto backend = runtime.get<std::string>("io_backend", "hardware");
  if (backend != "hardware" && backend != "talos")
    runtime.fail("io_backend", "must be hardware or talos");
  config.talos_backend = backend == "talos";
#ifndef DUST_ENABLE_SIMULATOR
  if (config.talos_backend)
    runtime.fail(
      "io_backend",
      "Talos simulator backend is not compiled in; configure with -DENABLE_SIMULATOR=ON");
#endif

  config.simulate_gimbal = runtime.get("simulate_gimbal", false);
  config.verbose_ekf = runtime.get("verbose_ekf", false);
  const auto timeout = [&](const char * field, std::chrono::milliseconds fallback) {
    const auto value = runtime.get(field, static_cast<int>(fallback.count()));
    if (value <= 0)
      runtime.fail(field, "must be positive");
    return std::chrono::milliseconds(value);
  };
  config.fire_timeout = timeout("fire_timeout_ms", config.fire_timeout);
  config.control_timeout = timeout("control_timeout_ms", config.control_timeout);
  config.feedback_timeout = timeout("feedback_timeout_ms", config.feedback_timeout);
  config.plan_timeout = timeout("plan_timeout_ms", config.plan_timeout);
  return config;
}
}  // namespace

ApplicationConfig load_application_config(const std::filesystem::path & path)
{
  const tools::YamlConfig yaml(path);
  ApplicationConfig config;
  config.runtime = parse_runtime(yaml);
  config.visualization = visualization::RuntimeVisualizationConfig::parse(yaml);
  config.diagnostics = PlannerDiagnosticsConfig::parse(yaml);
  config.detector = YoloConfig::parse(yaml);
  config.solver = SolverConfig::parse(yaml);
  config.tracker = TrackerConfig::parse(yaml);
  config.planner = PlannerConfig::parse(yaml);
  if (!config.runtime.talos_backend)
    config.camera = io::CameraConfig::parse(yaml);
  if (config.runtime.simulate_gimbal || config.runtime.talos_backend)
    config.gimbal.simulate = true;
  else
    config.gimbal = io::GimbalConfig::parse(yaml);
  return config;
}
}  // namespace auto_aim::runtime
