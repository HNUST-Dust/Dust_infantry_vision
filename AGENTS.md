# Repository Guidelines

This document describes how to contribute to `sp_vision`, the DUST infantry vision system (auto-aim, auto-buff, and omni-perception) for RoboMaster. It is a C++17, CMake-based project that runs without ROS, though optional ROS 2 message packages live under `src/`.

## Project Structure & Module Organization

- `tasks/` — feature modules: `auto_aim`, `auto_buff`, `omniperception`. Each is a CMake library (e.g. `add_library(auto_aim OBJECT ...)`).
- `io/` — hardware drivers: Hikrobot/MindVision/USB cameras, serial, gimbal, DM IMU, and the ROS 2 bridge.
- `tools/` — shared utilities: EKF, PID, logger, plotter, recorder, CRC, math helpers.
- `tests/` — standalone test programs, one per module (e.g. `auto_aim_test.cpp`).
- `calibration/` — camera, hand-eye, and robot-world hand-eye calibration executables.
- `configs/` — YAML runtime configuration (e.g. `standard3.yaml`); `assets/` — models and demo recordings; `install/` and `autostart.sh` — deployment and autostart.

## Build, Test, and Development Commands

Dependencies: OpenCV, fmt, Eigen3, spdlog, yaml-cpp, nlohmann-json, and OpenVINO 2024.6.0 at `/opt/intel/openvino_2024.6.0` (hardcoded in `CMakeLists.txt`).

```bash
cmake -B build                        # configure the build
make -C build -j$(nproc)              # compile all targets
./build/infantry_debug -c configs/standard3.yaml   # run the main program
./build/auto_aim_test                 # run a module test
```

## Coding Style & Naming Conventions

- Format with clang-format: 2-space indentation, Google-style braces. Preserve hand-aligned tables with `// clang-format off` / `// clang-format on`.
- Use `snake_case` for functions and variables, `PascalCase` for types, `SCREAMING_SNAKE` for constants (e.g. `COLORS`).
- Headers use `*_hpp` names and include guards such as `AUTO_AIM__ARMOR_HPP`; group code in namespaces (`auto_aim`, `tools`).
- Comments are commonly written in Chinese; match the language of the surrounding file.

## Testing Guidelines

Tests are standalone executables in `tests/`, registered in the root `CMakeLists.txt`, and named `*_test.cpp`. Build, then run the binary directly:

```bash
make -C build -j$(nproc)
./build/camera_test
./build/planner_test
```

There is no CI or coverage gate. Hardware-dependent behavior (cameras, gimbal, serial) must be verified on the robot, and the results documented in the pull request.

## Commit & Pull Request Guidelines

Git history uses short, single-line summaries describing the change, often in Chinese, e.g. `新前哨站自瞄` ("new outpost auto-aim"). Follow that pattern: one concise line stating what changed, with the body reserved for why.

Pull requests should state what and why, list changed configs/assets, and describe how the change was tested (simulated video vs. on-robot). Link the related issue when one exists.
