# Repository Guidelines

## Project Structure & Module Organization
This is a C++17 vision stack built with CMake. Application entry points live in `src/` (`infantry.cpp`, debug variants). Core feature modules are under `tasks/`: `auto_aim/` and `omniperception/`. Hardware and communication abstractions are in `io/`, shared utilities in `tools/`, calibration programs in `calibration/`, and runnable test programs in `tests/`. Robot and camera parameters are YAML files in `configs/`; model weights and demo data are in `assets/`. Treat `build/`, `install/`, and `log/` as generated output.

## Build, Test, and Development Commands
Install the SDKs and libraries described in `readme.md` first, including OpenCV, OpenVINO, Eigen, fmt, spdlog, yaml-cpp, and nlohmann-json. Build from the repository root:

```bash
cmake -B build
cmake --build build -j$(nproc)
```

Primary binaries are emitted under `build/`, for example:

```bash
./build/infantry configs/standard3.yaml
./build/camera_test
./build/gimbal_test
```

Use `cmake --build build --target <target>` for focused iteration, such as `camera_test` or `infantry_debug`.

## Coding Style & Naming Conventions
Follow the existing C++ style: two-space indentation, K&R braces for functions and control flow, `snake_case` for files, variables, and functions, and PascalCase for classes such as `Tracker` or `ThreadSafeQueue`. Keep headers beside their implementation files (`detector.hpp` with `detector.cpp`). Prefer existing helpers in `tools/` and established module boundaries before adding new utilities.

## Testing Guidelines
Tests are standalone CMake executables rather than a centralized CTest suite. Add new test files under `tests/` with names ending in `_test.cpp`, then register them in the root `CMakeLists.txt` with `add_executable` and `target_link_libraries`. Run the relevant binary directly from `build/`. Hardware-dependent tests should document required devices and config files in comments or usage output.

## Commit & Pull Request Guidelines
Recent history uses short, direct commit summaries, often in Chinese, without strict prefixes. Keep commits focused and describe the changed behavior, for example `修复相机线程退出逻辑` or `Add planner offline test`. Pull requests should include a concise summary, affected modules, build/test commands run, linked issues if any, and screenshots or logs when changing visualization, detection output, or hardware behavior.

## Security & Configuration Tips
Do not commit machine-specific secrets, absolute device IDs, or large generated logs. Keep reusable configuration in `configs/` and document robot-specific deviations. Verify OpenVINO path changes carefully because `CMakeLists.txt` currently points to `/opt/intel/openvino_2024.6.0/runtime/cmake/`.
