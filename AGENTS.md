# Repository Guidelines

## Project Structure & Module Organization
This is a C++17 vision stack built with CMake. Application entry points live in `src/` (`infantry.cpp`, `infantry_debug.cpp`). Core feature modules are under `tasks/`: `auto_aim/` and `omniperception/`. Hardware and communication abstractions are in `io/`: camera drivers (`hikrobot/`, `mindvision/`, `usbcamera/`), gimbal, cboard/IMU, serial, and SocketCAN. Shared utilities are in `tools/`, calibration programs in `calibration/`, and runnable test programs in `tests/`. Robot and camera parameters are YAML files in `configs/`; model weights and demo data are in `assets/`. Treat `build-ninja/`, `build/`, `install/`, and `logs/` as generated output.

## Build, Test, and Development Commands
Install the SDKs and libraries described in `readme.md` first, including OpenCV, OpenVINO, Eigen, fmt, spdlog, yaml-cpp, nlohmann-json, ccache, and Ninja. Build from the repository root:

```bash
cmake -B build-ninja -G Ninja
cmake --build build-ninja -j$(nproc)
```

Binaries are emitted under `build-ninja/`, for example:

```bash
./build-ninja/infantry configs/standard3.yaml
./build-ninja/camera_test configs/standard3.yaml
./build-ninja/planner_test_offline configs/standard3.yaml
```

Use `cmake --build build-ninja --target <target>` for focused iteration, such as `infantry`, `auto_aim_test`, or `planner_test_offline`.

## Coding Style & Naming Conventions
Follow the existing C++ style: two-space indentation, K&R braces for functions and control flow, `snake_case` for files, variables, and functions, and PascalCase for classes such as `Tracker` or `ThreadSafeQueue`. Keep headers beside their implementation files (`detector.hpp` with `detector.cpp`). Prefer existing helpers in `tools/` and established module boundaries before adding new utilities.

## Testing Guidelines
Tests are standalone CMake executables rather than a centralized CTest suite. Add new test files under `tests/` with names ending in `_test.cpp`, then register them in the root `CMakeLists.txt` with `add_executable` and `target_link_libraries`. Run the relevant binary directly from `build-ninja/`. Prefer offline tests where possible; hardware-dependent tests should document required devices and config files in comments or usage output.

## Commit & Pull Request Guidelines
Recent history uses short, direct commit summaries, often in Chinese, without strict prefixes. Keep commits focused and describe the changed behavior, for example `修复相机线程退出逻辑` or `Add planner offline test`. Pull requests should include a concise summary, affected modules, build/test commands run, linked issues if any, and screenshots or logs when changing visualization, detection output, or hardware behavior.

## Security & Configuration Tips
Do not commit machine-specific secrets, absolute device IDs, or large generated logs. Keep reusable configuration in `configs/` and document robot-specific deviations. OpenVINO is pinned to `/opt/intel/openvino_2024.6.0/runtime/cmake` in `CMakeLists.txt`, `tasks/auto_aim/CMakeLists.txt`, and `tasks/omniperception/CMakeLists.txt`; update all three together when changing the SDK path.

## AI Memory Workflow
Use `.codex/memory.md` as local private memory for AI sessions in this repository. At the start of each new conversation or task, read it before planning or editing. At the end of each meaningful conversation, update it with a concise summary of goals, decisions, files touched, commands/tests run, and unresolved next steps. Keep at most 10 recent conversation entries; newer entries should be more detailed, while older entries should be compressed or promoted into long-term preferences/project facts before removal. Do not store secrets, private device IDs, full logs, or raw chat transcripts.
