# Repository Guidelines

## Project Structure & Module Organization
This is a C++17 vision stack built with CMake. Application entry points live in `src/` (`infantry.cpp`, `infantry_debug.cpp`). Core feature modules are under `tasks/`: `auto_aim/` and `omniperception/`. Hardware and communication abstractions are in `io/`: camera drivers (`hikrobot/`, `mindvision/`, `usbcamera/`), gimbal, DM IMU, and serial. Shared utilities are in `tools/`, calibration programs in `calibration/`, and runnable test programs in `tests/`. Robot and camera parameters are YAML files in `configs/`; model weights and demo data are in `assets/`. Treat `build-ninja/`, `build/`, `install/`, and `logs/` as generated output.

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
```

Use `cmake --build build-ninja --target <target>` for focused iteration, such as `infantry` or `auto_aim_test`.

## Coding Style & Naming Conventions
Follow the existing C++ style: two-space indentation, K&R braces for functions and control flow, `snake_case` for files, variables, and functions, and PascalCase for classes such as `Tracker` or `ThreadSafeQueue`. Keep headers beside their implementation files (`detector.hpp` with `detector.cpp`). Prefer existing helpers in `tools/` and established module boundaries before adding new utilities.

## Testing Guidelines
Tests are standalone CMake executables rather than a centralized CTest suite. Add new test files under `tests/` with names ending in `_test.cpp`, then register them in the root `CMakeLists.txt` with `add_executable` and `target_link_libraries`. Run the relevant binary directly from `build-ninja/`. Prefer offline tests where possible; hardware-dependent tests should document required devices and config files in comments or usage output.

## Commit & Pull Request Guidelines
Recent history uses short, direct commit summaries, often in Chinese, without strict prefixes. Keep commits focused and describe the changed behavior, for example `修复相机线程退出逻辑` or `Add planner offline test`. Pull requests should include a concise summary, affected modules, build/test commands run, linked issues if any, and screenshots or logs when changing visualization, detection output, or hardware behavior. Commit messages must not carry AI co-author or generator attribution trailers (for example `Co-Authored-By: Claude <noreply@anthropic.com>`); keep only a summary and, when useful, a body.

## Security & Configuration Tips
Do not commit machine-specific secrets, absolute device IDs, or large generated logs. Keep reusable configuration in `configs/` and document robot-specific deviations. OpenVINO is pinned to `/opt/intel/openvino_2024.6.0/runtime/cmake` by the single `set(OpenVINO_DIR ... CACHE PATH ...)` line in the root `CMakeLists.txt`; `tasks/auto_aim/CMakeLists.txt` and `tasks/omniperception/CMakeLists.txt` only call `target_link_libraries(... openvino::runtime)` and define no such variable, so there is nothing else to keep in sync. Because that variable is `CACHE` without `FORCE`, an existing build directory needs `-DOpenVINO_DIR=...` or a fresh configure to pick up a change.

## AI Memory Workflow
The files below are local private memory and are ignored by Git:

- `.codex/memory.md`: stable project facts and durable decisions only; keep it concise.
- `.codex/current.md`: the active task, current state, files involved, validation, blockers, and exact next steps.
- `.codex/current.json`: a small machine-readable copy of the active task state.
- `.codex/sessions/YYYY-MM-DD-topic.md`: one retrospective summary for each meaningful task or conversation.

At the start of a conversation or task, read `.codex/memory.md` and `.codex/current.md` before planning or editing. Read only the relevant files under `.codex/sessions/` when historical reasoning is needed; do not load the complete archive by default.

Update `.codex/current.md` and `.codex/current.json` while work is in progress, especially after a material decision, completed phase, or newly discovered blocker. At the end of a meaningful task, write or update its session summary with the goal, decisions, files touched, commands or tests run, outcome, and unresolved next steps. Promote only information that will remain useful across multiple future tasks into `.codex/memory.md`; replace or remove stale facts rather than appending conflicting statements.

Do not store secrets, credentials, private device identifiers, complete logs, generated artifacts, or raw chat transcripts. Keep memory summaries factual and compact. Never force-add or commit these ignored memory files.
