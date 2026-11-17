# Repository Guidelines

## Project Structure & Module Organization
This is a C++17 vision stack built with CMake. The single application entry point is `src/infantry.cpp` (it serves both production and local-window debugging). Core feature modules are under `tasks/`: `auto_aim/` and `omniperception/`. Hardware and communication abstractions are in `io/`: camera drivers (`hikrobot/`, `mindvision/`, `usbcamera/`), gimbal, DM IMU, and serial. Shared utilities are in `tools/`, calibration programs in `calibration/`, and runnable test programs in `tests/`. Robot and camera parameters are YAML files in `configs/`; model weights and demo data are in `assets/`. Treat `build-ninja/`, `build/`, `install/`, and `logs/` as generated output.

## Build, Test, and Development Commands
Run the SDK-free regression suite from the repository root:

```bash
cmake --preset tools-tests
cmake --build --preset tools-tests
ctest --preset tools-tests
```

Read [Build and runtime architecture](docs/architecture.md) when changing build dependencies,
startup configuration, or runtime outputs. It lists the offline and robot presets and their
dependencies. For SDK installation and device-specific setup, see [readme.md](readme.md).

## Coding Style & Naming Conventions
Use clang-format 14 with the root `.clang-format`: two-space indentation, a 100-column limit, new-line braces for functions/types/namespaces and attached braces for control flow. Run `./scripts/format.sh --write` to format maintained sources and `./scripts/format.sh --check` before submitting changes; the script defines the scope and excludes vendor code, including TinyMPC. Keep formatting-only changes separate from functional changes. Use `snake_case` for files, variables, and functions, and PascalCase for classes such as `Tracker` or `ThreadSafeQueue`. Keep headers beside their implementation files (`detector.hpp` with `detector.cpp`). Prefer existing helpers in `tools/` and established module boundaries before adding new utilities.

## Testing Guidelines
Place new `*_test.cpp` files under the relevant directory in `tests/` and register
executable targets in `tests/CMakeLists.txt`. Register unattended regressions with CTest using
the matching label and a timeout. For tests that need files, declare the inputs and working
directory. Build manual hardware programs with `BUILD_HARDWARE_TOOLS` and document the
devices and config required to run each executable.

## Commit & Pull Request Guidelines
Recent history uses short, direct commit summaries, often in Chinese, without strict prefixes. Keep commits focused and describe the changed behavior, for example `修复相机线程退出逻辑` or `Add planner offline test`. Pull requests should include a concise summary, affected modules, build/test commands run, linked issues if any, and screenshots or logs when changing visualization, detection output, or hardware behavior. Commit messages must not carry AI co-author or generator attribution trailers (for example `Co-Authored-By: Claude <noreply@anthropic.com>`); keep only a summary and, when useful, a body.

## Security & Configuration Tips
Do not commit machine-specific secrets, absolute device IDs, or large generated logs. Keep reusable configuration in `configs/` and document robot-specific deviations. Set `-DOpenVINO_DIR=...` at configure time to override the OpenVINO package path used
by the inference target.

## AI Memory Workflow
The files below are local private memory and are ignored by Git:

- `.codex/memory.md`: stable project facts and durable decisions only; keep it concise.
- `.codex/current.md`: the active task, current state, files involved, validation, blockers, and exact next steps.
- `.codex/current.json`: a small machine-readable copy of the active task state.
- `.codex/sessions/YYYY-MM-DD-topic.md`: one retrospective summary for each meaningful task or conversation.

At the start of a conversation or task, read `.codex/memory.md` and `.codex/current.md` before planning or editing. Read only the relevant files under `.codex/sessions/` when historical reasoning is needed; do not load the complete archive by default.

Update `.codex/current.md` and `.codex/current.json` while work is in progress, especially after a material decision, completed phase, or newly discovered blocker. At the end of a meaningful task, write or update its session summary with the goal, decisions, files touched, commands or tests run, outcome, and unresolved next steps. Promote only information that will remain useful across multiple future tasks into `.codex/memory.md`; replace or remove stale facts rather than appending conflicting statements.

Do not store secrets, credentials, private device identifiers, complete logs, generated artifacts, or raw chat transcripts. Keep memory summaries factual and compact. Never force-add or commit these ignored memory files.
