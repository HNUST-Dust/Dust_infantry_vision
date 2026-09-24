---
name: replay-auto-aim
description: Reproduce and isolate auto-aim detection, tracking, solving, aiming, and planning issues from recorded video in this repository. Use for false positives, missed armors, tracker state changes, aim jitter, fire decisions, or latency regressions.
---

# Replay Auto-Aim

Use recorded inputs before using a camera or gimbal. The goal is a tight, repeatable loop that can go red on the reported symptom.

## Workflow

1. State the symptom as an observable assertion: frame range, expected armor/target, tracker state, command tolerance, or timing threshold.
2. Verify `assets/demo/demo.avi` and `assets/demo/demo.txt`, select a small frame range, and run from the repository root.
3. Choose the narrowest executable:
   - `detector_video_test` for YOLO versus traditional detection and corner geometry;
   - `auto_aim_test` for YOLO, Solver, Tracker, and Aimer together — the widest offline replay;
   - there is **no** standalone offline planner executable in this checkout. `Planner`
     (`tasks/auto_aim/planner/planner.cpp`) is reachable only through `src/auto_aim_runtime.cpp`,
     the shared runtime behind the single entry `src/infantry.cpp`, and that runtime requires a
     live camera — `--simulate-gimbal` only swaps the gimbal for a simulated pose/serial path,
     not the camera. Observe trajectory and fire values instead through the `Plotter` UDP stream
     or the Foxglove `/vision/telemetry` keys `plan_yaw`, `plan_yaw_vel`, `plan_yaw_acc`,
     `plan_pitch`, `plan_pitch_vel`, `plan_pitch_acc`, `fire`, and `mode`
     (0 = IDLE, 1 = AUTO_AIM, 2 = FIRE); the Foxglove stream itself needs `--foxglove`.
4. If OpenVINO GPU is unavailable, use a temporary copied YAML with `device: CPU`; never overwrite the robot config during diagnosis.
5. Compare one variable at a time: model, detector mode, config threshold, frame interval, or calibration. Record frame index, detection count, tracker state, command, and stage timings.
6. Minimise the frame range until removing any remaining input makes the symptom disappear. Turn that replay into a regression test at a public seam before changing implementation.

## Evidence and boundaries

- `auto_aim_test` and `detector_video_test` use OpenCV windows and require a display; report headless limitations explicitly.
- `Plotter` sends JSON to UDP `127.0.0.1:9870`; capture or disable that observer only when it is part of the repro.
- Do not infer planner correctness from a process that merely stays alive. Check target yaw/pitch, trajectory values, and fire flag.
- Remove temporary debug logging and rerun the original frame range after the fix. Use `$diagnosing-bugs` for difficult reproductions; add the retained regression test as a standalone executable under `tests/` following the existing pattern, and register it in `tests/CMakeLists.txt` (that file already carries the three CTest cases).

Read [replay-matrix.md](references/replay-matrix.md) for command syntax and pipeline facts.
