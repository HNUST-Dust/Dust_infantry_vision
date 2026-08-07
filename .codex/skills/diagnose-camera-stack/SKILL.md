---
name: diagnose-camera-stack
description: Diagnose HikRobot, MindVision, or USB camera discovery, SDK, streaming, frame, and frame-rate failures in this repository. Use when camera initialization fails, frames are empty, FPS is wrong, or exposure/gain/VID-PID settings look suspect.
---

# Diagnose Camera Stack

Use this skill for camera-only failures. Keep the diagnosis read-only until the user explicitly asks to change a device setting.

## Workflow

1. Establish the exact symptom and whether it reproduces with hardware disconnected. Record the config path, camera name, VID:PID, expected resolution/FPS, and the first error line.
2. Run `scripts/camera_preflight.sh [config]` from the repository root. Treat missing devices, permissions, SDK libraries, and model binaries as separate findings.
3. Confirm the build before touching hardware:
   - `cmake --build build-ninja --target camera_test -j$(nproc)`
   - `cmake --build build-ninja --target camera_detect_test -j$(nproc)`
   - `./build-ninja/camera_test -h`
4. Exercise the smallest relevant executable. `camera_test` proves acquisition; `camera_detect_test` adds the detector. Do not jump to `infantry` until both are understood.
5. Compare observed frame timestamps/FPS and image dimensions with `exposure_ms`, `gain` or `gamma`, `frame_rate`, `camera_name`, and `vid_pid`. For HikRobot also check `/opt/MVS/lib/64` and `LD_LIBRARY_PATH`; MindVision uses the repository SDK library.
6. If acquisition is healthy, hand off algorithm symptoms to `$replay-auto-aim`. If the failure is a config/model mismatch, hand off to `$audit-vision-config`.

## Evidence and stop conditions

- Capture command, exit status, stderr, device listing, and the first failing layer. Redact machine-specific serials and private paths when reporting.
- A successful process start is not proof of streaming. Require non-empty frames and a measured rate.
- Stop before changing exposure, gain, camera firmware, udev rules, or SDK installation. Ask the user before any privileged or hardware-mutating action.
- GUI tests need a display. Report `DISPLAY`/headless limitations instead of substituting an unverified result.

Read [camera-matrix.md](references/camera-matrix.md) for repository-specific facts and expected commands.
