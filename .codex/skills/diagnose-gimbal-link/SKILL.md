---
name: diagnose-gimbal-link
description: Safely diagnose this repository's gimbal and cboard serial links, packet framing, CRC, timing, quaternion interpolation, and reconnect behavior. Use for /dev/gimbal failures, stale state, CRC errors, dropped packets, or incorrect control data.
---

# Diagnose Gimbal Link

This skill is for communication diagnosis. Default actions are read-only and must not move the gimbal or fire.

## Workflow

1. Classify the symptom: cannot open, no receive data, CRC failures, stale state, incorrect angles, failed writes, or reconnect loop. Record config path and the exact device node.
2. Run `scripts/gimbal_preflight.sh [config]`. It checks YAML, device existence, read/write permissions, and the built test binary without opening the port.
3. Verify the link in layers:
   - `cmake --build build-ninja --target gimbal_test -j$(nproc)`
   - `cmake --build build-ninja --target gimbal_response_test -j$(nproc)`
   - `cmake --build build-ninja --target cboard_test -j$(nproc)` when the cboard path is involved
4. Inspect `io/gimbal/gimbal.hpp` and `gimbal.cpp` before proposing a protocol change. The packed receive frame is header `SP`, quaternion `wxyz`, angle/velocity fields, bullet data, and CRC16; the transmit frame contains mode, yaw/pitch position, velocity, acceleration, and CRC16.
5. For angle or timing symptoms, compare the serial timestamp, `Gimbal::q()` interpolation interval, radians conversion, and the configured `R_gimbal2imubody`. Do not infer a calibration fault from a CRC or transport failure.
6. If the link is healthy but behavior is wrong in the vision pipeline, hand off to `$replay-auto-aim` or `$audit-vision-config`.

## Safety and evidence

- Never run `fire_test`, send ad-hoc frames, disable CRC, change udev rules, or restart hardware without explicit user approval.
- Treat `skip_cboard_crc` as a diagnostic exception only; report it and restore the normal CRC expectation after the experiment.
- Capture port path, permissions, error counts, received header/CRC summary, and process exit status. Redact host-specific identifiers.
- A successful serial open is not proof of a valid link. Require valid frames, CRC behavior, fresh timestamps, and plausible quaternion/state values.

Read [gimbal-protocol.md](references/gimbal-protocol.md) for repository-specific facts.
