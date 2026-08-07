---
name: audit-vision-config
description: Audit YAML, model paths, OpenVINO device selection, camera and serial settings, and camera/gimbal calibration values for this repository. Use when parameters appear ignored, deployment differs between machines, or pose and model behavior are suspicious.
---

# Audit Vision Config

Use this skill before changing a threshold, model, device, or calibration number. Report facts and mismatches; do not silently rewrite the user's config.

## Workflow

1. Identify the exact YAML file and the executable that consumes it. Run from the repository root because model and demo paths are relative.
2. Compare the YAML against the actual reads in code, not just the README. Search `tools::read`, `yaml[`, and `tools::load` in the relevant module.
3. Check file existence and compatibility for `classify_model`, `yolov5_model_path`, `yolov8_model_path`, and `yolo11_model_path`; ensure `yolo_name` selects the intended model and `device` is available.
4. Validate camera fields (`camera_name`, `vid_pid`, exposure, gain/gamma, frame rate) and gimbal fields (`com_port`, optional CRC setting) without opening hardware. Hand hardware failures to `$diagnose-camera-stack` or `$diagnose-gimbal-link`.
5. Validate calibration structure and units: 3x3 `camera_matrix`, distortion vector, 3x3 rotations, 3-vector translation, and consistent coordinate/frame conventions. Check rotation orthonormality and determinant near `+1`; flag implausible values rather than auto-correcting them.
6. Check that OpenVINO lookup paths are consistent in root `CMakeLists.txt`, `tasks/auto_aim/CMakeLists.txt`, and `tasks/omniperception/CMakeLists.txt`.
7. Produce a report with `missing`, `unused`, `conditionally required`, and `suspicious value` categories. A syntactically valid YAML file is not a valid deployment configuration.

## Boundaries

- Never overwrite `configs/standard3.yaml` or calibration values during an audit.
- Use a copied temporary config for CPU fallback or an experiment, and identify every changed key.
- Do not call a calibration result correct solely because reprojection runs; preserve the measured error and coordinate-frame assumptions.

Read [config-map.md](references/config-map.md) for project-specific keys and checks.
