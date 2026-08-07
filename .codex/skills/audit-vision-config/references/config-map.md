# Configuration Map

## Main files

- `configs/standard3.yaml`: robot runtime configuration; default model is YOLOv5 and default device is GPU.
- `configs/calibration.yaml`: camera calibration tool configuration.
- `tools/yaml.hpp`: shared YAML loading/reading helpers.

## Conditional fields

| Consumer | Required or conditional keys |
| --- | --- |
| `io::Camera` | `camera_name`, `exposure_ms`, `vid_pid`; HikRobot requires `gain` and optionally `frame_rate`; MindVision requires `gamma` |
| `io::Gimbal` | `com_port`; optional `skip_cboard_crc` |
| YOLO factory | `yolo_name`, selected model path, `device`, `min_confidence` |
| Classifier | `classify_model` |
| Traditional detector | threshold, light-bar/armor geometry limits, ROI fields when enabled |
| Solver | `camera_matrix`, `distort_coeffs`, `R_camera2gimbal`, `t_camera2gimbal`, `R_gimbal2imubody` |
| Tracker | detection and temporary-lost limits, enemy color |
| Planner/Aimer | offsets, delay/speed thresholds, TinyMPC weights and acceleration limits |

## Structural checks

- `camera_matrix` and each rotation must contain 9 numbers; `t_camera2gimbal` must contain 3.
- Distortion coefficients must match the calibration model used by the calibration executable.
- A rotation should be orthonormal within measurement tolerance and have determinant near `+1`.
- Relative model paths must be resolved from the repository root. An existing file with the wrong model family is still a configuration error.
- The OpenVINO directory is intentionally repeated in three CMake files; report drift across them.
