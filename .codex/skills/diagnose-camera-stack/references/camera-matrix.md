# Camera Matrix

## Source of truth

- Factory selection is in `io/camera.cpp`.
- HikRobot implementation: `io/hikrobot/hikrobot.cpp`; MindVision implementation: `io/mindvision/mindvision.cpp`.
- Shared settings are read from YAML with `tools/yaml.hpp`.
- The documented default is `configs/standard3.yaml`; it selects `hikrobot`, `vid_pid: "2bdf:0001"`, `exposure_ms: 1.0`, `gain: 16.9`, and `frame_rate: 165`.

## Checks

| Layer | Check | Expected evidence |
| --- | --- | --- |
| USB | `lsusb` | The configured VID:PID is present |
| SDK | `ldd build-ninja/camera_test` | HikRobot/MindVision and libusb dependencies resolve |
| Permissions | `stat` on device or SDK path | Current user can access the transport |
| Build | `cmake --build build-ninja --target camera_test` | Target links successfully |
| Stream | `camera_test` | Non-empty frames and stable timestamps |
| Detection | `camera_detect_test` | Frames reach the detector without camera errors |

Never infer a camera fault from a detector-only error. Isolate acquisition first.
