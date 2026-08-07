# Replay Matrix

## Inputs

`assets/demo/demo.avi` contains frames and `assets/demo/demo.txt` contains timestamp plus quaternion columns `t w x y z`. `auto_aim_test` derives both paths from the input stem, so the normal input is `assets/demo/demo`.

## Commands

```bash
./build-ninja/detector_video_test --help
./build-ninja/detector_video_test -c configs/standard3.yaml -s 0 -e 100 assets/demo/demo.avi
./build-ninja/auto_aim_test --help
./build-ninja/auto_aim_test -c configs/standard3.yaml -s 0 -e 100 assets/demo/demo
./build-ninja/planner_test_offline configs/standard3.yaml -d 3.0 -w 5.0
```

The video tests are GUI-oriented. Confirm the executable's actual parser output with `--help` if a flag spelling differs from the local OpenCV parser version.

## Pipeline ownership

- `tasks/auto_aim/yolo*.cpp`: OpenVINO inference and postprocessing.
- `tasks/auto_aim/detector.cpp`: traditional light-bar/armor geometry.
- `tasks/auto_aim/solver.cpp`: camera-to-gimbal/world pose and reprojection.
- `tasks/auto_aim/tracker.cpp`: lost/detecting/tracking/temp_lost/switching state machine and EKF updates.
- `tasks/auto_aim/aimer.cpp`: aim point and command/fire decision inputs.
- `tasks/auto_aim/planner/planner.cpp`: TinyMPC trajectory and fire thresholds.
