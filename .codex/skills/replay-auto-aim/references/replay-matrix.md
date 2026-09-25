# Replay Matrix

## Inputs

`assets/demo/demo.avi` contains frames and `assets/demo/demo.txt` contains timestamp plus quaternion columns `t w x y z`. `auto_aim_test` derives both paths from the input stem, so the normal input is `assets/demo/demo`.

## Commands

```bash
./build-ninja/detector_video_test --help
./build-ninja/detector_video_test configs/standard3.yaml assets/demo/demo.avi --start-index=0 --end-index=100
./build-ninja/detector_video_test configs/standard3.yaml assets/demo/demo.avi --tradition
./build-ninja/auto_aim_test --help
./build-ninja/auto_aim_test configs/standard3.yaml assets/demo/demo --start-index=0 --end-index=100
```

**There is no offline planner binary.** The only `Planner` caller is `src/auto_aim_runtime.cpp`
(the shared runtime behind the single entry `src/infantry.cpp`), which needs a live camera —
verified: the runtime constructs `io::Camera`, and `runtime.simulate_gimbal: true` in the config only
swaps the gimbal for a simulated pose/serial path. Read trajectory and fire values from the `Plotter` UDP stream
(`127.0.0.1:9870`) or the Foxglove `/vision/telemetry` keys listed in `SKILL.md`. `Aimer`/`Shooter`,
which `auto_aim_test` and `minimum_vision_system` replay, are **not** the production planning path —
`readme.md` says so explicitly.

### Command-line form (verified 2026-09-24)

Both binaries take `config-path` and the video/input path as **positional** arguments; neither has
a `-c` option. `-s`/`--start-index` and `-e`/`--end-index` take values, and `-t`/`--tradition` is a
boolean. OpenCV 4.5.4's parser only accepts `--key=value`, so the space-separated form silently
misbehaves — it does not error, it corrupts the positional list:

- `-c configs/standard3.yaml` — `-c` is not a known option, so it is ignored entirely and
  `configs/standard3.yaml` is taken as `config-path` positionally. It happens to work, which is
  why the broken form survived; do not rely on it.
- `-s 0 -e 100` — this is the real hazard. `-s` and `-e` read back the string `"true"`, and `0`
  and `100` are swallowed as **positional** arguments, displacing `input-path`. The run then tries
  to open a video literally named `0` (visible as a GStreamer `unexpected reference "0"` warning)
  and replays nothing. Confirmed decisively by putting `-c` last:
  `auto_aim_test -s 0 -e 1 assets/demo/demo -c configs/standard3.yaml` dies with
  `YAML::BadFile: bad file: 0`, because `config-path` became `0`.

So: put both paths positionally, write every value-taking flag as `--key=value`, and write boolean
switches alone. The corrected form runs the full chain headlessly on the demo clip (model loads,
per-frame `yolo/tracker/aimer` timing is logged, exit 0); it only needs a display for its
`cv::imshow` overlay, matching the "requires a display" note in `readme.md`.

## Pipeline ownership

- `tasks/auto_aim/yolo*.cpp`: OpenVINO inference and postprocessing.
- `tasks/auto_aim/detector.cpp`: traditional light-bar/armor geometry.
- `tasks/auto_aim/solver.cpp`: camera-to-gimbal/world pose and reprojection.
- `tasks/auto_aim/tracker.cpp`: lost/detecting/tracking/temp_lost/switching state machine and EKF updates.
- `tasks/auto_aim/aimer.cpp`: aim point and command/fire decision inputs. Replayed offline by
  `auto_aim_test` and `minimum_vision_system`; **not** used by the production entry.
- `tasks/auto_aim/planner/planner.cpp`: TinyMPC trajectory and fire thresholds. Its only caller is
  `src/auto_aim_runtime.cpp`, reached from the single application entry `src/infantry.cpp` (a thin
  argument parser that fills `auto_aim::runtime::RuntimeOptions` and calls `run()`).
