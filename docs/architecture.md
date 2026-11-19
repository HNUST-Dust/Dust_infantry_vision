# 构建与运行架构

## 构建依赖

- `tools_core` 提供队列、顺序交付和四元数缓存等头文件接口，依赖标准库、Eigen 和线程支持。`tools` 在此基础上提供数学、图像和诊断实现。
- `camera_interface` 提供 `CameraBase`，接口依赖 OpenCV Core。`camera_io` 根据配置选择启用的 `hikrobot_camera` 或 `mindvision_camera`。USB 相机实现可通过 `usb_camera` 单独链接。
- `gimbal_io` 和 `dm_imu_io` 各自链接 `serial`。启用模拟器时，`talos_io` 链接 Talos IPC。
- `aim_model` 提供装甲板数据。解算和 ROI 分属 `aim_geometry` 和 `aim_roi`，两者供 `aim_tracking` 使用。`aim_planner` 依赖跟踪模块和 TinyMPC。
- `aim_traditional_detector` 使用 OpenCV DNN。`aim_detection` 组合传统检测、网络推理和异步交付。OpenVINO 和可选 TensorRT 由网络检测构建分支查找并私有链接。
- `infantry_runtime` 装配运行链路。可选的 Foxglove 传输只在启用对应应用或标定功能时配置。

禁用 `ENABLE_HIKROBOT` 后可用系统 libusb 构建 MindVision。两个驱动同时启用时，MindVision 链接 MVS 自带的 libusb，以匹配 MVS 运行时。默认的 HikRobot 构建从 `HIKROBOT_MVS_ROOT` 查找 MVS，默认路径是 `/opt/MVS`。

## 启动配置

`src/app_config.cpp` 中的 `load_application_config` 读取一次主 YAML，调用各模块旁的配置解析函数，随后把配置对象交给构造函数。解析错误包含文件路径和字段。启动配置根据 IO 后端决定是否解析相机配置，根据模拟云台开关决定是否解析串口配置。

已有 YAML 键名和单位继续使用。模型相对路径优先按启动目录解析，找不到时再按 YAML 所在目录解析。日志和 trace 的输出目录沿用启动目录。独立调试程序可以继续使用接受 YAML 路径的构造入口。

## 运行与输出

`src/auto_aim_runtime.cpp` 装配检测、跟踪、规划和 IO，管理采集、规划、发送和遥测线程，以及停止和排空顺序。遥测线程采样后将 `TelemetrySnapshot` 交给 `src/diagnostics/telemetry.cpp` 转为 JSON，再发送给 Plotter 和可选的 Foxglove。规划追踪由 `planner_diagnostics` 管理。

`RuntimeVisualizer` 根据本地窗口开关和远程订阅决定是否保留源图，绘制叠加层并发布图像。绘制时借用检测帧和跟踪数据。Foxglove 队列持有自己的图像引用，本地缩放在发布之后执行。

### 检测耗时遥测

每条遥测中的 `capture_time_ms`、`detection_submit_ms`、`inference_time_ms`、`inference_wait_ms`、`postprocess_time_ms` 和 `result_delivery_ms` 来自同一个检测结果。`detection_sequence` 标识该帧，`detection_frame_t` 是以遥测 `t` 为同一起点的帧时间戳，单位为秒。首次结果前，帧号和六项耗时均为 0，帧时间戳为 null。遥测采样读取最近一次结果，连续两行的检测帧号可能重复或跳号。

### 最终发送遥测

`/vision/telemetry` 原有的 `mode`、`fire` 仍表示规划结果。`send_mode` 表示发送线程完成相对角更新、角度检查和超时门控后交给 IO 的模式（0 停止控制、1 控制但不允许开火、2 允许开火），不代表下位机已收到或实际发弹。

发送线程每轮复制一份完整快照，遥测线程约 50 Hz 读取。`send_valid` 表示至少已有一次发送；`send_sequence` 是发送序号；`send_t` 是发送调用前的单调时间，单位秒，与遥测 `t` 共用起点。`send_plan_id` 对应同次运行 Planner trace 的 `plan_id`，不是检测帧编号；trace 的相对时间另有起点，应按 ID 关联。

`send_target_yaw`、`send_target_pitch` 是使用该次发送反馈计算的目标角误差，`send_fire_tolerance` 是该计划的开火容差，单位均为 rad。有效性由 `send_target_valid` 表示；无有效目标时三项为 null。超时门控清空控制指令时，仍保留门控前的有效误差，便于分析拒绝原因。

`send_fire_block_mask` 可同时包含多个未满足条件，另有对应的 `send_block_*` 布尔字段供 Plot 直接选择：

| 位值 | 布尔字段后缀 | 含义 |
| --- | --- | --- |
| 1 | `no_control` | 无有效相对控制计划 |
| 2 | `planner_no_fire` | 规划结果未允许开火；结合 trace 检查选板、预瞄和规划门控 |
| 4 | `angle_outside` | 发送时合成目标角误差不小于开火容差 |
| 8 | `no_plan` | 尚无生成的计划 |
| 16 | `plan_expired` | 计划生成时间或有效期检查不通过 |
| 32 | `feedback_stale` | 云台反馈有效性或新鲜度检查不通过 |
| 64 | `no_observation` | 无真实匹配观测时间 |
| 128 | `observation_control_stale` | 真实观测超过控制时限 |
| 256 | `observation_fire_stale` | 真实观测超过开火时限；现有 `send_status` 此时仍可能为 ready |

`send_total_time_s` 和 `send_mode2_time_s` 在约 500 Hz 的发送循环累计。上一模式占有到下一次发送调用前的时间间隔；从首次发送开始计时，不含启动等待。50 Hz 遥测之间短暂出现的 mode=2 也会被计入。区间许可占比使用 `Δsend_mode2_time_s / Δsend_total_time_s`；最新模式快照本身不能完整呈现两条遥测之间的每次切换。

## 构建和验证

预设用到 CMake 3.21+、Ninja 和 C++17 工具链。从仓库根目录执行：

| 预设 | 依赖和适用范围 |
| --- | --- |
| `tools-tests` | Eigen 和线程支持。运行标为 `unit` 的通用回归 |
| `offline-tests` | OpenCV、yaml-cpp、fmt、spdlog、nlohmann-json 和仓库内 TinyMPC。运行通用与离线回归，其中 `detector_roi` 需要 `configs/standard3.yaml` 指向的分类模型 |
| `robot` | 加入应用、OpenVINO、相机驱动、标定和手动硬件程序。CTest 可离线运行，`runtime_data` 读取配置和模型路径并验证遥测 |

```bash
cmake --preset tools-tests
cmake --build --preset tools-tests
ctest --preset tools-tests

cmake --preset offline-tests
cmake --build --preset offline-tests --parallel 5
ctest --preset offline-tests

cmake --preset robot
cmake --build --preset robot --parallel 6
ctest --preset robot
```

`ctest --preset <预设> -N` 列出已注册回归。配置 `robot` 时可另加 `-DBUILD_PERFORMANCE_TOOLS=ON` 编译推理基准。硬件诊断由 `BUILD_HARDWARE_TOOLS` 控制，按设备要求手动运行。输入和设备要求见 [测试与调试程序](../readme.md#测试与调试程序)。
