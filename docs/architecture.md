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

### MPC 模式

主 YAML 的 `mpc_mode` 在启动时选择规划方式；缺键为 `measured_state`，standard3 显式启用 `reference_smoothing`。前者从实测状态出发输出 x[1]/u[0]；后者以当轮命中决策为第 30 列、使用参考首列初值，输出 x[30] 的位置供下位机位置控制器追赶，四个前馈字段置零。内部速度/加速度继续优化，约束只作用于模型轨迹。两种模式都保存固定绝对目标，并在每次发送时由最新反馈换算相对角及复核实际到位，保留现有所有超时门控。

平滑模式命令采样时刻 `command_at` 为本轮规划时刻；`valid_at` 仍为规划后 10ms 的发送截止。trace 的 mpc_sampling 区分实测反馈与 solver_x0，记录模式及时间轴、输出索引；optimized_feedforward 是内部优化值，output 是下发值。回放读取记录的初值与索引，旧记录缺元数据时沿用实测状态路径。

### 检测耗时遥测

每条遥测中的 `capture_time_ms`、`detection_submit_ms`、`inference_time_ms`、`inference_wait_ms`、`postprocess_time_ms` 和 `result_delivery_ms` 来自同一个检测结果。`detection_sequence` 标识该帧，`detection_frame_t` 是以遥测 `t` 为同一起点的帧时间戳，单位为秒。首次结果前，帧号和六项耗时均为 0，帧时间戳为 null。遥测采样读取最近一次结果，连续两行的检测帧号可能重复或跳号。

### 下位机弹速遥测

`/vision/telemetry` 的 `bullet_speed` 为下位机回传的最近一次实际弹速，单位 m/s，直接取自云台反馈，不使用规划器的弹速回退值。随现有遥测约 50 Hz 发布；在 Foxglove Plot 面板添加 `/vision/telemetry.bullet_speed` 即可查看曲线。两次弹速更新之间保持最近值；模拟云台模式下为模拟值。

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

### 按预测运动方向区分射击容差

`fire_directional_tolerance_enabled: true` 启用同板预测 yaw 方向门控；缺键为 false。
`fire_lag_tolerance_ratio: 0.5` 将运动后侧 yaw 额度缩为基础容差的一半，取值必须有限且在 (0,1]。
`fire_direction_min_yaw_rate: 0.05`（rad/s，有限且正数）区分可信静止和运动，独立于目标自转速度选档阈值。
四档基础容差与距离/自转速度选档继续使用原参数；前侧 yaw 与 pitch 半轴均为基础 R。
启用时基础容差必须为有限正数；ratio=1 或关闭开关恢复原对称到位判定。
配置在启动时读取，修改后需重新加载应用，不支持热更新。

内部误差 ey=wrap(实测yaw−预测弹道目标yaw)，ep=实测pitch−目标pitch。
以同一有效收敛 track 装甲板的局部预测 yaw 导数确定 s，a=s*ey；a<0 为落后、a>0 为前侧。
Ry 在后侧为 ratio*R、前侧为 R，到位要求 (ey/Ry)^2+(ep/R)^2<1，严格小于边界。
可信近零使用原对称圆；无可信相邻点或局部明显反向时，yaw 两侧均为 ratio*R、pitch 仍为 R。
换板、预瞄、回退及未收敛点不参与方向差分。方向不来自目标自转符号、实测云台速度或 MPC/下发速度。
新窗口为原窗口子集：它筛除跟随不足时的开火许可，不补偿云台控制滞后，也不增加预测提前量。

规划 settled、Plan::relative_to() 和发送端角度阻断诊断复用同一窗口算法。
实测模式 settled/发送参考分别为第0/1列，参考平滑模式均为第30列。
`fire_tolerance` 继续表示基础 R，相邻计划目标跳变的 2*R 检查保持原语义。
发送线程按最新反馈复核固定绝对目标；不能把规划阶段已拒绝的 fire 重新置为 true。

最终发送遥测新增 `send_fire_directional_enabled`、`send_fire_direction_state`（moving/stationary/unknown）、
`send_fire_yaw_direction`、`send_fire_reference_yaw_rate`（rad/s）、`send_fire_aligned_yaw_error`（rad，前正后负）、
`send_fire_yaw_tolerance`、`send_fire_lag_tolerance`（rad）、`send_fire_error_normalized_sq` 和 `send_fire_window_passed`。
现有 send_target_yaw 为目标减实测，与 ey 符号相反；moving 时 a=-s*send_target_yaw。
无有效目标时新增数值字段为 null；未知导数与非 moving 状态的 a 为 null。
原 send_fire_tolerance 仍为基础 R，不能单凭它推断整个窗口。
这些字段随同一 send_plan_id/send_sequence 快照发布，send_angle_outside 与该快照的窗口判定一致。

Planner trace 新增 `fire_windows`，分别保存 settled/command 的参考索引、绝对目标、方向、参数与判定结果；
配置快照包含三个新键，完整目标回放比较新增上下文并沿用构建指纹校验。
旧 trace 仍可在原有兼容范围内做轴回放，缺少新字段不能证明验证了方向门控。
仅比较窗口时应使用同一批发送反馈；不同实际运行不能把 mode=2 许可时间当命中率。
