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
