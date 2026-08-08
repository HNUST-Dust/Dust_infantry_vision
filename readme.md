# Dust Infantry Vision

这是一个基于 C++17、OpenCV、OpenVINO 和 Eigen 的步兵视觉系统，包含工业相机取流、YOLO 装甲板检测、数字识别、PnP 空间解算、EKF 目标跟踪、TinyMPC 云台轨迹规划、串口通信以及相机/手眼标定工具。

当前仓库以自瞄主链路为完整可用状态，`tasks/omniperception/` 作为全向感知模块参与构建，但主入口尚未直接启用完整多相机链路。

## 快速开始

### 1. 准备代码

将整个仓库复制到目标电脑，并保证模型、配置和演示数据都在仓库内。所有模型路径和部分配置路径是相对仓库根目录的，因此请始终在仓库根目录执行构建和运行命令。

```bash
cd /path/to/Dust_Infantry_vision
ls assets/demo/demo.avi
```

如果 `assets/demo/demo.avi` 不存在，演示程序无法运行，但不影响构建。

### 2. 构建

首次构建前需要安装依赖，见下方“新电脑环境配置”。依赖就绪后执行：

```bash
cmake -B build-ninja -G Ninja
cmake --build build-ninja -j$(nproc)
```

如果直接执行 `cmake -B build` 而没有指定 `-G Ninja`，CMake 默认会生成 Makefile。README 使用 Ninja 构建目录 `build-ninja/`，后续命令也都从该目录运行可执行文件。

也可以只编译某个目标：

```bash
cmake --build build-ninja --target infantry -j$(nproc)
cmake --build build-ninja --target auto_aim_test -j$(nproc)
```

可执行文件输出在 `build-ninja/` 目录。主要程序包括：

- `build-ninja/infantry`
- `build-ninja/infantry_debug`
- `build-ninja/camera_test`
- `build-ninja/camera_detect_test`
- `build-ninja/detector_video_test`
- `build-ninja/auto_aim_test`
- `build-ninja/gimbal_test`
- `build-ninja/cboard_test`
- `build-ninja/planner_test`
- `build-ninja/planner_test_offline`

### 3. 无硬件快速验证

只验证命令入口：

```bash
./build-ninja/infantry -h
./build-ninja/auto_aim_test -h
```

用仓库内置视频验证 YOLO、分类器、Tracker 和 Aimer 主链路。此命令会打开 OpenCV 显示窗口，需要桌面环境或远程桌面：

```bash
./build-ninja/auto_aim_test configs/standard3.yaml assets/demo/demo
```

如果电脑没有 Intel GPU 或 OpenCL，先修改 `configs/standard3.yaml`：

```yaml
device: CPU
```

只验证 Planner 和配置文件解析，不依赖相机、云台或显示窗口：

```bash
./build-ninja/planner_test_offline configs/standard3.yaml
```

该程序会持续运行并发送调试数据到 UDP `127.0.0.1:9870`，用 `Ctrl+C` 退出。

### 4. 实机运行

实机运行前确认：

- 相机型号和 `vid_pid` 与 `configs/standard3.yaml` 一致。
- 云台串口存在 `/dev/gimbal`，当前用户有读写权限。
- OpenVINO 设备可用，或者已将 `device` 改为 `CPU`。

运行生产主程序：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
./build-ninja/infantry configs/standard3.yaml
```

需要可视化调试窗口和 EKF 日志时运行：

```bash
./build-ninja/infantry_debug configs/standard3.yaml
```

单独测试相机：

```bash
./build-ninja/camera_test configs/standard3.yaml
```

## 新电脑环境配置

### 系统要求

推荐环境：

- Ubuntu 22.04 x86_64
- GCC / G++，支持 C++17
- CMake 3.16.3 或更高
- OpenCV 4.x
- OpenVINO 2024.6.0
- libusb-1.0
- ccache

仓库的相机 SDK 目录包含 `amd64` 和 `arm64` 两个架构的库，但项目主要在 Ubuntu 22.04 x86_64 上验证。

### 安装系统依赖

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake ninja-build ccache pkg-config \
  libopencv-dev libeigen3-dev libfmt-dev libspdlog-dev \
  libyaml-cpp-dev nlohmann-json3-dev libusb-1.0-0-dev \
  libgtk-3-dev ocl-icd-libopencl1
```

`ocl-icd-libopencl1` 用于 OpenVINO GPU 插件。如果只在 CPU 上运行，可以省略，但需要把配置文件中的 `device` 改为 `CPU`。

### 安装 OpenVINO 2024.6.0

项目 CMake 目前固定查找：

```text
/opt/intel/openvino_2024.6.0/runtime/cmake
```

因此新电脑上最简单的方式是把 OpenVINO 安装到相同路径，或者通过软链接指向实际安装目录。

Ubuntu 22.04 x86_64 官方归档包名为：

```text
l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz
```

可尝试从 OpenVINO 官方存储下载：

```bash
wget \
  https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6.0/l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz
```

下载完成后解压并建立软链接：

```bash
sudo mkdir -p /opt/intel
sudo tar -xzf l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz -C /opt/intel
sudo ln -sfn \
  /opt/intel/l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64 \
  /opt/intel/openvino_2024.6.0
```

每次新终端运行项目前建议加载 OpenVINO 环境：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
```

也可以把这一行加入 `~/.bashrc`。如果 OpenVINO 安装在其他目录且无法软链接到 `/opt/intel/openvino_2024.6.0`，需要同步修改三处 CMake 文件中的 `OpenVINO_DIR`：

- `CMakeLists.txt`
- `tasks/auto_aim/CMakeLists.txt`
- `tasks/omniperception/CMakeLists.txt`

### 相机 SDK 依赖

- HikRobot 相机：仓库 `io/hikrobot/lib/<arch>/libMvCameraControl.so` 可用于编译链接，但 USB 相机的运行时传输层通常还需要 HikRobot MVS SDK。建议在目标电脑安装 MVS SDK 到 `/opt/MVS`，并设置：

```bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
```

- MindVision 相机：`io/mindvision/lib/<arch>/libMVSDK.so` 已随仓库提供，不需要额外安装系统级 SDK。

### 串口和权限

当前 `configs/standard3.yaml` 使用：

```yaml
com_port: "/dev/gimbal"
```

如果目标电脑没有这个设备节点，需要为云台或主控板串口添加 udev 规则。例如，根据实际 USB 串口的 VID/PID 创建规则：

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="gimbal", MODE="0666", GROUP="dialout"
```

写入 `/etc/udev/rules.d/99-gimbal.rules` 后执行：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

同时确保当前用户属于 `dialout` 组，并重新登录：

```bash
sudo usermod -aG dialout $USER
```

## 项目结构

```text
src/                      主程序入口
  infantry.cpp            生产运行入口
  infantry_debug.cpp      调试运行入口
tasks/
  auto_aim/               自瞄核心算法
  omniperception/         全向感知模块
io/                       硬件和通信抽象
  camera.cpp              工业相机统一接口
  hikrobot/               HikRobot 相机驱动
  mindvision/             MindVision 相机驱动
  usbcamera/              USB/V4L2 相机驱动
  gimbal/                 云台串口收发
  cboard.cpp              主控板/IMU 串口
  dm_imu/                 DM 系列 IMU
  serial/                 跨平台串口库
  socketcan.hpp           SocketCAN 工具
tools/                    通用工具
  plotter.cpp             UDP 调试数据发送
  extended_kalman_filter  EKF 实现
  thread_safe_queue       线程安全队列
  thread_pool             线程池
  logger.cpp              日志封装
  trajectory.cpp          弹道模型
  pid.cpp                 PID
calibration/              标定工具
configs/                  YAML 配置
assets/                   模型、演示数据
tests/                    调试和测试程序
```

## 核心流程

主自瞄链路大致为：

```text
工业相机取流
  -> YOLO / 传统灯条检测
  -> 数字识别与装甲板分类
  -> 敌我颜色过滤、目标优先级
  -> PnP 求解装甲板空间位姿
  -> EKF 跟踪目标位置、速度、角速度
  -> 弹道预测与 TinyMPC 轨迹规划
  -> 串口下发云台角度、角速度、角加速度和射击信号
```

主要组件职责：

- `Detector`：传统方法提取灯条并组合装甲板，可对 YOLO 结果做几何修正。
- `YOLO`：通过 `NetDetector` 统一 letterbox、OpenVINO 预处理和 `InferRequest` 池；YOLOv5、YOLOv8、YOLO11 适配器只负责输出解码。
- `MultiThreadDetector`：采集线程使用有界异步请求入口，推理槽位耗尽时丢弃新帧，消费线程等待完成后继续跟踪。
- `Classifier`：使用 `assets/tiny_resnet.onnx` 识别装甲板数字。
- `Solver`：结合相机内参和云台外参，将装甲板从像素坐标解算到云台/世界坐标。
- `Tracker`：维护 lost/detecting/tracking/temp_lost/switching 状态机，使用 EKF 预测旋转目标。
- `Aimer` / `Shooter`：选择瞄准点并判断是否满足射击条件。
- `Planner`：使用 TinyMPC 求解 yaw/pitch 参考轨迹，输出控制量和射击标志。
- `Gimbal`：通过串口读取四元数、云台状态和弹速，发送视觉控制帧。
- `Plotter`：将 JSON 调试数据发送到 UDP `127.0.0.1:9870`。

## 配置文件

### `configs/standard3.yaml`

这是当前机器人配置，不适合直接用于所有电脑或所有机器人。部署到新机器人时建议复制一份，再修改相机、串口、标定和射击参数。

常用字段：

| 分类 | 字段 | 说明 |
| --- | --- | --- |
| 识别 | `enemy_color` | `red` 或 `blue` |
| 识别 | `yolo_name` | `yolov5`、`yolov8` 或 `yolo11` |
| 识别 | `yolov5_model_path` / `yolov8_model_path` / `yolo11_model_path` | 对应 OpenVINO IR 模型 |
| 识别 | `classify_model` | 数字识别 ONNX 模型 |
| 识别 | `device` | OpenVINO 设备，如 `GPU`、`CPU`、`AUTO` |
| 识别 | `infer_request_buffer_num` | 可并行复用的 OpenVINO 请求数，默认 `2` |
| 识别 | `min_confidence` | 目标最低置信度 |
| 识别 | `use_traditional` | YOLOv5 是否使用传统方法修正角点 |
| 相机 | `camera_name` | `hikrobot` 或 `mindvision` |
| 相机 | `exposure_ms` / `gain` / `frame_rate` | 相机曝光、增益、帧率 |
| 相机 | `vid_pid` | USB VID:PID，例如 `2bdf:0001` |
| 标定 | `camera_matrix` / `distort_coeffs` | 相机内参和畸变系数 |
| 标定 | `R_gimbal2imubody` | 云台坐标系到 IMU 机体系的旋转 |
| 标定 | `R_camera2gimbal` / `t_camera2gimbal` | 相机到云台的外参 |
| 串口 | `com_port` | 云台串口节点 |
| 跟踪 | `min_detect_count` / `max_temp_lost_count` | Tracker 状态机参数 |
| 射击 | `yaw_offset` / `pitch_offset` | 枪管相对瞄准点的机械补偿 |
| 射击 | `auto_fire` | 是否由视觉控制射击 |
| 规划 | `fire_thresh_high_speed` / `fire_thresh_low_speed` | 射击阈值 |
| 规划 | `max_yaw_acc` / `max_pitch_acc` | 云台角加速度约束 |
| 规划 | `Q_yaw` / `R_yaw` / `Q_pitch` / `R_pitch` | TinyMPC 权重 |

### `configs/calibration.yaml`

标定工具使用的配置，包含标定板规格、相机参数和云台到 IMU 的旋转矩阵。

## 常用测试程序

| 程序 | 用途 | 是否需要硬件 |
| --- | --- | --- |
| `infantry` | 生产自瞄主程序 | 需要相机和云台 |
| `infantry_debug` | 带调试窗口和 EKF 日志的主程序 | 需要相机和云台 |
| `camera_test` | 相机取流和帧率测试 | 需要相机 |
| `camera_detect_test` | 相机实时检测测试 | 需要相机 |
| `detector_video_test` | 视频文件检测测试 | 不需要相机，可播放 AVI |
| `auto_aim_test` | 内置演示视频自瞄链路测试 | 不需要相机/云台，需要显示环境 |
| `gimbal_test` | 云台串口收发测试 | 需要云台 |
| `cboard_test` | 主控板串口测试 | 需要主控板 |
| `planner_test` | Planner 实机测试 | 需要云台 |
| `planner_test_offline` | Planner 离线模拟 | 无 |
| `handeye_test` | 手眼标定结果验证 | 需要相机和主控板 |

测试程序是独立 CMake 可执行文件，没有统一注册到 CTest。每个程序通常支持 `-h` 查看参数。

## 标定工具

### 采集标定数据

```bash
./build-ninja/capture configs/calibration.yaml
```

程序会显示相机画面，按 `s` 保存图片和当前云台四元数，按 `q` 退出。输出目录默认是 `assets/img_with_q`。

### 相机内参标定

```bash
./build-ninja/calibrate_camera configs/calibration.yaml <图片文件夹>
```

### 手眼标定

```bash
./build-ninja/calibrate_handeye configs/calibration.yaml <图片文件夹>
```

## 常见问题

### CMake 找不到 OpenVINO

检查是否存在：

```bash
ls /opt/intel/openvino_2024.6.0/runtime/cmake/OpenVINOConfig.cmake
```

不存在时，按上方 OpenVINO 安装步骤解压并建立软链接，或者修改三处 CMake 文件中的 `OpenVINO_DIR`。

### 运行时报 GPU 或 OpenCL 错误

`configs/standard3.yaml` 默认 `device: GPU`。没有 Intel GPU 或没有 OpenCL 驱动时，改为：

```yaml
device: CPU
```

### 找不到 HikRobot 相机动态库

确认 MVS SDK 已安装，并设置：

```bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
```

### 串口打开失败

确认 `/dev/gimbal` 存在、当前用户属于 `dialout` 组，且 udev 规则已生效：

```bash
ls -l /dev/gimbal
id
```

### 模型或配置路径加载失败

始终在仓库根目录运行程序，因为配置中的 `assets/...` 是相对路径。

## 开发约定

- 构建产物集中在 `build/`、`build-ninja/`、`install/`、`log/`，不要提交。
- 代码风格和提交规范以仓库根目录 `AGENTS.md` 为准。
- 新增测试放在 `tests/`，命名以 `_test.cpp` 结尾，并在根目录 `CMakeLists.txt` 注册。
- 不要提交机器人私有的设备 ID、绝对路径和大型日志。
