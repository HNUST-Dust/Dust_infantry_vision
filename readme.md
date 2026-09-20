# Dust Infantry Vision

这是一个基于 C++17、OpenCV、OpenVINO 和 Eigen 的步兵视觉系统，包含工业相机取流、YOLO 装甲板检测、数字识别、PnP 空间解算、EKF 目标跟踪、TinyMPC 云台轨迹规划、串口通信以及相机/手眼标定工具。自瞄主链路使用带序号的有序异步检测，并按 Tracker 预测动态裁剪网络/灯条 ROI，减少误检并提升目标像素占比。

当前仓库以自瞄主链路为完整可用状态，`tasks/omniperception/` 保留源码、未接入默认构建，主入口尚未直接启用完整多相机链路。

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
- `build-ninja/detector_video_test`
- `build-ninja/auto_aim_test`
- `build-ninja/gimbal_test`

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

## 测试与调试程序

`tests/` 中的程序均作为独立可执行文件构建，主要用于算法回放、性能测量和硬件联调；除非特别说明，它们不会被 `ctest` 自动执行。

### 离线视觉与算法

- `auto_aim_test`：读取 `<input-path>.avi` 和同名 `.txt` 姿态记录，串联 YOLO、解算、跟踪、瞄准与开火判断，显示重投影和调试数据。

  ```bash
  ./build-ninja/auto_aim_test configs/standard3.yaml assets/demo/demo
  ```

- `detector_video_test`：对录像运行 YOLO 或传统灯条检测，输出装甲板四角点，适合比较两种检测路径。

  ```bash
  ./build-ninja/detector_video_test configs/standard3.yaml assets/demo/demo.avi
  ./build-ninja/detector_video_test configs/standard3.yaml assets/demo/demo.avi --tradition
  ```

### 有序交付与动态 ROI

- `ordered_delivery_test`：验证有序事件交付的容量限制、逆序完成、跳帧占位、关闭唤醒和严格序号语义，纯离线运行，无需参数。

  ```bash
  ./build-ninja/ordered_delivery_test
  ```

- `dynamic_roi_test`：验证网络 ROI 与灯条 ROI 的目标类型放大、边界裁剪、非法输入回退和丢失恢复。

  ```bash
  ./build-ninja/dynamic_roi_test
  ```

- `detector_roi_test`：在局部 ROI 内运行传统检测，验证灯条、角点与中心点回译到全图坐标。

  ```bash
  ./build-ninja/detector_roi_test configs/standard3.yaml
  ```

### 相机与实时检测

- `camera_test`：验证配置指定的工业相机采集，持续打印相邻帧 FPS；传入 `--display` 显示画面。

  ```bash
  ./build-ninja/camera_test configs/standard3.yaml --display
  ```

- `camera_thread_test`：创建多个 YOLO 实例并通过线程池并行处理工业相机帧，用于观察多线程检测吞吐与结果顺序。
- `minimum_vision_system`：硬件在环的小型完整链路，组合工业相机、DM IMU、异步检测、解算、跟踪、瞄准和射击判断；需要相机、DM IMU 和桌面显示环境。

### 异步推理与性能

- `async_detector_test`：验证 OpenVINO 异步请求池容量耗尽、后处理释放和请求复用，以及 `MultiThreadDetector` 的序号分配、跳帧/错误占位、关闭与有序排空语义。

  ```bash
  ./build-ninja/async_detector_test configs/standard3.yaml assets/demo/demo.avi
  ```

- `openvino_benchmark_test`：以录像首帧分别测量同步和异步 YOLO 的 FPS 与每帧耗时。

  ```bash
  ./build-ninja/openvino_benchmark_test configs/standard3.yaml assets/demo/demo.avi 120
  ```

  `YOLO::detect()` 是同步便利接口（提交到 `NetDetector` 后立即 `wait()`），仅用于离线测试与对比：`openvino_benchmark_test` 用它做同步基准，`detector_video_test -t` 用它与传统模式对比；生产链路必须走 `MultiThreadDetector`。

### 云台、串口与标定

- `quaternion_buffer_test`：无硬件回归，覆盖姿态插值、历史重复查询、缓存淘汰、无效四元数、通信间断和退出唤醒。

  ```bash
  ./build-ninja/quaternion_buffer_test
  ```

- `gimbal_test`：验证新版云台串口的姿态、角速度、弹速和弹丸计数读取；`--f` 会周期性发送开火命令，连接实机前必须确认安全条件。
- `gimbal_response_test`：向 CBoard/云台发送三角波、阶跃或圆周角度命令，并记录命令与 IMU 实际姿态，用于观察跟随和响应。
- `fire_test`：周期性切换普通控制与开火模式，单独验证发射控制链路；会连接真实云台，须在安全条件下运行。
- `handeye_test`：结合相机、CBoard 姿态和世界坐标网格进行重投影，用于检查手眼标定与 IMU-相机时间延迟。

上述硬件测试均从 YAML 读取串口和相机配置。运行前请核对设备连接、设备权限和当前配置，按 `q` 或 `Ctrl+C` 退出。

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

仓库的相机 SDK 目录包含 `amd64` 和 `arm64` 两个架构的回退库，但项目主要在 Ubuntu 22.04 x86_64 上验证。HikRobot USB 相机应安装完整 MVS SDK，避免控制库与 USB3 Vision 传输层版本不匹配。

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

也可以把这一行加入 `~/.bashrc`。如果 OpenVINO 安装在其他目录且无法软链接到 `/opt/intel/openvino_2024.6.0`，需要同步修改两处 CMake 文件中的 `OpenVINO_DIR`：

- `CMakeLists.txt`
- `tasks/auto_aim/CMakeLists.txt`

（`tasks/omniperception/` 未接入默认构建，其 `CMakeLists.txt` 仅在重新启用该模块时才有影响。）

### 相机 SDK 依赖

- HikRobot 相机：USB 相机的运行时传输层需要完整 HikRobot MVS SDK。默认安装到 `/opt/MVS` 后，CMake 会自动使用其中匹配的头文件、控制库和运行时路径，无需手工设置 `LD_LIBRARY_PATH`。若安装在其他目录，配置时指定：

```bash
cmake -B build-ninja -G Ninja -DHIKROBOT_MVS_ROOT=/path/to/MVS
```

若配置输出提示回退到仓库内的 `libMvCameraControl.so`，项目仍可完成无硬件构建，但 USB 相机可能因缺少匹配的 USB3 Vision 传输层而枚举失败。

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
    dynamic_roi           动态 ROI：网络 ROI 与灯条 ROI
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
  ordered_delivery.hpp    有序事件交付
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
  -> 云台姿态快照与动态 ROI（网络 ROI / 灯条 ROI）
  -> 有序异步检测（YOLO / 传统灯条）
  -> 数字识别与装甲板分类
  -> 敌我颜色过滤、目标优先级
  -> PnP 求解装甲板空间位姿
  -> EKF 跟踪目标位置、速度、角速度
  -> 弹道预测与 TinyMPC 轨迹规划
  -> 串口下发云台角度、角速度、角加速度和射击信号
```

主要组件职责：

- `Detector`：传统方法提取灯条并组合装甲板，可对 YOLO 结果做几何修正；`detect()` 支持在局部 ROI 内搜索，并把灯条、角点、中心与框回译为全图坐标。
- `YOLO`：通过 `NetDetector` 统一 letterbox、OpenVINO 预处理和 `InferRequest` 池；YOLOv5 适配器负责输出解码，后处理可携带传统灯条 ROI 用于角点细化。
- `MultiThreadDetector`：采集线程调用 `submit()` 分配自 1 递增的序号并异步推理，请求池满、空帧或错误时返回对应跳帧占位；`wait_pop()` 按序号有序交付 `Detection`，`close()`/`join()` 拒绝新帧并排空已接受任务。
- `Classifier`：使用 `assets/tiny_resnet.onnx` 识别装甲板数字。
- `Solver`：结合相机内参和云台外参，将装甲板从像素坐标解算到云台/世界坐标。
- `Tracker`：维护 lost/detecting/tracking/temp_lost/switching 状态机，使用 EKF 预测旋转目标；`focus_rois()` 根据预测四角生成网络 ROI 与灯条 ROI，按目标类型放大并在丢失 `lost_time` 后恢复全图。
- `Aimer` / `Shooter`：选择瞄准点并判断是否满足射击条件。
- `Planner`：使用 TinyMPC 求解 yaw/pitch 参考轨迹，输出控制量和射击标志。
- `Gimbal`：通过串口读取四元数、云台状态和弹速，发送视觉控制帧。
- `Camera`：统一工业相机取流接口，`stop()` 提供安全的停止与 SDK 清理入口（HikRobot 启动失败时清理流程无操作兜底）。
- `Plotter`：将 JSON 调试数据发送到 UDP `127.0.0.1:9870`。

## 配置文件

### `configs/standard3.yaml`

这是当前机器人配置，不适合直接用于所有电脑或所有机器人。部署到新机器人时建议复制一份，再修改相机、串口、标定和射击参数。

常用字段：

| 分类 | 字段 | 说明 |
| --- | --- | --- |
| 识别 | `enemy_color` | `red` 或 `blue` |
| 识别 | `yolov5_model_path` | YOLOv5 OpenVINO IR 模型 |
| 识别 | `classify_model` | 数字识别 ONNX 模型 |
| 识别 | `device` | OpenVINO 设备，如 `GPU`、`CPU`、`AUTO` |
| 识别 | `infer_request_buffer_num` | 可并行复用的 OpenVINO 请求数，缺省 `2`，当前配置 `5` |
| 识别 | `dynamic_roi.enabled` | 是否启用基于 Tracker 预测的动态 ROI |
| 识别 | `dynamic_roi.expand_ratio` | 普通装甲目标的 ROI 放大倍率，默认 `1.4` |
| 识别 | `dynamic_roi.base_expand_ratio` | 基地/前哨站目标的 ROI 放大倍率，默认 `3.0` |
| 识别 | `dynamic_roi.net_ratio` | 网络 ROI 宽高比调整系数，默认 `1.0` |
| 识别 | `dynamic_roi.lost_time` | 目标丢失该秒数后恢复全图搜索，默认 `0.5` |
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

## 测试程序速查

详细用途和示例命令见上方“测试与调试程序”。

| 程序 | 用途 | 是否需要硬件 |
| --- | --- | --- |
| `auto_aim_test` | 演示视频上的完整自瞄算法回放 | 不需要相机或云台，需要显示环境 |
| `detector_video_test` | 视频中的 YOLO 或传统检测对比 | 不需要相机 |
| `ordered_delivery_test` | 有序事件交付、跳帧占位与关闭语义 | 不需要硬件 |
| `dynamic_roi_test` | 动态 ROI 放大、裁剪与丢失恢复 | 不需要硬件 |
| `detector_roi_test` | 局部 ROI 内传统检测并回译全图坐标 | 不需要相机 |
| `camera_test` | 工业相机取流和帧率测试 | 需要相机 |
| `camera_thread_test` | 多 YOLO 实例并行检测 | 需要相机 |
| `minimum_vision_system` | 相机、DM IMU 与异步自瞄链路联调 | 需要相机和 DM IMU |
| `async_detector_test` | OpenVINO 异步请求池行为检查 | 不需要相机 |
| `openvino_benchmark_test` | 同步与异步推理性能对比 | 不需要相机 |
| `gimbal_test` / `gimbal_response_test` | 云台通信及动态响应测试 | 需要云台/主控板 |
| `fire_test` | 发射控制链路测试 | 需要云台，涉及开火命令 |
| `handeye_test` | 手眼标定与时间对齐检查 | 需要相机和主控板 |

## 标定工具

### 采集标定数据

```bash
./build-ninja/capture configs/calibration.yaml
```

程序会显示相机画面，按 `s` 保存图片和与该帧时间戳对齐的云台四元数，按 `q` 退出。输出目录默认是 `assets/img_with_q`。只有圆点识别成功、姿态对齐有效时才允许保存。

姿态缓存保留最近 1000 个有效样本，查询不消费历史；最多等待 20 ms，且只在相邻姿态间隔不超过 20 ms 时插值。不覆盖图像时间戳、过期或通信中断时，预览显示 `Pose unavailable`，保存请求返回状态 `save_rejected_pose_unavailable`。`infantry` / `infantry_debug` 将采集姿态随帧送入异步检测，处理结果及退出排空时复用该姿态；姿态不可用时跳过图像并暂停目标控制。

每组数据新增同编号 `.json`，记录图像及插值前后姿态的主机单调时钟时间戳（纳秒）；原有 JPG/TXT 格式不变。这些时间戳用于检查软件配对，尚未补偿相机曝光/传输和下位机通信延迟。

修复同步后应重新采集手眼数据，并使用新目录，避免混入旧样本：

```bash
./build-ninja/capture configs/calibration.yaml --output-folder=assets/img_with_q_synced
```

内参和手眼建议分别采集。内参采集时固定焦距、对焦和分辨率，增加标定板在画面中的大小，覆盖中心与边缘，并改变距离及两个方向的倾角。手眼采集时固定标定板和底盘，仅转动云台，在多个 yaw/pitch 姿态稳定后保存；实际圆心间距必须与 `center_distance_mm` 一致。用未参与求解的姿态验证固定板的世界坐标是否稳定，不能只看参与拟合的重投影误差。

远程标定时可以启用原生 Foxglove WebSocket，避免通过远程桌面传输整个桌面。首次配置会下载并校验官方 Foxglove C++ SDK 0.27.0：

```bash
cmake -B build-ninja -G Ninja -DENABLE_FOXGLOVE_CALIBRATION=ON
cmake --build build-ninja --target capture --parallel 4
./build-ninja/capture configs/calibration.yaml \
  --output-folder=assets/img_with_q \
  --foxglove --headless
```

服务默认只监听远端 `127.0.0.1:8765`。在操作电脑建立 SSH 隧道：

```bash
ssh -N -L 127.0.0.1:18765:127.0.0.1:8765 edge-108
```

Foxglove Studio 选择 **Open connection → Foxglove WebSocket**，连接 `ws://127.0.0.1:18765`：

- Image 面板选择 `/calibration/image/compressed`。
- Raw Messages 面板选择 `/calibration/status`，查看圆点识别、`orientation_valid`、姿态和保存状态；姿态无效时角度为 `null`。
- Service Call 面板调用 `/calibration/save` 保存下一张圆点识别成功的原图和对应四元数。
- Service Call 面板调用 `/calibration/quit` 安全退出。

预览默认是 10 FPS、JPEG 质量 80，保存的仍是相机原始分辨率图片。可通过 `--foxglove-fps`、`--jpeg-quality`、`--foxglove-host` 和 `--foxglove-port` 调整。输出目录已有连续编号的 JPG/TXT 时会从末尾续写；文件不成对或编号有缺口时程序会拒绝启动，避免覆盖已有标定数据。

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
