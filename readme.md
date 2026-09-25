# Dust Infantry Vision

这是一个基于 C++17、OpenCV、OpenVINO 和 Eigen 的步兵视觉系统，包含工业相机取流、YOLO 装甲板检测、数字识别、PnP 空间解算、EKF 目标跟踪、TinyMPC 云台轨迹规划、串口通信以及相机/手眼标定工具。自瞄主链路使用带序号的有序异步检测，并按 Tracker 预测动态裁剪网络/灯条 ROI，减少误检并提升目标像素占比。

当前仓库以自瞄主链路为完整可用状态。`tasks/omniperception/` 保留源码但未接入默认构建（根 `CMakeLists.txt` 中没有 `add_subdirectory(tasks/omniperception)`）；`io/usbcamera/`、`io/socketcan.hpp`、`auto_aim::Voter`、`tools::RansacSineFitter` 同样保留或参与编译，但当前没有调用方。详见“保留与未接入的模块”。

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

构建选项（均为 CMake 缓存变量）：

| 选项 | 默认值 | 说明 |
| --- | --- | --- |
| `ENABLE_CCACHE` | `ON` | 找到 `ccache` 时用它做编译器启动器 |
| `ENABLE_WARNINGS` | `OFF` | 开启后追加 `-Wall -Wextra -Wpedantic` |
| `ENABLE_FOXGLOVE_CALIBRATION` | `OFF` | 为 `capture` 启用原生 Foxglove WebSocket 预览与标定服务 |
| `HIKROBOT_MVS_ROOT` | `/opt/MVS` | HikRobot MVS SDK 安装根目录 |
| `OpenVINO_DIR` | `/opt/intel/openvino_2024.6.0/runtime/cmake/` | OpenVINO CMake 包目录 |

可执行文件输出在 `build-ninja/` 目录。主要程序包括：

- `build-ninja/infantry`（唯一运行入口，默认开本地可视化窗口）
- `build-ninja/camera_test`、`build-ninja/detector_video_test`、`build-ninja/auto_aim_test`
- `build-ninja/gimbal_test`、`build-ninja/handeye_test`
- `build-ninja/capture`、`build-ninja/calibrate_camera`、`build-ninja/calibrate_handeye`、`build-ninja/calibrate_robotworld_handeye`

### 3. 无硬件快速验证

先跑不依赖任何硬件的自动回归（CTest 只注册了两个用例）：

```bash
ctest --test-dir build-ninja --output-on-failure
```

`quaternion_buffer` 总是注册；`foxglove_calibration_protocol` 仅在启用 `ENABLE_FOXGLOVE_CALIBRATION` 时注册，`foxglove_vision_protocol` 仅在启用 `ENABLE_FOXGLOVE_VISION` 时注册。其余测试程序都是独立可执行文件，需要手动运行。

只验证命令入口：

```bash
./build-ninja/infantry -h
./build-ninja/auto_aim_test -h
```

用仓库内置视频验证 YOLO、分类器、Tracker 和 Aimer 主链路。此命令会打开 OpenCV 显示窗口，需要桌面环境或远程桌面：

```bash
./build-ninja/auto_aim_test configs/standard3.yaml assets/demo/demo
```

`auto_aim_test` 接收的是**不带扩展名**的输入前缀：上面的命令会读取 `assets/demo/demo.avi` 和同名 `assets/demo/demo.txt`（`t w x y z` 姿态记录）。`-s` / `--start-index` 与 `-e` / `--end-index` 可限制帧范围。

`configs/standard3.yaml` 当前使用 `device: CPU`，无需 Intel GPU 即可跑通。有 Intel GPU / OpenCL 时可改为 `GPU` 提速。

### 4. 实机运行

实机运行前确认：

- 相机型号和 `vid_pid` 与 `configs/standard3.yaml` 一致。
- 云台串口存在 `/dev/gimbal`，当前用户有读写权限。
- OpenVINO 设备可用（当前配置为 `CPU`）。

运行生产主程序：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
./build-ninja/infantry configs/standard3.yaml
```

`infantry` 默认打开本地可视化窗口，显示检测叠加图，按 `q` 退出（与 SIGINT 同一条退出路径）。运行模式开关统一放在配置文件的 `runtime` 段（见 `configs/standard3.yaml`），命令行只剩配置文件路径：

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `runtime.simulate_gimbal` | `false` | 置为 `true` 时使用虚拟云台姿态并只做串口输出，不依赖真实云台，可用于无设备联调 |
| `runtime.headless` | `false` | 置为 `true` 时关闭检测可视化和窗口事件，适合无桌面的远程运行；无桌面开机自启必须为 `true` |
| `runtime.verbose_ekf` | `false` | 置为 `true` 时每帧输出 EKF 11 维状态；关闭时无输出 |
| `runtime.foxglove` | `false` | 置为 `true` 时启用 Foxglove 图像与遥测推送；监听地址、端口、帧率、缩放、发布线程调度和 JPEG 质量在 `foxglove` 段配置 |

`runtime` 段整体缺失或缺少某个键时用上表的默认值；值不是布尔（例如 `headless: 3`）会在启动时报错并以退出码 2 结束。原来的命令行开关 `--simulate-gimbal`、`--headless`、`--foxglove`、`--verbose-ekf` 已移除，旧写法会被 `infantry` 拦截并报错退出（见下文「自瞄图像与数据」）。

单独测试相机：

```bash
./build-ninja/camera_test configs/standard3.yaml
```

## 测试与调试程序

`tests/` 中的程序均作为独立可执行文件构建，主要用于算法回放、性能测量和硬件联调。除 CTest 注册的两个用例外，其余不会被 `ctest` 自动执行。

多数程序的位置参数默认值非空（默认 `configs/standard3.yaml`，`handeye_test` 默认 `configs/calibration.yaml`），因此**不带参数直接运行会按默认配置启动，而不是打印帮助**；要查看帮助请显式传 `-h`。

### 自动回归（CTest）

- `quaternion_buffer_test`：无硬件回归，覆盖姿态插值、历史重复查询、缓存淘汰、无效四元数、通信间断和退出唤醒。链接仅需 Eigen 与 Threads。

  ```bash
  ./build-ninja/quaternion_buffer_test
  ```

- `foxglove_calibration_test`：配合 `tests/foxglove_calibration_protocol_test.py` 以合成图像驱动 Foxglove 服务端，校验通道、服务与图像/状态负载。仅在 `-DENABLE_FOXGLOVE_CALIBRATION=ON` 时构建。用法为 `foxglove_calibration_test <port>`，通常由 CTest 调用。
- `foxglove_vision_test`：配合 `tests/foxglove_vision_protocol_test.py` 校验自瞄的图像/遥测通道、图像缩放与时间戳，以及单端口和双端口的主题隔离。仅在 `-DENABLE_FOXGLOVE_VISION=ON` 时构建。用法为 `foxglove_vision_test <image_port> [data_port] [duration_ms]`，通常由 CTest 调用。

### 离线视觉与算法

- `auto_aim_test`：读取 `<input-path>.avi` 和同名 `.txt` 姿态记录，串联 YOLO、解算、跟踪、瞄准与开火判断，显示重投影和调试数据。需要显示环境；不会向硬件发送任何命令。

  ```bash
  ./build-ninja/auto_aim_test configs/standard3.yaml assets/demo/demo
  ```

- `detector_video_test`：对录像运行 YOLO 或传统灯条检测，输出装甲板四角点，适合比较两种检测路径。`-t` / `--tradition` 切换传统模式。输入是**带扩展名**的视频路径。该程序会无条件构造 YOLO 与传统检测器，因此即使使用 `--tradition` 也需要 OpenVINO 可用。

  ```bash
  ./build-ninja/detector_video_test configs/standard3.yaml assets/demo/demo.avi
  ./build-ninja/detector_video_test configs/standard3.yaml assets/demo/demo.avi --tradition
  ```

### 有序交付与动态 ROI

- `ordered_delivery_test`：验证有序事件交付的容量限制、逆序完成、跳帧占位、关闭唤醒和严格序号语义，纯离线运行，无需参数。

  ```bash
  ./build-ninja/ordered_delivery_test
  ```

- `dynamic_roi_test`：验证网络 ROI 与灯条 ROI 的目标类型放大、边界裁剪、非法输入回退和丢失恢复。无需参数、无需配置与模型。

  ```bash
  ./build-ninja/dynamic_roi_test
  ```

- `detector_roi_test`：在局部 ROI 内运行传统检测，验证灯条、角点与中心点回译到全图坐标。数字识别走 OpenCV DNN，需要 `assets/tiny_resnet.onnx`。

  ```bash
  ./build-ninja/detector_roi_test configs/standard3.yaml
  ```

### 相机与实时检测

- `camera_test`：验证配置指定的工业相机采集，持续打印相邻帧 FPS；传入 `-d` / `--display` 显示画面。

  ```bash
  ./build-ninja/camera_test configs/standard3.yaml --display
  ```

- `camera_thread_test`：创建多个 YOLO 实例并通过线程池并行处理工业相机帧，用于观察多线程检测吞吐与结果顺序。线程数在代码中固定为 8。

  ```bash
  ./build-ninja/camera_thread_test configs/standard3.yaml
  ```

- `minimum_vision_system`：硬件在环的小型完整链路，组合工业相机、DM IMU、异步检测、解算、跟踪、瞄准和射击判断；需要相机、DM IMU 和桌面显示环境。射击判断会被计算，但不会下发到硬件。

  ```bash
  ./build-ninja/minimum_vision_system configs/standard3.yaml
  ```

### 异步推理与性能

- `async_detector_test`：验证 OpenVINO 异步请求池容量耗尽、后处理释放和请求复用，以及 `MultiThreadDetector` 的序号分配、跳帧/错误占位、关闭与有序排空语义。需要 OpenVINO 与一段视频，只用到视频首帧。

  ```bash
  ./build-ninja/async_detector_test configs/standard3.yaml assets/demo/demo.avi
  ```

- `openvino_benchmark_test`：以录像首帧分别测量同步和异步 YOLO 的 FPS 与每帧耗时，第三个参数是迭代次数（默认 120）。

  ```bash
  ./build-ninja/openvino_benchmark_test configs/standard3.yaml assets/demo/demo.avi 120
  ```

  `YOLO::detect()` 是同步便利接口（提交到 `NetDetector` 后立即 `wait()`），源码中标注为仅用于离线测试与对比：`openvino_benchmark_test` 用它做同步基准，`detector_video_test -t` 用它与传统模式对比；生产链路必须走 `MultiThreadDetector`。

### 云台、串口与标定

以下程序都会打开真实串口，**连接实机前请确认安全条件**。

- `gimbal_test`：验证云台串口的姿态、角速度、弹速和弹丸计数读取；`-f` 会周期性发送开火命令（默认 `false`，不加则只读不发）。

  ```bash
  ./build-ninja/gimbal_test configs/standard3.yaml
  ```

- `gimbal_response_test`：通过 `Gimbal` 发送三角波、阶跃或圆周角度命令，并记录命令与 IMU 实际姿态。`-a` / `--delta-angle`（默认 8）、`-m` / `--signal-mode`（默认 `triangle_wave`，另有 `step`、`circle`）、`-x` / `--axis`（默认 `yaw`）、`-c` / `--circle` 可调。全程使用控制模式，不开火；姿态不可用或退出时发送停止控制命令。

  ```bash
  ./build-ninja/gimbal_response_test configs/standard3.yaml --signal-mode circle
  ```

- `fire_test`：周期性切换普通控制与开火模式，单独验证发射控制链路；**会周期性发送开火命令**，须在安全条件下运行。
- `handeye_test`：结合相机、`Gimbal::orientation_at()` 同步姿态和世界坐标网格进行重投影，用于检查手眼标定与 IMU-相机时间延迟；姿态不可用时跳过重投影。默认配置是 `configs/calibration.yaml`（不是 `standard3.yaml`），并读取其中的 `height`、`grid_num`、`grid_size`、`delay`。`-d` / `--display` 显示画面。

  ```bash
  ./build-ninja/handeye_test configs/calibration.yaml --display
  ```

上述硬件测试均从 YAML 读取串口和相机配置。运行前请核对设备连接、设备权限和当前配置，按 `q`（仅在打开了窗口时）或 `Ctrl+C` 退出。

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

项目 CMake 目前只在**根 `CMakeLists.txt`** 中固定查找：

```text
/opt/intel/openvino_2024.6.0/runtime/cmake/
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

也可以把这一行加入 `~/.bashrc`。如果 OpenVINO 安装在其他目录且无法软链接到 `/opt/intel/openvino_2024.6.0`，修改根 `CMakeLists.txt` 中的 `OpenVINO_DIR` 即可（`tasks/auto_aim/CMakeLists.txt` 与 `tasks/omniperception/CMakeLists.txt` 只是链接 `openvino::runtime`，不含该变量）。

注意该变量是**不带 `FORCE` 的缓存变量**：如果构建目录里已经存在旧的缓存值，仅修改文件不会生效。此时用命令行覆盖或删除缓存条目后重新配置：

```bash
cmake -B build-ninja -G Ninja -DOpenVINO_DIR=/your/openvino/runtime/cmake
# 或者
rm -rf build-ninja && cmake -B build-ninja -G Ninja
```

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

如果目标电脑没有这个设备节点，需要为云台串口添加 udev 规则。例如，根据实际 USB 串口的 VID/PID 创建规则：

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

### systemd 服务（可选）

`systemd/infantry.service` 提供开机自启模板，使用 `Type=exec`、`KillSignal=SIGINT`、`TimeoutStopSec=10`，并预设 MVS SDK 的运行环境变量。其中的 `WorkingDirectory` 与 `ExecStart` 使用绝对路径，当前指向本检出的 `/home/rmul/Dust_infantry_vision`，换到别的机器或用户名下部署时需要相应修改。`ExecStart` 只传配置文件路径：运行模式开关已移入 `configs/standard3.yaml` 的 `runtime` 段，命令行不再有 `--simulate-gimbal`/`--headless`。用这份模板做无桌面开机自启时，必须把该段的 `headless` 置为 `true`（`infantry` 默认开本地窗口，没有 `DISPLAY` 时会失败）；`simulate_gimbal` 接实机时保持 `false`。

## 项目结构

```text
src/                      主程序入口
  infantry.cpp            唯一运行入口（生产与调试，含重投影可视化窗口）
  auto_aim_runtime.cpp    运行编排：相机/检测/跟踪/规划三线程与运行期选项
tasks/
  auto_aim/               自瞄核心算法
    armor / detector / classifier / solver / target / tracker
    aimer / shooter / voter
    dynamic_roi           动态 ROI：网络 ROI 与灯条 ROI
    yolo + net_detector   OpenVINO 推理与请求池
    yolos/yolov5          YOLOv5 输出解码适配器
    multithread/          有序异步检测（MultiThreadDetector）
    planner/              TinyMPC 轨迹规划，tinympc/ 为内置求解器
  omniperception/         全向感知模块（保留源码，未接入默认构建）
io/                       硬件和通信抽象
  camera.cpp              工业相机统一接口与工厂
  hikrobot/               HikRobot 相机驱动
  mindvision/             MindVision 相机驱动
  usbcamera/              USB/V4L2 相机驱动（未接入 io::Camera）
  gimbal/                 云台串口收发
  dm_imu/                 DM 系列 IMU
  command.hpp             瞄准命令 POD，供 Aimer/Decider 与部分测试使用
  serial/                 跨平台串口库
  socketcan.hpp           SocketCAN 工具（当前无调用方）
tools/                    通用工具
  plotter.cpp             UDP 调试数据发送
  extended_kalman_filter  EKF 实现（含 NIS/NEES 诊断）
  trajectory.cpp          弹道模型
  quaternion_buffer.hpp   有界姿态历史与插值
  ordered_delivery.hpp    有序事件交付
  latency_stats.hpp       延迟分位数统计
  thread_safe_queue       线程安全队列
  thread_pool             线程池
  recorder.cpp            录像与姿态记录
  ransac_sine_fitter.cpp  RANSAC 正弦拟合（当前无调用方）
  crc / exiter / logger / math_tools / img_tools / pid / yaml
calibration/              标定工具
visualization/            Foxglove 图像与遥测推送、检测叠加绘制
configs/                  YAML 配置
assets/                   模型、演示数据
systemd/                  systemd 服务模板
tests/                    调试和测试程序
```

### 保留与未接入的模块

为避免误判，以下内容存在于仓库中但不参与默认运行链路：

- `tasks/omniperception/`：源码保留，根 `CMakeLists.txt` 未 `add_subdirectory`，不参与构建；`tasks/auto_aim/tracker.cpp` 只包含其纯头文件 `detection.hpp` 使用类型定义。
- `io/usbcamera/`：已编译进 `io`，但 `io::Camera` 工厂只识别 `hikrobot` 与 `mindvision`，无法选用。
- `io/socketcan.hpp`：头文件存在，全仓无引用。
- `auto_aim::Voter`、`tools::RansacSineFitter`：参与编译，但无调用方。
- `YOLO::detect()`、`tools::create_yolos()`：仅供离线测试与 `camera_thread_test` 使用。

## 核心流程

主自瞄链路大致为：

```text
工业相机取流（同时快照云台姿态）
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

- `Detector`：传统方法提取灯条并组合装甲板，可对 YOLO 结果做几何修正；`detect()` 支持在局部 ROI 内搜索，并把灯条、角点、中心与框回译为全图坐标。构造时会创建 `patterns/` 目录（见“常见问题”）。
- `YOLO`：通过 `NetDetector` 统一预处理和 `InferRequest` 池；`YOLOV5` 适配器负责输出解码，后处理可携带传统灯条 ROI 用于角点细化。网络输入固定为 640×640，ROI 缩放后贴到左上角，不做居中 letterbox。
- `MultiThreadDetector`：采集线程调用 `submit()` 分配自 1 递增的序号并异步推理，请求池满、空帧或错误时返回对应跳帧占位；`wait_pop()` 按序号有序交付 `Detection`，`close()`/`join()` 拒绝新帧并排空已接受任务。被跳过或失败的帧同样会以占位 `Detection` 交付（`inferred == false`、装甲板列表为空），因此**消费方必须检查 `inferred`**。有序交付的容量是请求池容量的 2 倍。
- `Classifier`：使用 `assets/tiny_resnet.onnx` 识别装甲板数字（OpenCV DNN，非 OpenVINO）。
- `Solver`：唯一负责像素、云台、世界三者坐标转换的组件。持有 `camera_matrix`、`distort_coeffs`、`R_gimbal2imubody` 以及 `R_camera2gimbal` / `t_camera2gimbal`，同时提供 `solve(Armor&)`（像素 → 世界）与 `reproject_armor()` / `world2pixel()`（世界 → 像素）。装甲板俯仰角按固定值建模（普通装甲 +15°，前哨站 −15°），yaw 通过重投影搜索细化。
- `Tracker`：维护 lost/detecting/tracking/temp_lost/switching 状态机，使用 11 维 EKF（含 2/3/4 装甲板模型与前哨站专用处理）预测旋转目标；`focus_rois()` 根据预测四角生成网络 ROI 与灯条 ROI。
- `Aimer` / `Shooter`：用于 `auto_aim_test` 算法回放和 `minimum_vision_system` 的瞄准、射击判断；生产入口使用 `Planner`。`Aimer` 输出 `io::Command`。
- `Planner`：生产链路使用的规划器。由目标预测构造 60 步 yaw/pitch 参考轨迹，用两个独立的 TinyMPC 问题分别求解 yaw 与 pitch，输出 `Plan{control, fire, target_yaw, yaw, yaw_vel, yaw_acc, target_pitch, pitch, pitch_vel, pitch_acc}`。控制量取预测时域中点（0.3 s 处），射击判定比较参考轨迹与求解结果在该点附近的偏差。
- `Gimbal`：通过串口读取四元数、云台状态和弹速，发送视觉控制帧。
- `Camera`：统一工业相机取流接口，`stop()` 提供安全的停止与 SDK 清理入口（HikRobot 启动失败时清理流程无操作兜底）。取流结束后队列关闭，`read()` 返回空图。
- `Plotter`：将 JSON 调试数据发送到 UDP `127.0.0.1:9870`，无需接收端即可运行。

### 姿态与时间同步

姿态是反复出错的来源，改动时请遵守以下约定：

- 姿态在采集时刻快照，并**随每一帧一起传递**；处理异步结果时**禁止**重新消费云台队列。
- `Gimbal::orientation_at(t)` 用 `QuaternionBuffer` 保留最近 1000 个姿态，按主机时间 slerp 插值，查询不消费历史。等待与相邻样本间隔上限均为 20 ms，无有效覆盖时返回空值。
- `infantry` 每帧只快照一次；姿态不可用时**跳过该帧图像**并暂停目标控制。

## 配置文件

### `configs/standard3.yaml`

这是当前机器人配置，不适合直接用于所有电脑或所有机器人。部署到新机器人时建议复制一份，再修改相机、串口、标定和射击参数。绝大多数键是**必填**的，缺失时会抛出 `Missing YAML key`；可选的有 `infer_request_buffer_num`（缺省 `2`）、`skip_gimbal_crc`（缺省 `false`）、`dynamic_roi.net_ratio`（缺省 `1.0`），以及 `dynamic_roi`、`armor_association`、`foxglove` 和 `runtime` 四个节点本身（节点不存在时用内置默认值；`runtime` 的四个开关见上文「运行生产主程序」）。

| 分类 | 字段 | 说明 |
| --- | --- | --- |
| 识别 | `enemy_color` | `red` 或 `blue`（非 `red` 一律按 blue 处理） |
| 识别 | `yolov5_model_path` | YOLOv5 OpenVINO IR 模型，当前 `assets/0708.xml`（配套 `.bin`） |
| 识别 | `classify_model` | 数字识别 ONNX 模型，当前 `assets/tiny_resnet.onnx` |
| 识别 | `device` | OpenVINO 设备，`GPU`、`CPU`、`AUTO`；当前为 `CPU` |
| 识别 | `infer_request_buffer_num` | 可并行复用的 OpenVINO 请求数，缺省 `2`，当前配置 `5` |
| 识别 | `min_confidence` | 目标最低置信度 |
| 识别 | `use_traditional` | YOLOv5 是否用传统方法细化角点 |
| ROI | `use_roi` | 是否使用固定网络 ROI |
| ROI | `roi.x` / `roi.y` / `roi.width` / `roi.height` | 固定 ROI，像素；`width` / `height` 为 `-1` 时延伸到画面边缘 |
| 动态 ROI | `dynamic_roi.enabled` | 是否启用基于 Tracker 预测的动态 ROI |
| 动态 ROI | `dynamic_roi.expand_ratio` | 普通装甲/前哨站目标的 ROI 放大倍率，默认 `1.4` |
| 动态 ROI | `dynamic_roi.base_expand_ratio` | **仅基地**目标的 ROI 放大倍率，默认 `3.0` |
| 动态 ROI | `dynamic_roi.net_ratio` | 网络 ROI 宽高比下限，默认 `1.0` |
| 动态 ROI | `dynamic_roi.lost_time` | 丢失后 ROI 线性恢复全图的秒数，默认 `0.5` |
| 传统检测 | `threshold` | 二值化灰度阈值 |
| 传统检测 | `max_angle_error` | 灯条倾角容差（度） |
| 传统检测 | `min_lightbar_ratio` / `max_lightbar_ratio` | 灯条长宽比范围 |
| 传统检测 | `min_lightbar_length` | 灯条最短长度（像素） |
| 传统检测 | `min_armor_ratio` / `max_armor_ratio` | 装甲板长宽比范围 |
| 传统检测 | `max_side_ratio` | 两侧灯条长度比上限 |
| 传统检测 | `max_rectangular_error` | 装甲板矩形度误差容差（度） |
| 跟踪 | `min_detect_count` | 进入 tracking 前需要的连续检出次数 |
| 跟踪 | `max_temp_lost_count` | 普通目标 temp_lost 容忍次数 |
| 跟踪 | `outpost_max_temp_lost_count` | 前哨站的 temp_lost 容忍次数 |
| 关联 | `armor_association.enabled` | 是否启用图像空间装甲板关联 |
| 关联 | `armor_association.center_gate_px` | 中心像素门限，默认 `100.0` |
| 关联 | `armor_association.corner_gate_px` | 角点像素门限，默认 `140.0` |
| 关联 | `armor_association.angle_gate_rad` | 角度门限（弧度），默认 `0.8` |
| 关联 | `armor_association.perimeter_ratio_gate` | 周长比门限，默认 `0.6` |
| 关联 | `armor_association.image_observation.enabled` | 是否使用 8 维像素观测更新 EKF |
| 关联 | `armor_association.image_observation.point_sigma_px` | 像素观测噪声（像素），默认 `8.0` |
| 相机 | `camera_name` | `hikrobot` 或 `mindvision` |
| 相机 | `exposure_ms` / `gain` / `frame_rate` | 相机曝光、增益、帧率（HikRobot 用；`frame_rate` 缺省 `165`） |
| 相机 | `gamma` | MindVision 专用 |
| 相机 | `vid_pid` | USB VID:PID，例如 `2bdf:0001` |
| 标定 | `camera_matrix` / `distort_coeffs` | 相机内参和畸变系数 |
| 标定 | `R_gimbal2imubody` | 云台坐标系到 IMU 机体系的旋转 |
| 标定 | `R_camera2gimbal` / `t_camera2gimbal` | 相机到云台的外参（旋转为行主序 3×3，平移单位为米） |
| 串口 | `com_port` | 云台串口节点 |
| 串口 | `skip_gimbal_crc` | 可选诊断开关，默认 `false`，保持 CRC 校验 |
| 瞄准 | `yaw_offset` / `pitch_offset` | 枪管相对瞄准点的机械补偿（度） |
| 瞄准 | `comming_angle` / `leaving_angle` | 旋转目标的进入/离开角门限（度，拼写与源码一致） |
| 瞄准 | `decision_speed` | 高/低速判定阈值（rad/s） |
| 瞄准 | `high_speed_delay_time` / `low_speed_delay_time` | 高/低速下的额外预测时延（秒） |
| 射击 | `auto_fire` | 是否由视觉控制射击（`Shooter` 使用） |
| 射击 | `first_tolerance` / `second_tolerance` | 近距离/远距离射击角度容差（度） |
| 射击 | `judge_distance` | 近/远距离判定阈值（米） |
| 规划 | `fire_thresh_high_speed` / `fire_thresh_low_speed` | 高/低速下的射击阈值 |
| 规划 | `max_yaw_acc` / `max_pitch_acc` | 云台角加速度约束（rad/s²） |
| 规划 | `Q_yaw` / `R_yaw` / `Q_pitch` / `R_pitch` | TinyMPC 权重 |
| 运行模式 | `runtime.simulate_gimbal` | 虚拟云台姿态与串口输出，不依赖真机，默认 `false` |
| 运行模式 | `runtime.headless` | 关闭检测可视化与窗口事件，无桌面运行必须为 `true`，默认 `false` |
| 运行模式 | `runtime.verbose_ekf` | 每帧输出 EKF 11 维状态，默认 `false` |
| 运行模式 | `runtime.foxglove` | 启用 Foxglove 图像与遥测推送，默认 `false` |
| Foxglove | `foxglove.host` | 监听地址，默认 `127.0.0.1` |
| Foxglove | `foxglove.port` | 图像端口，默认 `8766`；`data_port` 为 0 时同时承载遥测 |
| Foxglove | `foxglove.data_port` | 非 0 时遥测走独立端口，默认 `0` |
| Foxglove | `foxglove.fps` | 图像发布帧率上限，默认 `30` |
| Foxglove | `foxglove.scale` | 发布前缩放系数，默认 `0.5`，范围 `(0,1]` |
| Foxglove | `foxglove.sched` | `auto`（默认，发布线程降级让出 CPU）或 `off` |
| Foxglove | `foxglove.jpeg_quality` | JPEG 质量，默认 `80`，范围 `1-100` |

`yaw_offset`、`pitch_offset`、`decision_speed`、`high_speed_delay_time`、`low_speed_delay_time` 同时被 `Aimer` 和 `Planner` 读取。

### `configs/calibration.yaml`

标定与手眼检查工具使用的配置。除 `configs/standard3.yaml` 中的相机、串口和标定字段（相机参数与 `standard3.yaml` 保持一致）外，还包含：

| 字段 | 说明 |
| --- | --- |
| `pattern_cols` / `pattern_rows` | 标定板列数/行数，当前 `10` / `7`；由三个 `calibrate_*` 程序读取 |
| `center_distance_mm` | 相邻圆心距（毫米），当前 `40` |
| `height` | 云台旋转中心距地面高度（米），`handeye_test` 世界网格用，当前 `0.40` |
| `grid_num` | 世界网格范围，当前 `8` |
| `grid_size` | 世界网格相邻点间距（米），当前 `0.20` |
| `delay` | 手眼重投影的姿态时间补偿（毫秒），当前 `0` |

标定板规格：使用 `calib.io_circles_200x150_7x10_12_5.pdf`（页面 200×150 mm、10 列 7 行、圆心距 12 mm、圆直径 5 mm）经 27 寸屏放大显示。实测相邻圆心距为 **40 mm**，与 `center_distance_mm: 40` 一致；PDF 上的 12 mm 是显示前的原始尺寸，与实物无关。

注意 `capture` 目前**硬编码** 10 列 7 行的圆点图案（启动时会打印“默认标定板尺寸为10列7行”），并不读取 `pattern_cols` / `pattern_rows`；换用其他规格标定板需要改代码。

### `assets/`

模型与演示数据：

- `assets/0708.xml` + `0708.bin`：当前默认 YOLOv5 OpenVINO IR。
- `assets/tiny_resnet.onnx`：数字识别模型。
- `assets/yolov5.xml` + `yolov5.bin`、`assets/best2-sim.onnx`：备用模型，当前配置未引用。
- `assets/demo/demo.avi` + `demo.txt`：演示视频与 `t w x y z` 姿态记录。
- `assets/img_with_q/`、`assets/img_with_q_synced/`：标定采集输出目录，本地生成、未纳入版本管理，不要混用新旧样本。

## 测试程序速查

详细用途和示例命令见上方“测试与调试程序”。

| 程序 | 用途 | 是否需要硬件 | 是否需要显示环境 |
| --- | --- | --- | --- |
| `quaternion_buffer_test` | 姿态缓冲插值、淘汰与关闭语义（CTest） | 不需要 | 不需要 |
| `ordered_delivery_test` | 有序事件交付、跳帧占位与关闭语义 | 不需要 | 不需要 |
| `dynamic_roi_test` | 动态 ROI 放大、裁剪与丢失恢复 | 不需要 | 不需要 |
| `detector_roi_test` | 局部 ROI 内传统检测并回译全图坐标 | 不需要相机 | 不需要 |
| `async_detector_test` | OpenVINO 异步请求池行为检查 | 不需要相机 | 不需要 |
| `openvino_benchmark_test` | 同步与异步推理性能对比 | 不需要相机 | 不需要 |
| `auto_aim_test` | 演示视频上的完整自瞄算法回放 | 不需要相机或云台 | **需要** |
| `detector_video_test` | 视频中的 YOLO 或传统检测对比 | 不需要相机 | 不需要 |
| `camera_test` | 工业相机取流和帧率测试 | 需要相机 | 仅 `--display` 时 |
| `camera_thread_test` | 多 YOLO 实例并行检测 | 需要相机 | 不需要 |
| `minimum_vision_system` | 相机、DM IMU 与异步自瞄链路联调 | 需要相机和 DM IMU | **需要** |
| `gimbal_test` | 云台串口读取，`-f` 可开火 | 需要云台 | 不需要 |
| `gimbal_response_test` | 云台动态响应标定，不发开火命令 | 需要云台 | 不需要 |
| `fire_test` | 发射控制链路测试 | 需要云台，**周期性开火** | 不需要 |
| `handeye_test` | 手眼标定与时间对齐检查 | 需要相机和云台 | 仅 `--display` 时 |
| `foxglove_calibration_test` | Foxglove 协议回归（需开启构建选项） | 不需要 | 不需要 |
| `foxglove_vision_test` | 自瞄 Foxglove 协议回归与双端口隔离（需开启构建选项） | 不需要 | 不需要 |

需要 OpenVINO 的程序：`auto_aim_test`、`detector_video_test`、`camera_thread_test`、`async_detector_test`、`openvino_benchmark_test`、`minimum_vision_system`。

## 标定工具

### 采集标定数据

```bash
./build-ninja/capture configs/calibration.yaml
```

程序会显示相机画面，按 `s` 保存图片和与该帧时间戳对齐的云台四元数，按 `q` 退出。输出目录默认是 `assets/img_with_q`。只有圆点识别成功、姿态对齐有效时才允许保存；拒绝时会给出 `save_rejected_pose_unavailable` 或 `save_rejected_grid_not_detected`。

命令行参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `@config-path` | `configs/calibration.yaml` | 位置参数，YAML 配置 |
| `-o` / `--output-folder` | `assets/img_with_q` | 输出目录 |
| `--headless` | 关 | 不创建 OpenCV 窗口，必须与 `--foxglove` 一起使用 |
| `--foxglove` | 关 | 启用 Foxglove WebSocket |
| `--foxglove-host` | `127.0.0.1` | 监听地址 |
| `--foxglove-port` | `8765` | 监听端口 |
| `--foxglove-fps` | `10` | 预览帧率 |
| `--jpeg-quality` | `80` | 预览 JPEG 质量 |

带值的开关必须写成 `--key=value`：OpenCV 的 `CommandLineParser` 不解析空格分隔形式，写错时 `capture`
会打印 `[cli] --xxx was parsed as the literal string "true"; use --xxx=<value>` 并以退出码 2 结束，
不会静默使用错误的值。受检查的开关是 `--output-folder`（含 `-o` 别名）、`--foxglove-host`、
`--foxglove-port`、`--foxglove-fps`、`--jpeg-quality`。`--headless=false` / `--foxglove=false`
现在会正确地关闭对应功能。

空格写法还有一个更隐蔽的后果：后面那个值会被当成**位置参数**，也就是把 `config-path` 顶掉，
于是程序去加载一个错误的 YAML（文件不存在时会打印 `[Calibration] Failed to load YAML file: ...`
并以退出码 2 结束）。另外 `--config-path=foo.yaml` 这种拼法会被**静默忽略**——`keys` 里只声明了
位置参数 `@config-path`，解析器识别不了未知键，配置文件只能用位置参数给定。

`capture` 没有虚拟云台开关，因此必须有可用的 `/dev/gimbal`。

姿态缓存保留最近 1000 个有效样本，查询不消费历史；最多等待 20 ms，且只在相邻姿态间隔不超过 20 ms 时插值。不覆盖图像时间戳、过期或通信中断时，预览显示 `Pose unavailable`，保存请求返回状态 `save_rejected_pose_unavailable`。`infantry` 将采集姿态随帧送入异步检测，处理结果及退出排空时复用该姿态；姿态不可用时跳过图像并暂停目标控制。

每组数据写入三个同编号文件：`.jpg`（相机原始分辨率的未标注原图）、`.txt`（四元数，顺序为 **w x y z**）、`.json`（时间元数据）。JSON 字段固定为：

```json
{"clock":"host_steady_clock","image_timestamp_ns":0,"orientation_before_ns":0,"orientation_after_ns":0}
```

时间戳均为主机单调时钟纳秒值，用于检查软件配对，尚未补偿相机曝光/传输和下位机通信延迟。输出目录已有连续编号的 JPG/TXT 时会从末尾续写；JPG/TXT 不成对或编号有缺口时程序会拒绝启动，避免覆盖已有标定数据。

修复同步后应重新采集手眼数据，并使用新目录，避免混入旧样本：

```bash
./build-ninja/capture configs/calibration.yaml --output-folder=assets/img_with_q_synced
```

内参和手眼建议**分开采集**。内参采集时固定焦距、对焦和分辨率，增加标定板在画面中的大小，覆盖中心与边缘，并改变距离及两个方向的倾角。手眼采集时固定标定板和底盘，仅转动云台，在多个 yaw/pitch 姿态稳定后保存；实际圆心间距必须与 `center_distance_mm` 一致。用未参与求解的姿态验证固定板的世界坐标是否稳定，不能只看参与拟合的重投影误差。

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

预览默认是 10 FPS、JPEG 质量 80，保存的仍是相机原始分辨率图片。可通过 `--foxglove-fps`、`--jpeg-quality`、`--foxglove-host` 和 `--foxglove-port` 调整。

### 自瞄图像与数据（infantry）

`infantry` 可以把 EKF/规划数据和检测叠加图推送到 Foxglove，不必远程桌面。
与标定程序共用同一份官方 Foxglove C++ SDK 0.27.0，两个开关同时打开也只下载一次：

```bash
cmake -B build-ninja -G Ninja -DENABLE_FOXGLOVE_VISION=ON
# 与标定一起打开：-DENABLE_FOXGLOVE_CALIBRATION=ON -DENABLE_FOXGLOVE_VISION=ON
cmake --build build-ninja --target infantry -j$(nproc)
# 先把 configs/standard3.yaml 的 runtime 段设为 foxglove: true、headless: true
./build-ninja/infantry configs/standard3.yaml
```

服务默认只监听远端 `127.0.0.1:8766`（与 `capture` 的 8765 分开，同机可同时运行）。在操作电脑建立 SSH 隧道：

```bash
ssh -N -L 127.0.0.1:18766:127.0.0.1:8766 edge-108
```

Foxglove Studio 选择 **Open connection → Foxglove WebSocket**，连接 `ws://127.0.0.1:18766`：

- Image 面板选择 `/vision/image/compressed`。内容与本地窗口一致：检测 ROI、装甲板角点、
  重投影装甲板与瞄准点、Tracker 状态和中心准星。开不开本地窗口，推送的都是同一份叠加图。
- Plot / Raw Messages 面板选择 `/vision/telemetry`，字段与 UDP `127.0.0.1:9870` 的调试数据完全一致。
- Raw Messages 面板选择 `/vision/status`，查看连接数、发布/丢帧计数、当前工作模式和发布线程的调度状态。

带宽或时延不足时把图像与遥测拆成两条独立连接：两个端口属于不同的 Foxglove context，
各自有独立的积压队列，慢速图像客户端不会挤占遥测的带宽和时延。

```bash
# 先把 configs/standard3.yaml 的 runtime 段设为 foxglove: true、headless: true，
# 再把 foxglove.data_port 设为 8767
./build-ninja/infantry configs/standard3.yaml
ssh -N -L 127.0.0.1:18766:127.0.0.1:8766 -L 127.0.0.1:18767:127.0.0.1:8767 edge-108
# Foxglove 里建立两个连接：图像 ws://127.0.0.1:18766，遥测 ws://127.0.0.1:18767
```

Foxglove 的启用开关同样在配置文件的 `runtime` 段（`runtime.foxglove`），命令行上已经没有 Foxglove 开关。
监听与编码参数取自 `configs/standard3.yaml` 的 `foxglove` 段（节点不存在或缺键时用内置默认值）：

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `foxglove.host` | `127.0.0.1` | 监听地址 |
| `foxglove.port` | `8766` | 图像端口；单端口模式下同时承载遥测 |
| `foxglove.data_port` | `0` | 非 0 时启用双端口，遥测只走该端口 |
| `foxglove.fps` | `30` | 图像发布帧率上限，超出的帧直接丢弃、不排队 |
| `foxglove.scale` | `0.5` | 发布前缩放系数，`1.0` 为原始分辨率 |
| `foxglove.sched` | `auto` | 发布线程降级让出 CPU 给推理（`auto`）或保持默认优先级（`off`） |
| `foxglove.jpeg_quality` | `80` | JPEG 质量 |

取值非法（端口越界、`fps <= 0`、`scale` 不在 `(0,1]`、JPEG 质量不在 `1-100`、`sched` 不是 `auto|off`）
会在启动时报错并以退出码 2 结束，不会静默退回默认值。

原来的命令行开关 `--foxglove-host`、`--foxglove-port`、`--foxglove-data-port`、`--foxglove-fps`、
`--foxglove-scale`、`--foxglove-sched`、`--jpeg-quality` 已移除，运行模式开关 `--simulate-gimbal`、
`--headless`、`--foxglove`、`--verbose-ekf` 也一并移除（前者改到 `foxglove` 段，后者改到 `runtime` 段）。
`CommandLineParser` 对未声明的键是**静默忽略**的，所以 `infantry` 显式检测这些旧写法（`--key` 与
`--key=value` 两种拼法都查），命中即报错退出（退出码 2）并提示改到哪个 YAML 段，不会让人误以为参数
已经生效。

`infantry` 现在没有任何布尔开关，命令行上只剩位置参数 `@config-path` 和 `-h`/`--help`：`--headless true a.yaml`
这类「布尔开关后面跟空格分隔的值」的陷阱随之消失——这种写法现在会被当成已移除开关直接报错退出，不会再
偷偷顶掉 `@config-path`。`capture` 仍有 `--headless` 与 `--foxglove` 两个布尔开关，对它来说该陷阱依旧
存在，必须裸写。

实测（MV-CS016-10UC，0.5 倍缩放、10 FPS、JPEG 质量 80）：单帧约 15-20 KB，图像码率约 0.2 MB/s，
遥测约 100 Hz。图像码率随画面内容变化，`foxglove.scale: 1.0` 或提高帧率会成倍上升，此时建议拆端口。

### 不拖慢推理：传输线程的调度

模块内部有两个发布线程，都只做「把数据送出去」，不参与检测与规划：

- `foxglove-image`：`cv::resize` + JPEG 编码，跑在 **SCHED_IDLE**（同 CPU 上没有别的可运行线程时才跑，
  比 nice 19 还低）。编码被抢占只会让画面帧率下降——帧本来就会丢最旧，不影响主链路。
- `foxglove-data`：遥测与状态的序列化与发送，只降 **nice 19**、不降到 SCHED_IDLE。它必须能及时送出
  遥测（100 Hz、单条微秒级），降到 SCHED_IDLE 有被饿死的风险。两个队列都有界且丢最旧。

100 Hz 的规划线程（发云台指令的那条）不再做 JSON 序列化：`publish_telemetry` 只做订阅判断、取时间戳和
入队，`dump()` 与 SDK 调用都在 `foxglove-data` 上完成。`foxglove.sched: off` 可整体关掉降级（排查用）。

`/vision/status` 里可以直接读到效果：`sched_mode`（请求值）、`image_sched`/`data_sched`
（`{policy, nice, errno, err}`，**实际生效值**）、`encode_ms_last`/`encode_ms_ema`（缩放+编码的单帧耗时）、
`published_telemetry`、`dropped_messages`（数据队列满丢弃数）、`queued_messages`（当前队列深度）。
若 `errno` 非 0 说明降级没生效（受限环境下会显示 `EPERM`，此时至少 nice 19 仍然生效）。
注意 Foxglove SDK 自身还会起一组 `tokio-rt-worker` 线程用于 WebSocket I/O，它们由 SDK 创建，
优先级不受本模块控制；真正吃 CPU 的图像编码已经在 `foxglove-image` 上。

没有 Foxglove 订阅、也不开本地窗口时，程序不绘制叠加、不编码、不保留检测源帧；JPEG 编码在模块内部的
发布线程完成，不占用检测主循环。视觉模块不注册任何 service，Foxglove 客户端只能观看，
不能保存、退出或控制机器人。

### 相机内参标定

```bash
./build-ninja/calibrate_camera configs/calibration.yaml <图片文件夹>
```

### 手眼标定

```bash
./build-ninja/calibrate_handeye configs/calibration.yaml <图片文件夹>
./build-ninja/calibrate_robotworld_handeye configs/calibration.yaml <图片文件夹>
```

三个 `calibrate_*` 程序的参数形式相同（`@config-path` 默认 `configs/calibration.yaml`，`@input-folder` 默认 `assets/img_with_q`），都是**无界面批处理**程序：逐张输出圆点识别结果，失败样本自动跳过，不显示图片也不等待按键。`calibrate_camera` 输出内参与重投影误差；`calibrate_handeye` 用 `cv::calibrateHandEye` 输出 `R_camera2gimbal` / `t_camera2gimbal`；`calibrate_robotworld_handeye` 用 `cv::calibrateRobotWorldHandEye`，并额外打印标定板到世界坐标系原点的水平距离，便于用卷尺交叉校验尺度。

### 视频切分

```bash
./build-ninja/split_video records/Big/2024-05-14_11-6-26 -s 0 -e 100 -p records/Big/out
```

`split_video` 按帧下标切分记录文件：`@input-path` 是不带扩展名的输入前缀（读取 `.avi` 与 `.txt`），`-s` / `--start-index`、`-e` / `--end-index` 指定帧范围，`-p` / `--output-path` 指定输出前缀。

## 常见问题

### CMake 找不到 OpenVINO

检查是否存在：

```bash
ls /opt/intel/openvino_2024.6.0/runtime/cmake/OpenVINOConfig.cmake
```

不存在时，按上方 OpenVINO 安装步骤解压并建立软链接，或者修改根 `CMakeLists.txt` 中的 `OpenVINO_DIR`。该变量是缓存变量且未加 `FORCE`，已有构建目录时请改用 `-DOpenVINO_DIR=...` 或删除构建目录后重新配置。

### 运行时报 GPU 或 OpenCL 错误

`configs/standard3.yaml` 当前使用 `device: CPU`。若已改为 `GPU` 而机器没有 Intel GPU 或 OpenCL 驱动，改回：

```yaml
device: CPU
```

### 编译时报 ccache 临时目录只读

在沙箱或受限环境下，ccache 的临时目录可能不可写，导致构建失败。指定可写目录即可：

```bash
env CCACHE_TEMPDIR=/tmp/ccache-tmp CCACHE_DIR=/tmp/ccache cmake --build build-ninja -j$(nproc)
```

### 找不到 HikRobot 相机动态库

确认 MVS SDK 已安装，并设置：

```bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
```

若配置阶段提示回退到仓库内置的 `libMvCameraControl.so`，相机可能枚举失败并返回 `MV_E_RESOURCE`，此时应安装完整 MVS SDK 并重新配置。

### 串口打开失败

确认 `/dev/gimbal` 存在、当前用户属于 `dialout` 组，且 udev 规则已生效：

```bash
ls -l /dev/gimbal
id
```

### 模型或配置路径加载失败

始终在仓库根目录运行程序，因为配置中的 `assets/...` 是相对路径。报错 `Missing YAML key: <key>` 说明配置文件缺少必填键。

### 运行后多出 `patterns/`、`logs/`、`MvSdkLog/` 等目录

这些都是运行时产物，不需要提交：

- `patterns/`：`Detector` 构造时创建，用于保存调试图。
- `logs/`：`tools::logger()` 创建，写入滚动日志 `logs/infantry.log`（单文件上限 10 MB，保留 4 个历史文件）。
- `records/`：`tools::Recorder` 的录像输出目录。
- `MvSdkLog/`：HikRobot MVS SDK 的运行日志。

### 调试数据看不到

`Plotter` 把 JSON 发到 UDP `127.0.0.1:9870`，没有接收端时数据会被丢弃，但不影响程序运行。需要一个监听该端口的绘图工具才能看到曲线。

## 开发约定

- 构建产物集中在 `build/`、`build-ninja/`、`install/`、`log/`、`logs/`、`records/`、`neo/`，不要提交。
- 代码风格和提交规范以仓库根目录 `AGENTS.md` 为准。
- 新增测试放在 `tests/`，命名以 `_test.cpp` 结尾，并在根目录 `CMakeLists.txt` 注册；需要自动回归的用 `add_test` 注册到 CTest。
- 处理姿态时遵守“姿态随帧传递”的约定，不要在异步结果处理阶段重新消费云台队列。
- 不要提交机器人私有的设备 ID、绝对路径和大型日志。
- 项目私有记忆保存在 git 忽略的 `.codex/` 下（`memory.md` 为长期事实，`current.md`/`current.json` 为当前任务状态，`sessions/` 为单次任务回顾），不要强制提交。
