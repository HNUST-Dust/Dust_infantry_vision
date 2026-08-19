# PC Link Interrupt IN 瞬时降频问题交接

## 1. 问题摘要

PC Link 已完成从 CDC ACM 到 USB 2.0 High-Speed Vendor Interrupt 的迁移：

- Interface 0，Vendor Specific `0xff`；
- Interrupt OUT `0x01`，29 字节控制命令；
- Interrupt IN `0x81`，43 字节遥测；
- 两个 endpoint 均为 MPS 64、`bInterval=4`，即 1 ms；
- 设备实际枚举为 `480M`；
- OUT 首次 arm、持续 re-arm 和 1000 帧安全压力测试已经通过；
- IN/OUT 均未发现长度、帧头或 CRC 错误。

当前剩余问题是：部分运行中，Interrupt IN 会出现一段约 3.3 秒的瞬时降频。正常时稳定为约 1000 frame/s；异常窗口内约 1.65% 的相邻内核 completion 间隔从 1 ms 变为约 2 ms。窗口结束后自动恢复，USB 不断线，也没有传输错误。

## 2. 测试方式

Host 使用 4 个持续挂起的 libusb async Interrupt IN transfer：

```text
EP: 0x81
单帧: 43 bytes
pending transfer: 4
测试时长: 30 s
OUT: 不发送
```

同时采集 Linux Bus 003 usbmon：

```bash
sudo timeout 35 cat /sys/kernel/debug/usb/usbmon/3u \
  | tee /tmp/dust_usbmon_bus3.log >/dev/null &

sleep 1
./build/gimbal_usb_async_benchmark configs/standard3.yaml 30
wait
```

usbmon 用于区分：

- USB总线/设备实际 completion 时间；
- libusb用户态 callback 被 Linux延迟处理。

## 3. 两轮实测证据

### 3.1 正常轮次

第一份有效 usbmon 抓包中，EP `0x81` 内核 completion 间隔：

| 指标 | 结果 |
|---|---:|
| 样本数 | 30,005 |
| 平均 | 999.91 us |
| P50 | 1000 us |
| P95 | 1048 us |
| P99 | 1084 us |
| 最大 | 1119 us |
| `>= 1.25 ms` | 0 |

该轮证明在相同硬件、描述符和 Host程序下，固件能够连续稳定地完成 1 kHz IN。

### 3.2 异常轮次

用户态程序输出：

```text
Elapsed: 30.004 s
IN: 29519 frames, 983.84 frame/s
Errors: timeout=0, short=0, cancelled=0, no_device=0,
        other=0, submit=0, header=0, crc=0, quaternion=0
Callback interval: P50=999.8 us, P95=1077.2 us,
                   P99=2000.5 us, max=2185.5 us
```

对应 usbmon 内核 completion：

| 指标 | 结果 |
|---|---:|
| 相邻间隔样本 | 29,518 |
| 平均 | 1016.45 us |
| P50 | 1000 us |
| P95 | 1068 us |
| P99 | 2000 us |
| 最大 | 2104 us |
| `>= 1.25 ms` | 487，1.65% |
| `>= 1.75 ms` | 487，1.65% |

487 次漏周期不是均匀分布，而是全部集中在一个 3.301 秒窗口：

| 相对秒 | completion 数 | 约 2 ms间隔数 |
|---:|---:|---:|
| 0 | 1000 | 0 |
| 1 | 1001 | 0 |
| 2 | 1000 | 0 |
| 3 | 999 | 1 |
| 4 | 791 | 209 |
| 5 | 854 | 146 |
| 6 | 874 | 126 |
| 7 | 995 | 5 |
| 8–29 | 每秒约1000 | 0 |

窗口结束后无需 reset 或重连即自行恢复。

## 4. Host 请求队列证据

异常轮次中：

```text
submit count     = 29519
completion count = 29519
```

completion 后紧邻的下一次 submit 空窗：

| 指标 | 结果 |
|---|---:|
| 平均 | 43.20 us |
| P50 | 27 us |
| P95 | 117 us |
| P99 | 128 us |
| 最大 | 189 us |

Host 始终配置 4 个 pending transfer，且重新提交最慢只有 189 us。因此异常窗口不是 Host 请求队列耗尽，也不是同步 API 完成后没有及时重新提交。

## 5. 已排除项目

根据现有证据，可以排除或基本排除：

- VID/PID、interface 或 endpoint 地址错误；
- Full-Speed 枚举，设备实际为 480M High-Speed；
- USB权限或 interface 被其他业务进程占用；
- Host 没有持续提交 IN 请求；
- 43 字节长度、`SP` 帧头或 CRC错误；
- USB断线、reset 或自动重连；
- 持续带宽不足；
- 其他 Bus 003 设备在异常窗口内产生可见流量竞争；
- 单纯的 libusb用户态 callback 延迟，因为第二轮 usbmon 内核 completion 本身也出现 2 ms间隔。

## 6. 当前判断

该问题是间歇性的 device/endpoint IN 降频，而不是固定性能上限。

当 Host 已有 pending Interrupt IN 请求时，如果设备端 EP `0x81` 没有准备好数据，设备会对 Host轮询返回 NAK。当前异常表现与“部分 1 ms周期没有及时启动/完成下一次 IN write”一致。

优先检查下位机：

1. 1 ms telemetry producer 是否在异常窗口被其他任务延迟；
2. `usbd_ep_start_write()` 是否每个目标周期都被调用；
3. 上一次 IN 尚为 busy 时如何处理下一帧，是覆盖、跳过还是排队；
4. IN completion callback 是否存在耗时处理、日志或锁竞争；
5. USB task/ISR优先级是否被其他中断或高优先级任务压制；
6. 启动后约 3–7 秒是否执行传感器初始化、Flash操作、日志刷新、参数保存或其他临界区；
7. DCache clean、DMA buffer ownership 和双缓冲切换是否可能暂时阻塞；
8. 是否存在固定持续数秒的控制/校准状态切换。

这不是要求固件盲目提高发送频率，而是确认为什么相同固件有时能完整维持 1 kHz，有时会在单个短窗口内跳过约 487 个发送机会。

## 7. 请求增加的下位机 diagnostics

现有 `platform::GetUsbSessionDiagnostics()` 已覆盖 OUT。请增加 IN 和 telemetry producer 侧计数：

```cpp
uint32_t telemetry_publish_count;
uint32_t in_start_ok_count;
uint32_t in_start_busy_count;
uint32_t in_start_error_count;
int32_t  last_in_start_error;
uint32_t in_complete_count;
uint32_t in_complete_error_count;
uint32_t in_last_nbytes;
uint32_t in_busy_drop_count;
uint32_t in_buffer_overwrite_count;
uint32_t usb_isr_max_duration_us;
uint32_t telemetry_max_interval_us;
```

至少需要区分：

- telemetry 已生成但 IN endpoint busy；
- telemetry producer 本身没有按 1 ms运行；
- `usbd_ep_start_write()` 调用失败；
- write 已启动但 completion晚到；
- 新帧覆盖了仍归 USB/DMA 使用的 buffer。

建议提供每秒 snapshot，而不是在 USB ISR 中打印日志：

```text
second
telemetry_publish delta
in_start_ok delta
in_start_busy delta
in_start_error delta
in_complete delta
in_busy_drop delta
telemetry_max_interval_us
```

正式配置没有 UART日志时，通过调试器周期读取 diagnostics，或者保存在固定大小的 RAM ring buffer 中。不要在高频回调里直接打印，以免日志本身制造抖动。

## 8. 建议的固件时间记录

建议使用单调递增的高精度 MCU timer，记录最近至少 4096 个事件：

```cpp
struct PcLinkInTraceEvent {
  uint32_t timestamp_us;
  uint8_t event;       // publish/start_ok/start_busy/start_error/complete
  int16_t status;
  uint16_t nbytes;
};
```

ring buffer 写入必须无阻塞。Host 会提供 usbmon 中异常窗口的相对时间，双方对齐后判断 2 ms间隔发生时固件处于哪个阶段。

## 9. 下一轮测试矩阵

每轮均使用 4 个 pending async IN、30 秒 usbmon：

1. MCU刚上电后立即测试；
2. MCU上电稳定10秒后测试；
3. MCU上电稳定60秒后测试；
4. 完整断电重启后重复三次；
5. 如果有可选业务模块，分别关闭/开启可疑初始化或周期任务；
6. 同步导出下位机 diagnostics/ring trace。

重点确认异常窗口是否总在测试开始后约 3–7 秒，还是与 MCU启动后的绝对时间、特定任务或状态切换相关。

## 10. 验收标准

建议修复验收条件：

- 连续 10 分钟 EP81 内核 completion 平均约 1000 frame/s；
- P99 completion interval 不超过 1.25 ms；
- 不出现连续数秒的 2 ms间隔聚集窗口；
- `in_start_error_count == 0`；
- `in_busy_drop_count == 0`，或明确证明该计数符合设计且不会降低 1 kHz遥测；
- 长度、帧头、CRC、disconnect错误均为0；
- OUT 160 Hz并行工作时保持同样指标；
- USB reset/拔插后无需重启 MCU即可恢复。

## 11. 双方下一步

### 下位机侧

- 提供 CherryUSB/HPM SDK commit；
- 标出 telemetry producer、IN start_write、completion callback代码位置；
- 增加第7节 diagnostics和必要的 RAM trace；
- 检查启动后数秒内的任务和中断活动；
- 回传正常/异常轮次的计数差异。

### 上位机侧

- 保持当前 4 pending async基准用于复现；
- 每轮保存完整 usbmon和程序输出；
- 解析每秒 completion和异常窗口；
- 与下位机 timestamp/diagnostics 对齐；
- 固件修复后执行10分钟验收和160 Hz OUT全双工复测。

相关总体协议文档见 `docs/pc_link_firmware_handoff.md`。

## 12. 下位机修改后回归结果

下位机完成新一轮修改后，上位机进行了以下回归：

### Async IN-only 30 秒

```text
IN: 30004 frames, 1000.01 frame/s
P50: 1.000 ms
P95: 1.075 ms
P99: 1.095 ms
max: 1.252 ms
USB/长度/CRC错误: 0
```

该轮出现1帧不合理四元数，因此严格结果为FAIL；随后为基准增加首个异常四元数的帧号和值记录。后续10秒诊断复测未复现：10004帧、1000.03 frame/s、P99 1.090 ms、最大1.170 ms、全部错误及四元数异常为0。该单帧暂作为重新配置/状态初始化边界观察项，不作为USB时序退化证据。

### 160 Hz OUT生产负载30秒

| 指标 | IN | OUT |
|---|---:|---:|
| 成功帧 | 30002 | 4800 |
| 帧率 | 1000.03 frame/s | 159.99 frame/s |
| P50 | 1.000 ms | 0.779 ms transfer completion |
| P95 | 1.077 ms | 1.195 ms |
| P99 | 1.115 ms | 1.284 ms |
| 最大 | 1.207 ms | 1.403 ms |
| 传输/协议错误 | 0 | 0 |

与修改前相同的30秒、160 Hz OUT测试（IN 981.79 frame/s、P99约1.998 ms）相比，本轮恢复到完整1 kHz，未出现3.3秒降频窗口。当前回归结论为“修改有效，短时与生产负载测试通过”；最终关闭问题仍建议执行至少10分钟带usbmon的160 Hz OUT验收，并同步确认固件新增IN diagnostics没有异常计数。

### 最大速率全双工30秒

OUT 不限速持续发送 `mode=0` 全零安全命令，IN持续读取遥测：

| 指标 | IN | OUT |
|---|---:|---:|
| 成功帧 | 30001 | 29943 |
| 帧率 | 1000.02 frame/s | 998.09 frame/s |
| 有效吞吐 | 344.01 kbit/s | 231.56 kbit/s |
| P50 | 1.000 ms | 1.000 ms |
| P95 | 1.081 ms | 1.083 ms |
| P99 | 1.127 ms | 1.138 ms |
| 最大 | 1.240 ms | 2.093 ms |
| timeout/短包/断线/协议错误 | 0 | 0 |

全双工有效载荷合计为71,945.49 B/s，即575.56 kbit/s。修改前最大压力同类测试约为IN 962.44 frame/s、OUT 960.54 frame/s、双向P99约2.02 ms；修改后基本达到1 ms Interrupt polling的双向上限，且IN最大值保持1.24 ms。OUT仅有单个极端最大值约2.09 ms，但P99仍为1.138 ms且没有错误。

## 13. Host 异步 OUT 改造

生产 `Gimbal::send()` 已从同步 `libusb_interrupt_transfer()` 改为非阻塞异步OUT：

- 一个OUT transfer保持在途；
- 在途期间只保存最新待发命令；
- 新命令覆盖尚未提交的旧命令，防止控制积压；
- completion callback立即提交最新待发命令；
- 独立libusb event thread处理完成事件；
- 关闭时短暂等待最后命令完成，超时才取消；
- 重连后重新claim interface、分配transfer并重建event thread；
- 保留同步write API供安全测试与A/B基准使用；
- diagnostics统计submitted/completed/overwritten/transfer_errors/submit_errors。

30秒异步OUT实测：

| 指标 | async 160 Hz | async 1000 Hz提交 |
|---|---:|---:|
| accepted calls | 4800 | 29993 |
| USB submitted/completed | 4800/4800 | 29359/29359 |
| USB实际OUT帧率 | 159.86 frame/s | 977.79 frame/s |
| latest-only覆盖 | 0 | 634 |
| enqueue P50 | 17.8 us | 0.2 us |
| enqueue P99 | 37.6 us | 25.5 us |
| transfer/submit错误 | 0 | 0 |
| IN帧率 | 999.02 frame/s | 999.00 frame/s |
| IN P99 | 1.122 ms | 1.117 ms |

160 Hz生产负载下所有命令均实际完成且无覆盖，业务调用从约1 ms同步等待下降到几十微秒内返回。1000 Hz压力下USB受1 ms轮询上限约束，latest-only按设计覆盖634个旧待发命令，没有形成积压。同步安全回归的1/10/1000个mode=0帧也全部通过。
