# PC Link Vendor Interrupt 下位机修复与联调交接

## 1. 交接目标

将 HPMicro + CherryUSB 下位机的 PC Link 从“只能上报遥测”修复为双向可用：

- 下位机通过 Interrupt IN `0x81` 每 1 ms 上报一帧 43 字节遥测；
- 上位机通过 Interrupt OUT `0x01` 单次发送一帧 29 字节控制命令；
- 下位机持续接收命令，检查长度、帧头和 CRC 后才更新控制状态；
- 拔插、错误包和暂时没有控制命令时不产生旧命令重放或异常动作。

## 2. 已确认的主机侧证据

2026-08-19 在目标 NUC 上确认：

| 项目 | 实测结果 |
|---|---|
| VID:PID | `34b7:ffff` |
| USB 速度 | `480M` High-Speed |
| Interface | `0`，Vendor Specific `0xff` |
| OUT endpoint | `0x01`，Interrupt，MPS 64，`bInterval=4` |
| IN endpoint | `0x81`，Interrupt，MPS 64，`bInterval=4` |
| IN 遥测 | 43 字节帧可正常接收，主程序已获得四元数 |
| OUT 控制 | 每次 29 字节写入均 `LIBUSB_ERROR_TIMEOUT`，`transferred=0` |
| 进程占用 | 只有一个 `infantry_debug` 业务进程 claim Interface 0 |

由此可以排除 VID/PID、枚举速度、端点地址、描述符类型、上位机帧长和接口冲突。`0/29` 超时表示 USB Device 没有 ACK OUT transaction；数据尚未进入下位机，因此当前不是帧头或 CRC 解析问题。

## 3. 下位机首要检查项

最可能的故障是 EP `0x01` 没有启动接收，或只启动了一次但完成后没有重新启动。

如果确认已经调用 `usbd_ep_start_read()`，按以下顺序排查次级原因：传入了错误 `busid`、把 OUT `0x01` 写成 IN `0x81`、completion callback 没有注册到 `0x01`、在 configuration 完成前 arm 后又被端点配置清除、USB ISR/CherryUSB task 没有持续执行，或者固件主动 stall 了端点。STALL 通常会让 libusb 很快返回 `LIBUSB_ERROR_PIPE`，因此在当前持续 TIMEOUT 症状下优先级较低。

负责下位机的 Agent 应逐项确认：

1. USB configuration 完成后调用一次 `usbd_ep_start_read()`，为 `0x01` 挂起接收；
2. OUT transfer 完成回调确实注册到 EP `0x01`；
3. 每次完成、包括收到异常长度包后，都再次调用 `usbd_ep_start_read()`；
4. 接收缓冲区不是栈变量，生命周期覆盖整个 USB 传输；
5. 缓冲区满足 HPM USB DMA/CherryUSB 所要求的 section 和 alignment；
6. 开启 DCache 时按当前 HPM CherryUSB port 要求执行 cache invalidate/使用 non-cache RAM；
7. 不要在 USB ISR/endpoint callback 中做耗时控制计算，只完成校验、复制/交换最新命令和 re-arm；
8. USB reset、disconnect、重新 configuration 后能重新初始化接收状态。

## 4. 推荐固件结构

下面是逻辑示例，CherryUSB API 参数和事件回调签名必须以当前固件版本为准：

```c
#define PC_LINK_EP_OUT   0x01U
#define PC_LINK_RX_SIZE  29U

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX
static uint8_t pc_link_rx_buffer[PC_LINK_RX_SIZE];

static void pc_link_arm_out(void)
{
    int ret = usbd_ep_start_read(
        BUS_ID,
        PC_LINK_EP_OUT,
        pc_link_rx_buffer,
        sizeof(pc_link_rx_buffer));

    /* 失败时增加计数和限频日志，不能悄悄忽略。 */
}

static void pc_link_out_complete(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if (ep == PC_LINK_EP_OUT) {
        if (nbytes == PC_LINK_RX_SIZE &&
            pc_link_rx_buffer[0] == 'S' &&
            pc_link_rx_buffer[1] == 'P' &&
            pc_link_crc_is_valid(pc_link_rx_buffer, PC_LINK_RX_SIZE)) {
            /* 将完整命令复制/发布到控制任务；只保留最新合法命令。 */
            pc_link_publish_latest_command(pc_link_rx_buffer);
        } else {
            /* 分别统计 bad_length、bad_header、bad_crc。 */
        }

        /* 无论本次数据是否合法，都必须挂起下一次接收。 */
        pc_link_arm_out();
    }
}

static void pc_link_configured(void)
{
    pc_link_reset_command_state();
    pc_link_arm_out();
}
```

如果 CherryUSB 回调运行在中断上下文，建议使用双缓冲、消息队列或短临界区发布命令。不得在重新 arm 前等待控制任务处理完毕，否则主机下一次 OUT 可能被 NAK。

RX buffer 在重新 arm 后重新归 USB 控制器所有，控制任务不得继续引用它。推荐先复制 29 字节到独立 snapshot，再尽快 re-arm，通过临界区、双缓冲交换或序号锁把整条命令原子发布给控制任务，避免读取到半更新的 mode/yaw/pitch。

## 5. 29 字节控制帧

所有多字节字段为小端：

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 2 | `'S','P'` |
| 2 | 1 | `mode`：0 不控制，1 控制不开火，2 控制并开火 |
| 3 | 4 | `yaw_angle` float32 |
| 7 | 4 | `yaw_velocity` float32 |
| 11 | 4 | `yaw_acceleration` float32 |
| 15 | 4 | `pitch_angle` float32 |
| 19 | 4 | `pitch_velocity` float32 |
| 23 | 4 | `pitch_acceleration` float32 |
| 27 | 2 | CRC16，小端 |

CRC 覆盖前 27 字节：初值 `0xffff`，反射多项式 `0x8408`，LSB first，无最终异或。

安全要求：只有长度、帧头、CRC 全部合法时才能更新命令。异常包不得改变当前控制状态。下位机命令看门狗默认为 100 ms，由 `CONFIG_WBR_CONTROL_PC_LINK_COMMAND_TIMEOUT_MS` 配置。超过该时间没有收到新的合法命令，或者发生 USB reset、disconnect、reconfiguration，下位机会发布新的 `mode=0` 空闲命令，禁止继续沿用旧控制状态。

## 6. 下位机临时诊断计数

下位机通过 `platform::GetUsbSessionDiagnostics()` 提供以下联调计数；正式配置未启用 UART 日志后端时，通过调试器调用 getter 或观察 `g_usb_diag`：

- `configured_count`
- `reset_count` / `disconnect_count`
- `out_arm_ok_count` / `out_arm_error_count` / `last_out_arm_error`
- `out_complete_count`
- `out_last_nbytes`
- `command_valid_count`
- `bad_length_count`
- `bad_header_count`
- `bad_crc_count`

判断顺序：

1. 枚举后 `configured_count >= 1`；
2. `out_arm_ok_count >= 1` 且 `out_arm_error_count == 0`；
3. Host 每成功发送一帧，`out_complete_count` 增长且 `out_last_nbytes == 29`；
4. 合法命令使 `command_valid_count` 增长，三类错误计数保持为 0。

如果 `out_arm_ok_count` 增长但没有 completion，需要继续检查 HPM USB device controller OUT endpoint、DMA/cache 和 endpoint index/direction 配置。

## 7. 最小联调顺序

1. 下位机启动但不上电执行机构，确认枚举为 `34b7:ffff`、单接口、480M；
2. 确认 configuration 事件后 `out_arm_ok_count >= 1`；
3. 上位机启动只发送 `mode=0` 的安全测试程序，确认不再出现 OUT timeout；
4. 先发送 1 帧，再发送间隔 20–100 ms 的 10 帧；如果仅第一帧成功，直接检查 completion 后漏 re-arm；
5. 核对每次成功写入都是 29 字节，下位机 completion 的 `nbytes` 也是 29；
6. 确认合法帧计数增长，错误计数为 0；
7. 人为发送错误 CRC/长度测试只允许在安全台架程序中进行，确认不会更新控制状态，且随后的合法帧仍能接收；
8. 连续发送 1000 个合法命令，要求 Host 无 timeout、固件无 re-arm failure；
9. 测试拔插，确认重新枚举后 OUT 被重新 arm，旧命令被清空；
10. 最后才运行完整 `infantry_debug --headless=true`，观察控制、延迟和错误计数。

## 8. 双方交付内容

### 下位机 Agent 回传

- CherryUSB 版本或 commit；
- HPM SDK/USB device controller port 版本；
- endpoint callback 注册和首次 arm 的代码位置；
- OUT 缓冲区声明、section/alignment 和 cache 策略；
- 修复 commit/diff；
- 上述诊断计数的实测结果；
- 断线重连和异常帧测试结果。

### 上位机侧提供

- 当前单次 Interrupt OUT 长度固定为 29，endpoint 固定为 `0x01`；
- 默认 libusb transfer timeout 为 20 ms；
- 当前失败日志为 `LIBUSB_ERROR_TIMEOUT, transferred 0/29`；
- 联调时提供一帧 `mode=0` 的真实 29 字节 hex 和 CRC golden vector；
- 修复下位机后，上位机负责复测成功写入率、接收帧率、P99 延迟及拔插恢复。

如果固件报告 arm 成功但 callback 始终不触发，双方下一步使用 Linux `usbmon`/Wireshark 对齐 Host transfer 时间戳和固件 configured/arm 日志，确认总线上是 NAK、STALL 还是没有到达 OUT token。抓包前停止完整 `infantry_debug`，使用最小安全单帧程序，避免混入高频业务流量。

## 9. 当前结论

2026-08-19 第一版修复实测结果：

- 第一个 `mode=0`、全零设定值、CRC `0x7199` 的 29 字节帧成功，Host `transferred=29`；
- 间隔 20 ms 后发送第二帧，Host `LIBUSB_ERROR_TIMEOUT`、`transferred=0`；
- 由于第二帧失败，10 帧阶段立即停止，1000 帧阶段没有执行；
- golden frame 为 `53 50 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 99 71`。

该结果证明首次 arm 已成功，当前故障收敛为第一次 OUT completion 后没有成功 re-arm。下位机应立即读取 `out_complete_count`、`out_arm_ok_count`、`out_arm_error_count` 和 `last_out_arm_error`：若 completion 为 1 且 arm ok 仍为 1，检查 callback 是否执行 re-arm；若 arm error 增长，按错误码检查 callback 上下文、buffer ownership、cache/对齐和 HPM USB controller 状态。

上位机不应通过增加 timeout 掩盖问题；这会继续阻塞视觉控制线程并增加延迟。应先让下位机 EP `0x01` 持续处于可接收状态。固件修复前应停止完整视觉进程，避免每次控制循环额外等待 20 ms。

### 最终复测

下位机完成 re-arm 修复后，目标 NUC 实际运行：

```text
PASS count=1 mode=0 bytes=29 crc=0x7199
PASS count=10 mode=0 bytes=29 crc=0x7199
PASS count=1000 mode=0 bytes=29 crc=0x7199
```

首次 arm、completion 后连续 re-arm、29 字节帧长和 CRC golden vector 均已通过 Host 验收，测试期间没有 OUT timeout。PC Link Interrupt OUT 修复项可以关闭；下一阶段是在安全条件下运行完整 `infantry_debug --headless=true`，同步观察 Host 错误日志、`VisionLatency` 和下位机 diagnostics，并完成拔插自动重连验收。

### 10 秒全双工性能基准

使用 `gimbal_usb_benchmark`，IN 持续读取 43 字节遥测，同时 OUT 无间隔持续发送 29 字节 `mode=0` 全零安全命令：

| 指标 | IN | OUT |
|---|---:|---:|
| 成功帧 | 9677 | 9771 |
| 帧率 | 967.64 frame/s | 977.04 frame/s |
| 有效载荷 | 332.87 kbit/s | 226.67 kbit/s |
| transfer P50 | 999.8 us | 999.9 us |
| transfer P95 | 1097.7 us | 1086.2 us |
| transfer P99 | 2004.7 us | 2005.0 us |
| 最大值 | 2134.7 us | 2162.8 us |

全双工合计有效载荷为 69,942.37 B/s，即 559.54 kbit/s。两个方向均为 0 timeout、0 短包、0 disconnect/other error；IN 另为 0 帧头错误、0 CRC 错误、0 不合理四元数。相对 1 ms Interrupt polling 的 1000 frame/s 上限，IN/OUT 利用率约为 96.8%/97.7%。P99 约 2 ms 表示少量同步 transfer 错过一个 polling slot；该结果是 Host USB transfer 完成时间，不代表视觉到执行器的端到端控制延迟。

### 与 CDC ACM 的同条件仅 IN 对比

`gimbal_usb_benchmark ... 10 in-only` 不提交任何 OUT transfer。首轮受到一次明显的主机调度异常影响，仅得到 942.64 frame/s、P95 1.954 ms；随后两次重复测试稳定：

| 指标 | CDC ACM 历史测试 | Vendor IN-only 复测 1 | Vendor IN-only 复测 2 |
|---|---:|---:|---:|
| 帧率 | 1000.14 frame/s | 1000.04 frame/s | 1000.05 frame/s |
| 有效吞吐 | 344.05 kbit/s | 344.01 kbit/s | 344.02 kbit/s |
| P50 | 1.000 ms | 1.000 ms | 1.000 ms |
| P95 | 1.092 ms | 1.078 ms | 1.069 ms |
| P99 | 1.147 ms | 1.137 ms | 1.095 ms |
| 最大 | 1.243 ms | 1.202 ms | 1.181 ms |
| CRC/帧错误 | 0 | 0 | 0 |

稳定样本中两种 transport 的单向吞吐差异小于 0.02%，均达到固件 1 kHz 遥测周期；Vendor 的分位数略低，但差异很小。由于 Vendor 曾出现一次无协议错误但 Host 调度整体退化的 10 秒样本，而 CDC 只有一轮历史数据，不能据此宣称 Vendor 的长期抖动一定更好。协议层优势主要是明确的一次 transfer 一帧、无需串口搜帧/重同步，以及已经验证的约 1 kHz 双向能力。

### OUT 频率扫描和长时对照

10 秒频率扫描结果：

| OUT 频率 | IN frame/s | IN P95 | IN P99 | OUT frame/s | 错误 |
|---:|---:|---:|---:|---:|---:|
| 0（IN-only） | 999.89 | 1.080 ms | 1.127 ms | 0 | 0 |
| 100 Hz | 1000.06 | 1.079 ms | 1.121 ms | 100.00 | 0 |
| 160 Hz | 999.98 | 1.080 ms | 1.114 ms | 160.00 | 0 |
| 500 Hz | 1000.05 | 1.077 ms | 1.110 ms | 499.98 | 0 |
| max | 962.44 | 1.110 ms | 2.020 ms | 960.54 | 0 |

短测表明 500 Hz 以内的 OUT 不会明显影响 IN，只有双向同时逼近 1 kHz 时出现明显 polling-slot miss。

但 30 秒长测发现同步单请求模型自身存在周期性尾部：

| 条件 | IN frame/s | P50 | P95 | P99 | 最大 | 错误 |
|---|---:|---:|---:|---:|---:|---:|
| IN-only | 982.46 | 1.000 ms | 1.084 ms | 2.001 ms | 2.154 ms | 0 |
| OUT 160 Hz | 981.79 | 1.000 ms | 1.098 ms | 1.998 ms | 2.150 ms | 0 |

两档长测几乎相同，说明生产相关的 160 Hz OUT 不是长时 2 ms 尾部的原因。更可能是当前同步 libusb 每方向只挂一个请求，完成后由普通 Linux 用户态线程重新提交；线程偶尔晚醒会错过一个 1 ms polling slot。下一步应以 2–4 个持续挂起的异步 IN transfer 做 A/B 测试，并用 usbmon 区分总线 completion 和用户态通知延迟。

### 异步 IN A/B

新增 `gimbal_usb_async_benchmark`，始终保持 4 个 EP `0x81` transfer 挂起，不发送 OUT。30 秒重复结果：

| 轮次 | IN frame/s | P50 | P95 | P99 | 最大 | 错误 |
|---|---:|---:|---:|---:|---:|---:|
| async 1 | 992.20 | 1.000 ms | 1.078 ms | 1.143 ms | 2.137 ms | 0 |
| async 2 | 987.46 | 1.000 ms | 1.082 ms | 1.971 ms | 2.188 ms | 0 |

另一次 10 秒异步测试稳定达到 999.97 frame/s、P99 1.143 ms、最大 1.224 ms。相比同步 30 秒的约 982 frame/s，异步多请求平均有所改善，但第二轮仍出现 2 ms P99，因此不能把根因完全归结为同步 resubmit 空窗，也暂不应直接替换生产 transport。需要用 Bus 003 usbmon 将内核 URB completion 时间与用户态 callback 时间对齐。当前自动抓包因 `sudo` 需要交互密码而未取得数据。

### usbmon 根因确认

用户以 root 权限完成 Bus 003 抓包，共 60,012 行、约 30,000 次 EP `0x81` submit/completion。内核 URB completion 的相邻间隔为：

| 样本数 | 平均 | P50 | P95 | P99 | 最大 | `>=1.25 ms` |
|---:|---:|---:|---:|---:|---:|---:|
| 30,005 | 999.91 us | 1000 us | 1048 us | 1084 us | 1119 us | 0 |

completion 后紧邻的下一次 submit 空窗：平均 45.1 us、P50 30 us、P95 120 us、P99 130 us、最大 169 us，没有一次超过 250 us。所有 completion 状态正常。

因此 USB总线、xHCI调度、固件 1 kHz IN、CherryUSB re-arm 均没有 2 ms间隔；用户态基准看到的 2 ms来自 libusb event handling/普通 Linux线程偶尔晚处理或批量分发 callback。4 个异步请求能保证内核始终有请求挂起，避免总线丢帧，但用户态消费时间仍会受到调度影响。若业务确实要求用户态 P99 接近 1.1 ms，应采用异步多请求生产 transport，并给 USB event thread 设置合适的 CPU affinity/调度优先级；是否需要这样做应根据完整视觉控制端到端延迟决定。

后续另一轮抓包出现了不同状态，说明上面的结论只适用于第一轮而不是所有运行：异步用户态得到 983.84 frame/s、P99 2.001 ms；对应 usbmon 内核 completion 也为平均 1016.45 us、P99 2000 us，共 487/29,518（1.65%）个间隔接近 2 ms。Host 始终维持请求：submit/completion 均为 29,519，completion 到下一 submit P99 128 us、最大 189 us。

487 个漏周期全部集中在一个 3.301 秒窗口：该窗口前后每秒均约 1000 completion；窗口内三个完整秒仅约 791、854、874 completion，随后恢复。抓包中没有其他 Bus 003 流量竞争。这排除了持续带宽瓶颈，也不符合 Host 请求队列耗尽；更像下位机在该窗口没有每毫秒及时调用/完成 EP81 IN write，或设备控制器短暂持续 NAK。下一步应在固件 diagnostics 增加 `in_start_ok/error`、`in_complete_count`、最近一次错误、每秒 telemetry publish/IN complete 数，并与 usbmon 时间戳对齐；同时重复多轮抓包确认窗口是否总在启动后约 3–7 秒出现。
