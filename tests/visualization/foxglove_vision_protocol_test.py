#!/usr/bin/env python3
"""End-to-end protocol test for the vision Foxglove publisher.

Runs the publisher twice: once in single-port mode (image + telemetry share one
server) and once in two-port mode, where the test also proves the isolation
property: a stalled image consumer must not starve the telemetry connection.
"""

from __future__ import annotations

import base64
import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import time

IMAGE_TOPIC = "/vision/image/compressed"
TELEMETRY_TOPIC = "/vision/telemetry"
STATUS_TOPIC = "/vision/status"

COLLECT_SECONDS = 2.5
TELEMETRY_MIN_MESSAGES = 150      # ~100 Hz expected; well below that still proves no rate limit
TELEMETRY_MAX_GAP_SECONDS = 0.25  # telemetry must stay responsive
DURATION_MS = 6000


def read_exact(sock: socket.socket, count: int) -> bytes:
    result = bytearray()
    while len(result) < count:
        block = sock.recv(count - len(result))
        if not block:
            raise ConnectionError("WebSocket closed unexpectedly")
        result.extend(block)
    return bytes(result)


def read_frame(sock: socket.socket) -> tuple[int, bytes]:
    first, second = read_exact(sock, 2)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", read_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", read_exact(sock, 8))[0]
    if second & 0x80:
        mask = read_exact(sock, 4)
        data = read_exact(sock, length)
        payload = bytes(value ^ mask[index % 4] for index, value in enumerate(data))
    else:
        payload = read_exact(sock, length)
    return first & 0x0F, payload


def send_frame(sock: socket.socket, opcode: int, payload: bytes) -> None:
    mask = os.urandom(4)
    masked = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
    size = len(payload)
    if size < 126:
        header = bytes([0x80 | opcode, 0x80 | size])
    elif size < 65536:
        header = bytes([0x80 | opcode, 0x80 | 126]) + struct.pack("!H", size)
    else:
        header = bytes([0x80 | opcode, 0x80 | 127]) + struct.pack("!Q", size)
    sock.sendall(header + mask + masked)


def send_json(sock: socket.socket, value: dict) -> None:
    send_frame(sock, 1, json.dumps(value, separators=(",", ":")).encode())


def websocket_upgrade(sock: socket.socket, port: int) -> None:
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall(
        (
            "GET / HTTP/1.1\r\n"
            f"Host: 127.0.0.1:{port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Protocol: foxglove.sdk.v1\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        ).encode()
    )
    response = bytearray()
    while b"\r\n\r\n" not in response:
        response.extend(read_exact(sock, 1))
    lines = bytes(response).split(b"\r\n")
    assert b"101" in lines[0], f"WebSocket upgrade failed: {lines[0]!r}"
    expected = base64.b64encode(
        hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()
    )
    headers = dict(line.split(b":", 1) for line in lines[1:] if b":" in line)
    assert any(
        name.lower() == b"sec-websocket-accept" and value.strip() == expected
        for name, value in headers.items()
    )


def jpeg_dimensions(data: bytes) -> tuple[int, int]:
    """Extract width/height from the JPEG SOF marker."""
    index = 2
    while index + 9 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        if marker in (0xC0, 0xC1, 0xC2, 0xC3):
            height = struct.unpack_from(">H", data, index + 5)[0]
            width = struct.unpack_from(">H", data, index + 7)[0]
            return width, height
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            index += 2
            continue
        index += 2 + struct.unpack_from(">H", data, index + 2)[0]
    raise AssertionError("no SOF marker found in the JPEG payload")


class Client:
    """Minimal Foxglove WebSocket client: advertise, subscribe, collect."""

    def __init__(self, port: int, process: subprocess.Popen[str]):
        deadline = time.monotonic() + 5
        while True:
            if process.poll() is not None:
                raise RuntimeError(
                    f"publisher exited early: {process.stdout.read()}"
                )
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=1)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)
        websocket_upgrade(self.sock, port)
        self.sock.settimeout(0.2)

        self.channels: dict[str, int] = {}
        self.capabilities: set[str] = set()
        self.supported_encodings: list[str] = []
        self.images: list[tuple[int, bytes]] = []   # (log_time_ns, jpeg bytes)
        self.telemetry: list[tuple[int, dict]] = []
        self.status: list[dict] = []
        self._subscriptions: dict[int, str] = {}
        self._next_subscription = 1
        self._drain_server_info()

    def _drain_server_info(self) -> None:
        deadline = time.monotonic() + 5
        while not self.channels and time.monotonic() < deadline:
            self._pump()

    def _pump(self) -> None:
        try:
            opcode, payload = read_frame(self.sock)
        except TimeoutError:
            return
        if opcode == 9:
            send_frame(self.sock, 10, payload)
            return
        if opcode != 1 and opcode != 2:
            return
        if opcode == 1:
            message = json.loads(payload)
            if message.get("op") == "serverInfo":
                self.capabilities.update(message.get("capabilities", []))
                self.supported_encodings = message.get("supportedEncodings", [])
                for channel in message.get("channels", []):
                    self.channels[channel["topic"]] = channel["id"]
            # 通道可以在 serverInfo 之后才发广告，所以每条文本消息都要收集一次
            for channel in message.get("channels", []):
                self.channels[channel["topic"]] = channel["id"]
            if message.get("op") == "serviceCallFailure":
                raise AssertionError(f"service call failed: {message}")
            return

        assert payload, "empty Foxglove binary message"
        assert payload[0] == 1, f"unexpected Foxglove binary opcode: {payload[0]}"
        subscription_id = struct.unpack_from("<I", payload, 1)[0]
        log_time = struct.unpack_from("<Q", payload, 5)[0]
        body = payload[13:]
        topic = self._subscriptions.get(subscription_id)
        if topic == IMAGE_TOPIC:
            self.images.append((log_time, body))
        elif topic == TELEMETRY_TOPIC:
            self.telemetry.append((log_time, json.loads(body)))
        elif topic == STATUS_TOPIC:
            self.status.append(json.loads(body))

    def subscribe(self, topics: list[str]) -> None:
        subscriptions = []
        for topic in topics:
            assert topic in self.channels, f"{topic} is not advertised"
            index = self._next_subscription
            self._next_subscription += 1
            self._subscriptions[index] = topic
            subscriptions.append({"id": index, "channelId": self.channels[topic]})
        send_json(self.sock, {"op": "subscribe", "subscriptions": subscriptions})

    def collect(self, seconds: float, pumped: list["Client"] | None = None) -> None:
        """Pump for `seconds`, also draining any clients listed in `pumped`."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for client in [self] + list(pumped or []):
                client._pump()

    def close(self) -> None:
        try:
            send_frame(self.sock, 8, struct.pack("!H", 1000))
        except OSError:
            pass
        self.sock.close()


def free_port() -> int:
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        return reservation.getsockname()[1]


def start(binary: str, image_port: int, data_port: int) -> subprocess.Popen[str]:
    return subprocess.Popen(
        [binary, str(image_port), str(data_port), str(DURATION_MS)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def finish(process: subprocess.Popen[str]) -> str:
    process.wait(timeout=20)
    output = process.stdout.read()
    assert process.returncode == 0, output
    return output


def assert_telemetry_quality(telemetry: list[tuple[int, dict]], label: str) -> None:
    assert len(telemetry) >= TELEMETRY_MIN_MESSAGES, (
        f"{label}: only {len(telemetry)} telemetry messages, expected "
        f">= {TELEMETRY_MIN_MESSAGES}"
    )
    values = [payload["counter"] for _, payload in telemetry]
    assert values == sorted(values) and len(set(values)) == len(values), (
        f"{label}: telemetry counter is not strictly increasing"
    )
    stamps = [stamp for stamp, _ in telemetry]
    max_gap = max(b - a for a, b in zip(stamps, stamps[1:])) / 1e9
    assert max_gap < TELEMETRY_MAX_GAP_SECONDS, (
        f"{label}: telemetry gap of {max_gap:.3f}s exceeds "
        f"{TELEMETRY_MAX_GAP_SECONDS}s"
    )
    assert telemetry[-1][1]["tracker_state"] == 2


def assert_scheduling_status(status: dict, label: str) -> None:
    # 发布线程的调度降级：测试程序用默认的 --foxglove-sched=auto。
    # data_sched 的 nice 是确定性断言——提高 nice 值不需要任何特权，必定生效；
    # image_sched 只断言策略名可读：SCHED_IDLE 在受限环境下可能返回 EPERM，
    # 此时会自动退回 nice 19（errno 里能看到），不应该让用例失败。
    assert status["sched_mode"] == "auto", f"{label}: sched_mode={status['sched_mode']}"
    assert status["data_sched"]["nice"] == 19, f"{label}: data_sched={status['data_sched']}"
    assert status["data_sched"]["errno"] == 0, f"{label}: data_sched={status['data_sched']}"
    assert status["image_sched"]["policy"] != "unknown", (
        f"{label}: image_sched={status['image_sched']}"
    )
    assert status["image_sched"]["nice"] == 19, f"{label}: image_sched={status['image_sched']}"
    assert isinstance(status["encode_ms_last"], (int, float)) and status["encode_ms_last"] >= 0, (
        f"{label}: encode_ms_last={status['encode_ms_last']!r}"
    )
    assert isinstance(status["encode_ms_ema"], (int, float)), (
        f"{label}: encode_ms_ema={status['encode_ms_ema']!r}"
    )
    assert isinstance(status["dropped_messages"], int) and status["dropped_messages"] >= 0, (
        f"{label}: dropped_messages={status['dropped_messages']!r}"
    )
    assert 0 <= status["queued_messages"] <= 256, (
        f"{label}: queued_messages={status['queued_messages']!r}"
    )


def run_single_port(binary: str) -> None:
    port = free_port()
    process = start(binary, port, 0)
    client = None
    try:
        client = Client(port, process)
        assert {IMAGE_TOPIC, TELEMETRY_TOPIC, STATUS_TOPIC} <= client.channels.keys(), (
            f"single port advertised {sorted(client.channels)}"
        )
        assert "json" in client.supported_encodings
        assert "services" not in client.capabilities, "vision stream must not expose services"

        client.subscribe([IMAGE_TOPIC, TELEMETRY_TOPIC, STATUS_TOPIC])
        client.collect(COLLECT_SECONDS)

        assert client.images, "no compressed image received"
        width, height = jpeg_dimensions(client.images[0][1])
        assert (width, height) == (640, 480), f"unexpected image size {width}x{height}"

        drift = abs(client.images[0][0] / 1e9 - time.time())
        assert drift < 60, f"image log_time is {drift:.1f}s away from wall clock"

        images = len(client.images)
        telemetry = len(client.telemetry)
        status = len(client.status)
        assert_telemetry_quality(client.telemetry, "single port")
        assert client.status, "no status message received"
        assert_scheduling_status(client.status[-1], "single port")

        # 断言全部通过后先断开，再等发布端按 duration 自行退出（同时验证析构顺序不死锁）
        client.close()
        client = None
        output = finish(process)
        assert "FOXGLOVE_VISION_DONE" in output, output
        print(
            f"PASS single_port images={images} telemetry={telemetry} status={status}"
        )
    finally:
        if client is not None:
            client.close()
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)


def run_split_ports(binary: str) -> None:
    image_port, data_port = free_port(), free_port()
    process = start(binary, image_port, data_port)
    image_client = data_client = None
    try:
        image_client = Client(image_port, process)
        data_client = Client(data_port, process)

        assert IMAGE_TOPIC in image_client.channels, "image port lost the image topic"
        assert TELEMETRY_TOPIC not in image_client.channels, (
            "image port must not advertise telemetry"
        )
        assert STATUS_TOPIC not in image_client.channels, (
            "image port must not advertise status"
        )
        assert TELEMETRY_TOPIC in data_client.channels and STATUS_TOPIC in data_client.channels, (
            f"data port advertised {sorted(data_client.channels)}"
        )
        assert IMAGE_TOPIC not in data_client.channels, (
            "data port must not advertise the image topic"
        )

        image_client.subscribe([IMAGE_TOPIC])
        data_client.subscribe([TELEMETRY_TOPIC, STATUS_TOPIC])

        # 先确认图像端口自己确实在发图（两个连接都读）
        image_client.collect(0.6, pumped=[data_client])
        assert image_client.images, "no compressed image received on the image port"
        width, height = jpeg_dimensions(image_client.images[0][1])
        assert (width, height) == (640, 480), f"unexpected image size {width}x{height}"

        # 隔离证明：从此不再读图像连接，让它的积压被塞满；遥测必须照常满速到达。
        # 只统计停顿窗口内的遥测，避免把停顿前的样本算进来。
        data_client.telemetry.clear()
        data_client.collect(COLLECT_SECONDS)

        assert_telemetry_quality(data_client.telemetry, "split ports")
        assert data_client.status, "no status message received"
        assert data_client.status[-1]["split_mode"] is True
        assert data_client.status[-1]["data_port"] == data_port
        assert data_client.status[-1]["image_clients"] >= 1
        # 图像连接被停读的这段时间里遥测仍然满速，说明数据线程没有被图像积压拖住。
        assert_scheduling_status(data_client.status[-1], "split ports")

        images = len(image_client.images)
        telemetry = len(data_client.telemetry)
        status = len(data_client.status)
        image_client.close()
        data_client.close()
        image_client = data_client = None
        output = finish(process)
        assert "FOXGLOVE_VISION_DONE" in output, output
        print(
            f"PASS split_ports image={images} telemetry={telemetry} status={status}"
        )
    finally:
        for client in (image_client, data_client):
            if client is not None:
                client.close()
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)


def run(binary: str) -> None:
    run_single_port(binary)
    run_split_ports(binary)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(f"Usage: {sys.argv[0]} <foxglove_vision_test>")
    run(sys.argv[1])
