#!/usr/bin/env python3
"""End-to-end protocol test for the calibration Foxglove publisher."""

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


def call_service(sock: socket.socket, service_id: int, call_id: int) -> None:
    encoding = b"json"
    payload = struct.pack("<BIII", 2, service_id, call_id, len(encoding)) + encoding + b"{}"
    send_frame(sock, 2, payload)


def connect(port: int, process: subprocess.Popen[str]) -> socket.socket:
    deadline = time.monotonic() + 5
    while True:
        if process.poll() is not None:
            raise RuntimeError(f"publisher exited before accepting connections: {process.stdout.read()}")
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=1)
        except OSError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.05)


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


def run(binary: str) -> None:
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]

    process = subprocess.Popen(
        [binary, str(port)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    sock: socket.socket | None = None
    try:
        sock = connect(port, process)
        websocket_upgrade(sock, port)
        sock.settimeout(1)

        channels: dict[str, int] = {}
        services: dict[str, int] = {}
        responses: dict[int, dict] = {}
        subscribed = False
        save_called = False
        quit_called = False
        image_count = 0
        latest_status: dict = {}
        capabilities: set[str] = set()
        deadline = time.monotonic() + 10

        while time.monotonic() < deadline:
            try:
                opcode, payload = read_frame(sock)
            except TimeoutError:
                continue
            if opcode == 9:
                send_frame(sock, 10, payload)
                continue
            if opcode == 8:
                break
            if opcode == 1:
                message = json.loads(payload)
                if message.get("op") == "serverInfo":
                    capabilities.update(message.get("capabilities", []))
                    assert "json" in message.get("supportedEncodings", [])
                for channel in message.get("channels", []):
                    channels[channel["topic"]] = channel["id"]
                for service in message.get("services", []):
                    services[service["name"]] = service["id"]
                if message.get("op") == "serviceCallFailure":
                    raise AssertionError(f"service call failed: {message}")
            elif opcode == 2:
                assert payload, "empty Foxglove binary message"
                message_opcode = payload[0]
                if message_opcode == 1:
                    assert len(payload) >= 13
                    subscription_id = struct.unpack_from("<I", payload, 1)[0]
                    body = payload[13:]
                    if subscription_id == 1:
                        assert b"\xff\xd8" in body and b"\xff\xd9" in body
                        image_count += 1
                    elif subscription_id == 2:
                        latest_status = json.loads(body)
                elif message_opcode == 3:
                    assert len(payload) >= 13
                    service_id, call_id, encoding_size = struct.unpack_from("<III", payload, 1)
                    start = 13
                    encoding = payload[start : start + encoding_size]
                    assert encoding == b"json"
                    responses[call_id] = json.loads(payload[start + encoding_size :])
                    assert service_id in services.values()
                else:
                    raise AssertionError(f"unexpected Foxglove binary opcode: {message_opcode}")

            required_channels = {
                "/calibration/image/compressed",
                "/calibration/status",
            }
            required_services = {"/calibration/save", "/calibration/quit"}
            if not subscribed and required_channels <= channels.keys():
                send_json(
                    sock,
                    {
                        "op": "subscribe",
                        "subscriptions": [
                            {"id": 1, "channelId": channels["/calibration/image/compressed"]},
                            {"id": 2, "channelId": channels["/calibration/status"]},
                        ],
                    },
                )
                subscribed = True
            if (
                not save_called
                and required_services <= services.keys()
                and latest_status.get("orientation_valid") is False
            ):
                assert latest_status["yaw_degree"] is None
                assert latest_status["pitch_degree"] is None
                assert latest_status["roll_degree"] is None
                assert "services" in capabilities
                call_service(sock, services["/calibration/save"], 101)
                save_called = True
            if (
                save_called
                and responses.get(101, {}).get("success") is True
                and latest_status.get("saved_count") == 1
                and latest_status.get("last_event") == "saved_1"
                and image_count > 0
                and not quit_called
            ):
                assert latest_status["grid_detected"] is True
                assert latest_status["orientation_valid"] is True
                assert latest_status["yaw_degree"] == 12.5
                call_service(sock, services["/calibration/quit"], 102)
                quit_called = True
            if quit_called and responses.get(102, {}).get("success") is True:
                break

        assert subscribed, "calibration topics were not advertised"
        assert save_called and responses.get(101, {}).get("success") is True
        assert latest_status.get("last_event") == "saved_1"
        assert image_count > 0
        assert quit_called and responses.get(102, {}).get("success") is True
        process.wait(timeout=5)
        output = process.stdout.read()
        assert process.returncode == 0, output
        assert "FOXGLOVE_TEST_QUIT saved_count=1" in output
        print(
            "PASS topics=2 services=2 "
            f"images={image_count} last_event={latest_status['last_event']}"
        )
    finally:
        if sock is not None:
            try:
                send_frame(sock, 8, struct.pack("!H", 1000))
            except OSError:
                pass
            sock.close()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(f"Usage: {sys.argv[0]} <foxglove_calibration_test>")
    run(sys.argv[1])
