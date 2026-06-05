#!/usr/bin/env python3
"""Offline PC-side tester for the ESP32 line-car state machine.

The script intentionally uses only the Python standard library so it can run
after the computer switches to the ESP32 SoftAP and loses internet access.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import random
import socket
import struct
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


DEFAULT_HOST = "192.168.4.1"
DEFAULT_LOG_ROOT = "logs/state-machine-tests"
DEFAULT_PARAM_ID = "pid.kp"
DEFAULT_PARAM_VALUE = 7500


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def slug_timestamp() -> str:
    return datetime.now().strftime("%Y%m%d-%H%M%S")


def json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"), sort_keys=True)


def compact(data: Any, limit: int = 240) -> str:
    text = json_dumps(data) if not isinstance(data, str) else data
    return text if len(text) <= limit else text[: limit - 3] + "..."


def read_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("socket closed while reading")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class SimpleWebSocket:
    def __init__(self, host: str, path: str = "/ws", port: int = 80, timeout: float = 4.0):
        self.host = host
        self.path = path
        self.port = port
        self.timeout = timeout
        self.sock: socket.socket | None = None

    def connect(self) -> None:
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        sock.settimeout(self.timeout)
        request = (
            f"GET {self.path} HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "User-Agent: dcar-liner-state-machine-tester/1.0\r\n"
            "\r\n"
        )
        sock.sendall(request.encode("ascii"))
        response = b""
        while b"\r\n\r\n" not in response:
            chunk = sock.recv(4096)
            if not chunk:
                break
            response += chunk
            if len(response) > 8192:
                break
        header = response.decode("latin1", errors="replace")
        if " 101 " not in header.splitlines()[0]:
            sock.close()
            raise ConnectionError(f"WebSocket upgrade failed: {header.splitlines()[0] if header else 'empty response'}")
        expected = base64.b64encode(
            hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()
        ).decode("ascii")
        if expected not in header:
            sock.close()
            raise ConnectionError("WebSocket upgrade missing expected accept key")
        self.sock = sock

    def close(self) -> None:
        if not self.sock:
            return
        try:
            self._send_frame(b"", opcode=0x8)
        except OSError:
            pass
        try:
            self.sock.close()
        finally:
            self.sock = None

    def send_json(self, payload: dict[str, Any]) -> None:
        self._send_frame(json_dumps(payload).encode("utf-8"), opcode=0x1)

    def recv_json(self) -> dict[str, Any]:
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = max(0.1, deadline - time.monotonic())
            if not self.sock:
                raise ConnectionError("WebSocket is not connected")
            self.sock.settimeout(remaining)
            opcode, payload = self._recv_frame()
            if opcode == 0x1:
                return json.loads(payload.decode("utf-8", errors="replace"))
            if opcode == 0x8:
                raise ConnectionError("WebSocket closed by peer")
            if opcode == 0x9:
                self._send_frame(payload, opcode=0xA)
            if time.monotonic() > deadline:
                raise TimeoutError("timed out waiting for text WebSocket frame")

    def request_json(self, payload: dict[str, Any]) -> dict[str, Any]:
        self.send_json(payload)
        return self.recv_json()

    def _send_frame(self, payload: bytes, opcode: int) -> None:
        if not self.sock:
            raise ConnectionError("WebSocket is not connected")
        first = 0x80 | opcode
        length = len(payload)
        if length <= 125:
            header = struct.pack("!BB", first, 0x80 | length)
        elif length <= 65535:
            header = struct.pack("!BBH", first, 0x80 | 126, length)
        else:
            header = struct.pack("!BBQ", first, 0x80 | 127, length)
        mask = random.randbytes(4) if hasattr(random, "randbytes") else os.urandom(4)
        masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        self.sock.sendall(header + mask + masked)

    def _recv_frame(self) -> tuple[int, bytes]:
        if not self.sock:
            raise ConnectionError("WebSocket is not connected")
        first, second = read_exact(self.sock, 2)
        opcode = first & 0x0F
        masked = bool(second & 0x80)
        length = second & 0x7F
        if length == 126:
            length = struct.unpack("!H", read_exact(self.sock, 2))[0]
        elif length == 127:
            length = struct.unpack("!Q", read_exact(self.sock, 8))[0]
        mask = read_exact(self.sock, 4) if masked else b""
        payload = read_exact(self.sock, length) if length else b""
        if masked:
            payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        return opcode, payload


@dataclass
class CheckResult:
    name: str
    ok: bool
    note: str


class TestRun:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.started = time.monotonic()
        self.log_dir = Path(args.log_dir) / slug_timestamp()
        self.log_dir.mkdir(parents=True, exist_ok=False)
        self.events_path = self.log_dir / "events.jsonl"
        self.summary_path = self.log_dir / "summary.md"
        self.results: list[CheckResult] = []

    def event(self, kind: str, name: str, **fields: Any) -> None:
        event = {
            "ts": now_iso(),
            "elapsed_ms": int((time.monotonic() - self.started) * 1000),
            "kind": kind,
            "name": name,
            **fields,
        }
        with self.events_path.open("a", encoding="utf-8") as handle:
            handle.write(json_dumps(event) + "\n")
        status = fields.get("status")
        suffix = f" {status}" if status else ""
        detail = fields.get("note") or fields.get("error") or ""
        print(f"[{event['elapsed_ms']:>6} ms] {kind}:{name}{suffix} {detail}".rstrip())

    def check(self, name: str, ok: bool, note: str) -> bool:
        self.results.append(CheckResult(name, ok, note))
        self.event("check", name, status="PASS" if ok else "FAIL", note=note)
        return ok

    def http_get(self, path: str) -> Any:
        url = f"http://{self.args.host}{path}"
        self.event("http", path, request={"method": "GET", "url": url})
        started = time.monotonic()
        try:
            with urllib.request.urlopen(url, timeout=self.args.timeout) as response:
                body = response.read().decode("utf-8", errors="replace")
                try:
                    parsed = json.loads(body)
                except json.JSONDecodeError:
                    parsed = body
                self.event(
                    "http",
                    path,
                    status="OK",
                    duration_ms=int((time.monotonic() - started) * 1000),
                    response=parsed,
                    note=compact(parsed),
                )
                return parsed
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            self.event(
                "http",
                path,
                status="ERROR",
                duration_ms=int((time.monotonic() - started) * 1000),
                error=str(exc),
            )
            raise

    def ws_request(self, ws: SimpleWebSocket, payload: dict[str, Any], name: str | None = None) -> Any:
        label = name or payload.get("type", "unknown")
        self.event("ws", label, request=payload)
        started = time.monotonic()
        try:
            response = ws.request_json(payload)
            self.event(
                "ws",
                label,
                status="OK",
                duration_ms=int((time.monotonic() - started) * 1000),
                request=payload,
                response=response,
                note=compact(response),
            )
            return response
        except (ConnectionError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            self.event(
                "ws",
                label,
                status="ERROR",
                duration_ms=int((time.monotonic() - started) * 1000),
                request=payload,
                error=str(exc),
            )
            raise

    def write_summary(self) -> None:
        passed = sum(1 for item in self.results if item.ok)
        failed = len(self.results) - passed
        lines = [
            "# ESP32 State Machine Test Summary",
            "",
            f"- Started: `{now_iso()}`",
            f"- Host: `{self.args.host}`",
            f"- Include AUTO_RUNNING test: `{self.args.include_auto}`",
            f"- Result: `{'PASS' if failed == 0 else 'FAIL'}`",
            f"- Checks: `{passed}` pass, `{failed}` fail",
            f"- Event log: `{self.events_path.name}`",
            "",
            "## Checks",
            "",
        ]
        for item in self.results:
            lines.append(f"- `{'PASS' if item.ok else 'FAIL'}` {item.name}: {item.note}")
        lines.extend(
            [
                "",
                "## Notes",
                "",
                "- The default run does not send `start_auto`, so it should not start closed-loop driving.",
                "- The run sends one preflight `stop` before formal state checks, so repeated tests start from `SAFE_IDLE` when the firmware accepts stop.",
                "- Run again with `--include-auto` only when the car is lifted or otherwise physically safe.",
                "- If HTTP fails but the ESP32 root page opens in a browser, capture this log folder for routing/API analysis.",
                "",
            ]
        )
        self.summary_path.write_text("\n".join(lines), encoding="utf-8")


def get_nested(data: Any, *keys: str) -> Any:
    current = data
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def vehicle_field(data: Any, name: str) -> Any:
    if not isinstance(data, dict):
        return None
    return (
        get_nested(data, "vehicle", name)
        or get_nested(data, "telemetry", "vehicle", name)
        or data.get(name)
    )


def response_kind(data: Any) -> str | None:
    if isinstance(data, dict):
        value = data.get("status")
        if isinstance(value, str):
            return value
        value = data.get("message")
        if isinstance(value, str):
            return value
        value = data.get("type")
        if isinstance(value, str):
            return value
    return None


def run(args: argparse.Namespace) -> int:
    test = TestRun(args)
    ws: SimpleWebSocket | None = None
    try:
        test.event("run", "start", host=args.host, log_dir=str(test.log_dir))

        health = test.http_get("/api/health")
        test.check("HTTP /api/health responds", isinstance(health, dict), f"response={compact(health)}")

        params = test.http_get("/api/params")
        test.check("HTTP /api/params responds", isinstance(params, dict), f"response={compact(params)}")

        schema = test.http_get("/api/schema")
        test.check("HTTP /api/schema responds", isinstance(schema, dict), f"response={compact(schema)}")

        ws = SimpleWebSocket(args.host, timeout=args.timeout)
        ws.connect()
        test.event("ws", "connect", status="OK", note=f"ws://{args.host}/ws")

        before_reset = test.http_get("/api/telemetry")
        before_state = vehicle_field(before_reset, "motion_state")
        before_fault = vehicle_field(before_reset, "fault")
        test.event(
            "run",
            "preflight_state_before_stop",
            note=f"motion_state={before_state}, fault={before_fault}",
            telemetry=before_reset,
        )

        reset = test.ws_request(ws, {"type": "stop"}, "preflight_safety_stop")
        reset_state = vehicle_field(reset, "motion_state")
        reset_fault = vehicle_field(reset, "fault")
        test.check(
            "preflight safety stop reaches SAFE_IDLE",
            reset_state == "SAFE_IDLE" and reset_fault in {None, "NONE"},
            f"motion_state={reset_state}, fault={reset_fault}",
        )

        telemetry = test.http_get("/api/telemetry")
        initial_state = get_nested(telemetry, "vehicle", "motion_state")
        initial_fault = get_nested(telemetry, "vehicle", "fault")
        test.check(
            "Current state after preflight stop is SAFE_IDLE",
            initial_state == "SAFE_IDLE" and initial_fault == "NONE",
            f"motion_state={initial_state}, fault={initial_fault}",
        )

        ws_telemetry = test.ws_request(ws, {"type": "get_telemetry"})
        test.check("WebSocket get_telemetry responds", isinstance(ws_telemetry, dict), f"response={compact(ws_telemetry)}")

        blocked = test.ws_request(
            ws,
            {"type": "set_param", "id": args.param_id, "value": args.param_value},
            "set_param_before_tuning",
        )
        blocked_status = response_kind(blocked)
        test.check(
            "set_param is blocked before PARAM_TUNING",
            blocked_status == "enter_tuning_required",
            f"status={blocked_status}, response={compact(blocked)}",
        )

        entered = test.ws_request(ws, {"type": "enter_tuning"})
        tune_state = vehicle_field(entered, "motion_state")
        tune_session = vehicle_field(entered, "debug_session")
        test.check("enter_tuning reaches PARAM_TUNING", tune_state == "PARAM_TUNING", f"motion_state={tune_state}")
        test.check("enter_tuning sets TUNING_ACTIVE", tune_session == "TUNING_ACTIVE", f"debug_session={tune_session}")

        updated = test.ws_request(ws, {"type": "set_param", "id": args.param_id, "value": args.param_value})
        updated_status = response_kind(updated)
        test.check("set_param works inside PARAM_TUNING", updated_status in {"ok", "param_updated"}, f"status={updated_status}, response={compact(updated)}")

        manual = test.ws_request(
            ws,
            {"type": "manual_motion", "motion": args.manual_motion, "duration_ms": args.manual_duration_ms},
        )
        manual_state = vehicle_field(manual, "motion_state")
        test.check("manual_motion enters MANUAL_TEST", manual_state == "MANUAL_TEST", f"motion_state={manual_state}")

        time.sleep((args.manual_duration_ms + 350) / 1000.0)
        after_manual = test.ws_request(ws, {"type": "get_telemetry"}, "get_telemetry_after_manual_timeout")
        after_manual_state = vehicle_field(after_manual, "motion_state")
        test.check("manual_motion timeout returns to PARAM_TUNING", after_manual_state == "PARAM_TUNING", f"motion_state={after_manual_state}")

        exited = test.ws_request(ws, {"type": "exit_tuning"})
        exit_state = vehicle_field(exited, "motion_state")
        test.check("exit_tuning returns SAFE_IDLE", exit_state == "SAFE_IDLE", f"motion_state={exit_state}")

        armed = test.ws_request(ws, {"type": "arm_auto"})
        armed_state = vehicle_field(armed, "motion_state")
        test.check("arm_auto reaches AUTO_ARMED", armed_state == "AUTO_ARMED", f"motion_state={armed_state}")

        if args.include_auto:
            started = test.ws_request(ws, {"type": "start_auto"})
            auto_state = vehicle_field(started, "motion_state")
            test.check("start_auto reaches AUTO_RUNNING or immediate FAULT", auto_state in {"AUTO_RUNNING", "FAULT"}, f"motion_state={auto_state}")
            time.sleep(args.auto_hold_ms / 1000.0)
        else:
            test.event("run", "skip_start_auto", note="safe default; pass --include-auto to test AUTO_RUNNING")

        stopped = test.ws_request(ws, {"type": "stop"})
        stop_state = vehicle_field(stopped, "motion_state")
        test.check("stop returns SAFE_IDLE", stop_state == "SAFE_IDLE", f"motion_state={stop_state}")

    except Exception as exc:
        test.event("run", "fatal", status="ERROR", error=repr(exc))
        test.check("test run completed without fatal exception", False, repr(exc))
    finally:
        if ws:
            try:
                test.ws_request(ws, {"type": "stop"}, "final_safety_stop")
            except Exception as exc:
                test.event("ws", "final_safety_stop", status="ERROR", error=repr(exc))
            ws.close()
            test.event("ws", "close", status="OK")
        test.write_summary()
        test.event("run", "summary", status="OK", summary=str(test.summary_path))

    failed = [item for item in test.results if not item.ok]
    print()
    print(f"Log folder: {test.log_dir}")
    print(f"Summary:    {test.summary_path}")
    print(f"Events:     {test.events_path}")
    print(f"Result:     {'PASS' if not failed else 'FAIL'} ({len(test.results) - len(failed)} pass, {len(failed)} fail)")
    return 0 if not failed else 1


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run an offline PC-side test sequence against the ESP32 line-car HTTP/WebSocket debug API.",
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"ESP32 SoftAP IP address, default: {DEFAULT_HOST}")
    parser.add_argument("--timeout", type=float, default=4.0, help="HTTP/WebSocket timeout in seconds, default: 4")
    parser.add_argument("--log-dir", default=DEFAULT_LOG_ROOT, help=f"Log root directory, default: {DEFAULT_LOG_ROOT}")
    parser.add_argument("--param-id", default=DEFAULT_PARAM_ID, help=f"Parameter id used for write-gate test, default: {DEFAULT_PARAM_ID}")
    parser.add_argument("--param-value", type=int, default=DEFAULT_PARAM_VALUE, help=f"Parameter value used for write-gate test, default: {DEFAULT_PARAM_VALUE}")
    parser.add_argument("--manual-motion", default="forward", choices=["forward", "backward", "left", "right", "stop"], help="Manual test motion, default: forward")
    parser.add_argument("--manual-duration-ms", type=int, default=500, help="Manual test duration in milliseconds, default: 500")
    parser.add_argument("--include-auto", action="store_true", help="Also send start_auto briefly; use only when the car is physically safe")
    parser.add_argument("--auto-hold-ms", type=int, default=300, help="How long to hold AUTO_RUNNING before stop when --include-auto is set")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
