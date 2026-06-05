#!/usr/bin/env python3
"""Upload a DCar-Liner firmware image to the ESP32 Wi-Fi OTA endpoint."""

from __future__ import annotations

import argparse
import datetime as _dt
import http.client
import json
from pathlib import Path
import sys
from typing import Optional


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Upload a built DCar-Liner .bin image to http://192.168.4.1/api/ota."
    )
    parser.add_argument(
        "firmware",
        type=Path,
        help="Firmware image, for example build-esp32c3/DCar-Liner.bin",
    )
    parser.add_argument("--host", default="192.168.4.1", help="ESP32 Wi-Fi host")
    parser.add_argument("--port", type=int, default=80, help="ESP32 HTTP port")
    parser.add_argument("--path", default="/api/ota", help="OTA upload path")
    parser.add_argument("--timeout", type=float, default=45.0, help="HTTP timeout seconds")
    parser.add_argument("--source-ip", default=None, help="Optional local source IP, for example 192.168.4.2")
    parser.add_argument("--no-log", action="store_true", help="Do not write logs/ota-upload-tests")
    return parser.parse_args()


def write_log(args: argparse.Namespace, status: int, reason: str, body: bytes) -> Optional[Path]:
    if args.no_log:
        return None

    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    log_dir = Path("logs") / "ota-upload-tests" / stamp
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / "response.txt").write_bytes(body)

    try:
        parsed = json.loads(body.decode("utf-8"))
        pretty_body = json.dumps(parsed, ensure_ascii=False, indent=2)
    except (UnicodeDecodeError, json.JSONDecodeError):
        pretty_body = body.decode("utf-8", errors="replace")

    summary = "\n".join(
        [
            "# DCar-Liner OTA Upload",
            "",
            f"- firmware: `{args.firmware}`",
            f"- bytes: `{args.firmware.stat().st_size}`",
            f"- endpoint: `http://{args.host}:{args.port}{args.path}`",
            f"- http_status: `{status} {reason}`",
            "",
            "```json",
            pretty_body,
            "```",
            "",
        ]
    )
    (log_dir / "summary.md").write_text(summary, encoding="utf-8")
    return log_dir


def main() -> int:
    args = parse_args()
    if not args.firmware.is_file():
        print(f"firmware not found: {args.firmware}", file=sys.stderr)
        return 2

    body_size = args.firmware.stat().st_size
    headers = {
        "Content-Type": "application/octet-stream",
        "Content-Length": str(body_size),
        "User-Agent": "DCar-Liner-OTA/1.0",
    }

    print(f"Uploading {args.firmware} ({body_size} bytes) to http://{args.host}:{args.port}{args.path}")
    source_address = (args.source_ip, 0) if args.source_ip else None
    conn = http.client.HTTPConnection(args.host, args.port, timeout=args.timeout, source_address=source_address)
    try:
        with args.firmware.open("rb") as firmware:
            conn.request("POST", args.path, body=firmware, headers=headers)
        response = conn.getresponse()
        response_body = response.read()
    finally:
        conn.close()

    log_dir = write_log(args, response.status, response.reason, response_body)
    print(f"HTTP {response.status} {response.reason}")
    print(response_body.decode("utf-8", errors="replace"))
    if log_dir is not None:
        print(f"log: {log_dir}")
    return 0 if 200 <= response.status < 300 else 1


if __name__ == "__main__":
    raise SystemExit(main())
