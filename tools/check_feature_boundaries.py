#!/usr/bin/env python3
"""Static boundary checks for DCtrl remote bridge and line-trace isolation."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def require_absent(text: str, tokens: list[str], context: str) -> None:
    for token in tokens:
        require(token not in text, f"{context} must not contain {token!r}")


def c_function_body(text: str, name: str) -> str:
    marker = f" {name}("
    start = -1
    brace = -1
    search_from = 0
    while True:
        start = text.find(marker, search_from)
        require(start >= 0, f"missing function {name}")
        paren_depth = 0
        signature_end = -1
        for index in range(start + len(marker) - 1, len(text)):
            if text[index] == "(":
                paren_depth += 1
            elif text[index] == ")":
                paren_depth -= 1
                if paren_depth == 0:
                    signature_end = index
                    break
        require(signature_end >= 0, f"unterminated signature for {name}")
        brace = text.find("{", signature_end)
        semicolon = text.find(";", signature_end)
        if brace >= 0 and (semicolon < 0 or brace < semicolon):
            break
        search_from = signature_end + 1

    require(brace >= 0, f"missing function body for {name}")
    depth = 0
    for index in range(brace, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[brace : index + 1]
    raise AssertionError(f"unterminated function {name}")


def main() -> None:
    ble = read("components/debug_server/debug_server_ble.c")
    app_main = read("main/app_main.c")
    vehicle_state_h = read("components/vehicle_state/include/vehicle_state.h")
    line_trace_controller = read("components/control/line_trace_controller.c")
    motion_policy = read("components/control/motion_policy.c")
    remote_tester = read("tools/ble_remote_tester.swift")

    require("chassis_uart_write_raw" in ble, "remote BLE writes must forward raw bytes to chassis UART")
    require("vehicle_state_enter_remote_bridge" in ble, "remote BLE writes must claim REMOTE_BRIDGE ownership")
    remote_write = c_function_body(ble, "write_remote_request")
    require('"OK\\n"' not in remote_write, "remote bridge success path must stay silent")
    require('"E:UART\\n"' in remote_write, "remote bridge must still notify UART errors")
    require("telemetry_record_remote_bridge_success" in remote_write, "remote bridge must record success telemetry")
    require("telemetry_record_remote_bridge_error" in remote_write, "remote bridge must record error telemetry")
    require("telemetry_end_remote_bridge_session" in ble, "remote bridge telemetry session must end on BLE disconnect")
    require(ble.count("s_rx_stream_len = 0;") >= 2, "BLE stream parser buffer must be cleared on connect and disconnect")
    require_absent(
        ble,
        [
            "REMOTE_V1",
            "REMOTE_FEED_PERIOD_MS",
            "REMOTE_WATCHDOG_MS",
            "remote_target",
            "remote_legacy",
            "remote_execute_motion",
            "remote_process_legacy_bytes",
            "dctrl_remote_ctl",
        ],
        "debug_server_ble.c",
    )

    require("CONTROLLER_TASK_STARTING" in line_trace_controller, "controller start must reserve STARTING state")
    require("s_controller_task_state != CONTROLLER_TASK_STOPPED" in line_trace_controller, "duplicate controller starts must be blocked")
    require_absent(
        remote_tester,
        [
            "0x44, 0x43",
            "13-byte remote v1",
            "DC 01 01",
        ],
        "ble_remote_tester.swift",
    )
    require("Motion_Velocity" in remote_tester, "remote tester must send DFLink Motion_Velocity")

    require("VEHICLE_MOTION_REMOTE_BRIDGE" in vehicle_state_h, "vehicle state must expose REMOTE_BRIDGE")
    require(
        "line_trace_controller_start" not in app_main,
        "app_main must not start the line-trace controller at boot",
    )
    require("set_noop_intent(intent, LINE_TRACE_PHASE_STOPPED)" in motion_policy, "disabled line-trace mode must no-op")
    require("set_noop_intent(intent, LINE_TRACE_PHASE_ACQUIRE_LINE)" in motion_policy, "auto-armed line-trace mode must no-op")

    print("Feature boundary checks passed")


if __name__ == "__main__":
    main()
