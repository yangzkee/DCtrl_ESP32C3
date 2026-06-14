# Tools

## ESP32 State Machine Tester

`state_machine_tester.py` is a computer-side, offline-safe test runner for the ESP32 line-car debug API. It uses only the Python standard library, so it still works after the computer disconnects from the internet and joins the ESP32 SoftAP.

Default Wi-Fi fallback target after the firmware receives BLE compact command `W1`:

- Wi-Fi SSID: `DCar-Liner-F47785` or the current `DCar-Liner-XXXXXX` shown by the firmware.
- Wi-Fi password: `DCar-Liner123`.
- ESP32 URL: `http://192.168.4.1`.
- WebSocket: `ws://192.168.4.1/ws`.

Run the safe default test:

```sh
python3 tools/state_machine_tester.py
```

The safe default first sends one preflight `stop`, then verifies HTTP reachability, WebSocket commands, tuning lock, manual test timeout, `AUTO_ARMED`, and `stop`. It does not send `start_auto`.

Run the optional auto-running test only when the car is lifted, restrained, or otherwise safe:

```sh
python3 tools/state_machine_tester.py --include-auto
```

Each run writes:

- `logs/state-machine-tests/<timestamp>/events.jsonl`
- `logs/state-machine-tests/<timestamp>/summary.md`

After reconnecting to the normal network, inspect the printed log folder or share the folder path for analysis.

## BLE Debug Tester

`ble_debug_tester.swift` is a Mac-side CoreBluetooth test runner for the BLE debug transport. It does not need a phone or internet connection.

Default target:

- BLE device name prefix: `DCtrl`.
- Runtime BLE name: `DCtrl`.
- Service UUID: `7b3a0001-8d4d-4b9a-b5c7-0f7c4c415243`.
- RX characteristic: `7b3a0002-8d4d-4b9a-b5c7-0f7c4c415243`.
- TX characteristic: `7b3a0003-8d4d-4b9a-b5c7-0f7c4c415243`.

Run:

```sh
swift tools/ble_debug_tester.swift --timeout 25
```

The tester scans, connects, writes `{"type":"get_telemetry"}`, reads direct or chunked TX responses, validates byte count and JSON validity, then writes:

- `logs/ble-debug-tests/<timestamp>/events.jsonl`
- `logs/ble-debug-tests/<timestamp>/summary.md`

Compact PID/gear frames for the WeChat path:

```sh
swift tools/ble_debug_tester.swift --timeout 25 --command G --append-newline --expect-prefix P
swift tools/ble_debug_tester.swift --timeout 25 --command S9000,0,0,2 --append-newline --expect-prefix OK
```

The compact `G` response is `P<kp>,<ki>,<kd>,<gear>`. The compact `S` command writes and saves the four values; invalid ranges return `E:RANGE`.

## BLE Remote Tester

`ble_remote_tester.swift` targets the independent DCtrl remote service and sends raw 21-byte DFLink `Motion_Velocity` frames through the transparent bridge. It is the Mac-side smoke tester for the same wire contract used by the WeChat joystick: `Vx/Vy` are encoded as m/s and `Vz` is encoded as rad/cmd after the business-level deg/cmd conversion.

```sh
swift tools/ble_remote_tester.swift --timeout 25 --motion zero
swift tools/ble_remote_tester.swift --timeout 25 --motion forward
swift tools/ble_remote_tester.swift --timeout 25 --motion strafe
swift tools/ble_remote_tester.swift --timeout 25 --motion yaw
```

For non-zero motions the tester repeats the active frame every `50 ms` for `500 ms`, then sends three zero-speed frames. The remote bridge success path is intentionally silent; the tester passes when all CoreBluetooth writes complete and no `E:*` remote error arrives. `E:BUSY`, `E:UART`, `E:LEN`, `E:COPY`, disconnect, or timeout fails the run.
