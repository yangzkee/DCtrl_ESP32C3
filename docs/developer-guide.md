# Developer Guide

## Current Firmware Shape

This project is an ESP-IDF firmware skeleton for an ESP32-S3 line-following car, designed so the same module boundaries can later move to ESP32-C3.

The firmware has these core modules:

- `board`: target profile, UART ports, pins, baud rates, and buffer sizes.
- `chassis`: serial motion command output for the car chassis.
- `line_sensor`: serial input parser for the eight-channel infrared line sensor.
- `vehicle_state`: main motion state, wireless tuning session state, fault reason, and manual-test timeout.
- `control`: motion-control pipeline split into contracts, input normalization, line interpretation, motion intent, and execution output.
- `param_store`: live tunable parameter registry plus NVS persistence.
- `debug_protocol`: JSON request/response protocol for fallback Wi-Fi/BLE diagnostics.
- `debug_server`: on-demand Wi-Fi SoftAP, HTTP API, WebSocket endpoint, HTTP OTA upload, and BLE GATT debug transport.
- `telemetry`: latest line sensor, line UART diagnostics, motion command, PID output, and parameter version snapshot.

## Feature Mode Ownership

The chassis UART is a shared physical resource. Current active modes are `SAFE_IDLE`, `REMOTE_BRIDGE`, `PARAM_TUNING`, `MANUAL_TEST`, `AUTO_ARMED`, `AUTO_RUNNING`, `OTA_UPDATE`, and `FAULT`. Only the active mode may write motion bytes to the chassis. `REMOTE_BRIDGE` is intentionally thin: it forwards DCtrl BLE writes to `chassis_uart_write_raw()` and does not parse velocity, cache targets, convert yaw units, run a local feed loop, or send success ACKs on the high-rate path. Remote bridge success is silent; errors still notify as `E:*`. The bridge only updates observation telemetry counters for field diagnosis; those counters must not influence motion behavior. Remote telemetry sessions are scoped to one continuous write run: BLE disconnect ends a session, and a gap above 3000 ms before the next write starts a new session.

The shared remote-motion coordinate truth is `X > 0` forward, `Y > 0` left translation, and `Z/Yaw > 0` left/counterclockwise rotation. DHelper maps joystick, button, and motion-control inputs into that coordinate frame before encoding DFLink; firmware bridge code must not add another sign layer.

`line_trace_controller` must not be started from `app_main`. It starts only when `manual_motion`, `arm_auto`, or `start_auto` enters a line-control mode, and exits when the state returns to tuning, idle, remote bridge, OTA, or fault. The start path must reserve a `STARTING` state before `xTaskCreate()` so concurrent BLE/Wi-Fi requests cannot create duplicate controller tasks. Disabled and idle line-trace states must not send periodic `chassis_uart_stop()`; only explicit stop, disconnect, manual-test timeout, OTA entry, or fault handling may send a one-shot zero-speed frame.

Future features must follow `docs/vehicle-feature-mode-maintenance.md` before touching chassis UART ownership. A new feature must define its goal, inputs, outputs, independent test, failure rules, and integration boundary before it is wired into shared motion output.

## Current Line Map Scope

The current project map is intentionally simple: a single black-tape irregular circle on the floor. The irregularity is the natural placement error from hand-taping, not a designed maze. The vehicle only needs to stay on that loop and continue following the line.

Out of scope for the current motion code:

- Maze solving.
- Branch or junction classification.
- Route memory.
- Left/right turn choice at intersections.
- Search behavior after line loss.

These features should be added later as new policy states or a separate route planner. They should not be mixed into the current circular-line policy.

## Motion Control Module Contract

`components/control/line_trace_controller.c` owns hardware integration:

- Reads live parameters from `param_store`.
- Reads the line sensor through `line_sensor_uart`.
- Maps `vehicle_state` into a line-following run request.
- Sends the final command to `chassis_uart`.
- Publishes telemetry.

The motion-control code is now split into five layers. The first refactor was intentionally behavior-preserving; current strategy changes should still preserve the stable line-following feel unless this document explicitly names the changed behavior.

- `Layer 0: motion_contracts`: the single source of truth for motion constants and rules, including line active-bit quality, offset curve, speed-gear mapping, timed recovery sweep behavior, speed-retention ratio, and common clamp helpers.
- `Layer 1: motion_inputs`: converts vehicle/system state and raw sensor samples into a normalized policy input. It applies mechanical operations such as run-mode mapping and optional sensor-bit inversion, but it does not decide what the line shape means.
- `Layer 2: line_interpreter`: interprets the normalized line sample into geometric facts, including active sensor count, line quality, lost-line flag, curved offset error, and a simple current-frame pattern state.
- `Layer 3: motion_policy`: turns run mode plus interpreted geometry into a motion intent. This is where the existing phase transitions, PID calculation, adaptive speed reduction, and lost-line recovery state machine live.
- `Layer 4: motion_executor`: turns the motion intent into the existing `line_trace_policy_output_t` contract. The output carries a shared `control_plan_t`; the controller remains the only layer that actually applies vehicle-state actions or sends commands to `chassis_uart`.

`control_plan_t` is the shared control-command contract between strategy and execution. It separates chassis actions (`NONE`, `SEND_MOTION`, `STOP`) from vehicle-state actions (`NONE`, `STOP_TO_IDLE`, `ENTER_FAULT`). Strategy code must not call `vehicle_state_stop()` or `chassis_uart_send_motion()` directly; it only fills this plan. The controller is the execution boundary that interprets the plan.

`components/control/line_trace_policy.c` remains the stable public wrapper around this pipeline. It has no UART or NVS access. Its input is the run request, sensor result, manual command, and PID/speed config. Its output is the line phase, shared control plan, and controller telemetry.

Current line phases:

- `STOPPED`: boot, idle, tuning, fault, or any state where motion is not allowed.
- `MANUAL_TEST`: short tuning-session manual command.
- `ACQUIRE_LINE`: auto is armed and the vehicle remains stopped while the operator starts explicitly.
- `TRACK_LINE`: normal circular-line following with PID steering.
- `LINE_LOST`: no active line bits while running; the policy keeps sending DFLink `Motion_Velocity` with `linear_mm_s=0` and an expanding timed left/right search. The fixed sequence is left `0.8 s`, right `1.6 s`, left `3.2 s`, right `6.4 s`, left `12.8 s`, right `25.6 s`, then right `30.0 s`. If the line still does not return after the final segment, the plan requests `STOP_TO_IDLE`.
- `SENSOR_FAULT`: sensor timeout or parse/read error; while running, the policy keeps sending a safe zero-linear search command instead of faulting immediately.

Lost-line recovery is intentionally split into two outputs. The motion output is the safe recovery command through `control_plan_t`: `SEND_MOTION` with `linear_mm_s=0` and `angular_mdeg_s` set to the current gear's normal turn limit. Gear 1/2/3 use `8000/10000/10000 mdeg/cmd`. The diagnostic output records the first geometric relation inferred when the line comes back: `UNDERSTEER` means the successful recovery direction matched the last valid tracking turn, while `OSCILLATION_SKEW` means it came back in the opposite direction. Telemetry exposes `segment_index`, `elapsed_ms`, and `target_ms` for the timed sweep. This relation is telemetry only for now; it does not yet change PID gains, speed, or future steering decisions.

To modify the line-following behavior, choose the layer by intent:

- Change constants, speed-gear rules, black/white semantics, line-quality thresholds, or protocol-level motion limits in `motion_contracts`.
- Change how system state, sensor samples, future ODOM, or future vehicle pose are cached and normalized in `motion_inputs`.
- Change line geometry, current-frame pattern classification, or future N-frame trend classification in `line_interpreter`.
- Change line-following phases, PID behavior, adaptive speed policy, lost-line search intent, or vehicle-state action requests in `motion_policy`.
- Change final output shaping that is independent of strategy, such as future ramping or slew-rate limiting, in `motion_executor`.

Keep UART, BLE, NVS, and board-pin details out of `line_interpreter`, `motion_policy`, and `motion_executor`.

Motion pipeline module definitions:

- `motion_contracts`
  - `Goal`: keep canonical motion rules in one place so future changes do not scatter through controller code.
  - `Inputs`: gear number, line bits, offset, active sensor count, generic numeric values.
  - `Outputs`: speed profile, line quality, curved error, adaptive speed, and clamped values.
  - `Independent Test`: C3/S3 builds plus deterministic checks of gear mapping, line quality, and offset curve.
  - `Failure Rules`: no hardware access, no persistent state, no side effects.
  - `Next Integration Boundary`: future protocol and physical-unit constants should enter here first.
- `motion_inputs`
  - `Goal`: normalize everything that can affect chassis motion into a stable input snapshot.
  - `Inputs`: `vehicle_state_snapshot_t`, raw `line_sensor_sample_t`, sensor polarity setting, future ODOM/pose/strategy-target fields.
  - `Outputs`: `line_trace_policy_input_t`.
  - `Independent Test`: fixed vehicle states must map to the expected run modes; inverted sensor bits must recalculate offset.
  - `Failure Rules`: no line-meaning decisions and no PID math.
  - `Next Integration Boundary`: future ODOM and global pose should be added here as normalized cached fields.
- `line_interpreter`
  - `Goal`: estimate the vehicle-line geometric relationship from normalized inputs.
  - `Inputs`: normalized run/sensor input.
  - `Outputs`: active sensor count, quality, lost-line flag, curved error, and current-frame line pattern state.
  - `Independent Test`: fixed `bits/offset` samples must produce stable quality, lost-line, and left/right/center/lost pattern outputs.
  - `Failure Rules`: no chassis command generation.
  - `Next Integration Boundary`: future N-frame pattern recognition belongs here, not in PID or UART drivers.
- `motion_policy`
  - `Goal`: decide what motion the vehicle intends to perform.
  - `Inputs`: run mode, interpreted line geometry, PID parameters, speed profile, runtime memory.
  - `Outputs`: `motion_intent_t` with phase, `control_plan_t`, telemetry fields, and optional vehicle-state action.
  - `Independent Test`: fixed input sequences must preserve the stable release outputs for stop, armed, track, line-lost, sensor-fault, and manual-test cases.
  - `Failure Rules`: no hardware send and no parameter storage.
  - `Next Integration Boundary`: future route strategy can feed this layer as a higher-level target.
- `motion_executor`
  - `Goal`: convert a motion intent into the controller output contract.
  - `Inputs`: `motion_intent_t`.
  - `Outputs`: `line_trace_policy_output_t` containing `control_plan_t`.
  - `Independent Test`: field-by-field copy should preserve the control plan and telemetry values.
  - `Failure Rules`: no direct UART send; `line_trace_controller` remains the only hardware-send owner.
  - `Next Integration Boundary`: future smoothing that is independent of policy can be added here.

Line-following quality is mainly affected by:

- Whether the line sensor black output is `0` as expected; `sensor.invert_bits` is only for opposite-output modules.
- Sensor height, alignment, and whether the black tape is wide enough for the 8-channel array.
- Loop period and sensor timeout.
- Speed gear, which maps both forward speed and turn limit.
- PID gains and integral limit.
- Turn saturation from `max_turn_mdeg_s`.
- Lost-line policy; current behavior is zero-linear safe Z search, while BLE
  disconnect remains the immediate stop condition.

## Required Peripherals

- USB-C serial connection for flashing, logs, and recovery.
- USB-C is still the first-install and recovery path. OTA can update the app image only after an OTA-capable firmware and partition table have already been flashed once over USB.
- Chassis serial link: ESP TX/RX to the car chassis serial command interface.
- Eight-channel infrared line sensor serial link: ESP RX/TX to the VGTI/UART interface.
- Common ground between ESP32 board, chassis controller, and sensor module.
- 3.3 V logic compatibility, or level shifting if either peripheral uses 5 V UART logic.
- Stable power budget for ESP32-S3, sensor, and chassis controller. Motor power must not back-feed the ESP module.
- BLE Low Energy debug channel. This is the default field tuning path and stays compatible with ESP32-C3.
- On-demand Wi-Fi SoftAP debug channel. This remains a fallback diagnostic path and is not powered by default.

## Default ESP32-S3 Profile

- Chassis UART: `UART1`, TX `GPIO17`, RX `GPIO18`, `115200 8N1`.
- Line sensor UART: `UART2`, TX `GPIO15`, RX `GPIO16`, `115200 8N1`.

These pins are placeholders until the exact ESP32-S3 development board pinout and wiring are confirmed. Avoid strapping pins and pins already used by USB, flash, PSRAM, or boot mode.

## ESP32-C3 Migration Rule

Business code must not depend on raw GPIO numbers or UART port numbers. All target-specific choices stay in `components/board/board_profile.c`.

The C3 profile currently reserves:

- Chassis UART: `UART1`, TX `GPIO4`, RX `GPIO5`.
- Line sensor UART: `UART0`, TX `GPIO2`, RX `GPIO3`.
- Console: primary output on USB Serial/JTAG, so `UART0` is reserved for the line sensor driver.

Wire UARTs crossed: ESP TX goes to the peripheral RX, and ESP RX goes to the peripheral TX.

- Chassis wiring on the current C3 profile: `GPIO4` -> chassis RX, `GPIO5` <- chassis TX, plus common GND.
- Line sensor wiring on the current C3 profile: `GPIO2` -> line sensor RX, `GPIO3` <- line sensor TX, plus common GND.

The first C3 board smoke test on `/dev/cu.usbmodem11201` confirmed the firmware boots as `esp32c3-portable-dual-uart-profile`. The C3 line-sensor wiring was later moved off `GPIO20/GPIO21` after field telemetry showed the line UART reading back `$0,0,1#` instead of module `$D,...#` frames. Keep future C3 pin changes inside `components/board/board_profile.c`.

## ESP-IDF Environment

ESP-IDF is installed locally at `/Users/schwartz/esp/esp-idf`, with toolchains and the Python environment under `/Users/schwartz/.espressif`.

Installed and verified version:

```sh
ESP-IDF v5.5.4
```

The installation includes both `esp32s3` and `esp32c3` target toolchains. The project helper below loads `export.sh`, enters the project root, and forwards arguments to `idf.py`:

```sh
./scripts/idf.sh --version
```

## Build Commands

```sh
./scripts/idf.sh set-target esp32s3
./scripts/idf.sh build
./scripts/idf.sh -p /dev/cu.usbserial-1120 flash monitor
```

Current ESP32-S3 build verification passes and generates `build/DCar-Liner.bin`.

The firmware now uses a custom 4 MB OTA partition table, `partitions_ota_4mb.csv`, for both S3 and C3 builds. Each OTA app slot is `0x1D0000` bytes. This replaces the earlier large single-app layout so wireless firmware updates can switch between `ota_0` and `ota_1`.

When a board is still running the older single-app firmware, flash once over USB so the bootloader sees the new partition table:

```sh
./scripts/idf.sh -B build-esp32c3 -DSDKCONFIG=sdkconfig.esp32c3 -p /dev/cu.usbmodem11201 flash
```

## Wireless Debug API

Default wireless mode:

- BLE starts by default as the low-power tuning/debug entry.
- DCtrl remote bridge starts as a transparent BLE UART service. The mini program sends 21-byte DFLink frames; firmware forwards BLE write bytes to the chassis UART unchanged. For remote rotation, the mini program owns business-level `Vz deg/cmd` and encodes it to DFLink wire-level `rad/cmd` before BLE write.
- Wi-Fi SoftAP is off by default to reduce power draw.
- The compact BLE command `W1` starts the fallback Wi-Fi SoftAP and HTTP/WebSocket server.

Fallback SoftAP after `W1`:

- SSID: `DCar-Liner-XXXXXX`, where the suffix comes from the ESP32 Wi-Fi MAC.
- Password: `DCar-Liner123`.
- URL: `http://192.168.4.1/`.
- WebSocket: `ws://192.168.4.1/ws`.

HTTP endpoints:

- `GET /api/health`: small health check for HTTP reachability, free heap, and HTTP task stack headroom.
- `GET /api/schema`: parameter definitions, ranges, defaults, current values.
- `GET /api/scheme`: compatibility alias for `/api/schema`.
- `GET /api/params`: current parameter values.
- `GET /api/telemetry`: current line sensor state, line UART diagnostics, motion command, controller state, and parameter version.
- `GET /api/ota/status`: running, boot, and next OTA partition metadata.
- `POST /api/ota`: binary firmware upload endpoint. Send the raw `.bin` image body with `Content-Type: application/octet-stream`.

The root debug page at `http://192.168.4.1/` is only a fallback diagnostic page with static links to the API endpoints. The normal tuning UI is the WeChat mini program over compact BLE frames; Wi-Fi remains a maintenance and fallback diagnostics channel.

The HTTP debug server follows the stability pattern from ESP-IDF examples and mature ESP32 Wi-Fi configuration projects:

- HTTP server stack is explicitly configured to `8192` bytes.
- Large JSON responses use heap buffers, not the HTTP handler stack.
- Normal HTTP API responses add no-cache, CORS, and `Connection: close` headers.
- The server keeps LRU socket purging enabled, so stale browser connections can be released.
- `/api/health` should be tested first; if it works repeatedly but `/api/schema` fails, inspect response size or heap headroom.

BLE debug transport:

- Stack: NimBLE BLE GATT, not Classic Bluetooth SPP. This keeps the firmware compatible with ESP32-C3, which supports BLE but not Classic Bluetooth.
- Device name prefix: `DCtrl`.
- Default runtime device name: `DCtrl`. BLE rename commands are disabled; `N=*` only clears old stored names and confirms the fixed name.
- Service UUID: `7b3a0001-8d4d-4b9a-b5c7-0f7c4c415243`.
- RX characteristic UUID: `7b3a0002-8d4d-4b9a-b5c7-0f7c4c415243`, write compact frames or JSON requests here.
- TX characteristic UUID: `7b3a0003-8d4d-4b9a-b5c7-0f7c4c415243`, read compact frames or JSON responses here.
- TX notifications are optional. When enabled, the firmware sends a small `ble_response_ready` notice after a request is handled; the client should then read TX.
- Short TX reads return the JSON response directly.
- Long TX reads return one or more chunk envelopes. Each envelope has `type:"ble_chunk"`, `seq`, `offset`, `total`, `done`, and `data`. The client must append `data` until `done:true`, then parse the assembled JSON.
- The RX stream buffer is scoped to one BLE connection. Firmware clears any partial compact/JSON request on connect and disconnect so an unfinished write cannot pollute the next request or next client session.

Compact BLE frames for the DHelper mini program `line-tuning` feature:

- `G\n`: read the four phone-facing params.
- Response: `P<kp>,<ki>,<kd>,<gear>\n`, for example `P9000,0,0,2`.
- `S<kp>,<ki>,<kd>,<gear>\n`: write and save the four params.
- `N\n`: read the fixed BLE advertising name.
- Response: `NDCtrl\n`.
- `N=<name>\n`: disabled compatibility command; returns `E:FIXED\n`.
- `N=*\n`: clear any old custom-name NVS keys and restore/confirm the fixed `DCtrl` name.
- Successful response: `OK\n`.
- Error responses: `E:PARSE\n`, `E:RANGE\n`, `E:FIXED\n`, `E:FAULT\n`, `E:BUSY\n`, or `E:SAVE\n`.
- `W1\n`: start the fallback Wi-Fi debug server.

Do not reintroduce editable BLE names. The mini program must scan/connect by fixed `DCtrl` name or the DCtrl service UUID.

The DHelper mini program line-tracing controls use BLE JSON requests on the same RX/TX
characteristics:

- `一键自检`: sends `get_telemetry`, sends `clear_fault` only when telemetry
  reports `FAULT`, then sends `arm_auto`. The start button remains disabled
  unless the final observed state is `AUTO_ARMED`.
- `开始循线`: sends `start_auto` and expects `AUTO_RUNNING`.
- `停止`: sends `stop`.

The phone-facing params are intentionally limited:

- `pid.kp`: `7000-11000`
- `pid.ki`: `0-300`
- `pid.kd`: `0-3000`
- `speed.gear`: `1-3`

`speed.gear` maps both the protocol line speed and the per-command Z angle-increment limit:

- Gear 1: maximum protocol line speed `250 mm/s` (`Vx=0.25`), Z limit `8000 mdeg/cmd`
- Gear 2: maximum protocol line speed `600 mm/s` (`Vx=0.6`), Z limit `10000 mdeg/cmd`
- Gear 3: maximum protocol line speed `1000 mm/s` (`Vx=1.0`), Z limit `10000 mdeg/cmd`

During auto line tracing, these are maximum line speeds. When the active line
position drifts away from the center pair `x4/x5`, the policy reduces linear
speed with a slow-then-fast quadratic curve. At the outermost offsets it keeps
roughly 45% of the selected gear speed. The PID steering input uses the same
slow-then-fast idea: the raw offset is normalized and squared before entering
the PID calculation, so small offsets produce gentle turn increments while large
offsets ramp up much more aggressively.

Example JSON BLE request flow:

1. Scan for BLE devices and connect to the selected device.
2. Write `{"type":"get_telemetry"}` to RX.
3. Read TX. If the response is `ble_chunk`, keep reading and append `data` until `done:true`.
4. Parse the final JSON response.

Computer-side BLE validation:

```sh
swift tools/ble_debug_tester.swift --timeout 25
```

The tester uses macOS CoreBluetooth directly. By default it scans for names starting with `DCtrl`, connects, writes the selected command, follows chunked TX responses, accepts JSON or compact BLE responses, and writes logs to `logs/ble-debug-tests/<timestamp>/`. Use `--name` only when you need to target one exact BLE name.

Compact frame examples:

```sh
swift tools/ble_debug_tester.swift --timeout 25 --command G --append-newline --expect-prefix P
swift tools/ble_debug_tester.swift --timeout 25 --command S9000,0,0,2 --append-newline --expect-prefix OK
```

WebSocket messages:

```json
{"type":"get_schema"}
{"type":"get_params"}
{"type":"get_telemetry"}
{"type":"enter_tuning"}
{"type":"set_param","id":"pid.kp","value":7500}
{"type":"save_params"}
{"type":"reset_defaults"}
{"type":"exit_tuning"}
{"type":"arm_auto"}
{"type":"start_auto"}
{"type":"stop"}
{"type":"clear_fault"}
{"type":"manual_motion","motion":"forward","duration_ms":500}
```

Read-only debug messages are allowed while the vehicle is idle or running. Mutating parameter messages use the stop-to-tune policy: `set_param`, `save_params`, and `reset_defaults` are accepted only after `enter_tuning` has moved the vehicle into `PARAM_TUNING`. `save_params` writes the current values to NVS. Do not save on every slider movement.

BLE compact `S<kp>,<ki>,<kd>,<gear>` also enters tuning briefly, validates the four phone-facing values, saves them to NVS, exits tuning, and returns `OK` or an error. It does not use OTA partitions and does not rewrite firmware.

Vehicle motion states:

- `BOOT_INIT`: startup and peripheral initialization.
- `SAFE_IDLE`: default safe state; the controller sends stop commands.
- `PARAM_TUNING`: wireless tuning session is active; the chassis remains stopped.
- `MANUAL_TEST`: short, low-speed manual chassis test from the tuning session.
- `AUTO_ARMED`: auto line tracing is prepared but still stopped.
- `AUTO_RUNNING`: closed-loop line tracing is allowed to send motion commands.
- `OTA_UPDATE`: maintenance update is writing a new firmware image; chassis is stopped, tuning writes are rejected, and the controller pauses line-sensor reads.
- `FAULT`: chassis send failure, OTA failure, or invalid safety state; clear explicitly with `clear_fault`. Normal line loss is handled by the search phase, not by entering `FAULT`.

`manual_motion` accepts either `motion` as `forward`, `backward`, `left`, `right`, or `stop`, or numeric `linear_mm_s` and `angular_mdeg_s`. Manual commands are limited to `1500 mm/s`, `10000 mdeg/cmd`, and `50-3000 ms`; the linear value is sent as the DFLink protocol speed value, and the Z value is a per-command angle increment. Protocol `Vx=0.5` is a measured comfortable manual-test speed, while automatic line tracing currently tops out at `Vx=1.0`.

## Wireless Firmware OTA

OTA is a maintenance operation, not the phone-side tuning path. It uses the Wi-Fi fallback server and the 4 MB OTA partition table. It is compatible with the vehicle state model because the upload handler forces `OTA_UPDATE`, sends a chassis stop, rejects tuning writes, and restarts only after the new image has been fully written and selected as the next boot partition.

Module definition:

- `Goal`: update the ESP32 app image without a USB cable after the OTA-capable firmware is installed once.
- `Inputs`: raw `DCar-Liner.bin` over `POST /api/ota` after the operator starts Wi-Fi with BLE `W1` and connects the computer to the ESP32 SoftAP.
- `Outputs`: the image is written to the inactive OTA slot, `esp_ota_set_boot_partition()` selects it, and the board reboots after a short response window.
- `Independent Test`: `GET /api/ota/status`, then upload `build-esp32c3/DCar-Liner.bin` with `tools/ota_upload.py`, then reconnect and confirm BLE/HTTP still reports `DCar-Liner`.
- `Failure Rules`: oversized images are rejected before writing; invalid images or receive/write failures enter `FAULT` with `OTA_FAILED`; USB flashing remains the recovery path.
- `Next Integration Boundary`: add an operator UI button only after the manual script path is stable.

Computer-side upload command after the computer is connected to `DCar-Liner-XXXXXX` Wi-Fi:

```sh
python3 tools/ota_upload.py build-esp32c3/DCar-Liner.bin
```

If macOS or a VPN routes `192.168.4.1` through another interface, bind the upload to the Wi-Fi source IP assigned by the ESP32 SoftAP:

```sh
python3 tools/ota_upload.py --source-ip 192.168.4.2 build-esp32c3/DCar-Liner.bin
```

For ESP32-S3, use:

```sh
python3 tools/ota_upload.py build/DCar-Liner.bin
```

Each upload writes `logs/ota-upload-tests/<timestamp>/summary.md` unless `--no-log` is passed.

## Computer-Side State Machine Test Tool

The project includes an offline-safe PC test runner at `tools/state_machine_tester.py`. It uses only the Python standard library, so it can run after the computer switches from normal Wi-Fi to the ESP32 SoftAP and loses internet access.

Safe default run:

```sh
python3 tools/state_machine_tester.py
```

Default target:

- Host: `192.168.4.1`.
- HTTP: `/api/health`, `/api/telemetry`, `/api/params`, `/api/schema`.
- WebSocket: `/ws`.
- Log root: `logs/state-machine-tests/`.

The safe default sends one preflight `stop`, then verifies HTTP reachability, WebSocket reachability, the stop-to-tune write gate, `enter_tuning`, one low-speed `manual_motion`, automatic return from `MANUAL_TEST` to `PARAM_TUNING`, `exit_tuning`, `arm_auto`, and `stop`. It deliberately skips `start_auto`, so it should not start closed-loop driving.

To include the `AUTO_RUNNING` transition, run only when the car is lifted, restrained, or otherwise physically safe:

```sh
python3 tools/state_machine_tester.py --include-auto
```

Each run creates:

- `logs/state-machine-tests/<timestamp>/events.jsonl`: every HTTP/WebSocket request, response, check, error, and safety stop.
- `logs/state-machine-tests/<timestamp>/summary.md`: pass/fail summary for quick review.

`GET /api/telemetry` includes:

- `vehicle.motion_state`
- `vehicle.debug_session`
- `vehicle.fault`
- `line.active_count`
- `line.quality`
- `motion.linear_mm_s`
- `motion.angular_mdeg_s`
- `remote_bridge.success`
- `remote_bridge.errors`
- `remote_bridge.session_id`
- `remote_bridge.session_success`
- `remote_bridge.last_gap_ms`
- `remote_bridge.max_gap_ms`
- `remote_bridge.last_error_code`
- `controller.line_phase`
- `controller.lost_line`
- `controller.pid_output_mdeg_s`

Current tunable parameters:

- `pid.kp`, `pid.ki`, `pid.kd`, `pid.integral_limit`
- `speed.gear`
- `control.base_speed_mm_s`
- `control.max_turn_mdeg_s`
- `control.loop_period_ms`
- `sensor.timeout_ms`
- `sensor.invert_bits`

For migration smoke testing, use a separate C3 build directory and SDK config so the active ESP32-S3 config stays intact:

```sh
./scripts/idf.sh -B build-esp32c3 -DSDKCONFIG=sdkconfig.esp32c3 set-target esp32c3 build
```

Current ESP32-C3 smoke build verification passes and generates `build-esp32c3/DCar-Liner.bin`.

## Preserved Protocol References

- `references/protocols/dflink-chassis-protocol-20260528.md`: copied source file for DFLink chassis UART.
- `references/protocols/line-sensor-8ch-protocol-from-screenshot.md`: extracted line sensor protocol from the user-provided screenshot.
- `references/protocols/interface-selection.md`: current interface decision.
- `references/wifi-debug-research-20260603.md`: 30-project Wi-Fi debug/config research and adopted stability pattern.

## Chassis Protocol

The chassis driver sends DFLink `Motion_Velocity` for continuous line-following speed commands:

- Frame: `0xDF target source A B LEN payload 0xFD sumL sumH`.
- Target id: `0x01`.
- Source id: `0x97`.
- A=`0x02`, B=`0x62`, LEN=`12`.
- Payload: `Vx`, `Vy`, `Vz` as DFLink F32 fixed-point values.
- `linear_mm_s` is the DFLink protocol speed command expressed as mm/s, not a
  guaranteed real-world speed. `500 mm/s` sends `Vx=0.5` in the DFLink payload.
- The current field-calibration estimate is `protocol speed : real speed = 10:1`.
  This ratio is used to interpret measured chassis speed later, not to scale the
  value before sending it to the chassis.
- `Vy` is fixed at `0`.
- `angular_mdeg_s` is now treated as a per-command Z angle increment in millidegrees and maps to DFLink `Vz` as radians per command. The chassis must receive commands continuously to keep moving.

For DCtrl remote control, the firmware does not call this conversion path. The mini program is the source of truth for DFLink remote frames, including `Vz deg/cmd` to wire `rad/cmd` conversion, and the ESP32-C3 remote service only forwards raw bytes. This preserves the old Bluetooth-serial behavior and prevents phone-side and firmware-side motion strategies from fighting each other.

The official firmware also includes probe-style chassis diagnostics, so the
temporary standalone probe firmware is no longer part of the maintained path.
On ESP32-C3 the current official chassis profile is `UART1 TX GPIO4 / RX GPIO5`
at `115200` baud for the user-selected 115200 chassis port. The BLE compact
diagnostic commands are:

- `C?`: report the official firmware chassis UART pins and baud rate.
- `Cready`: send the DFLink ready/access query and parse any response frames.
- `Cversion`: send the DFLink version query and parse any response frames.
- `Cparams`: send the DFLink parameter query and parse any response frames.
- `Crepeat`: send ready, version, and params queries in sequence.
- `Codom`: request odometry at 10 Hz and listen briefly for response frames.
- `Cxyz`: send one low-speed `Motion_Velocity` test frame with `Vx=0.05 m/s`,
  `Vy=0`, and `Vz=0.10 rad/cmd`; this mainly verifies the XYZ speed command path
  and may not produce a response.
- `Cxyzrx`: send the same low-speed XYZ frame, then listen briefly for chassis
  return bytes on the official RX pin.
- `Cxyz500`: send a protocol line-speed command of `linear_mm_s=500`,
  `angular_mdeg_s=0`; this emits DFLink `Vx=0.5`, `Vy=0`, `Vz=0`.
- `Cxyz500rx`: send the same `Vx=0.5` line-speed command, then listen briefly
  for chassis return bytes.
- `Cxyz0`: send one zero `Motion_Velocity` frame through the same XYZ path.
- `Cstop0`: send the official zero-velocity stop frame.

Each compact response starts with `C:` and reports `txpin`, `rxpin`, `baud`,
transmitted/received bytes, parsed frame count, checksum-ok count,
checksum-bad count, the last `A/B/LEN`, and `err`. The same diagnostics are
available through the JSON debug protocol:

```json
{"type":"chassis_diag","command":"repeat"}
```

Supported JSON command values match the compact suffixes: `status`, `ready`,
`version`, `params`, `repeat`, `odom`, `xyz`, `xyzrx`, `xyz500`, `xyz500rx`,
`xyz0`, and `stop0`.

## Line Sensor Protocol

The line sensor driver uses USART digital mode:

```text
$0,0,1#
```

The digital response is:

```text
$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
```

The current hardware reports black line as `0` and white background as `1`, so every zero channel value becomes one active line bit. If a future sensor module reports the opposite polarity, tune `sensor.invert_bits` through the wireless debug API.

`GET /api/telemetry` also includes a `line_uart` diagnostic object for bring-up:

- `ready`: whether the UART driver initialized.
- `requests`: how many digital-mode request commands have been sent.
- `frames`: how many complete `#`-terminated frames have been received.
- `digital_frames`: how many received frames matched the `$D,x1:...#` digital protocol.
- `invalid_frames`: how many complete frames failed the digital parser.
- `timeouts`: how many read attempts timed out without a complete frame.
- `last_error`: latest UART write/read/parser error code.
- `last_frame`: latest raw frame, capped for diagnostics.

If `requests` increases but `frames` stays at `0`, first check sensor TX/RX direction, common ground, power, and baud rate. If `frames` increases but `digital_frames` stays at `0`, inspect `last_frame` and update the parser or request command to match the module's actual output.

## Protocol TODOs

- Confirm the line sensor baud rate.
- Confirm whether digital value `1` means black line or white background on the physical module.
- Tune base speed and turn gain after physical testing.
