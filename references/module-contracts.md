# ESP32 Line Trace Module Contracts

## Board Profile

- Goal: Keep target-chip differences in one place.
- Inputs: ESP-IDF target selection, chosen board wiring, UART role requirements.
- Outputs: `board_profile_t` containing UART port, TX/RX pins, baud rate, and buffer sizes.
- Independent Test: Build once with `idf.py set-target esp32s3`, then with `idf.py set-target esp32c3`.
- Current C3 Wiring: chassis `UART1 TX GPIO4 / RX GPIO5`; line sensor `UART0 TX GPIO2 / RX GPIO3`; C3 console uses USB Serial/JTAG so `UART0` is dedicated to line sensing.
- Failure Rules: If a C3 board cannot expose two usable UART paths, do not edit chassis or sensor code. Change only the C3 board profile or add an external UART bridge profile.
- Next Integration Boundary: Chassis and line sensor drivers consume only `board_uart_config_t`.

## Chassis UART

- Goal: Send normalized motion commands to a serial-controlled car chassis.
- Inputs: `chassis_motion_cmd_t` with protocol line speed in mm/s and per-command Z angle increment in mdeg/cmd.
- Outputs: One DFLink `Motion_Velocity` frame per command. Linear commands are protocol values, so `linear_mm_s=500` emits payload `Vx=0.5`. The current field estimate is `protocol:real = 10:1` and is not applied before sending.
- Independent Test: Connect a USB-UART adapter or logic analyzer and verify `DF 01 97 02 62 0C ... FD sumL sumH`; use BLE `Cxyz500rx` for the `Vx=0.5` protocol-speed test.
- Failure Rules: If target/source id or scaling differs, update only the DFLink encoder constants, not the controller.
- Next Integration Boundary: Controller calls `chassis_uart_send_motion()`.

## Eight-Channel Line Sensor UART

- Goal: Read an 8-bit line sensor state and convert it to a small signed line offset.
- Inputs: USART digital frames after sending `$0,0,1#`.
- Outputs: `line_sensor_sample_t` with raw bits, weighted offset, and timestamp.
- Independent Test: Feed `$D,x1:0,x2:1,x3:1,x4:1,x5:1,x6:1,x7:1,x8:1#` over UART and verify bit 0 plus offset.
- Failure Rules: Ignore non-digital frames; on timeout, resend `$0,0,1#`. During auto-running the controller treats transient read problems as zero-linear safe Z search; outside auto-running it keeps the chassis stopped.
- Next Integration Boundary: Controller calls `line_sensor_uart_read_sample()`.

## Line Trace Controller

- Goal: Convert line offset into chassis motion only when the vehicle state allows motion.
- Inputs: `line_sensor_sample_t`, live parameters from `param_store`, and `vehicle_state_snapshot_t`.
- Outputs: stop commands in safe/tuning/armed/fault/OTA states, short manual-test commands in `MANUAL_TEST`, PID motion commands in `AUTO_RUNNING`, slow-then-fast quadratic line-speed reduction when the active line drifts away from `x4/x5`, slow-then-fast quadratic turn growth from normalized offset, and zero-linear timed speed-command recovery when line bits are lost.
- Speed Profile: gear 1 sends up to `250 mm/s` with `8000 mdeg/cmd` turn limit; gear 2 sends up to `600 mm/s` with `10000 mdeg/cmd`; gear 3 sends up to `1000 mm/s` with `10000 mdeg/cmd`. At the outermost tracked offsets, adaptive slowdown keeps roughly 45% of the selected gear speed.
- Independent Test: Verify boot telemetry shows `SAFE_IDLE`, `start_auto` is required before PID motion, lost line enters `LINE_LOST` phase with `linear_mm_s=0`, and BLE disconnect while active sends stop.
- Failure Rules: Line loss and transient sensor read issues during auto-running do not enter `FAULT`; line loss uses the fixed timed left/right speed-command sweep and transient sensor read issues keep a conservative zero-linear search command. If the final 30 s timed segment finishes without line recovery, automatic line tracing exits to `SAFE_IDLE`. BLE disconnect while active immediately stops and returns to `SAFE_IDLE`. Chassis send failure still enters `CHASSIS_SEND_FAILED`.
- Next Integration Boundary: Later PID tuning and chassis direction confirmation.

## Vehicle State

- Goal: Own the vehicle work model, including main motion state, wireless tuning session state, OTA maintenance state, fault reason, and manual-test timeout.
- Inputs: Debug protocol commands such as `enter_tuning`, `exit_tuning`, `arm_auto`, `start_auto`, `stop`, `clear_fault`, and `manual_motion`, plus OTA handler requests to enter/finish `OTA_UPDATE`.
- Outputs: `vehicle_state_snapshot_t` for controller and telemetry.
- Independent Test: Feed debug protocol commands and verify valid transitions plus rejected parameter writes outside `PARAM_TUNING`.
- Failure Rules: `FAULT` and `OTA_UPDATE` cannot enter tuning directly. OTA may be entered from fault as a recovery path. Manual motion is accepted only in `PARAM_TUNING`.
- Next Integration Boundary: Computer-side web tool mirrors these states and enables only valid actions.

## Parameter Store

- Goal: Keep all live-tunable parameters in one registry with ranges, defaults, and optional persistence.
- Inputs: Parameter id plus typed value from debug protocol or future local controls.
- Outputs: Current typed value, parameter schema, version counter, and NVS persistence on explicit save.
- Independent Test: Set each parameter inside and outside its allowed range, then reset defaults and save.
- Failure Rules: Reject unknown ids, wrong types, and out-of-range values; never write flash on every live update.
- Next Integration Boundary: Wi-Fi/BLE transports call `debug_protocol`; control loop reads typed params.

## Debug Protocol

- Goal: Provide a JSON protocol for HTTP/WebSocket fallback diagnostics and long BLE diagnostics, including stop-to-tune and motion-state commands.
- Inputs: JSON message with `type` such as `get_schema`, `enter_tuning`, `set_param`, `manual_motion`, `arm_auto`, `start_auto`, `stop`, or `get_telemetry`.
- Outputs: JSON response with schema, params, telemetry, state acknowledgements, update acknowledgements, or error messages.
- Independent Test: Feed synthetic JSON messages and check response shape, state transitions, parameter gate, and manual-motion safety limits.
- Failure Rules: Mutating parameter messages outside `PARAM_TUNING` return `enter_tuning_required`; invalid states return state-aware errors.
- Next Integration Boundary: Fallback Wi-Fi diagnostics and Mac BLE tester use the same protocol; the DHelper mini program `line-tuning` feature uses the compact BLE PID/gear frame.

## Wireless Debug Server

- Goal: Keep BLE available for low-power tuning, and expose Wi-Fi SoftAP plus HTTP/WebSocket only as an on-demand fallback.
- Inputs: `debug_server_config_t`, compact BLE frames, debug protocol requests, and optional browser/WebSocket clients.
- Outputs: BLE device name defaults to fixed `DCtrl`; after `W1`, SoftAP `DCar-Liner-XXXXXX`, HTTP API at `192.168.4.1`, WebSocket endpoint at `/ws`, and OTA endpoints at `/api/ota/status` and `/api/ota`.
- Independent Test: Use `tools/ble_debug_tester.swift --command G --append-newline --expect-prefix P`; then send `W1` and call `/api/schema`, `/api/params`, `/api/telemetry`, and `/api/ota/status`.
- Name Control: `N` reads the BLE name, `N=<name>` saves a complete 1-20 UTF-8 byte name using CJK Chinese/ASCII letters/digits/`-`/`_`, and `N=*` restores the default Bluetooth-MAC name.
- Failure Rules: Compact parameter writes return `E:RANGE`, `E:FAULT`, `E:BUSY`, or `E:SAVE`; Wi-Fi startup failures do not disable BLE.
- Next Integration Boundary: DHelper mini program `line-tuning` uses only compact `G`/`S` PID+gear frames; Wi-Fi remains a fallback diagnostics path.

## Firmware OTA

- Goal: Update the ESP32 app image wirelessly after one USB install of the OTA-capable partition table.
- Inputs: Raw `DCar-Liner.bin` body posted to `POST /api/ota` while connected to the ESP32 SoftAP.
- Outputs: Write to the inactive OTA app partition, select it for boot, send a JSON success response, and reboot.
- Independent Test: Build S3/C3, USB-flash the OTA-capable C3 firmware once, start Wi-Fi with BLE `W1`, check `GET /api/ota/status`, then run `python3 tools/ota_upload.py build-esp32c3/DCar-Liner.bin`.
- Failure Rules: Oversized firmware is rejected; receive/write/image validation failures enter `FAULT` with `OTA_FAILED`; USB flashing remains the recovery path.
- Next Integration Boundary: Keep OTA as a computer maintenance workflow until repeated manual uploads are stable.

## Telemetry

- Goal: Hold the latest observable vehicle, control, sensor, and motion state for wireless debug clients.
- Inputs: Latest vehicle state snapshot, line sample, motion command, lost-line state, line quality, and PID output.
- Outputs: JSON telemetry snapshot including `vehicle.motion_state`, `vehicle.debug_session`, `vehicle.fault`, line diagnostics, motion command, uptime, and parameter version.
- Independent Test: Update telemetry with known samples and verify the built JSON.
- Failure Rules: Build failure returns `ESP_ERR_INVALID_SIZE` rather than truncating JSON silently.
- Next Integration Boundary: Stream telemetry periodically over WebSocket after the UI workflow is defined.
