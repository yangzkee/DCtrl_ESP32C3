# Vehicle Feature Mode Maintenance

更新时间：2026-06-14

## Current Truth

ESP32-C3 exposes one BLE name, `DCtrl`. The remote-control service is a transparent serial bridge: the mini program writes DFLink chassis bytes, and the firmware forwards those bytes to the chassis UART without motion parsing, target caching, unit conversion, local feed scheduling, or success ACKs on the high-rate path. Remote bridge success is silent; only `E:*` errors notify the client.

The remote bridge may update low-cost telemetry counters for diagnosis. These counters are observation only: write counts, forwarded bytes, current-session last/max write gaps, and error totals must never drive motion decisions, smoothing, watchdogs, or ACK behavior. A remote telemetry session starts on the first remote write and ends on BLE disconnect or after more than 3000 ms of no remote writes before the next write.

The shared vehicle coordinate truth is fixed at the DFLink command boundary: `X > 0` means forward, `Y > 0` means left translation, and `Z/Yaw > 0` means left/counterclockwise rotation. The mini program owns input mapping into this coordinate frame; the ESP32-C3 bridge must forward the resulting bytes unchanged.

Line tracing, line tuning, OTA, diagnostics, and remote bridge features share the same physical chassis UART. Only the active vehicle mode may write motion bytes. Inactive features must not poll sensors, run PID, feed cached motion, or send periodic stop frames.

## Active Modes

- `SAFE_IDLE`: no feature owns motion output.
- `REMOTE_BRIDGE`: DCtrl remote bridge owns raw chassis UART forwarding.
- `PARAM_TUNING`: line-tuning parameter edits are allowed; normal line control is not running.
- `MANUAL_TEST`: tuning manual-test command owns the line controller briefly.
- `AUTO_ARMED` / `AUTO_RUNNING`: line-trace controller owns sensor reads and chassis motion.
- `OTA_UPDATE`: firmware update owns the device and forces a one-shot stop before writing flash.
- `FAULT`: motion output is disabled until explicitly cleared.

## Rules For Future Work

- Do not start `line_trace_controller` from `app_main`; start it only from a state transition that enters manual test or automatic line tracing.
- Do not create `line_trace_controller` tasks without first reserving startup ownership. The controller start path must keep a `STARTING` state or equivalent in the critical section so concurrent BLE/Wi-Fi requests cannot create duplicate tasks.
- Do not send periodic `chassis_uart_stop()` from disabled, idle, tuning, remote, OTA, or fault states. One-shot stops are allowed only on explicit stop, disconnect, manual-test timeout, OTA entry, or fault handling.
- Do not add a second remote-motion protocol. Remote control must reuse the mini program DFLink encoder and firmware raw UART bridge.
- Do not restore `REMOTE_V1`, 13-byte DCtrl semantic motion frames, remote target caches, watchdog-fed local motion loops, or ESP32-side speed/yaw conversion.
- Do not make remote bridge success noisy again. `OK` notifications on every forwarded BLE write are forbidden because they compete with the high-rate control path; only error notifications are allowed.
- Do not remove `remote_bridge` telemetry from `get_telemetry`; it is the field diagnostic path for proving whether BLE writes reached ESP32-C3 during stutter reports. `remote_bridge.max_gap_ms` is session-scoped, so idle time between separate tests must not pollute the value.
- Do not let partial BLE debug RX streams survive a connection boundary. Clear the compact/JSON stream buffer on connect and disconnect so a truncated command from one session cannot become a prefix for the next command.
- Do not smooth DCtrl remote rotation inside ESP32-C3. The mini program owns the business-level `Vz deg/cmd` profile and must encode it to DFLink wire-level `rad/cmd`; the bridge still forwards bytes unchanged.
- Do not invert, reinterpret, or special-case the shared coordinate signs inside the remote bridge. Remote DFLink bytes already use `X > 0` forward, `Y > 0` left, and `Z/Yaw > 0` left/counterclockwise.
- Do not reintroduce `WHEELTEC` as a runtime device name, scan compatibility path, priority match, service alias, code module, UI copy, or test fixture.
- Do not reintroduce editable BLE names. `N` reads `DCtrl`, `N=*` clears old custom-name storage and restores/confirms `DCtrl`, and `N=<name>` is disabled.
- Do not write `chassis_uart_send_motion()`, `chassis_uart_stop()`, or `chassis_uart_write_raw()` from a new feature until that feature has an explicit vehicle mode and documented ownership boundary.
- When adding a feature, document its goal, inputs, outputs, independent test, failure rules, and next integration boundary before wiring it into shared UART control.

## Required Checks

- Mini program: `npm run check` and `npm test` from `miniapp/dcar-ble-remote`.
- Firmware static boundary check: `python3 tools/check_feature_boundaries.py`.
- Firmware build: `./scripts/idf.sh -B build-esp32c3 -DSDKCONFIG=sdkconfig.esp32c3 build`.
- Remote smoke tester: `swift tools/ble_remote_tester.swift --timeout 25 --motion zero` and `--motion yaw`; this tool must send raw 21-byte DFLink `Motion_Velocity` frames, not the retired 13-byte remote semantic frame.
- Hardware smoke test after flashing: connect to BLE `DCtrl`, confirm remote movement, release zero-speed, disconnect zero-speed, then separately verify line-tuning and line-trace entry still work.
