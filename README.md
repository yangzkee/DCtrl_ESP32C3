# ESP32-S3/C3 Line Trace Car Firmware

This project controls a serial-command car chassis and reads an eight-channel infrared line sensor over VGTI/UART.

## What To Configure

### Hardware

- ESP32-S3 board for the first bring-up.
- USB-C serial path for flashing, logging, and recovery.
- One UART link from ESP32-S3 to the car chassis command port.
- One UART link from ESP32-S3 to the eight-channel infrared line sensor VGTI/UART port.
- Common GND across ESP32, chassis controller, and sensor module.
- 3.3 V UART logic, or level shifters when a peripheral uses 5 V logic.
- Stable power isolation so motor power noise does not reset the ESP32 board.
- Wi-Fi SoftAP debug channel for wireless parameter tuning.
- Reserved BLE debug transport boundary for later close-range tuning.

### Default ESP32-S3 Wiring

| Role | UART | ESP TX | ESP RX | Baud |
| --- | --- | --- | --- | --- |
| Chassis command serial | UART1 | GPIO17 | GPIO18 | 115200 |
| Eight-channel line sensor | UART2 | GPIO15 | GPIO16 | 115200 |

### ESP32-C3 Migration

All chip-specific UART and GPIO choices live in `components/board/board_profile.c`.

The control, chassis, and sensor modules must keep using `board_profile_t` instead of hardcoded pins. If the final ESP32-C3 board cannot expose two independent UART links cleanly, change only the board profile or add an external UART bridge profile.

## Current Protocols

- Chassis output uses DFLink `Motion_Velocity`: frame head `0xDF`, target `0x01`, source `0x97`, A=`0x02`, B=`0x62`, LEN=`12`, payload `Vx,Vy,Vz` as fixed-point F32.
- Line sensor input uses USART digital mode. The firmware sends `$0,0,1#` and parses `$D,x1:0,...,x8:0#`.

Protocol reference files are preserved under `references/protocols/`.

For the eight-channel line sensor, USART digital mode is the first implementation choice. It needs only two pins, has a complete known protocol from the screenshot, and stays compatible with ESP32-C3 migration. IO direct read costs eight GPIOs. I2C remains a future option after its address/register protocol is confirmed.

## Wireless Debug Scaffold

The firmware starts a Wi-Fi SoftAP named `DCar-Liner-XXXXXX` with password `DCar-Liner123`.

After connecting the computer to that hotspot:

- Open `http://192.168.4.1/` for a minimal debug landing page.
- Read parameter schema from `http://192.168.4.1/api/schema`.
- Read current parameters from `http://192.168.4.1/api/params`.
- Read current telemetry from `http://192.168.4.1/api/telemetry`.
- Use WebSocket `ws://192.168.4.1/ws` for live tuning.

Example WebSocket messages:

```json
{"type":"get_schema"}
```

```json
{"type":"set_param","id":"pid.kp","value":7500}
```

```json
{"type":"save_params"}
```

Live updates change RAM immediately. Flash persistence happens only after `save_params`.
