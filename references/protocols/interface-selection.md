# Interface Selection For This Firmware

## Chassis

Use DFLink `Motion_Velocity` over UART:

- Frame: `0xDF target source A B LEN payload 0xFD sumL sumH`
- Target id: `0x01`
- Source id: `0x97`
- Command: A=`0x02`, B=`0x62`, LEN=`12`
- Payload: `Vx`, `Vy`, `Vz` as DFLink F32 fixed-point values.
- Encoding: business value * 10000, then signed int32 little-endian.

The current controller maps:

- `linear_mm_s` to `Vx` in m/s.
- `Vy` to `0`.
- `angular_mdeg_s` to `Vz` as a per-command Z angle increment.

## Eight-Channel Line Sensor

Recommended first implementation: USART digital mode.

Reason:

- It needs only two ESP32 pins, which is better for ESP32-C3 migration than eight IO pins.
- The screenshot gives a complete USART command and response format.
- It can be switched to analog mode later without rewiring.
- It keeps the existing firmware boundary as one serial sensor driver.

IO direct read is electrically simple and lowest-latency, but it costs eight GPIOs and is a poor fit for ESP32-C3 migration.

I2C would also be pin-efficient and digital-only, but the screenshot does not include the I2C address, register map, or read transaction format. It remains a good future option after the exact I2C protocol is available.
