# DCar-Liner v1 First Tuning Stable

Date: 2026-06-06

This is the first accepted stable tuning version of DCar-Liner after real vehicle testing.

## Status

- Field result: stable and highly usable in the user's real line-following test.
- Working branch policy: `main` remains the live trunk for future development.
- Stable backup branch: `release/v1-first-tuning-stable`.
- Stable tag: `v1.0-first-tuning-stable`.

## Current Auto Line-Tracing Strategy

- Gear 1 maximum line speed: `250 mm/s` (`Vx=0.25`)
- Gear 2 maximum line speed: `600 mm/s` (`Vx=0.6`)
- Gear 3 maximum line speed: `1000 mm/s` (`Vx=1.0`)
- Gear turn limits: `8000 / 10000 / 10000 mdeg/cmd`
- Adaptive slowdown: slow-then-fast quadratic curve from center `x4/x5` toward edge offsets.
- Adaptive turning: normalized squared offset enters PID, so small offsets turn gently and large offsets ramp Z faster.
- Lost-line recovery: zero linear speed, bounded Z search, opposite the last normal tracking turn direction.
- Safety stop: BLE disconnect during active vehicle states sends chassis stop and returns to `SAFE_IDLE`.

## Firmware Artifacts

The matching local firmware binaries were generated before this record:

- ESP32-C3: `build-esp32c3/DCar-Liner.bin`
  - SHA-256: `12c1eb6b00df61bfb9d2a8adbff94deb3b089421397229bbf00a57ac7c01cbec`

The source of truth for all current behavior strategies is:

- `docs/strategy-register.html`
