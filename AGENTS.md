# AGENTS.md

This file provides guidance to coding agents working in this repository.

## Crate scope

`xy-modbus` is a `no_std` Modbus-RTU driver whose high-level API targets the
XY7025 programmable buck converter. Other devices use the raw framing/transport
layers; related variants share core function codes but differ in fixed-point
scales, physical limits, optional registers, and memory-group layouts.

## On-device test example

`examples/esp32c6-test/` is a standalone Cargo project (own
`rust-toolchain.toml`, `sdkconfig.defaults`, `.cargo/config.toml`, separate
`[workspace]`) that runs a 25-test sweep against a real XY7025 over UART1
on an ESP32-C6 (GPIO16/17, 115200 8N1). It snapshots persistent
configuration as exact register words, sweeps every public API method
(V/I/protection sweeps, output enable/disable, all M0–M9 groups, lifecycle
via `into_transport` + `Xy::with_slave`), and verifies exact restoration at
the end while leaving the live output safely off. Includes asserted
raw-transport checks for S-OTP / T-IN-OFFSET / SLEEP firmware quirks.

Build/flash from inside that directory: `./flash.sh`. The example pulls
`xy-modbus` via `path = "../.."` and is not picked up by the crate root's
`cargo` commands.

## Known XY7025 firmware quirks (verified empirically by `examples/esp32c6-test/`)

These were discovered while running the on-device suite and are now
encoded in the driver / datasheet — keep them in mind when extending:

- **S-OTP scale is 1, not 10.** Raw register value equals the displayed
  degrees in the unit selected by `F-C` (raw 95 with unit=°F is 95 °F).
  The third-party tinkering4fun PDF's "scale 10" entry is wrong for this
  firmware. `decode_group` / `encode_group` use scale=1; tests assert raw
  95 → 95.0.
- **Group writes (M0–M9) clamp S-OTP to 110 °C / 230 °F** in the current
  display unit and apply firmware unit conversion that introduces ±1°
  rounding. Single-register writes via `ModbusTransport::write_single_holding`
  bypass both — they round-trip raw values exactly.
- **T-IN/T-EX-OFFSET writes are silently ignored over Modbus.** Reads
  work. The driver intentionally does not expose `set_temp_offset_*` —
  use the front-panel calibration menu. Removed in
  `device/mod.rs` with a "setters intentionally absent" comment so the
  decision is discoverable.
- **Backlight floor is 1, not 0.** Writing 0 reads back as 1 — the
  display can't be fully extinguished via Modbus.
- **SLEEP cap is 9 minutes.** Any write ≥10 reads back as 9. 0 disables.
- **External-probe temperature (`T_EX`) is raw-only.** Bring-up had no
  thermistor connected, so the connected-probe scale remains unverified. The
  high-level API exposes only the verified internal sensor; use raw register
  access for explicit external-probe validation.

## Reference docs

- `DATASHEET.md` — full register map, CRC algorithm, wire-level examples,
  firmware quirks, recommended bring-up checklist.
- `docs-archive/` — third-party reverse-engineering notes and original
  vendor PDFs that informed the register map.
