# AGENTS.md

This file provides guidance to coding agents working in this repository.

## Crate scope

`xy-modbus` is a `no_std` Modbus-RTU driver whose high-level API targets the
XY7025 programmable buck converter. Other devices use the raw framing/transport
layers; related variants share core function codes but differ in fixed-point
scales, physical limits, optional registers, and memory-group layouts.

## Workspace boundary

This crate is **excluded** from the parent `esp32-battery` workspace
(`exclude = ["logic", "xy-modbus"]` in `../Cargo.toml`). The parent's
`./run_tests.sh` does **not** cover it. Verify changes with the crate's own
cargo commands run from this directory:

```sh
cargo test
cargo clippy --all-targets --all-features -- -D warnings
cargo fmt --check
cargo build --no-default-features         # exercise pure-no_std path
cargo build --features defmt
```

`cargo test <name>` runs a single test. Tests use `extern crate std;` inside
`#[cfg(test)]` modules — the crate itself stays `no_std`.

## Architecture

Three layers, cleanly separated; each can be used standalone:

1. **`framing`** — pure on-wire codec. `build_read_request`,
   `build_write_*`, `parse_*`, `crc16_modbus`, `MAX_ADU`. No I/O, no allocation.
2. **`transport`** — `ModbusTransport` trait (3 function codes: 0x03, 0x06,
   0x10) plus `RtuError`. `ModbusError` belongs to `framing`. Implementers own
   UART timing; the bundled UART uses a ~50 ms inter-frame gap and 500 ms
   per-read inactivity timeout (see `DATASHEET.md` §2).
3. **`device`** — `Xy<T: ModbusTransport>` high-level API: one method per
   logical operation. Fixed-point conversion (`to_reg` / `from_reg`) lives
   here with the verified XY7025 scales and physical write limits. Getting the
   scale family wrong silently yields readings off by 10×, so
   `verify_scale_family` should run during bring-up.

`uart::UartTransport` (gated behind the default `embedded-io` feature) is a
ready-made transport over any `embedded-io` UART. Disable default features to
bring your own.

`regs.rs` is the single source of truth for register addresses and
`DEFAULT_SLAVE` — the device layer references these constants, never raw
addresses.

`types/` splits value types by concern: `enums` (BaudRate, ProtectionStatus,
RegMode, TempUnit), `status` (live readings, setpoints, safety limits, totals,
on-time, temperature), `group` (M0–M9 memory group params).

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

## Project-specific conventions

- Always derive `Debug` on structs; `defmt` derives are feature-gated where
  applicable.
- Update `CHANGELOG.md` whenever public behavior or the public API changes.
- No backwards-compat shims — rename freely, rewrite callers.
- Tests must verify exact computed values (e.g. CRC bytes, decoded floats),
  not just "doesn't panic".
- WiFi-pairing block (regs 0x0030–0x0034) is documented in `DATASHEET.md`
  but intentionally not exposed at the high-level API yet.
- The crate exposes the protocol only — power-on / fault-recovery policy
  is the caller's responsibility (see `DATASHEET.md` §7).

## Reference docs

- `DATASHEET.md` — full register map, CRC algorithm, wire-level examples,
  firmware quirks, recommended bring-up checklist.
- `docs-archive/` — third-party reverse-engineering notes and original
  vendor PDFs that informed the register map.
