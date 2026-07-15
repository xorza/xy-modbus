# Changelog

This unreleased section summarizes the complete working-tree diff from the
repository's default mainline branch, `master`, at `v0.1.1` (`b515e54`). The
repository has no branch named `main`.

## Unreleased

### Added

- Added compact, typed `InputField`, `InputError`, and `XyError` values to
  distinguish invalid API input, invalid device register data, and Modbus
  transport failures without storing string pointers in embedded errors.
- Added `Temperature`, which pairs each temperature value with its `TempUnit`
  and supports explicit unit conversion.
- Added model-aware validation for voltage, current, protection, group, slave,
  display, sleep, baud-rate, and temperature-unit writes before any I/O occurs.
- Added `Xy::read_raw_holding` and `Xy::write_raw_holding` for raw register access
  through the configured transport and slave.
- Added operation context to UART I/O errors, identifying read, write, and flush
  failures.
- Added exact framing and device tests for invalid counts, malformed responses,
  exception function codes, trailing bytes, invalid register values, boundary
  inputs, model limits, and lossless 32-bit conversions.
- Added `REVIEW.md` with the applied review recommendations and the design
  decisions intentionally deferred for later work.

### Changed

- **Breaking:** Removed the unused `serde` feature and serialization derives
  from public types.
- **Breaking:** High-level `Xy` methods now return `XyError`; `Xy::with_slave`
  validates its address and returns `Result`.
- **Breaking:** Removed the trivial `Xy::slave`, `Xy::model`, and `Xy::transport`
  accessors. Raw access now uses the configured-slave methods, while
  `Xy::into_transport` still recovers the transport.
- **Breaking:** `GroupParams` now embeds `Setpoints` and `SafetyLimits`, and its
  32-bit charge and energy fields use `f64`.
- **Breaking:** `Status` now embeds `Setpoints` instead of duplicating `v_set`
  and `i_set` fields.
- **Breaking:** `Totals` charge and energy fields use `f64`, and
  `Temperatures` now returns unit-tagged `Temperature` values.
- **Breaking:** `GroupParams::s_otp` is now a unit-tagged `Temperature`;
  `write_group` converts it to the active unit and returns the values read back
  after firmware clamping or rounding.
- **Breaking:** Custom models now require explicit scales and physical limits;
  `verify_scale_family` returns `ScaleCheck::Compatible` or `Inconclusive`
  without claiming exact model identity. Wire-encoded enums no longer expose
  `Unknown(u16)` variants.
- **Breaking:** `RtuError::Io` now carries an `IoOperation` and portable
  `IoErrorKind`. `BlockingRead` extends `embedded_io::ErrorType` so read and
  write failures preserve the same error classification. `ModbusError` is owned
  by `framing` and `RtuError` by `transport`.
- **Breaking:** Framing, transport, and UART types now have one canonical public
  path through their modules; their duplicate crate-root aliases were removed.
- **Breaking:** `FrameError::InvalidLength` is now `InvalidQuantity`;
  `ModbusError` and `RtuError` also report invalid read/write quantities instead
  of allowing parser or UART assertions.
- **Breaking:** `UartTransport::release` was replaced by `into_parts`, returning
  the named `UartParts` struct. `BlockingRead` now lives with the UART transport.
- Consolidated the public surface in `lib.rs`, narrowed internal visibility, and
  removed internal re-export hubs, duplicate protocol-layer aliases, unused
  register constants, dead-code suppression, tuple returns, and trivial helpers.
- Set explicit dependency versions in the library and ESP32-C6 example
  manifests, matching their verified lockfile resolutions.
- Disabled `esp-idf-hal` default features in the library adapter so application
  startup policy, including `binstart`, remains with the final ESP application.
- Updated the ESP32-C6 hardware example for validated APIs, raw register access,
  nested group parameters, precise totals, optional external temperature, and
  the revised error model.
- Ignored the project-local `.venv` used for ESP-IDF tooling.
- Updated `README.md`, `DATASHEET.md`, and `AGENTS.md` to match the supported
  model scope, validation behavior, framing rules, UART timeout semantics,
  safety API, and packaged documentation links.
- Renamed the repository guidance to `AGENTS.md`, retained `CLAUDE.md` as a
  compatibility symlink, and excluded both names from the published crate.

### Fixed

- Restarted the complete pre-transmit quiet interval whenever draining observes
  RX activity, and report drain failures before writing a request.
- Rejected non-finite, negative, out-of-model-range, and unrepresentable write
  values instead of silently clamping or normalizing them.
- Enforced the documented XY7025 LVP minimum of 10 V for standalone and group
  protection writes.
- Rejected invalid group hours/minutes and runtime minutes/seconds read from the
  device instead of returning malformed time values.
- Guarded live M0 writes against the V-SET/S-OVP ordering hazard without
  assuming that FC10 register application is atomic.
- Preserved all `u32` register-pair values across decode/encode round trips by
  using `f64` for cumulative counters and group limits.
- Recognized both documented XY7025-family model codes, `0x6100` and `0x6500`,
  while treating unknown codes as inconclusive rather than mismatches.
- Decoded disconnected external temperature probes as `None` when the device
  reports its `888.8` sentinel.
- Rejected invalid boolean, protection, regulation-mode, temperature-unit,
  baud-rate, backlight, sleep, slave-address, and group register values instead
  of coercing them into valid states or exposing them as domain enum variants.
- Validated read quantities before frame construction, checked exception replies
  against the requested function, and rejected otherwise-valid frames with
  trailing bytes.
- Derived response lengths from validated slave/function/byte-count prefixes so
  complete malformed replies return header errors instead of timing out while
  waiting for request-predicted bytes.
- Documented the UART timeout as a per-read inactivity timeout, matching its
  actual behavior.
- Corrected the backlight range and group-energy unit documentation, and made
  memory-group address calculation enforce its index precondition.
- Replaced alignment-sensitive wire-frame diagrams with byte-layout tables and
  corrected package-safe reference links and source-line anchors.
