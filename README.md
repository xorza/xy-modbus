# xy-modbus

<img src="docs/XY7025.png" alt="XY7025 board" width="400">

`no_std` Modbus-RTU driver for the XY-series programmable buck
converters (XY7025, XY6020L, XY6015, XY-SK60, XY-SK120, XY-SK120X).
These modules share a common register layout — the differences between
models are mechanical (max V/A/W), not protocol.

> **Hardware verification:** only the XY7025 has been tested against
> real hardware (see `examples/esp32c6-test/`). Other models are
> supported on the basis of the shared register layout documented by
> third-party reverse engineering, but are **unverified**.

## Usage

```rust,ignore
use xy_modbus::{Model, SafetyLimits, Xy};

let mut xy = Xy::new(my_transport, Model::Xy7025);

xy.set_protection(SafetyLimits { lvp_v: 22.0, ovp_v: 15.0, ocp_a: 15.0 })?;
xy.set_voltage(13.5)?;
xy.set_current_limit(10.0)?;
xy.set_output(true)?;

let s = xy.read_status()?;
```

`Model` selects per-register scales for I-OUT, POWER, S-OCP, S-OPP —
the wrong family silently shifts current and power readings by 10×.
Call `xy.verify_model()?` at boot to catch a misconfiguration against
the device's `MODEL` register.

## Transport

The default `embedded-io` feature ships a `UartTransport` over any
`BlockingRead + embedded_io::Write` pair. For `esp-idf-hal`, enable
the `esp-idf-hal` feature and use `Xy::from_esp_uart(uart, model)`.
To bring your own, implement [`ModbusTransport`] directly — the
`framing` module exposes the on-wire codec.

The transport implementer owns UART timing. The XY-series wants
~50 ms between frames and ~500 ms response window; see
[`DATASHEET.md`](DATASHEET.md) §2.

## Cargo features

| Feature       | Default | Purpose                                                                |
|---------------|---------|------------------------------------------------------------------------|
| `embedded-io` | yes     | Bundled `UartTransport` over `BlockingRead + embedded_io::Write`.      |
| `esp-idf-hal` | no      | `Xy::from_esp_uart` constructor for `esp_idf_hal::UartDriver`.         |
| `defmt`       | no      | `defmt::Format` derives on public types.                               |
| `serde`       | no      | `Serialize`/`Deserialize` derives on public types.                     |

## Boot / safety policy

This crate exposes the device protocol; it intentionally does **not**
prescribe a power-on / fault-recovery policy. See
[`DATASHEET.md`](DATASHEET.md) §7 for the recommended bring-up
checklist (program protection *before* raising V-SET, force OUTPUT_EN
off until verification passes, etc.).

## References

- [`DATASHEET.md`](DATASHEET.md) — full register map, CRC algorithm,
  wire-level examples, known firmware quirks.
- `examples/esp32c6-test/` — on-device 26-test sweep against a real
  XY7025 over UART, snapshots and restores every writable register.
- API reference on [docs.rs](https://docs.rs/xy-modbus).

## License

Licensed under either of [MIT](LICENSE-MIT) or
[Apache-2.0](LICENSE-APACHE) at your option.
