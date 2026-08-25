//! Host-side type check for `esp32c6-test`'s conformance suite.
//!
//! Not meant to be run — the mock transport refuses every transaction. Its
//! only job is to make `suite.rs` compile somewhere other than a flashed
//! ESP32, so the compiler and clippy can see it on every change.

#[path = "../../esp32c6-test/src/suite.rs"]
mod suite;

use xy_modbus::transport::ModbusTransport;
use xy_modbus::{RtuError, Xy};

/// Refuses everything: this binary exists to be compiled, not executed.
struct NeverAnswers;

impl ModbusTransport for NeverAnswers {
    fn read_holding(&mut self, _slave: u8, _addr: u16, _dst: &mut [u16]) -> Result<(), RtuError> {
        Err(RtuError::Timeout)
    }

    fn write_single_holding(&mut self, _slave: u8, _addr: u16, _v: u16) -> Result<(), RtuError> {
        Err(RtuError::Timeout)
    }

    fn write_multiple_holdings(
        &mut self,
        _slave: u8,
        _addr: u16,
        _values: &[u16],
    ) -> Result<(), RtuError> {
        Err(RtuError::Timeout)
    }
}

fn main() {
    let _ = suite::run(Xy::new(NeverAnswers), 115_200);
}
