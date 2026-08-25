#![cfg(feature = "selftest")]

//! Runs the on-device conformance suite against a simulated XY7025.
//!
//! The suite ships in the crate behind the `selftest` feature. Running it here
//! against a simulator is what keeps it honest: every driver change is checked
//! against it by `cargo test --all-features`, instead of only when someone
//! flashes a board.
//!
//! [`Firmware`] below is a register file that reproduces the quirks
//! `DATASHEET.md` records. It is a stand-in, not a substitute — it cannot tell
//! us how real firmware behaves, only that the suite and the driver still
//! agree with what we already believe about it.

use std::collections::HashMap;
use std::time::Duration;

use xy_modbus::framing::{self, MAX_READ_REGS, MAX_WRITE_REGS};
use xy_modbus::regs::{Group, Reg};
use xy_modbus::selftest;
use xy_modbus::transport::ModbusTransport;
use xy_modbus::{RtuError, Xy};

/// A simulated XY7025 register file.
///
/// Writes are stored and read back, except where real firmware is known to do
/// something else — each exception below cites the behaviour it reproduces.
#[derive(Debug)]
struct Firmware {
    regs: HashMap<u16, u16>,
}

impl Firmware {
    fn new() -> Self {
        let mut regs = HashMap::new();
        // Seed the registers whose zero value the driver would reject, so a
        // fresh device is in a state the suite can snapshot.
        for (reg, value) in [
            (Reg::Model, 0x6500),
            (Reg::Version, 0x0071),
            (Reg::VIn, 2400),
            (Reg::SlaveAddr, 1),
            (Reg::BaudCode, 6),
            (Reg::Backlight, 3),
        ] {
            regs.insert(reg.addr(), value);
        }
        Self { regs }
    }

    /// M0's first two words *are* the live V-SET / I-SET at `0x0000`, so both
    /// addresses must name one cell — the driver's crossing guards read one
    /// and write the other.
    fn cell(addr: u16) -> u16 {
        if addr == Reg::VSet.addr() || addr == Reg::ISet.addr() {
            Group::M0.addr() + addr
        } else {
            addr
        }
    }

    fn get(&self, addr: u16) -> u16 {
        self.regs.get(&Self::cell(addr)).copied().unwrap_or(0)
    }

    fn set(&mut self, addr: u16, value: u16) {
        // EXTRACT-M copies a stored group over the live set rather than
        // storing the group number.
        if addr == Reg::ExtractM.addr() {
            let group = Group::ALL[usize::from(value.min(9))];
            for offset in 0..Group::LEN {
                let word = self.get(group.addr() + offset as u16);
                self.regs.insert(Group::M0.addr() + offset as u16, word);
            }
            return;
        }
        // Calibration offsets ignore Modbus writes entirely.
        if addr == Reg::TInOffset.addr() || addr == Reg::TExOffset.addr() {
            return;
        }
        let value = if addr == Reg::Backlight.addr() {
            value.max(1) // the display can never be fully extinguished
        } else if addr == Reg::Sleep.addr() {
            value.min(9) // nine-minute ceiling
        } else {
            value
        };
        self.regs.insert(Self::cell(addr), value);
    }
}

/// Honours the `ModbusTransport` contract, which requires rejecting a
/// malformed request *before* any I/O. `framing` publishes the address and
/// quantity rules so an implementation need not restate them — this doubles as
/// a worked example of writing a transport.
impl ModbusTransport for Firmware {
    fn read_holding(&mut self, slave: u8, addr: u16, dst: &mut [u16]) -> Result<(), RtuError> {
        framing::validate_read_slave(slave)?;
        if dst.is_empty() || dst.len() > MAX_READ_REGS {
            return Err(RtuError::InvalidQuantity(dst.len()));
        }
        for (i, slot) in dst.iter_mut().enumerate() {
            *slot = self.get(addr + i as u16);
        }
        Ok(())
    }

    fn write_single_holding(&mut self, slave: u8, addr: u16, value: u16) -> Result<(), RtuError> {
        framing::validate_write_slave(slave)?;
        self.set(addr, value);
        Ok(())
    }

    fn write_multiple_holdings(
        &mut self,
        slave: u8,
        addr: u16,
        values: &[u16],
    ) -> Result<(), RtuError> {
        framing::validate_write_slave(slave)?;
        if values.is_empty() || values.len() > MAX_WRITE_REGS {
            return Err(RtuError::InvalidQuantity(values.len()));
        }
        for (i, &value) in values.iter().enumerate() {
            self.set(addr + i as u16, value);
        }
        Ok(())
    }
}

/// Forwards the suite's progress to stderr, so `cargo test -- --nocapture`
/// shows which check failed rather than just a count.
struct StderrLog;

impl log::Log for StderrLog {
    fn enabled(&self, _: &log::Metadata<'_>) -> bool {
        true
    }

    fn log(&self, record: &log::Record<'_>) {
        eprintln!("[{}] {}", record.level(), record.args());
    }

    fn flush(&self) {}
}

/// The whole suite must pass against a device that behaves as documented.
///
/// A failure here means either the suite drifted from the driver, or the
/// driver drifted from what `DATASHEET.md` says the hardware does.
#[test]
fn suite_passes_against_documented_firmware_behaviour() {
    static LOGGER: StderrLog = StderrLog;
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Info);

    let report = selftest::run(Xy::new(Firmware::new()), 115_200, Duration::ZERO);
    assert_eq!(
        report.failed,
        0,
        "{} of {} suite checks failed against the simulator",
        report.failed,
        report.passed + report.failed
    );
    // Guard against the suite silently shrinking to nothing.
    assert!(
        report.passed >= 20,
        "only {} checks ran; the suite table lost entries",
        report.passed
    );
}
