//! Bootstrap for the on-device XY7025 conformance suite.
//!
//! Wiring (UART1 @ 115200 8N1):
//!   ESP GPIO16 (TX) -> XY RX
//!   ESP GPIO17 (RX) -> XY TX
//!   common ground.
//!
//! Everything the suite does — including its warnings about writing to the
//! device — lives in [`suite`]. This file only builds the UART and hands over.

mod suite;

use std::thread;
use std::time::Duration;

use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::uart::UartDriver;
use esp_idf_hal::uart::config::Config;
use esp_idf_hal::units::Hertz;
use log::info;

use xy_modbus::Xy;

const BAUD: u32 = 115_200;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("xy-modbus ESP32-C6 conformance suite starting");

    let peripherals = Peripherals::take().expect("peripherals");
    let uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio16,
        peripherals.pins.gpio17,
        None::<AnyIOPin>,
        None::<AnyIOPin>,
        &Config::new().baudrate(Hertz(BAUD)),
    )
    .expect("UART1 init");

    let report = suite::run(Xy::from_esp_uart(uart), BAUD);

    // Nothing else to do on a microcontroller, and exiting `main` would reboot
    // into a run loop; park so the report stays on the monitor.
    info!(
        "parked — {} passed, {} failed",
        report.passed, report.failed
    );
    loop {
        thread::sleep(Duration::from_secs(60));
    }
}
