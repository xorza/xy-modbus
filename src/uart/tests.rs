extern crate std;

use std::vec::Vec;

use embedded_io::ErrorType;

use super::*;
use crate::framing::crc16_modbus;
use crate::transport::{BlockingRead, ModbusError};

/// Mock UART: `stale` bytes are visible immediately (simulating
/// junk in the RX FIFO before the request); `response` bytes only
/// become available after the first byte is written.
struct MockUart {
    tx: Vec<u8>,
    stale: Vec<u8>,
    stale_pos: usize,
    response: Vec<u8>,
    resp_pos: usize,
    armed: bool,
}

impl MockUart {
    fn new(response: Vec<u8>) -> Self {
        Self {
            tx: Vec::new(),
            stale: Vec::new(),
            stale_pos: 0,
            response,
            resp_pos: 0,
            armed: false,
        }
    }

    fn with_stale(mut self, stale: Vec<u8>) -> Self {
        self.stale = stale;
        self
    }
}

impl BlockingRead for MockUart {
    type Error = core::convert::Infallible;
    fn read(&mut self, buf: &mut [u8], _timeout_ms: u32) -> Result<usize, Self::Error> {
        let mut written = 0;
        while written < buf.len() && self.stale_pos < self.stale.len() {
            buf[written] = self.stale[self.stale_pos];
            self.stale_pos += 1;
            written += 1;
        }
        if self.armed {
            while written < buf.len() && self.resp_pos < self.response.len() {
                buf[written] = self.response[self.resp_pos];
                self.resp_pos += 1;
                written += 1;
            }
        }
        Ok(written)
    }
}

impl ErrorType for MockUart {
    type Error = core::convert::Infallible;
}

impl embedded_io::Write for MockUart {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if !buf.is_empty() {
            self.armed = true;
        }
        self.tx.extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

struct NoDelay;
impl DelayNs for NoDelay {
    fn delay_ns(&mut self, _: u32) {}
}

fn frame_with_crc(mut bytes: Vec<u8>) -> Vec<u8> {
    let crc = crc16_modbus(&bytes);
    bytes.push(crc as u8);
    bytes.push((crc >> 8) as u8);
    bytes
}

#[test]
fn read_holding_round_trip() {
    // Slave 1 read 3 regs at 0x0000 → returns [10, 20, 30].
    let resp = frame_with_crc(std::vec![0x01, 0x03, 0x06, 0, 10, 0, 20, 0, 30]);
    let uart = MockUart::new(resp);
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);

    let mut out = [0u16; 3];
    t.read_holding(0x01, 0x0000, &mut out).unwrap();
    assert_eq!(out, [10, 20, 30]);

    // Verify the request that went out matches the canonical encoding.
    let (uart, _) = t.release();
    let expected_req = build_read_request(0x01, 0x0000, 3);
    assert_eq!(uart.tx, expected_req);
}

#[test]
fn write_single_round_trip() {
    // Echo response.
    let req = build_write_single_request(0x01, 0x0012, 0x0001);
    let uart = MockUart::new(req.to_vec());
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    t.write_single_holding(0x01, 0x0012, 0x0001).unwrap();
}

#[test]
fn write_multiple_round_trip() {
    let resp = frame_with_crc(std::vec![0x01, 0x10, 0x00, 0x52, 0x00, 0x03]);
    let uart = MockUart::new(resp);
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    t.write_multiple_holdings(0x01, 0x0052, &[1000, 1500, 1250])
        .unwrap();
}

#[test]
fn exception_response_propagates() {
    let frame = frame_with_crc(std::vec![0x01, 0x83, 0x02]);
    let uart = MockUart::new(frame);
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 1];
    let err = t.read_holding(0x01, 0x0000, &mut out).unwrap_err();
    assert_eq!(err, RtuError::Modbus(ModbusError::Exception(0x02)));
}

#[test]
fn timeout_when_no_data() {
    let uart = MockUart::new(Vec::new());
    let mut t = UartTransport::new(uart, NoDelay).with_timing(3, 0);
    let mut out = [0u16; 1];
    assert_eq!(
        t.read_holding(0x01, 0x0000, &mut out).unwrap_err(),
        RtuError::Timeout
    );
}

/// UART that records every `delay_ms` for assertions on timing.
struct CountingDelay {
    total_ms: u32,
}
impl DelayNs for CountingDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.total_ms += ns / 1_000_000;
    }
}

/// UART that fails (read or write) on demand.
struct FailingUart {
    fail_read: bool,
    fail_write: bool,
    write_returns_zero: bool,
    response: Vec<u8>,
    resp_pos: usize,
    armed: bool,
}
impl ErrorType for FailingUart {
    type Error = embedded_io::ErrorKind;
}
impl BlockingRead for FailingUart {
    type Error = embedded_io::ErrorKind;
    fn read(&mut self, buf: &mut [u8], _timeout_ms: u32) -> Result<usize, Self::Error> {
        if self.fail_read {
            return Err(embedded_io::ErrorKind::Other);
        }
        let mut n = 0;
        while n < buf.len() && self.resp_pos < self.response.len() {
            buf[n] = self.response[self.resp_pos];
            self.resp_pos += 1;
            n += 1;
        }
        Ok(n)
    }
}
impl embedded_io::Write for FailingUart {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if self.fail_write {
            return Err(embedded_io::ErrorKind::Other);
        }
        self.armed = true;
        if self.write_returns_zero {
            return Ok(0);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[test]
fn bad_crc_propagates() {
    // Build a real-looking 3-reg read response, then flip the CRC.
    let mut frame = std::vec![0x01u8, 0x03, 0x06, 0, 10, 0, 20, 0, 30];
    let crc = crc16_modbus(&frame);
    frame.push((crc as u8) ^ 0xFF);
    frame.push((crc >> 8) as u8);
    let uart = MockUart::new(frame);
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 3];
    assert_eq!(
        t.read_holding(0x01, 0x0000, &mut out).unwrap_err(),
        RtuError::Modbus(ModbusError::BadCrc)
    );
}

#[test]
fn wrong_slave_in_response_propagates() {
    // Slave field = 0x02 but request was for 0x01.
    let frame = frame_with_crc(std::vec![0x02, 0x03, 0x02, 0x00, 0x05]);
    let uart = MockUart::new(frame);
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 1];
    assert_eq!(
        t.read_holding(0x01, 0x0000, &mut out).unwrap_err(),
        RtuError::Modbus(ModbusError::BadSlave(0x02))
    );
}

#[test]
fn write_returning_zero_is_io_error() {
    let uart = FailingUart {
        fail_read: false,
        fail_write: false,
        write_returns_zero: true,
        response: Vec::new(),
        resp_pos: 0,
        armed: false,
    };
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 1];
    assert_eq!(
        t.read_holding(0x01, 0x0000, &mut out).unwrap_err(),
        RtuError::Io
    );
}

#[test]
fn write_error_is_io_error() {
    let uart = FailingUart {
        fail_read: false,
        fail_write: true,
        write_returns_zero: false,
        response: Vec::new(),
        resp_pos: 0,
        armed: false,
    };
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    assert_eq!(
        t.write_single_holding(0x01, 0x0012, 0x0001).unwrap_err(),
        RtuError::Io
    );
}

#[test]
fn read_error_mid_frame_is_io_error() {
    // Arm the UART with one byte of "response" but make read() return Err.
    let uart = FailingUart {
        fail_read: true,
        fail_write: false,
        write_returns_zero: false,
        response: std::vec![0xAA],
        resp_pos: 0,
        armed: false,
    };
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 1];
    assert_eq!(
        t.read_holding(0x01, 0x0000, &mut out).unwrap_err(),
        RtuError::Io
    );
}

#[test]
fn pre_tx_silence_applies_inter_frame_gap() {
    // Even with stale RX present (which makes drain_rx do work), the
    // inter_frame_ms gap must be observed before TX.
    let response = frame_with_crc(std::vec![0x01, 0x03, 0x02, 0x00, 0x05]);
    let uart = MockUart::new(response);
    let mut t = UartTransport::new(uart, CountingDelay { total_ms: 0 }).with_timing(50, 7);
    let mut out = [0u16; 1];
    t.read_holding(0x01, 0x0000, &mut out).unwrap();
    let (_, delay) = t.release();
    // Exactly one pre-TX silence of 7 ms; no other delay_ms calls on the
    // happy path (response is ready immediately, so read_exact never sleeps).
    assert_eq!(delay.total_ms, 7);
}

#[test]
#[should_panic(expected = "broadcast")]
fn slave_zero_panics_on_read() {
    let uart = MockUart::new(Vec::new());
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 1];
    let _ = t.read_holding(0x00, 0x0000, &mut out);
}

#[test]
#[should_panic(expected = "broadcast")]
fn slave_zero_panics_on_write_single() {
    let uart = MockUart::new(Vec::new());
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let _ = t.write_single_holding(0x00, 0x0000, 0);
}

#[test]
#[should_panic(expected = "broadcast")]
fn slave_zero_panics_on_write_multiple() {
    let uart = MockUart::new(Vec::new());
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let _ = t.write_multiple_holdings(0x00, 0x0000, &[0]);
}

/// `read_exact` must aggregate across multiple `read()` calls when the
/// UART hands back a single byte at a time — a real concern with FIFOs
/// that drain incrementally.
#[test]
fn read_exact_aggregates_byte_at_a_time() {
    struct DribbleUart {
        response: Vec<u8>,
        pos: usize,
        armed: bool,
    }
    impl ErrorType for DribbleUart {
        type Error = core::convert::Infallible;
    }
    impl BlockingRead for DribbleUart {
        type Error = core::convert::Infallible;
        fn read(&mut self, buf: &mut [u8], _timeout_ms: u32) -> Result<usize, Self::Error> {
            if !self.armed || self.pos >= self.response.len() || buf.is_empty() {
                return Ok(0);
            }
            buf[0] = self.response[self.pos];
            self.pos += 1;
            Ok(1)
        }
    }
    impl embedded_io::Write for DribbleUart {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.armed = true;
            Ok(buf.len())
        }
        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    let resp = frame_with_crc(std::vec![0x01, 0x03, 0x06, 0, 10, 0, 20, 0, 30]);
    let uart = DribbleUart {
        response: resp,
        pos: 0,
        armed: false,
    };
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 3];
    t.read_holding(0x01, 0x0000, &mut out).unwrap();
    assert_eq!(out, [10, 20, 30]);
}

/// Exception responses are only 5 bytes — `read_response` must short-circuit
/// instead of waiting for the full expected length, otherwise the call would
/// time out.
#[test]
fn exception_short_circuits_read() {
    let frame = frame_with_crc(std::vec![0x01, 0x83, 0x02]);
    let uart = MockUart::new(frame);
    // Request 3 regs (would expect 11 bytes) but server returns 5-byte exception.
    let mut t = UartTransport::new(uart, NoDelay).with_timing(10, 0);
    let mut out = [0u16; 3];
    assert_eq!(
        t.read_holding(0x01, 0x0000, &mut out).unwrap_err(),
        RtuError::Modbus(ModbusError::Exception(0x02))
    );
}

#[test]
fn release_returns_inner_uart_and_delay() {
    let uart = MockUart::new(Vec::new());
    let t = UartTransport::new(uart, NoDelay).with_timing(123, 7);
    let (uart, _delay) = t.release();
    // Sanity: tx buffer is empty, no traffic happened.
    assert!(uart.tx.is_empty());
}

#[test]
fn pre_existing_rx_is_drained() {
    // Stale garbage byte that would otherwise corrupt the parse,
    // followed by the real response.
    let response = frame_with_crc(std::vec![0x01, 0x03, 0x02, 0x00, 0x05]);
    let uart = MockUart::new(response).with_stale(std::vec![0xAA, 0xBB, 0xCC]);
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 1];
    t.read_holding(0x01, 0x0000, &mut out).unwrap();
    assert_eq!(out, [5]);
}
