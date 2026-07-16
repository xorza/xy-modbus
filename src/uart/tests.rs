extern crate std;

use std::vec::Vec;

use embedded_io::ErrorType;

use super::*;
use crate::framing::{ModbusError, crc16_modbus};

/// Mock UART: `stale` bytes are visible immediately (simulating
/// junk in the RX FIFO before the request); `response` bytes only
/// become available after the first byte is written.
#[derive(Debug)]
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

#[derive(Debug)]
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
    let parts = t.into_parts();
    let expected_req = framing::build_read_request(0x01, 0x0000, 3).unwrap();
    assert_eq!(parts.uart.tx, expected_req);
}

#[test]
fn write_single_round_trip() {
    // Echo response.
    let req = framing::build_write_single_request(0x01, 0x0012, 0x0001);
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
#[derive(Debug)]
struct CountingDelay {
    total_ms: u32,
}
impl DelayNs for CountingDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.total_ms += ns / 1_000_000;
    }
}

/// UART that fails (read or write) on demand.
#[derive(Debug)]
struct FailingUart {
    fail_drain: bool,
    fail_read: bool,
    fail_write: bool,
    fail_flush: bool,
    write_returns_zero: bool,
    response: Vec<u8>,
    resp_pos: usize,
    armed: bool,
}
impl ErrorType for FailingUart {
    type Error = embedded_io::ErrorKind;
}
impl BlockingRead for FailingUart {
    fn read(&mut self, buf: &mut [u8], _timeout_ms: u32) -> Result<usize, Self::Error> {
        if self.fail_drain && !self.armed {
            return Err(embedded_io::ErrorKind::InvalidData);
        }
        if self.fail_read && self.armed {
            return Err(embedded_io::ErrorKind::ConnectionReset);
        }
        let mut n = 0;
        while self.armed && n < buf.len() && self.resp_pos < self.response.len() {
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
            return Err(embedded_io::ErrorKind::BrokenPipe);
        }
        self.armed = true;
        if self.write_returns_zero {
            return Ok(0);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        if self.fail_flush {
            Err(embedded_io::ErrorKind::BrokenPipe)
        } else {
            Ok(())
        }
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
        fail_drain: false,
        fail_read: false,
        fail_write: false,
        fail_flush: false,
        write_returns_zero: true,
        response: Vec::new(),
        resp_pos: 0,
        armed: false,
    };
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 1];
    assert_eq!(
        t.read_holding(0x01, 0x0000, &mut out).unwrap_err(),
        RtuError::Io {
            operation: IoOperation::Write,
            kind: IoErrorKind::WriteZero,
        }
    );
}

#[test]
fn write_error_is_io_error() {
    let uart = FailingUart {
        fail_drain: false,
        fail_read: false,
        fail_write: true,
        fail_flush: false,
        write_returns_zero: false,
        response: Vec::new(),
        resp_pos: 0,
        armed: false,
    };
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    assert_eq!(
        t.write_single_holding(0x01, 0x0012, 0x0001).unwrap_err(),
        RtuError::Io {
            operation: IoOperation::Write,
            kind: IoErrorKind::BrokenPipe,
        }
    );
}

#[test]
fn flush_error_identifies_flush_operation() {
    let uart = FailingUart {
        fail_drain: false,
        fail_read: false,
        fail_write: false,
        fail_flush: true,
        write_returns_zero: false,
        response: Vec::new(),
        resp_pos: 0,
        armed: false,
    };
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    assert_eq!(
        t.write_single_holding(0x01, 0x0012, 0x0001).unwrap_err(),
        RtuError::Io {
            operation: IoOperation::Flush,
            kind: IoErrorKind::BrokenPipe,
        }
    );
}

#[test]
fn interrupted_io_retries_the_current_transaction() {
    #[derive(Debug)]
    struct InterruptingUart {
        tx: Vec<u8>,
        response: Vec<u8>,
        response_pos: usize,
        armed: bool,
        drain_interrupted: bool,
        response_interrupted: bool,
        write_calls: u8,
        flush_calls: u8,
    }

    impl ErrorType for InterruptingUart {
        type Error = embedded_io::ErrorKind;
    }

    impl BlockingRead for InterruptingUart {
        fn read(&mut self, buf: &mut [u8], _timeout_ms: u32) -> Result<usize, Self::Error> {
            if !self.armed {
                if !self.drain_interrupted {
                    self.drain_interrupted = true;
                    return Err(embedded_io::ErrorKind::Interrupted);
                }
                return Ok(0);
            }
            if !self.response_interrupted {
                self.response_interrupted = true;
                return Err(embedded_io::ErrorKind::Interrupted);
            }
            if buf.is_empty() || self.response_pos == self.response.len() {
                return Ok(0);
            }
            buf[0] = self.response[self.response_pos];
            self.response_pos += 1;
            Ok(1)
        }
    }

    impl embedded_io::Write for InterruptingUart {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.write_calls += 1;
            if self.write_calls == 2 {
                return Err(embedded_io::ErrorKind::Interrupted);
            }
            let written = buf.len().min(2);
            self.tx.extend_from_slice(&buf[..written]);
            self.armed = true;
            Ok(written)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            self.flush_calls += 1;
            if self.flush_calls == 1 {
                Err(embedded_io::ErrorKind::Interrupted)
            } else {
                Ok(())
            }
        }
    }

    let response = frame_with_crc(std::vec![0x01, 0x03, 0x02, 0x00, 0x05]);
    let uart = InterruptingUart {
        tx: Vec::new(),
        response,
        response_pos: 0,
        armed: false,
        drain_interrupted: false,
        response_interrupted: false,
        write_calls: 0,
        flush_calls: 0,
    };
    let mut transport = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 1];
    transport.read_holding(0x01, 0x0000, &mut out).unwrap();
    assert_eq!(out, [5]);

    let parts = transport.into_parts();
    assert_eq!(
        parts.uart.tx,
        framing::build_read_request(0x01, 0x0000, 1).unwrap()
    );
    assert!(parts.uart.drain_interrupted);
    assert!(parts.uart.response_interrupted);
    assert_eq!(parts.uart.write_calls, 5);
    assert_eq!(parts.uart.flush_calls, 2);
    assert_eq!(parts.uart.response_pos, parts.uart.response.len());
}

#[test]
fn read_error_mid_frame_is_io_error() {
    // Arm the UART with one byte of "response" but make read() return Err.
    let uart = FailingUart {
        fail_drain: false,
        fail_read: true,
        fail_write: false,
        fail_flush: false,
        write_returns_zero: false,
        response: std::vec![0xAA],
        resp_pos: 0,
        armed: false,
    };
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    let mut out = [0u16; 1];
    assert_eq!(
        t.read_holding(0x01, 0x0000, &mut out).unwrap_err(),
        RtuError::Io {
            operation: IoOperation::Read,
            kind: IoErrorKind::ConnectionReset,
        }
    );
}

#[test]
fn pre_tx_activity_restarts_the_full_inter_frame_gap() {
    let response = frame_with_crc(std::vec![0x01, 0x03, 0x02, 0x00, 0x05]);
    let cases = [(std::vec![], 7), (std::vec![0xAA, 0xBB, 0xCC], 14)];
    for (stale, expected_delay_ms) in cases {
        let uart = MockUart::new(response.clone()).with_stale(stale);
        let mut transport =
            UartTransport::new(uart, CountingDelay { total_ms: 0 }).with_timing(50, 7);
        let mut out = [0u16; 1];
        transport.read_holding(0x01, 0x0000, &mut out).unwrap();
        assert_eq!(transport.into_parts().delay.total_ms, expected_delay_ms);
    }
}

#[test]
fn pre_tx_drain_error_is_reported_before_write() {
    let uart = FailingUart {
        fail_drain: true,
        fail_read: false,
        fail_write: false,
        fail_flush: false,
        write_returns_zero: false,
        response: Vec::new(),
        resp_pos: 0,
        armed: false,
    };
    let mut transport = UartTransport::new(uart, CountingDelay { total_ms: 0 }).with_timing(50, 7);
    let mut out = [0u16; 1];
    assert_eq!(
        transport.read_holding(0x01, 0x0000, &mut out),
        Err(RtuError::Io {
            operation: IoOperation::Read,
            kind: IoErrorKind::InvalidData,
        })
    );
    let parts = transport.into_parts();
    assert!(!parts.uart.armed);
    assert_eq!(parts.delay.total_ms, 7);
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
fn broadcast_writes_transmit_without_waiting_for_response() {
    let uart = MockUart::new(Vec::new());
    let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
    t.write_single_holding(0, 0x0012, 1).unwrap();
    t.write_multiple_holdings(0, 0x0052, &[1000, 1500, 1250])
        .unwrap();

    assert_eq!(
        t.into_parts().uart.tx,
        [
            0x00, 0x06, 0x00, 0x12, 0x00, 0x01, 0xE9, 0xDE, 0x00, 0x10, 0x00, 0x52, 0x00, 0x03,
            0x06, 0x03, 0xE8, 0x05, 0xDC, 0x04, 0xE2, 0x65, 0x11,
        ]
    );
}

/// `read_exact` must aggregate across multiple `read()` calls when the
/// UART hands back a single byte at a time — a real concern with FIFOs
/// that drain incrementally.
#[test]
fn read_exact_aggregates_byte_at_a_time() {
    #[derive(Debug)]
    struct DribbleUart {
        response: Vec<u8>,
        pos: usize,
        armed: bool,
    }
    impl ErrorType for DribbleUart {
        type Error = core::convert::Infallible;
    }
    impl BlockingRead for DribbleUart {
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
fn malformed_complete_responses_return_header_errors_without_timeout() {
    let frame = frame_with_crc(std::vec![0x01, 0x03, 0x02, 0x00, 0x05]);
    let uart = MockUart::new(frame);
    let mut transport = UartTransport::new(uart, NoDelay).with_timing(10, 0);
    let mut out = [0u16; MAX_READ_REGS];
    assert_eq!(
        transport.read_holding(0x01, 0x0000, &mut out).unwrap_err(),
        RtuError::Modbus(ModbusError::BadHeader)
    );

    let frame = frame_with_crc(std::vec![0x01, 0x03, 0x02, 0x00, 0x05]);
    let uart = MockUart::new(frame);
    let mut transport = UartTransport::new(uart, NoDelay).with_timing(10, 0);
    assert_eq!(
        transport.write_single_holding(0x01, 0x0012, 1).unwrap_err(),
        RtuError::Modbus(ModbusError::BadHeader)
    );
}

#[test]
fn invalid_quantities_fail_before_uart_io() {
    let uart = MockUart::new(Vec::new());
    let mut transport = UartTransport::new(uart, NoDelay).with_timing(10, 0);

    let mut empty = [];
    assert_eq!(
        transport.read_holding(0x01, 0, &mut empty),
        Err(RtuError::InvalidQuantity(0))
    );
    let mut oversized = [0u16; MAX_READ_REGS + 1];
    assert_eq!(
        transport.read_holding(0x01, 0, &mut oversized),
        Err(RtuError::InvalidQuantity(MAX_READ_REGS + 1))
    );
    assert_eq!(
        transport.write_multiple_holdings(0x01, 0, &[]),
        Err(RtuError::InvalidQuantity(0))
    );
    let oversized = [0u16; MAX_WRITE_REGS + 1];
    assert_eq!(
        transport.write_multiple_holdings(0x01, 0, &oversized),
        Err(RtuError::InvalidQuantity(MAX_WRITE_REGS + 1))
    );
    assert!(transport.into_parts().uart.tx.is_empty());
}

#[test]
fn into_parts_returns_inner_uart_and_delay() {
    let uart = MockUart::new(Vec::new());
    let t = UartTransport::new(uart, NoDelay).with_timing(123, 7);
    let uart = t.into_parts().uart;
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
