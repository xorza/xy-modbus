//! Modbus-RTU on-wire framing — pure functions, no I/O.
//!
//! Use these to build a [`crate::ModbusTransport`] over your platform's
//! UART. The codec is general Modbus-RTU (function codes `0x03`, `0x06`,
//! `0x10`); nothing in here is XY-specific.
//!
//! CRC-16 is the standard reflected polynomial `0xA001`, seeded
//! `0xFFFF`, no final XOR. The CRC is appended low-byte first.

use crate::transport::ModbusError;

// ─── Constants ───────────────────────────────────────────────────────────────

pub(crate) const FN_READ_HOLDING: u8 = 0x03;
pub(crate) const FN_WRITE_SINGLE: u8 = 0x06;
pub(crate) const FN_WRITE_MULTIPLE: u8 = 0x10;

/// Modbus exception flag (high bit of the function-code byte). Per the
/// spec, function codes occupy 1..=127 — any FC byte with bit 7 set is
/// an exception response.
pub const EXCEPTION_BIT: u8 = 0x80;

/// Maximum Modbus-RTU ADU size (slave + PDU + CRC).
pub const MAX_ADU: usize = 256;

/// Maximum registers in a single `Read Holding Registers` request
/// (Modbus standard limit).
pub const MAX_READ_REGS: usize = 125;

/// Maximum registers in a single `Write Multiple Holdings` request
/// (Modbus standard limit).
pub const MAX_WRITE_REGS: usize = 123;

// ─── FrameError ──────────────────────────────────────────────────────────────

/// Why [`build_write_multiple_request`] could not assemble a frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FrameError {
    /// `values` was empty or exceeded [`MAX_WRITE_REGS`] (123).
    InvalidLength(usize),
    /// `out` was smaller than the assembled frame (header + payload + CRC).
    BufferTooSmall { needed: usize, actual: usize },
}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidLength(n) => write!(f, "invalid register count {n}"),
            Self::BufferTooSmall { needed, actual } => {
                write!(f, "buffer too small (need {needed}, have {actual})")
            }
        }
    }
}

impl core::error::Error for FrameError {}

// ─── CRC-16 ──────────────────────────────────────────────────────────────────

/// Standard Modbus-RTU CRC-16.
pub fn crc16_modbus(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

fn append_crc(buf: &mut [u8], len: usize) {
    let crc = crc16_modbus(&buf[..len]);
    buf[len] = crc as u8;
    buf[len + 1] = (crc >> 8) as u8;
}

// ─── Request builders ────────────────────────────────────────────────────────

/// Build a `Read Holding Registers` (FC `0x03`) request frame.
pub fn build_read_request(slave: u8, addr: u16, count: u16) -> [u8; 8] {
    let mut req = [0u8; 8];
    req[0] = slave;
    req[1] = FN_READ_HOLDING;
    req[2..4].copy_from_slice(&addr.to_be_bytes());
    req[4..6].copy_from_slice(&count.to_be_bytes());
    append_crc(&mut req, 6);
    req
}

/// Build a `Write Single Holding Register` (FC `0x06`) request frame.
pub fn build_write_single_request(slave: u8, addr: u16, value: u16) -> [u8; 8] {
    let mut req = [0u8; 8];
    req[0] = slave;
    req[1] = FN_WRITE_SINGLE;
    req[2..4].copy_from_slice(&addr.to_be_bytes());
    req[4..6].copy_from_slice(&value.to_be_bytes());
    append_crc(&mut req, 6);
    req
}

/// Build a `Write Multiple Holding Registers` (FC `0x10`) request into
/// `out`, returning the number of bytes written. `out` must be at
/// least `9 + 2 * values.len()` bytes.
pub fn build_write_multiple_request(
    slave: u8,
    addr: u16,
    values: &[u16],
    out: &mut [u8],
) -> Result<usize, FrameError> {
    if values.is_empty() || values.len() > MAX_WRITE_REGS {
        return Err(FrameError::InvalidLength(values.len()));
    }
    let bc = 2 * values.len();
    let len = 7 + bc + 2;
    if out.len() < len {
        return Err(FrameError::BufferTooSmall {
            needed: len,
            actual: out.len(),
        });
    }
    out[0] = slave;
    out[1] = FN_WRITE_MULTIPLE;
    out[2..4].copy_from_slice(&addr.to_be_bytes());
    out[4..6].copy_from_slice(&(values.len() as u16).to_be_bytes());
    out[6] = bc as u8;
    for (i, v) in values.iter().enumerate() {
        out[7 + 2 * i..9 + 2 * i].copy_from_slice(&v.to_be_bytes());
    }
    append_crc(out, 7 + bc);
    Ok(len)
}

// ─── Response parsers ────────────────────────────────────────────────────────

fn check_crc(resp: &[u8], len: usize) -> Result<(), ModbusError> {
    if resp.len() < len {
        return Err(ModbusError::ShortResponse(resp.len()));
    }
    let got = u16::from_le_bytes([resp[len - 2], resp[len - 1]]);
    let calc = crc16_modbus(&resp[..len - 2]);
    if got == calc {
        Ok(())
    } else {
        Err(ModbusError::BadCrc)
    }
}

fn check_exception(resp: &[u8], slave: u8) -> Result<(), ModbusError> {
    if resp.len() < 5 {
        return Err(ModbusError::ShortResponse(resp.len()));
    }
    if resp[0] != slave {
        return Err(ModbusError::BadSlave(resp[0]));
    }
    if resp[1] & EXCEPTION_BIT != 0 {
        check_crc(resp, 5)?;
        return Err(ModbusError::Exception(resp[2]));
    }
    Ok(())
}

/// Parse a `Read Holding Registers` response into `out`. The expected
/// register count is `out.len()`.
pub fn parse_read_response(resp: &[u8], slave: u8, out: &mut [u16]) -> Result<(), ModbusError> {
    check_exception(resp, slave)?;
    let count = out.len();
    let expected_len = 5 + 2 * count;
    if resp[1] != FN_READ_HOLDING || resp[2] as usize != 2 * count {
        return Err(ModbusError::BadHeader);
    }
    check_crc(resp, expected_len)?;
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = u16::from_be_bytes([resp[3 + 2 * i], resp[4 + 2 * i]]);
    }
    Ok(())
}

/// Parse a `Write Single Holding Register` response. Per Modbus spec the
/// response echoes the request byte-for-byte.
pub fn parse_write_single_response(resp: &[u8], req: &[u8; 8]) -> Result<(), ModbusError> {
    check_exception(resp, req[0])?;
    check_crc(resp, 8)?;
    if resp.len() < 8 || resp[..8] != req[..] {
        return Err(ModbusError::BadHeader);
    }
    Ok(())
}

/// Parse a `Write Multiple Holding Registers` response. The response is
/// always 8 bytes: slave, fc, start addr, qty, CRC.
pub fn parse_write_multiple_response(
    resp: &[u8],
    slave: u8,
    addr: u16,
    qty: u16,
) -> Result<(), ModbusError> {
    check_exception(resp, slave)?;
    check_crc(resp, 8)?;
    if resp[1] != FN_WRITE_MULTIPLE
        || u16::from_be_bytes([resp[2], resp[3]]) != addr
        || u16::from_be_bytes([resp[4], resp[5]]) != qty
    {
        return Err(ModbusError::BadHeader);
    }
    Ok(())
}

#[cfg(test)]
mod tests;
