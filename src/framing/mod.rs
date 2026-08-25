//! Modbus-RTU on-wire framing — pure functions, no I/O.
//!
//! Use these to build a [`crate::transport::ModbusTransport`] over your platform's
//! UART. The codec is general Modbus-RTU (function codes `0x03`, `0x06`,
//! `0x10`); nothing in here is XY-specific.
//! Address `0` is accepted for write broadcasts; read requests must use a
//! unicast slave address because broadcasts never receive a response.
//!
//! CRC-16 is the standard reflected polynomial `0xA001`, seeded
//! `0xFFFF`, no final XOR. The CRC is appended low-byte first.

use crate::error::RtuError;

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

/// Check a slave address for a request that expects a response. Address `0`
/// is [`RtuError::BroadcastRead`] because no slave answers a broadcast.
///
/// `ModbusTransport` implementations owe this check before any I/O; calling it
/// here keeps the `0..=247` rule in one place.
pub fn validate_read_slave(slave: u8) -> Result<(), RtuError> {
    match slave {
        0 => Err(RtuError::BroadcastRead),
        1..=247 => Ok(()),
        invalid => Err(RtuError::InvalidSlaveAddress(invalid)),
    }
}

/// Check a slave address for a write, where `0` is a legal broadcast.
pub fn validate_write_slave(slave: u8) -> Result<(), RtuError> {
    if slave <= 247 {
        Ok(())
    } else {
        Err(RtuError::InvalidSlaveAddress(slave))
    }
}

/// Build a unicast `Read Holding Registers` (FC `0x03`) request frame.
pub fn build_read_request(slave: u8, addr: u16, count: usize) -> Result<[u8; 8], RtuError> {
    validate_read_slave(slave)?;
    if count == 0 || count > MAX_READ_REGS {
        return Err(RtuError::InvalidQuantity(count));
    }
    let mut req = [0u8; 8];
    req[0] = slave;
    req[1] = FN_READ_HOLDING;
    req[2..4].copy_from_slice(&addr.to_be_bytes());
    // Bounded by `MAX_READ_REGS` above, so this cannot truncate.
    req[4..6].copy_from_slice(&(count as u16).to_be_bytes());
    append_crc(&mut req, 6);
    Ok(req)
}

/// Build a `Write Single Holding Register` (FC `0x06`) request frame.
///
/// Slave address `0` creates a broadcast request, for which no response exists.
pub fn build_write_single_request(slave: u8, addr: u16, value: u16) -> Result<[u8; 8], RtuError> {
    validate_write_slave(slave)?;
    let mut req = [0u8; 8];
    req[0] = slave;
    req[1] = FN_WRITE_SINGLE;
    req[2..4].copy_from_slice(&addr.to_be_bytes());
    req[4..6].copy_from_slice(&value.to_be_bytes());
    append_crc(&mut req, 6);
    Ok(req)
}

/// Build a `Write Multiple Holding Registers` (FC `0x10`) request into
/// `out`, returning the number of bytes written. `out` must be at
/// least `9 + 2 * values.len()` bytes.
///
/// Slave address `0` creates a broadcast request, for which no response exists.
pub fn build_write_multiple_request(
    slave: u8,
    addr: u16,
    values: &[u16],
    out: &mut [u8],
) -> Result<usize, RtuError> {
    validate_write_slave(slave)?;
    if values.is_empty() || values.len() > MAX_WRITE_REGS {
        return Err(RtuError::InvalidQuantity(values.len()));
    }
    let bc = 2 * values.len();
    let len = 7 + bc + 2;
    if out.len() < len {
        return Err(RtuError::BufferTooSmall {
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

#[cfg(feature = "embedded-io")]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum ResponseShape {
    ReadHolding { register_count: usize },
    WriteSingle,
    WriteMultiple,
}

#[cfg(feature = "embedded-io")]
pub(crate) fn response_adu_len(
    prefix: [u8; 3],
    slave: u8,
    shape: ResponseShape,
) -> Result<usize, RtuError> {
    if prefix[0] != slave {
        return Err(RtuError::BadSlave(prefix[0]));
    }
    let expected_fn = match shape {
        ResponseShape::ReadHolding { register_count } => {
            if register_count == 0 || register_count > MAX_READ_REGS {
                return Err(RtuError::InvalidQuantity(register_count));
            }
            FN_READ_HOLDING
        }
        ResponseShape::WriteSingle => FN_WRITE_SINGLE,
        ResponseShape::WriteMultiple => FN_WRITE_MULTIPLE,
    };
    if prefix[1] == expected_fn | EXCEPTION_BIT {
        return Ok(5);
    }
    if prefix[1] != expected_fn {
        return Err(RtuError::BadHeader);
    }
    match shape {
        ResponseShape::ReadHolding { register_count } => {
            let byte_count = 2 * register_count;
            if prefix[2] as usize != byte_count {
                return Err(RtuError::BadHeader);
            }
            Ok(5 + byte_count)
        }
        ResponseShape::WriteSingle | ResponseShape::WriteMultiple => Ok(8),
    }
}

fn check_crc(resp: &[u8], len: usize) -> Result<(), RtuError> {
    if resp.len() < len {
        return Err(RtuError::ShortResponse(resp.len()));
    }
    if resp.len() != len {
        return Err(RtuError::BadHeader);
    }
    let got = u16::from_le_bytes([resp[len - 2], resp[len - 1]]);
    let calc = crc16_modbus(&resp[..len - 2]);
    if got == calc {
        Ok(())
    } else {
        Err(RtuError::BadCrc)
    }
}

fn check_exception(resp: &[u8], slave: u8, expected_fn: u8) -> Result<(), RtuError> {
    if resp.len() < 5 {
        return Err(RtuError::ShortResponse(resp.len()));
    }
    if resp[0] != slave {
        return Err(RtuError::BadSlave(resp[0]));
    }
    if resp[1] == expected_fn | EXCEPTION_BIT {
        check_crc(resp, 5)?;
        return Err(RtuError::Exception(resp[2]));
    }
    if resp[1] & EXCEPTION_BIT != 0 {
        return Err(RtuError::BadHeader);
    }
    Ok(())
}

/// Parse a `Read Holding Registers` response into `out`. The expected
/// register count is `out.len()`; zero or more than [`MAX_READ_REGS`] returns
/// [`RtuError::InvalidQuantity`].
pub fn parse_read_response(resp: &[u8], slave: u8, out: &mut [u16]) -> Result<(), RtuError> {
    if out.is_empty() || out.len() > MAX_READ_REGS {
        return Err(RtuError::InvalidQuantity(out.len()));
    }
    check_exception(resp, slave, FN_READ_HOLDING)?;
    let count = out.len();
    let expected_len = 5 + 2 * count;
    if resp[1] != FN_READ_HOLDING || resp[2] as usize != 2 * count {
        return Err(RtuError::BadHeader);
    }
    check_crc(resp, expected_len)?;
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = u16::from_be_bytes([resp[3 + 2 * i], resp[4 + 2 * i]]);
    }
    Ok(())
}

/// Parse a `Write Single Holding Register` response. Per Modbus spec the
/// response echoes the request byte-for-byte.
pub fn parse_write_single_response(resp: &[u8], req: &[u8; 8]) -> Result<(), RtuError> {
    check_exception(resp, req[0], FN_WRITE_SINGLE)?;
    check_crc(resp, 8)?;
    if resp != req {
        return Err(RtuError::BadHeader);
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
) -> Result<(), RtuError> {
    check_exception(resp, slave, FN_WRITE_MULTIPLE)?;
    check_crc(resp, 8)?;
    if resp[1] != FN_WRITE_MULTIPLE
        || u16::from_be_bytes([resp[2], resp[3]]) != addr
        || u16::from_be_bytes([resp[4], resp[5]]) != qty
    {
        return Err(RtuError::BadHeader);
    }
    Ok(())
}

#[cfg(test)]
mod tests;
