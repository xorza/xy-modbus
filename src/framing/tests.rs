extern crate std;

use super::*;

#[test]
fn crc_known_vectors() {
    assert_eq!(crc16_modbus(&[]), 0xFFFF);
    assert_eq!(crc16_modbus(&[0x01]), 0x807E);
    // Read 1 reg at 0x001F from slave 1.
    assert_eq!(crc16_modbus(&[0x01, 0x03, 0x00, 0x1F, 0x00, 0x01]), 0xCCB5);
    // Write 0x0001 to reg 0x0012 from slave 1.
    assert_eq!(crc16_modbus(&[0x01, 0x06, 0x00, 0x12, 0x00, 0x01]), 0x0FE8);
}

#[test]
fn crc_detects_bit_flips() {
    let base = [0x01u8, 0x03, 0x00, 0x00, 0x00, 0x06];
    let base_crc = crc16_modbus(&base);
    for i in 0..base.len() {
        for bit in 0..8 {
            let mut f = base;
            f[i] ^= 1 << bit;
            assert_ne!(crc16_modbus(&f), base_crc);
        }
    }
}

#[test]
fn build_read_matches_known() {
    let req = build_read_request(0x01, 0x001F, 1).unwrap();
    assert_eq!(req, [0x01, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xB5, 0xCC]);
}

#[test]
fn build_write_single_matches_known() {
    let req = build_write_single_request(0x01, 0x0012, 0x0001).unwrap();
    assert_eq!(req, [0x01, 0x06, 0x00, 0x12, 0x00, 0x01, 0xE8, 0x0F]);
}

#[test]
fn build_write_multiple_layout() {
    // Wire-level example from DATASHEET §6: write LVP=1000, OVP=1500,
    // OCP=1250 to 0x0052..=0x0054, slave 1.
    let mut buf = [0u8; 32];
    let n = build_write_multiple_request(0x01, 0x0052, &[1000, 1500, 1250], &mut buf).unwrap();
    // 7 (header) + 6 (payload) + 2 (CRC) = 15
    assert_eq!(n, 15);
    assert_eq!(
        buf[..n],
        [
            0x01, 0x10, 0x00, 0x52, 0x00, 0x03, 0x06, 0x03, 0xE8, 0x05, 0xDC, 0x04, 0xE2, 0x67,
            0x90,
        ]
    );
}

#[test]
fn build_write_multiple_rejects_empty() {
    let mut buf = [0u8; 32];
    assert!(matches!(
        build_write_multiple_request(0x01, 0x0050, &[], &mut buf),
        Err(FrameError::InvalidQuantity(0))
    ));
}

#[test]
fn build_write_multiple_rejects_too_small_buffer() {
    // Need 9 + 2*3 = 15 bytes, give 10.
    let mut buf = [0u8; 10];
    assert!(matches!(
        build_write_multiple_request(0x01, 0x0050, &[1, 2, 3], &mut buf),
        Err(FrameError::BufferTooSmall {
            needed: 15,
            actual: 10
        })
    ));
}

#[test]
fn parse_write_multiple_rejects_qty_mismatch() {
    let mut frame = [0x01u8, 0x10, 0x00, 0x52, 0x00, 0x03, 0, 0];
    let crc = crc16_modbus(&frame[..6]);
    frame[6] = crc as u8;
    frame[7] = (crc >> 8) as u8;
    // Frame says qty=3 but caller expects 4.
    assert!(matches!(
        parse_write_multiple_response(&frame, 0x01, 0x0052, 4),
        Err(ModbusError::BadHeader)
    ));
}

#[test]
fn build_write_multiple_rejects_oversize() {
    let mut buf = [0u8; 16];
    assert!(build_write_multiple_request(0x01, 0x0050, &[0; 14], &mut buf).is_err());
}

fn read_resp(slave: u8, values: &[u16]) -> std::vec::Vec<u8> {
    let mut out = std::vec::Vec::new();
    out.push(slave);
    out.push(FN_READ_HOLDING);
    out.push((values.len() * 2) as u8);
    for v in values {
        out.extend_from_slice(&v.to_be_bytes());
    }
    let crc = crc16_modbus(&out);
    out.push(crc as u8);
    out.push((crc >> 8) as u8);
    out
}

#[test]
fn parse_read_six_regs() {
    let frame = read_resp(0x01, &[1360, 1000, 1350, 0, 0, 4800]);
    let mut out = [0u16; 6];
    parse_read_response(&frame, 0x01, &mut out).unwrap();
    assert_eq!(out, [1360, 1000, 1350, 0, 0, 4800]);
}

#[test]
fn parse_read_rejects_wrong_slave() {
    let frame = read_resp(0x02, &[0x1234]);
    let mut out = [0u16; 1];
    assert!(matches!(
        parse_read_response(&frame, 0x01, &mut out),
        Err(ModbusError::BadSlave(0x02))
    ));
}

#[test]
fn parse_read_rejects_bad_crc() {
    let mut frame = read_resp(0x01, &[0x1234]);
    let last = frame.len() - 1;
    frame[last] ^= 0xFF;
    let mut out = [0u16; 1];
    assert!(matches!(
        parse_read_response(&frame, 0x01, &mut out),
        Err(ModbusError::BadCrc)
    ));
}

#[test]
fn parse_read_exception_with_valid_crc() {
    let mut frame = std::vec![0x01u8, 0x83, 0x02];
    let crc = crc16_modbus(&frame);
    frame.push(crc as u8);
    frame.push((crc >> 8) as u8);
    let mut out = [0u16; 1];
    assert!(matches!(
        parse_read_response(&frame, 0x01, &mut out),
        Err(ModbusError::Exception(0x02))
    ));
}

#[test]
fn parse_read_exception_with_bad_crc_is_bad_crc() {
    let frame = [0x01u8, 0x83, 0x02, 0x00, 0x00];
    let mut out = [0u16; 1];
    assert!(matches!(
        parse_read_response(&frame, 0x01, &mut out),
        Err(ModbusError::BadCrc)
    ));
}

#[test]
fn parse_write_single_valid_echo() {
    let req = build_write_single_request(0x01, 0x0012, 0x0001).unwrap();
    parse_write_single_response(&req, &req).unwrap();
}

#[test]
fn parse_write_single_rejects_value_mismatch() {
    let req = build_write_single_request(0x01, 0x0012, 0x0001).unwrap();
    let mut resp = req;
    resp[5] = 0x02;
    let crc = crc16_modbus(&resp[..6]);
    resp[6] = crc as u8;
    resp[7] = (crc >> 8) as u8;
    assert!(matches!(
        parse_write_single_response(&resp, &req),
        Err(ModbusError::BadHeader)
    ));
}

#[test]
fn parse_write_single_exception_returns_exception() {
    let req = build_write_single_request(0x01, 0x0012, 0x0001).unwrap();
    let mut frame = std::vec![0x01u8, 0x86, 0x03];
    let crc = crc16_modbus(&frame);
    frame.push(crc as u8);
    frame.push((crc >> 8) as u8);
    assert!(matches!(
        parse_write_single_response(&frame, &req),
        Err(ModbusError::Exception(0x03))
    ));
}

#[test]
fn parse_write_multiple_valid() {
    // Standard echo response: slave, fc, addr, qty, CRC.
    let mut frame = [0x01u8, 0x10, 0x00, 0x52, 0x00, 0x03, 0, 0];
    let crc = crc16_modbus(&frame[..6]);
    frame[6] = crc as u8;
    frame[7] = (crc >> 8) as u8;
    parse_write_multiple_response(&frame, 0x01, 0x0052, 3).unwrap();
}

#[test]
fn parse_write_multiple_rejects_fc_mismatch() {
    // FC 0x03 instead of 0x10 — should be rejected as BadHeader.
    let mut frame = [0x01u8, 0x03, 0x00, 0x52, 0x00, 0x03, 0, 0];
    let crc = crc16_modbus(&frame[..6]);
    frame[6] = crc as u8;
    frame[7] = (crc >> 8) as u8;
    assert!(matches!(
        parse_write_multiple_response(&frame, 0x01, 0x0052, 3),
        Err(ModbusError::BadHeader)
    ));
}

/// Read at the maximum standard count (125) builds a frame whose response
/// would be 5 + 250 = 255 bytes — fits inside MAX_ADU.
#[test]
fn build_read_at_max_count_is_well_formed() {
    let req = build_read_request(0x01, 0x0000, MAX_READ_REGS as u16).unwrap();
    assert_eq!(u16::from_be_bytes([req[4], req[5]]), 125);
    let crc = u16::from_le_bytes([req[6], req[7]]);
    assert_eq!(crc, crc16_modbus(&req[..6]));
}

#[test]
fn build_read_rejects_invalid_requests() {
    assert_eq!(
        build_read_request(0x01, 0x0000, 0),
        Err(FrameError::InvalidQuantity(0))
    );
    assert_eq!(
        build_read_request(0x01, 0x0000, MAX_READ_REGS as u16 + 1),
        Err(FrameError::InvalidQuantity(MAX_READ_REGS + 1))
    );
    assert_eq!(
        build_read_request(0, 0x0000, 1),
        Err(FrameError::BroadcastRead)
    );

    let mut empty = [];
    assert_eq!(
        parse_read_response(&[], 0x01, &mut empty),
        Err(ModbusError::InvalidQuantity(0))
    );
    let mut oversized = [0u16; MAX_READ_REGS + 1];
    assert_eq!(
        parse_read_response(&[], 0x01, &mut oversized),
        Err(ModbusError::InvalidQuantity(MAX_READ_REGS + 1))
    );
}

#[test]
fn request_builders_validate_slave_addresses() {
    let cases = [
        (0, Err(FrameError::BroadcastRead), true),
        (1, Ok(()), true),
        (247, Ok(()), true),
        (248, Err(FrameError::InvalidSlaveAddress(248)), false),
        (255, Err(FrameError::InvalidSlaveAddress(255)), false),
    ];

    for (slave, read_expected, write_valid) in cases {
        assert_eq!(
            build_read_request(slave, 0, 1).map(|_| ()),
            read_expected,
            "read slave {slave}"
        );
        assert_eq!(
            build_write_single_request(slave, 0, 0).is_ok(),
            write_valid,
            "single write slave {slave}"
        );
        let mut out = [0; 11];
        assert_eq!(
            build_write_multiple_request(slave, 0, &[0], &mut out).is_ok(),
            write_valid,
            "multiple write slave {slave}"
        );
    }
}

#[cfg(feature = "embedded-io")]
#[test]
fn response_prefix_selects_and_validates_adu_length() {
    assert_eq!(
        response_adu_len(
            [0x01, FN_READ_HOLDING, 4],
            0x01,
            ResponseShape::ReadHolding { register_count: 2 },
        ),
        Ok(9)
    );
    assert_eq!(
        response_adu_len(
            [0x01, FN_READ_HOLDING | EXCEPTION_BIT, 2],
            0x01,
            ResponseShape::ReadHolding {
                register_count: MAX_READ_REGS,
            },
        ),
        Ok(5)
    );
    assert_eq!(
        response_adu_len([0x01, FN_WRITE_SINGLE, 0], 0x01, ResponseShape::WriteSingle,),
        Ok(8)
    );
    assert_eq!(
        response_adu_len(
            [0x01, FN_WRITE_MULTIPLE, 0],
            0x01,
            ResponseShape::WriteMultiple,
        ),
        Ok(8)
    );
    assert_eq!(
        response_adu_len(
            [0x01, FN_READ_HOLDING, 2],
            0x01,
            ResponseShape::ReadHolding { register_count: 2 },
        ),
        Err(ModbusError::BadHeader)
    );
    assert_eq!(
        response_adu_len(
            [0x02, FN_READ_HOLDING, 4],
            0x01,
            ResponseShape::ReadHolding { register_count: 2 },
        ),
        Err(ModbusError::BadSlave(0x02))
    );
}

/// Write Multiple at the maximum count (123) needs 9 + 246 = 255 bytes.
#[test]
fn build_write_multiple_at_max_count() {
    let mut buf = [0u8; MAX_ADU];
    let values = [0xABCDu16; MAX_WRITE_REGS];
    let n = build_write_multiple_request(0x01, 0x0050, &values, &mut buf).unwrap();
    assert_eq!(n, 9 + 2 * MAX_WRITE_REGS);
    // Byte count field equals 2 * qty.
    assert_eq!(buf[6] as usize, 2 * MAX_WRITE_REGS);
    // CRC is correct.
    let crc = u16::from_le_bytes([buf[n - 2], buf[n - 1]]);
    assert_eq!(crc, crc16_modbus(&buf[..n - 2]));
}

#[test]
fn build_write_multiple_rejects_oversize_payload() {
    let mut buf = [0u8; MAX_ADU];
    let oversized = std::vec![0u16; MAX_WRITE_REGS + 1];
    assert!(matches!(
        build_write_multiple_request(0x01, 0x0050, &oversized, &mut buf),
        Err(FrameError::InvalidQuantity(n)) if n == MAX_WRITE_REGS + 1
    ));
}

#[test]
fn parse_read_response_short_frame_is_short_response() {
    // 3 bytes < minimum 5.
    let frame = [0x01u8, 0x03, 0x02];
    let mut out = [0u16; 1];
    assert!(matches!(
        parse_read_response(&frame, 0x01, &mut out),
        Err(ModbusError::ShortResponse(3))
    ));
}

/// Single-register read response — minimal valid frame is 7 bytes.
#[test]
fn parse_read_response_single_register() {
    let frame = read_resp(0x01, &[0x1234]);
    assert_eq!(frame.len(), 7);
    let mut out = [0u16; 1];
    parse_read_response(&frame, 0x01, &mut out).unwrap();
    assert_eq!(out[0], 0x1234);
}

#[test]
fn parse_read_response_rejects_byte_count_mismatch() {
    // Header claims byte_count=4 but caller expects 1 register (count=2).
    let mut frame = std::vec![0x01u8, 0x03, 0x04, 0x00, 0x05];
    let crc = crc16_modbus(&frame);
    frame.push(crc as u8);
    frame.push((crc >> 8) as u8);
    let mut out = [0u16; 1];
    assert!(matches!(
        parse_read_response(&frame, 0x01, &mut out),
        Err(ModbusError::BadHeader)
    ));
}

#[test]
fn frame_error_display_strings() {
    use std::format;
    assert_eq!(
        format!("{}", FrameError::InvalidQuantity(0)),
        "invalid register quantity 0"
    );
    assert_eq!(
        format!("{}", FrameError::BroadcastRead),
        "read request cannot use broadcast address 0"
    );
    assert_eq!(
        format!("{}", FrameError::InvalidSlaveAddress(248)),
        "invalid Modbus slave address 248"
    );
    assert_eq!(
        format!(
            "{}",
            FrameError::BufferTooSmall {
                needed: 15,
                actual: 10,
            }
        ),
        "buffer too small (need 15, have 10)"
    );
}

#[test]
fn modbus_error_display_strings() {
    use std::format;
    assert_eq!(
        format!("{}", ModbusError::InvalidQuantity(0)),
        "invalid register quantity 0"
    );
    assert_eq!(
        format!("{}", ModbusError::ShortResponse(3)),
        "short response (3 bytes)"
    );
    assert_eq!(
        format!("{}", ModbusError::BadSlave(0x02)),
        "wrong slave id 0x02"
    );
    assert_eq!(format!("{}", ModbusError::BadHeader), "malformed header");
    assert_eq!(format!("{}", ModbusError::BadCrc), "CRC mismatch");
    assert_eq!(
        format!("{}", ModbusError::Exception(0x03)),
        "modbus exception 0x03"
    );
}

#[test]
fn parse_write_multiple_rejects_addr_mismatch() {
    let mut frame = [0x01u8, 0x10, 0x00, 0x52, 0x00, 0x03, 0, 0];
    let crc = crc16_modbus(&frame[..6]);
    frame[6] = crc as u8;
    frame[7] = (crc >> 8) as u8;
    assert!(matches!(
        parse_write_multiple_response(&frame, 0x01, 0x0050, 3),
        Err(ModbusError::BadHeader)
    ));
}

#[test]
fn parsers_reject_trailing_bytes() {
    let mut read = read_resp(0x01, &[0x1234]);
    read.push(0);
    let mut out = [0u16; 1];
    assert_eq!(
        parse_read_response(&read, 0x01, &mut out),
        Err(ModbusError::BadHeader)
    );

    let req = build_write_single_request(0x01, 0x0012, 1).unwrap();
    let mut write = req.to_vec();
    write.push(0);
    assert_eq!(
        parse_write_single_response(&write, &req),
        Err(ModbusError::BadHeader)
    );
}

#[test]
fn parse_read_rejects_exception_for_another_function() {
    let frame = frame_with_crc_for_test(&[0x01, FN_WRITE_SINGLE | EXCEPTION_BIT, 0x02]);
    let mut out = [0u16; 1];
    assert_eq!(
        parse_read_response(&frame, 0x01, &mut out),
        Err(ModbusError::BadHeader)
    );
}

fn frame_with_crc_for_test(bytes: &[u8]) -> std::vec::Vec<u8> {
    let mut frame = bytes.to_vec();
    let crc = crc16_modbus(&frame);
    frame.push(crc as u8);
    frame.push((crc >> 8) as u8);
    frame
}
