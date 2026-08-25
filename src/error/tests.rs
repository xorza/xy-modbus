extern crate std;

use core::error::Error;
use core::mem::size_of;
use std::format;

use super::*;

#[test]
fn input_error_display_identifies_each_invalid_input() {
    let cases = [
        (
            InputError::NonFinite {
                field: InputField::VoltageSetpoint,
            },
            "voltage setpoint must be finite",
        ),
        (
            InputError::OutOfRange {
                field: InputField::CurrentSetpoint,
            },
            "current setpoint is out of range",
        ),
        (
            InputError::VoltageSetpointAboveProtection,
            "voltage setpoint exceeds over-voltage protection",
        ),
    ];
    for (error, expected) in cases {
        assert_eq!(format!("{error}"), expected);
    }
}

/// Every variant renders, and each string is distinct — a copy-paste in the
/// `Display` arms would otherwise go unnoticed.
#[test]
fn rtu_error_display_covers_every_variant() {
    let cases = [
        (RtuError::InvalidQuantity(0), "invalid register quantity 0"),
        (
            RtuError::InvalidSlaveAddress(248),
            "invalid Modbus slave address 248",
        ),
        (
            RtuError::BroadcastRead,
            "read request cannot use broadcast address 0",
        ),
        (
            RtuError::BufferTooSmall {
                needed: 12,
                actual: 8,
            },
            "buffer too small (need 12, have 8)",
        ),
        (RtuError::ShortResponse(3), "short response (3 bytes)"),
        (RtuError::BadSlave(0x7F), "wrong slave id 0x7F"),
        (RtuError::BadHeader, "malformed header"),
        (RtuError::BadCrc, "CRC mismatch"),
        (RtuError::Exception(0x02), "modbus exception 0x02"),
        (RtuError::Timeout, "UART response timed out"),
        (RtuError::BusBusy, "UART bus did not become quiet"),
        (
            RtuError::Io {
                operation: IoOperation::Read,
                kind: IoErrorKind::ConnectionReset,
            },
            "UART read error (ConnectionReset)",
        ),
    ];
    for (error, expected) in cases {
        assert_eq!(format!("{error}"), expected);
    }
    // 12 variants, 12 distinct strings.
    let mut seen: std::vec::Vec<&str> = cases.iter().map(|(_, s)| *s).collect();
    seen.sort_unstable();
    let before = seen.len();
    seen.dedup();
    assert_eq!(seen.len(), before);
    assert_eq!(before, 12);
}

/// The split a retry loop depends on: a malformed request will fail the same
/// way forever, a bus fault may not.
#[test]
fn rtu_error_separates_request_faults_from_bus_faults() {
    for error in [
        RtuError::InvalidQuantity(0),
        RtuError::InvalidSlaveAddress(248),
        RtuError::BroadcastRead,
        RtuError::BufferTooSmall {
            needed: 12,
            actual: 8,
        },
    ] {
        assert!(error.is_request_error(), "{error:?}");
    }
    for error in [
        RtuError::ShortResponse(3),
        RtuError::BadSlave(2),
        RtuError::BadHeader,
        RtuError::BadCrc,
        RtuError::Exception(2),
        RtuError::Timeout,
        RtuError::BusBusy,
        RtuError::Io {
            operation: IoOperation::Write,
            kind: IoErrorKind::WriteZero,
        },
    ] {
        assert!(!error.is_request_error(), "{error:?}");
    }
}

#[test]
fn xy_error_preserves_sources_and_invalid_register_context() {
    let input = XyError::from(InputError::OutOfRange {
        field: InputField::VoltageSetpoint,
    });
    assert_eq!(format!("{input}"), "voltage setpoint is out of range");
    assert!(input.source().is_some());

    let rtu = XyError::from(RtuError::Timeout);
    assert_eq!(format!("{rtu}"), "UART response timed out");
    assert!(rtu.source().is_some());

    let register = XyError::InvalidRegisterValue {
        register: 0x0018,
        value: 300,
    };
    assert_eq!(
        format!("{register}"),
        "invalid value 300 in register 0x0018"
    );
    assert!(register.source().is_none());
}

#[cfg(feature = "embedded-io")]
#[test]
fn embedded_io_error_kinds_map_without_losing_detail() {
    let cases = [
        (embedded_io::ErrorKind::Other, IoErrorKind::Other),
        (embedded_io::ErrorKind::NotFound, IoErrorKind::NotFound),
        (
            embedded_io::ErrorKind::PermissionDenied,
            IoErrorKind::PermissionDenied,
        ),
        (
            embedded_io::ErrorKind::ConnectionRefused,
            IoErrorKind::ConnectionRefused,
        ),
        (
            embedded_io::ErrorKind::ConnectionReset,
            IoErrorKind::ConnectionReset,
        ),
        (
            embedded_io::ErrorKind::ConnectionAborted,
            IoErrorKind::ConnectionAborted,
        ),
        (
            embedded_io::ErrorKind::NotConnected,
            IoErrorKind::NotConnected,
        ),
        (embedded_io::ErrorKind::AddrInUse, IoErrorKind::AddrInUse),
        (
            embedded_io::ErrorKind::AddrNotAvailable,
            IoErrorKind::AddrNotAvailable,
        ),
        (embedded_io::ErrorKind::BrokenPipe, IoErrorKind::BrokenPipe),
        (
            embedded_io::ErrorKind::AlreadyExists,
            IoErrorKind::AlreadyExists,
        ),
        (
            embedded_io::ErrorKind::InvalidInput,
            IoErrorKind::InvalidInput,
        ),
        (
            embedded_io::ErrorKind::InvalidData,
            IoErrorKind::InvalidData,
        ),
        (embedded_io::ErrorKind::TimedOut, IoErrorKind::TimedOut),
        (
            embedded_io::ErrorKind::Interrupted,
            IoErrorKind::Interrupted,
        ),
        (
            embedded_io::ErrorKind::Unsupported,
            IoErrorKind::Unsupported,
        ),
        (
            embedded_io::ErrorKind::OutOfMemory,
            IoErrorKind::OutOfMemory,
        ),
        (embedded_io::ErrorKind::WriteZero, IoErrorKind::WriteZero),
    ];
    for (source, expected) in cases {
        assert_eq!(IoErrorKind::from(source), expected);
    }
}

#[test]
fn errors_have_compact_embedded_representations() {
    assert_eq!(size_of::<InputField>(), 1);
    assert_eq!(size_of::<InputError>(), 2);
    // Discriminant plus the widest payload (`BufferTooSmall`'s two `usize`s).
    assert_eq!(size_of::<RtuError>(), 3 * size_of::<usize>());
}
