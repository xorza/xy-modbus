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
            InputError::InvalidSlaveAddress { address: 248 },
            "invalid Modbus slave address 248",
        ),
        (
            InputError::InvalidGroup { group: 10 },
            "invalid memory group 10",
        ),
    ];
    for (error, expected) in cases {
        assert_eq!(format!("{error}"), expected);
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

#[test]
fn input_errors_have_compact_embedded_representations() {
    assert_eq!(size_of::<InputField>(), 1);
    assert_eq!(size_of::<InputError>(), 2);
}
