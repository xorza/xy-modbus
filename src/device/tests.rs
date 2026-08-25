extern crate std;
use std::vec;
use std::vec::Vec;

use super::*;
use crate::framing::ModbusError;

/// Scriptable transport for tests. Each script entry pairs a
/// register-or-value request with a canned response or error.
#[derive(Debug)]
enum Op {
    Read { addr: u16, values: Vec<u16> },
    WriteOne { addr: u16, value: u16 },
    WriteMany { addr: u16, values: Vec<u16> },
}

#[derive(Debug)]
struct MockTransport {
    script: Vec<Op>,
}

impl MockTransport {
    fn new(script: Vec<Op>) -> Self {
        Self { script }
    }
}

impl Drop for MockTransport {
    fn drop(&mut self) {
        if !std::thread::panicking() {
            assert!(
                self.script.is_empty(),
                "{} unconsumed mock ops",
                self.script.len()
            );
        }
    }
}

impl ModbusTransport for MockTransport {
    fn read_holding(&mut self, _slave: u8, addr: u16, dst: &mut [u16]) -> Result<(), RtuError> {
        let op = self.script.remove(0);
        match op {
            Op::Read { addr: a, values } => {
                assert_eq!(addr, a);
                assert_eq!(dst.len(), values.len());
                dst.copy_from_slice(&values);
                Ok(())
            }
            _ => panic!("expected read"),
        }
    }
    fn write_single_holding(&mut self, _slave: u8, addr: u16, value: u16) -> Result<(), RtuError> {
        let op = self.script.remove(0);
        match op {
            Op::WriteOne { addr: a, value: v } => {
                assert_eq!(addr, a);
                assert_eq!(value, v);
                Ok(())
            }
            _ => panic!("expected write_single"),
        }
    }
    fn write_multiple_holdings(
        &mut self,
        _slave: u8,
        addr: u16,
        values: &[u16],
    ) -> Result<(), RtuError> {
        let op = self.script.remove(0);
        match op {
            Op::WriteMany { addr: a, values: v } => {
                assert_eq!(addr, a);
                assert_eq!(values, v.as_slice());
                Ok(())
            }
            _ => panic!("expected write_multiple"),
        }
    }
}

/// Build a 19-reg fixture for `read_status` with the first six live regs
/// populated and the rest zeroed.
fn status_fixture(live: [u16; 6]) -> Vec<u16> {
    let mut v = vec![0u16; 0x13];
    v[..6].copy_from_slice(&live);
    v
}

#[test]
fn set_voltage_scales_correctly() {
    // 14.40 V → 1440 raw, against a live 72.00 V S-OVP.
    let mock = MockTransport::new(vec![
        Op::Read {
            addr: Reg::SOvp.addr(),
            values: vec![7200],
        },
        Op::WriteOne {
            addr: Reg::VSet.addr(),
            value: 1440,
        },
    ]);
    let mut xy = Xy::new(mock);
    xy.set_voltage(14.40).unwrap();

    // Equal is allowed: 14.40 V against a 14.40 V threshold.
    let mut xy = Xy::new(MockTransport::new(vec![
        Op::Read {
            addr: Reg::SOvp.addr(),
            values: vec![1440],
        },
        Op::WriteOne {
            addr: Reg::VSet.addr(),
            value: 1440,
        },
    ]));
    xy.set_voltage(14.40).unwrap();

    // One raw count above the live threshold is rejected. The mock panics on
    // drop with an unconsumed script, so this also proves no write was sent.
    let mut xy = Xy::new(MockTransport::new(vec![Op::Read {
        addr: Reg::SOvp.addr(),
        values: vec![1439],
    }]));
    assert_eq!(
        xy.set_voltage(14.40),
        Err(XyError::Input(InputError::VoltageSetpointAboveProtection))
    );
}

#[test]
fn set_protection_uses_bulk_write() {
    const LIMITS: SafetyLimits = SafetyLimits {
        lvp_v: 10.0,
        ovp_v: 15.0,
        ocp_a: 12.5,
    };

    // LVP=10.00, OVP=15.00, OCP=12.50 → raw 1000, 1500, 1250, against a live
    // 14.40 V setpoint.
    let mock = MockTransport::new(vec![
        Op::Read {
            addr: Reg::VSet.addr(),
            values: vec![1440],
        },
        Op::WriteMany {
            addr: Reg::SLvp.addr(),
            values: vec![1000, 1500, 1250],
        },
    ]);
    let mut xy = Xy::new(mock);
    xy.set_protection(LIMITS).unwrap();

    // Equal is allowed: a 15.00 V threshold against a 15.00 V setpoint.
    let mut xy = Xy::new(MockTransport::new(vec![
        Op::Read {
            addr: Reg::VSet.addr(),
            values: vec![1500],
        },
        Op::WriteMany {
            addr: Reg::SLvp.addr(),
            values: vec![1000, 1500, 1250],
        },
    ]));
    xy.set_protection(LIMITS).unwrap();

    // One raw count below the live setpoint is rejected before the bulk write.
    let mut xy = Xy::new(MockTransport::new(vec![Op::Read {
        addr: Reg::VSet.addr(),
        values: vec![1501],
    }]));
    assert_eq!(
        xy.set_protection(LIMITS),
        Err(XyError::Input(InputError::VoltageSetpointAboveProtection))
    );
}

#[test]
fn read_protection_decodes_three_regs() {
    let mock = MockTransport::new(vec![Op::Read {
        addr: Reg::SLvp.addr(),
        values: vec![1000, 1500, 1250],
    }]);
    let mut xy = Xy::new(mock);
    let l = xy.read_protection().unwrap();
    assert_eq!(l.lvp_v, 10.0);
    assert_eq!(l.ovp_v, 15.0);
    assert_eq!(l.ocp_a, 12.5);
}

#[test]
fn protection_status_decodes_known_and_rejects_unknown_codes() {
    let mock = MockTransport::new(vec![
        Op::Read {
            addr: Reg::Protect.addr(),
            values: vec![0],
        },
        Op::Read {
            addr: Reg::Protect.addr(),
            values: vec![4],
        },
        Op::Read {
            addr: Reg::Protect.addr(),
            values: vec![99],
        },
    ]);
    let mut xy = Xy::new(mock);
    assert_eq!(
        xy.read_protection_status().unwrap(),
        ProtectionStatus::Normal
    );
    assert_eq!(xy.read_protection_status().unwrap(), ProtectionStatus::Lvp);
    assert_eq!(
        xy.read_protection_status(),
        Err(XyError::InvalidRegisterValue {
            register: Reg::Protect.addr(),
            value: 99,
        })
    );
}

#[test]
fn read_status_decodes_19_regs_in_one_transaction() {
    // Registers 0x0000–0x0012, 19 total. Slot indices match register
    // addresses (0x10 = PROTECT, 0x11 = CVCC, 0x12 = OUTPUT_EN).
    // Pin all the cross-cutting fields the supervisor cares about.
    let mut values = [0u16; 0x13];
    values[0x00] = 1440; // V_SET → 14.40
    values[0x01] = 1000; // I_SET → 10.00 (XY7025 scale 100)
    values[0x02] = 1350; // V_OUT → 13.50
    values[0x03] = 50; // I_OUT → 0.50
    values[0x04] = 675; // P_OUT → 67.5 (scale 10)
    values[0x05] = 2400; // V_IN → 24.00
    values[0x10] = 4; // PROTECT = LVP
    values[0x11] = 1; // CVCC = ConstantCurrent
    values[0x12] = 1; // OUTPUT_EN = on

    let mock = MockTransport::new(vec![Op::Read {
        addr: Reg::VSet.addr(),
        values: values.to_vec(),
    }]);
    let mut xy = Xy::new(mock);
    let s = xy.read_status().unwrap();
    assert_eq!(
        s.setpoints,
        Setpoints {
            v_set: 14.40,
            i_set: 10.00,
        }
    );
    assert_eq!(s.v_out, 13.50);
    assert_eq!(s.i_out, 0.50);
    assert_eq!(s.p_out, 67.5);
    assert_eq!(s.v_in, 24.00);
    assert_eq!(s.protection, ProtectionStatus::Lvp);
    assert_eq!(s.reg_mode, RegMode::ConstantCurrent);
    assert!(s.output_on);
}

#[test]
fn read_totals_composes_high_low() {
    // ah = (high<<16 | low) / 1000
    // pick high=2, low=500 → raw=131_572 → 131.572 Ah.
    // wh: high=0, low=12345 → 12.345 Wh.
    // on_time h=1, m=23, s=45.
    let mock = MockTransport::new(vec![Op::Read {
        addr: Reg::AhLow.addr(),
        values: vec![500, 2, 12345, 0, 1, 23, 45],
    }]);
    let mut xy = Xy::new(mock);
    let t = xy.read_totals().unwrap();
    assert_eq!(t.charge_ah, 131.572);
    assert_eq!(t.energy_wh, 12.345);
    assert_eq!(
        t.on_time,
        OnTime {
            hours: 1,
            minutes: 23,
            seconds: 45
        }
    );
    assert_eq!(t.on_time.total_seconds(), 5025);
}

#[test]
fn read_group_decodes_14_regs() {
    let mock = MockTransport::new(vec![
        Op::Read {
            addr: Reg::TempUnit.addr(),
            values: vec![1],
        },
        Op::Read {
            addr: regs::group_addr(1),
            values: vec![
                1440, // v_set
                1000, // i_set
                1000, // s_lvp
                1500, // s_ovp
                1250, // s_ocp
                1800, // s_opp (W, scale 1)
                0,    // ohp_h
                0,    // ohp_m
                0,    // oah_l
                0,    // oah_h
                0,    // owh_l
                0,    // owh_h
                95,   // s_otp (scale 1 → 95.0; raw matches displayed °)
                0,    // s_ini
            ],
        },
    ]);
    let mut xy = Xy::new(mock);
    let g = xy.read_group(1).unwrap();
    assert_eq!(g.setpoints.v_set, 14.40);
    assert_eq!(g.safety_limits.ovp_v, 15.00);
    assert_eq!(g.s_opp_w, 1800.0);
    assert_eq!(g.s_oah_ah, 0.0);
    assert_eq!(g.s_owh_wh, 0.0);
    assert_eq!(
        g.s_otp,
        Temperature {
            value: 95.0,
            unit: TempUnit::Fahrenheit,
        }
    );
    assert!(!g.power_on_output);
}

#[test]
fn write_group_round_trips_through_encode() {
    let p = GroupParams {
        setpoints: Setpoints {
            v_set: 14.40,
            i_set: 10.00,
        },
        safety_limits: SafetyLimits {
            lvp_v: 10.00,
            ovp_v: 15.00,
            ocp_a: 12.50,
        },
        s_opp_w: 1800.0,
        s_ohp_h: 0,
        s_ohp_m: 0,
        s_oah_ah: 0.0,
        s_owh_wh: 0.0,
        s_otp: Temperature {
            value: 95.0,
            unit: TempUnit::Celsius,
        },
        power_on_output: true,
    };
    let stored = vec![1440, 1000, 1000, 1500, 1250, 1800, 0, 0, 0, 0, 0, 0, 94, 1];
    let mock = MockTransport::new(vec![
        Op::Read {
            addr: Reg::TempUnit.addr(),
            values: vec![0],
        },
        Op::WriteMany {
            addr: regs::group_addr(2),
            values: vec![1440, 1000, 1000, 1500, 1250, 1800, 0, 0, 0, 0, 0, 0, 95, 1],
        },
        Op::Read {
            addr: regs::group_addr(2),
            values: stored,
        },
    ]);
    let mut xy = Xy::new(mock);
    let written = xy.write_group(2, &p).unwrap();
    assert_eq!(written.s_otp.value, 94.0);
    assert_ne!(written.s_otp, p.s_otp);

    let mut invalid = p;
    invalid.safety_limits.lvp_v = 9.99;
    assert_eq!(
        xy.write_group(2, &invalid),
        Err(XyError::Input(InputError::OutOfRange {
            field: InputField::LowVoltageProtection
        }))
    );

    let mut invalid = p;
    invalid.s_otp.value = f32::NAN;
    assert_eq!(
        xy.write_group(2, &invalid),
        Err(XyError::Input(InputError::NonFinite {
            field: InputField::OverTemperatureProtection
        }))
    );

    let mut invalid = p;
    invalid.s_otp.value = 110.1;
    let mut invalid_xy = Xy::new(MockTransport::new(vec![Op::Read {
        addr: Reg::TempUnit.addr(),
        values: vec![0],
    }]));
    assert_eq!(
        invalid_xy.write_group(2, &invalid),
        Err(XyError::Input(InputError::OutOfRange {
            field: InputField::OverTemperatureProtection
        }))
    );

    let mut invalid = p;
    invalid.setpoints.v_set = 15.01;
    assert_eq!(
        xy.write_group(2, &invalid),
        Err(XyError::Input(InputError::VoltageSetpointAboveProtection))
    );
}

#[test]
fn live_group_rejects_protection_crossing_in_either_direction() {
    #[derive(Debug)]
    struct Case {
        /// Live M0 registers 0x0050..=0x0053 (V-SET, I-SET, S-LVP, S-OVP).
        current: [u16; 4],
        target_v_set: u16,
        target_ovp: u16,
        accepted: bool,
    }

    let cases = [
        // New V-SET 14.40 is above the live 13.00 S-OVP, so applying V-SET
        // before S-OVP would trip OVP.
        Case {
            current: [1200, 1000, 1000, 1300],
            target_v_set: 1440,
            target_ovp: 1500,
            accepted: false,
        },
        // New S-OVP 15.00 is below the live 16.00 V-SET, so applying S-OVP
        // before V-SET would trip OVP.
        Case {
            current: [1600, 1000, 1000, 1700],
            target_v_set: 1400,
            target_ovp: 1500,
            accepted: false,
        },
        // Both orderings clear: 1440 <= live 1500 and 1500 >= live 1400.
        Case {
            current: [1400, 1000, 1000, 1500],
            target_v_set: 1440,
            target_ovp: 1500,
            accepted: true,
        },
        // Equality on both bounds is clear — the comparisons are strict.
        Case {
            current: [1440, 1000, 1000, 1440],
            target_v_set: 1440,
            target_ovp: 1440,
            accepted: true,
        },
        // Device already violates V-SET <= S-OVP (live 50.00 V against a
        // 20.00 V threshold). Both crossing conditions hold at once here:
        // staging the new 40.00 V S-OVP first would have left the old 50.00 V
        // V-SET above it and tripped OVP, so this must be rejected outright.
        Case {
            current: [5000, 1000, 1000, 2000],
            target_v_set: 3000,
            target_ovp: 4000,
            accepted: false,
        },
    ];

    for case in cases {
        let p = GroupParams {
            setpoints: Setpoints {
                v_set: case.target_v_set as f32 / 100.0,
                i_set: 10.00,
            },
            safety_limits: SafetyLimits {
                lvp_v: 10.00,
                ovp_v: case.target_ovp as f32 / 100.0,
                ocp_a: 12.50,
            },
            s_opp_w: 1800.0,
            s_ohp_h: 0,
            s_ohp_m: 0,
            s_oah_ah: 0.0,
            s_owh_wh: 0.0,
            s_otp: Temperature {
                value: 95.0,
                unit: TempUnit::Celsius,
            },
            power_on_output: false,
        };
        let values = vec![
            case.target_v_set,
            1000,
            1000,
            case.target_ovp,
            1250,
            1800,
            0,
            0,
            0,
            0,
            0,
            0,
            95,
            0,
        ];
        let mut script = vec![
            Op::Read {
                addr: Reg::TempUnit.addr(),
                values: vec![0],
            },
            Op::Read {
                addr: GROUP_BASE,
                values: case.current.to_vec(),
            },
        ];
        if case.accepted {
            script.push(Op::WriteMany {
                addr: regs::group_addr(0),
                values: values.clone(),
            });
            script.push(Op::Read {
                addr: regs::group_addr(0),
                values,
            });
        }

        // A rejected case scripts only the two reads, so the mock's
        // drop-time check proves nothing was written.
        let mut xy = Xy::new(MockTransport::new(script));
        let result = xy.write_group(0, &p);
        if case.accepted {
            assert_eq!(result.unwrap(), p, "{case:?}");
        } else {
            assert_eq!(
                result,
                Err(XyError::Input(InputError::VoltageSetpointAboveProtection)),
                "{case:?}"
            );
        }
    }
}

#[test]
fn baud_round_trip() {
    for baud in [
        BaudRate::B2400,
        BaudRate::B4800,
        BaudRate::B9600,
        BaudRate::B14400,
        BaudRate::B19200,
        BaudRate::B38400,
        BaudRate::B56000,
        BaudRate::B57600,
        BaudRate::B115200,
    ] {
        assert_eq!(BaudRate::from_code(baud.code()), Ok(baud));
    }
    assert_eq!(BaudRate::from_code(99), Err(99));
    assert_eq!(BaudRate::B9600.baud(), 9600);
}

#[test]
fn group_encode_decode_round_trip() {
    // Pin all 14 register offsets in one go — an offset swap would
    // surface as a field mismatch here.
    // S-OAH raw = (2<<16) | 500 = 131_572 → 131.572 Ah (scale 1000).
    // S-OWH raw = 12_345 → 123.45 Wh (scale 100).
    let p = GroupParams {
        setpoints: Setpoints {
            v_set: 14.40,
            i_set: 10.00,
        },
        safety_limits: SafetyLimits {
            lvp_v: 10.00,
            ovp_v: 15.00,
            ocp_a: 12.50,
        },
        s_opp_w: 1800.0,
        s_ohp_h: 7,
        s_ohp_m: 30,
        s_oah_ah: 131.572,
        s_owh_wh: 123.45,
        s_otp: Temperature {
            value: 95.0,
            unit: TempUnit::Celsius,
        },
        power_on_output: true,
    };
    let regs = encode_group(&p)
        .unwrap()
        .for_unit(TempUnit::Celsius)
        .unwrap();
    // Pin the encoded oah/owh register pair layout (low, high).
    assert_eq!(regs[8..12], [500, 2, 12_345, 0]);
    let decoded = decode_group(&regs, regs::group_addr(0), TempUnit::Celsius).unwrap();
    assert_eq!(decoded.setpoints, p.setpoints);
    assert_eq!(decoded.safety_limits, p.safety_limits);
    assert_eq!(decoded.s_opp_w, p.s_opp_w);
    assert_eq!(decoded.s_ohp_h, p.s_ohp_h);
    assert_eq!(decoded.s_ohp_m, p.s_ohp_m);
    assert_eq!(decoded.s_oah_ah, p.s_oah_ah);
    assert_eq!(decoded.s_owh_wh, p.s_owh_wh);
    assert_eq!(decoded.s_otp, p.s_otp);
    assert_eq!(decoded.power_on_output, p.power_on_output);

    let fahrenheit_regs = encode_group(&p)
        .unwrap()
        .for_unit(TempUnit::Fahrenheit)
        .unwrap();
    assert_eq!(fahrenheit_regs[12], 203);
    assert_ne!(fahrenheit_regs[12], regs[12]);

    let mut fahrenheit_limit = p;
    fahrenheit_limit.s_otp = Temperature {
        value: 230.0,
        unit: TempUnit::Fahrenheit,
    };
    assert_eq!(
        encode_group(&fahrenheit_limit)
            .unwrap()
            .for_unit(TempUnit::Celsius)
            .unwrap()[12],
        110
    );
    fahrenheit_limit.s_otp.value = 230.1;
    assert_eq!(
        encode_group(&fahrenheit_limit)
            .unwrap()
            .for_unit(TempUnit::Fahrenheit),
        Err(InputError::OutOfRange {
            field: InputField::OverTemperatureProtection
        })
    );

    let mut source_unit_outlier = p;
    source_unit_outlier.s_otp.value = -1.0;
    assert_eq!(
        encode_group(&source_unit_outlier)
            .unwrap()
            .for_unit(TempUnit::Fahrenheit)
            .unwrap()[12],
        30
    );
    assert_eq!(
        encode_group(&source_unit_outlier)
            .unwrap()
            .for_unit(TempUnit::Celsius),
        Err(InputError::OutOfRange {
            field: InputField::OverTemperatureProtection
        })
    );
}

/// Pins (register address, raw value) for each one-shot setter. A wrong
/// REG_* constant or a wrong scale would surface here. Inlined per-call
/// rather than table-driven to keep `no_std` (no `Box`) and let each row
/// take its own typed argument.
#[test]
fn one_shot_setters_use_correct_addr_and_value() {
    macro_rules! check {
        ($addr:expr, $value:expr, $action:expr) => {{
            let mock = MockTransport::new(vec![Op::WriteOne {
                addr: $addr,
                value: $value,
            }]);
            let mut xy = Xy::new(mock);
            $action(&mut xy).unwrap();
        }};
        ($addr:expr, $value:expr, $action:expr, guard $guard:expr => $raw:expr) => {{
            let mock = MockTransport::new(vec![
                Op::Read {
                    addr: $guard,
                    values: vec![$raw],
                },
                Op::WriteOne {
                    addr: $addr,
                    value: $value,
                },
            ]);
            let mut xy = Xy::new(mock);
            $action(&mut xy).unwrap();
        }};
    }
    check!(Reg::VSet.addr(), 1440, |x: &mut Xy<_>| x.set_voltage(14.40), guard Reg::SOvp.addr() => 7200);
    check!(Reg::ISet.addr(), 500, |x: &mut Xy<_>| x
        .set_current_limit(5.00));
    check!(Reg::OutputEn.addr(), 1, |x: &mut Xy<_>| x.set_output(true));
    check!(Reg::OutputEn.addr(), 0, |x: &mut Xy<_>| x.set_output(false));
    check!(Reg::Protect.addr(), 0, |x: &mut Xy<_>| x
        .clear_protection_status());
    check!(Reg::Lock.addr(), 1, |x: &mut Xy<_>| x.set_lock(true));
    check!(Reg::Backlight.addr(), 3, |x: &mut Xy<_>| x.set_backlight(3));
    check!(Reg::Sleep.addr(), 9, |x: &mut Xy<_>| x.set_sleep_minutes(9));
    check!(Reg::Buzzer.addr(), 1, |x: &mut Xy<_>| x.set_buzzer(true));
    check!(Reg::SlaveAddr.addr(), 7, |x: &mut Xy<_>| x
        .set_slave_address(7));
    check!(Reg::SIni.addr(), 1, |x: &mut Xy<_>| x
        .set_power_on_output(true));
    check!(Reg::ExtractM.addr(), 3, |x: &mut Xy<_>| x.recall_group(3));
    check!(Reg::BaudCode.addr(), 6, |x: &mut Xy<_>| x
        .set_baud_rate(BaudRate::B115200));
    check!(Reg::TempUnit.addr(), 1, |x: &mut Xy<_>| x
        .set_temp_unit(TempUnit::Fahrenheit));
}

/// Pins (register address, returned raw, expected decoded value) for each
/// single-register getter. A wrong REG_* constant or scale would surface here.
#[test]
fn one_shot_getters_use_correct_addr_and_scale() {
    macro_rules! check {
        ($addr:expr, $raw:expr, $action:expr, $expected:expr) => {{
            let mock = MockTransport::new(vec![Op::Read {
                addr: $addr,
                values: vec![$raw],
            }]);
            let mut xy = Xy::new(mock);
            assert_eq!($action(&mut xy).unwrap(), $expected);
        }};
    }
    check!(
        Reg::VOut.addr(),
        1234,
        |x: &mut Xy<_>| x.read_voltage_out(),
        12.34
    );
    check!(
        Reg::IOut.addr(),
        500,
        |x: &mut Xy<_>| x.read_current_out(),
        5.00
    );
    check!(
        Reg::POut.addr(),
        675,
        |x: &mut Xy<_>| x.read_power_out(),
        67.5
    );
    check!(
        Reg::VIn.addr(),
        2400,
        |x: &mut Xy<_>| x.read_voltage_in(),
        24.00
    );
    check!(
        Reg::OutputEn.addr(),
        1,
        |x: &mut Xy<_>| x.read_output(),
        true
    );
    check!(Reg::Lock.addr(), 1, |x: &mut Xy<_>| x.read_lock(), true);
    check!(
        Reg::Backlight.addr(),
        4,
        |x: &mut Xy<_>| x.read_backlight(),
        4u8
    );
    check!(
        Reg::Sleep.addr(),
        9,
        |x: &mut Xy<_>| x.read_sleep_minutes(),
        9u16
    );
    check!(
        Reg::Buzzer.addr(),
        0,
        |x: &mut Xy<_>| x.read_buzzer(),
        false
    );
    check!(
        Reg::SlaveAddr.addr(),
        7,
        |x: &mut Xy<_>| x.read_slave_address(),
        7u8
    );
    check!(
        Reg::SIni.addr(),
        1,
        |x: &mut Xy<_>| x.read_power_on_output(),
        true
    );
    check!(
        Reg::Model.addr(),
        0x6500,
        |x: &mut Xy<_>| x.read_model(),
        0x6500u16
    );
    check!(
        Reg::Version.addr(),
        0x71,
        |x: &mut Xy<_>| x.read_version(),
        0x71u16
    );
    check!(
        Reg::TInOffset.addr(),
        15,
        |x: &mut Xy<_>| x.read_temp_offset_internal(),
        1.5
    );
    // 0xFFE7 = -25 as i16 → -2.5 °C; signed decoding must not read 6551.1.
    check!(
        Reg::TInOffset.addr(),
        0xFFE7,
        |x: &mut Xy<_>| x.read_temp_offset_internal(),
        -2.5
    );
    check!(
        Reg::TExOffset.addr(),
        20,
        |x: &mut Xy<_>| x.read_temp_offset_external(),
        2.0
    );
    check!(
        Reg::Cvcc.addr(),
        0,
        |x: &mut Xy<_>| x.read_reg_mode(),
        RegMode::ConstantVoltage
    );
    check!(
        Reg::Cvcc.addr(),
        1,
        |x: &mut Xy<_>| x.read_reg_mode(),
        RegMode::ConstantCurrent
    );
    check!(
        Reg::BaudCode.addr(),
        6,
        |x: &mut Xy<_>| x.read_baud_rate(),
        BaudRate::B115200
    );

    // 2-reg bulk reads.
    let mut xy = Xy::new(MockTransport::new(vec![Op::Read {
        addr: Reg::VSet.addr(),
        values: vec![1440, 1000],
    }]));
    let s = xy.read_setpoints().unwrap();
    assert_eq!((s.v_set, s.i_set), (14.40, 10.00));

    let mut xy = Xy::new(MockTransport::new(vec![Op::Read {
        addr: Reg::TIn.addr(),
        values: vec![345, 256, 0, 0, 0, 0, 1],
    }]));
    let t = xy.read_temperature_internal().unwrap();
    assert_eq!(
        t,
        Temperature {
            value: 34.5,
            unit: TempUnit::Fahrenheit,
        }
    );

    // T-IN is two's-complement signed. 0xFFCE = -50 as i16 -> -5.0 degC;
    // unsigned decoding would read 65486 / 10 = 6548.6.
    let mut xy = Xy::new(MockTransport::new(vec![Op::Read {
        addr: Reg::TIn.addr(),
        values: vec![0xFFCE, 0, 0, 0, 0, 0, 0],
    }]));
    assert_eq!(
        xy.read_temperature_internal().unwrap(),
        Temperature {
            value: -5.0,
            unit: TempUnit::Celsius,
        }
    );

    // Both ends of the i16 range: 0x8000 = -32768 -> -3276.8,
    // 0x7FFF = 32767 -> 3276.7.
    let mut xy = Xy::new(MockTransport::new(vec![
        Op::Read {
            addr: Reg::TIn.addr(),
            values: vec![0x8000, 0, 0, 0, 0, 0, 0],
        },
        Op::Read {
            addr: Reg::TIn.addr(),
            values: vec![0x7FFF, 0, 0, 0, 0, 0, 0],
        },
    ]));
    assert_eq!(xy.read_temperature_internal().unwrap().value, -3276.8);
    assert_eq!(xy.read_temperature_internal().unwrap().value, 3276.7);
}

#[test]
fn invalid_register_values_are_not_normalized() {
    let mut invalid_group_bool = vec![0; GROUP_LEN];
    invalid_group_bool[13] = 2;
    let mut invalid_group_hours = vec![0; GROUP_LEN];
    invalid_group_hours[6] = 100;
    let mut invalid_group_minutes = vec![0; GROUP_LEN];
    invalid_group_minutes[7] = 60;
    let mut invalid_output_status = status_fixture([0; 6]);
    invalid_output_status[Reg::OutputEn.addr() as usize] = 2;
    let mut invalid_protection_status = status_fixture([0; 6]);
    invalid_protection_status[Reg::Protect.addr() as usize] = 11;
    let mut invalid_reg_mode_status = status_fixture([0; 6]);
    invalid_reg_mode_status[Reg::Cvcc.addr() as usize] = 2;
    let mut xy = Xy::new(MockTransport::new(vec![
        Op::Read {
            addr: Reg::OutputEn.addr(),
            values: vec![2],
        },
        Op::Read {
            addr: Reg::Backlight.addr(),
            values: vec![6],
        },
        Op::Read {
            addr: Reg::Sleep.addr(),
            values: vec![10],
        },
        Op::Read {
            addr: Reg::Protect.addr(),
            values: vec![11],
        },
        Op::Read {
            addr: Reg::Cvcc.addr(),
            values: vec![2],
        },
        Op::Read {
            addr: Reg::TempUnit.addr(),
            values: vec![2],
        },
        Op::Read {
            addr: Reg::BaudCode.addr(),
            values: vec![9],
        },
        Op::Read {
            addr: Reg::TempUnit.addr(),
            values: vec![0],
        },
        Op::Read {
            addr: regs::group_addr(1),
            values: invalid_group_bool,
        },
        Op::Read {
            addr: Reg::TempUnit.addr(),
            values: vec![0],
        },
        Op::Read {
            addr: regs::group_addr(2),
            values: invalid_group_hours,
        },
        Op::Read {
            addr: Reg::TempUnit.addr(),
            values: vec![0],
        },
        Op::Read {
            addr: regs::group_addr(3),
            values: invalid_group_minutes,
        },
        Op::Read {
            addr: Reg::AhLow.addr(),
            values: vec![0, 0, 0, 0, 1, 60, 0],
        },
        Op::Read {
            addr: Reg::AhLow.addr(),
            values: vec![0, 0, 0, 0, 1, 0, 60],
        },
        Op::Read {
            addr: Reg::TIn.addr(),
            values: vec![0, 8888, 0, 0, 0, 0, 2],
        },
        Op::Read {
            addr: Reg::VSet.addr(),
            values: invalid_output_status,
        },
        Op::Read {
            addr: Reg::VSet.addr(),
            values: invalid_protection_status,
        },
        Op::Read {
            addr: Reg::VSet.addr(),
            values: invalid_reg_mode_status,
        },
    ]));

    for (result, register, value) in [
        (xy.read_output().map(|_| ()), Reg::OutputEn.addr(), 2),
        (xy.read_backlight().map(|_| ()), Reg::Backlight.addr(), 6),
        (xy.read_sleep_minutes().map(|_| ()), Reg::Sleep.addr(), 10),
        (
            xy.read_protection_status().map(|_| ()),
            Reg::Protect.addr(),
            11,
        ),
        (xy.read_reg_mode().map(|_| ()), Reg::Cvcc.addr(), 2),
        (xy.read_temp_unit().map(|_| ()), Reg::TempUnit.addr(), 2),
        (xy.read_baud_rate().map(|_| ()), Reg::BaudCode.addr(), 9),
        (xy.read_group(1).map(|_| ()), regs::group_addr(1) + 13, 2),
        (xy.read_group(2).map(|_| ()), regs::group_addr(2) + 6, 100),
        (xy.read_group(3).map(|_| ()), regs::group_addr(3) + 7, 60),
        (xy.read_totals().map(|_| ()), Reg::OutM.addr(), 60),
        (xy.read_totals().map(|_| ()), Reg::OutS.addr(), 60),
        (
            xy.read_temperature_internal().map(|_| ()),
            Reg::TempUnit.addr(),
            2,
        ),
        (xy.read_status().map(|_| ()), Reg::OutputEn.addr(), 2),
        (xy.read_status().map(|_| ()), Reg::Protect.addr(), 11),
        (xy.read_status().map(|_| ()), Reg::Cvcc.addr(), 2),
    ] {
        assert_eq!(
            result,
            Err(XyError::InvalidRegisterValue { register, value })
        );
    }
}

#[test]
fn temp_unit_round_trip() {
    // Write Celsius, read back Celsius (0); same for Fahrenheit (1).
    let mock = MockTransport::new(vec![
        Op::WriteOne {
            addr: Reg::TempUnit.addr(),
            value: 0,
        },
        Op::Read {
            addr: Reg::TempUnit.addr(),
            values: vec![0],
        },
        Op::WriteOne {
            addr: Reg::TempUnit.addr(),
            value: 1,
        },
        Op::Read {
            addr: Reg::TempUnit.addr(),
            values: vec![1],
        },
    ]);
    let mut xy = Xy::new(mock);
    xy.set_temp_unit(TempUnit::Celsius).unwrap();
    assert_eq!(xy.read_temp_unit().unwrap(), TempUnit::Celsius);
    xy.set_temp_unit(TempUnit::Fahrenheit).unwrap();
    assert_eq!(xy.read_temp_unit().unwrap(), TempUnit::Fahrenheit);
}

#[test]
fn rtu_error_propagates() {
    struct FailRead;
    impl ModbusTransport for FailRead {
        fn read_holding(&mut self, _: u8, _: u16, _: &mut [u16]) -> Result<(), RtuError> {
            Err(RtuError::Modbus(ModbusError::BadCrc))
        }
        fn write_single_holding(&mut self, _: u8, _: u16, _: u16) -> Result<(), RtuError> {
            unreachable!()
        }
        fn write_multiple_holdings(&mut self, _: u8, _: u16, _: &[u16]) -> Result<(), RtuError> {
            unreachable!()
        }
    }
    let mut xy = Xy::new(FailRead);
    assert!(matches!(
        xy.read_voltage_out(),
        Err(XyError::Rtu(RtuError::Modbus(ModbusError::BadCrc)))
    ));
}

#[test]
fn invalid_inputs_fail_before_transport_io() {
    assert_eq!(
        Xy::with_slave(MockTransport::new(vec![]), 0).unwrap_err(),
        InputError::InvalidSlaveAddress { address: 0 }
    );

    let mut xy = Xy::new(MockTransport::new(vec![]));
    assert_eq!(
        xy.set_protection(SafetyLimits {
            lvp_v: 9.99,
            ovp_v: 15.0,
            ocp_a: 12.5,
        }),
        Err(XyError::Input(InputError::OutOfRange {
            field: InputField::LowVoltageProtection
        }))
    );
    for result in [
        xy.set_voltage(f32::NAN),
        xy.set_voltage(f32::INFINITY),
        xy.set_voltage(70.01),
        xy.set_current_limit(25.01),
        xy.set_backlight(0),
        xy.set_backlight(6),
        xy.set_sleep_minutes(10),
        xy.set_slave_address(0),
        xy.set_slave_address(248),
        xy.recall_group(10),
    ] {
        assert!(matches!(result, Err(XyError::Input(_))));
    }
}

#[test]
fn register_conversions_preserve_precision_and_physical_ranges() {
    // Signedness and scale come from the register, not the call site: the same
    // raw word decodes differently depending on which register it came from.
    assert_eq!(decode_scaled(Reg::TIn, 0xFFCE), -5.0);
    assert_eq!(decode_scaled(Reg::VOut, 0xFFCE), 654.86);
    assert_ne!(
        decode_scaled(Reg::TIn, 0xFFCE),
        decode_scaled(Reg::VOut, 0xFFCE)
    );
    // Same word, same 10.0 scale, but P-OUT is unsigned and T-IN is not.
    assert_eq!(decode_scaled(Reg::POut, 0xFFCE), 6548.6);
    assert_eq!(decode_scaled(Reg::TIn, 0xFFCE), -5.0);

    // The two 32-bit pairs at the scales and ceilings the table declares:
    // charge 9999 Ah at 1000, energy 4_200_000 Wh at 100.
    for (low, scale, raws) in [
        (
            Reg::SOahLow,
            1000.0f64,
            [0u32, 1, 1_234_567, 9_999_000].as_slice(),
        ),
        (
            Reg::SOwhLow,
            100.0f64,
            [0u32, 1, 16_777_217, 420_000_000].as_slice(),
        ),
    ] {
        for &raw in raws {
            let value = decode_wide(low, raw as u16, (raw >> 16) as u16);
            assert_eq!(value, raw as f64 / scale, "{low:?} raw={raw}");
            let words = to_reg_wide(low, value).unwrap();
            let round_trip = ((words.high as u32) << 16) | words.low as u32;
            assert_eq!(round_trip, raw, "{low:?} raw={raw}");
        }
    }

    // Every shipped limit sits on the raw grid, so its ceiling encodes exactly.
    assert_eq!(to_reg(Reg::VSet, 70.0), Ok(7000));
    assert_eq!(to_reg(Reg::SLvp, 10.0), Ok(1000));
    assert_eq!(to_reg(Reg::SOpp, 2000.0), Ok(2000));
    assert_eq!(to_reg(Reg::SOcp, 27.0), Ok(2700));

    // A bound that is *not* grid-aligned must still be honoured after
    // rounding. 0.0078125 * 100 rounds to raw 1, which represents 0.01.
    assert_eq!(
        to_reg_within(Reg::VSet, 0.0078125, 0.0, 0.0078125),
        Err(InputError::OutOfRange {
            field: InputField::VoltageSetpoint
        })
    );
    // 0.004 * 100 rounds to raw 0, which represents 0.0 — below the floor.
    assert_eq!(
        to_reg_within(Reg::VSet, 0.004, 0.004, 1.0),
        Err(InputError::OutOfRange {
            field: InputField::VoltageSetpoint
        })
    );
}

#[test]
fn verify_scale_family_compatible_for_xy7025() {
    let mut xy = Xy::new(MockTransport::new(vec![
        Op::Read {
            addr: Reg::Model.addr(),
            values: vec![0x6500],
        },
        Op::Read {
            addr: Reg::Model.addr(),
            values: vec![0x6100],
        },
    ]));
    assert_eq!(
        xy.verify_scale_family().unwrap(),
        ScaleCheck::Compatible {
            device_code: 0x6500
        }
    );
    assert_eq!(
        xy.verify_scale_family().unwrap(),
        ScaleCheck::Compatible {
            device_code: 0x6100
        }
    );
}

#[test]
fn verify_scale_family_is_inconclusive_for_unknown_code() {
    let mut xy = Xy::new(MockTransport::new(vec![Op::Read {
        addr: Reg::Model.addr(),
        values: vec![0x7700],
    }]));
    assert_eq!(
        xy.verify_scale_family().unwrap(),
        ScaleCheck::Inconclusive {
            device_code: 0x7700
        }
    );
}
