extern crate std;
use core::num::NonZeroU16;
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

fn custom_model(current_scale: u16, power_scale: u16, opp_scale: u16) -> Model {
    Model::Custom {
        current_scale: NonZeroU16::new(current_scale).unwrap(),
        power_scale: NonZeroU16::new(power_scale).unwrap(),
        opp_scale: NonZeroU16::new(opp_scale).unwrap(),
    }
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

/// Same wire bytes decoded under XY7025 vs a Custom (SK-style) scale family
/// must yield 10× different physical values for I-SET, IOUT, S-OCP and POWER.
/// Locks in that `Model` actually changes behavior — a no-op `current_scale`
/// would silently report the same numbers.
#[test]
fn model_scales_diverge_between_xy7025_and_sk_custom() {
    // 1000 raw with /100 → 10.00 A, with /1000 → 1.000 A.
    // 675 raw with /10 → 67.5 W, with /100 → 6.75 W.
    let regs = [1440, 1000, 1350, 1000, 675, 2400];
    let xy7025_mock = MockTransport::new(vec![Op::Read {
        addr: REG_V_SET,
        values: status_fixture(regs),
    }]);
    let mut xy = Xy::new(xy7025_mock, Model::Xy7025);
    let s = xy.read_status().unwrap();
    assert_eq!(s.i_set, 10.00);
    assert_eq!(s.i_out, 10.00);
    assert_eq!(s.p_out, 67.5);

    let sk_mock = MockTransport::new(vec![Op::Read {
        addr: REG_V_SET,
        values: status_fixture(regs),
    }]);
    let mut xy = Xy::new(sk_mock, custom_model(1000, 100, 10));
    let s = xy.read_status().unwrap();
    assert_eq!(s.i_set, 1.000);
    assert_eq!(s.i_out, 1.000);
    assert_eq!(s.p_out, 6.75);

    // V_SET, V_OUT, V_IN scales are model-invariant (always /100).
    assert_eq!(s.v_set, 14.40);
    assert_eq!(s.v_out, 13.50);
    assert_eq!(s.v_in, 24.00);
}

/// `Model::Custom` lets users dial in scales for hardware not covered
/// by the preset variants. Verify the three scale getters route
/// through the supplied values verbatim.
#[test]
fn custom_model_routes_user_supplied_scales() {
    let m = custom_model(500, 25, 4);
    assert_eq!(m.current_scale(), 500.0);
    assert_eq!(m.power_scale(), 25.0);
    assert_eq!(m.opp_scale(), 4.0);
}

#[test]
fn set_voltage_scales_correctly() {
    // 14.40 V → 1440 raw.
    let mock = MockTransport::new(vec![Op::WriteOne {
        addr: REG_V_SET,
        value: 1440,
    }]);
    let mut xy = Xy::new(mock, Model::Xy7025);
    xy.set_voltage(14.40).unwrap();
}

#[test]
fn set_protection_uses_bulk_write() {
    // LVP=10.00, OVP=15.00, OCP=12.50 → raw 1000, 1500, 1250.
    let mock = MockTransport::new(vec![Op::WriteMany {
        addr: REG_S_LVP,
        values: vec![1000, 1500, 1250],
    }]);
    let mut xy = Xy::new(mock, Model::Xy7025);
    xy.set_protection(SafetyLimits {
        lvp_v: 10.0,
        ovp_v: 15.0,
        ocp_a: 12.5,
    })
    .unwrap();
}

#[test]
fn read_protection_decodes_three_regs() {
    let mock = MockTransport::new(vec![Op::Read {
        addr: REG_S_LVP,
        values: vec![1000, 1500, 1250],
    }]);
    let mut xy = Xy::new(mock, Model::Xy7025);
    let l = xy.read_protection().unwrap();
    assert_eq!(l.lvp_v, 10.0);
    assert_eq!(l.ovp_v, 15.0);
    assert_eq!(l.ocp_a, 12.5);
}

#[test]
fn protection_status_decodes_known_and_rejects_unknown_codes() {
    let mock = MockTransport::new(vec![
        Op::Read {
            addr: REG_PROTECT,
            values: vec![0],
        },
        Op::Read {
            addr: REG_PROTECT,
            values: vec![4],
        },
        Op::Read {
            addr: REG_PROTECT,
            values: vec![99],
        },
    ]);
    let mut xy = Xy::new(mock, Model::Xy7025);
    assert_eq!(
        xy.read_protection_status().unwrap(),
        ProtectionStatus::Normal
    );
    assert_eq!(xy.read_protection_status().unwrap(), ProtectionStatus::Lvp);
    assert_eq!(
        xy.read_protection_status(),
        Err(XyError::InvalidRegisterValue {
            register: REG_PROTECT,
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
        addr: REG_V_SET,
        values: values.to_vec(),
    }]);
    let mut xy = Xy::new(mock, Model::Xy7025);
    let s = xy.read_status().unwrap();
    assert_eq!(s.v_set, 14.40);
    assert_eq!(s.i_set, 10.00);
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
        addr: REG_AH_LOW,
        values: vec![500, 2, 12345, 0, 1, 23, 45],
    }]);
    let mut xy = Xy::new(mock, Model::Xy7025);
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
    let mock = MockTransport::new(vec![Op::Read {
        addr: group_addr(1),
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
    }]);
    let mut xy = Xy::new(mock, Model::Xy7025);
    let g = xy.read_group(1).unwrap();
    assert_eq!(g.setpoints.v_set, 14.40);
    assert_eq!(g.safety_limits.ovp_v, 15.00);
    assert_eq!(g.s_opp_w, 1800.0);
    assert_eq!(g.s_oah_ah, 0.0);
    assert_eq!(g.s_owh_wh, 0.0);
    assert_eq!(g.s_otp, 95.0);
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
        s_otp: 95.0,
        power_on_output: true,
    };
    let mock = MockTransport::new(vec![Op::WriteMany {
        addr: group_addr(2),
        values: vec![1440, 1000, 1000, 1500, 1250, 1800, 0, 0, 0, 0, 0, 0, 95, 1],
    }]);
    let mut xy = Xy::new(mock, Model::Xy7025);
    xy.write_group(2, &p).unwrap();
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
        s_otp: 95.0,
        power_on_output: true,
    };
    let regs = encode_group(&p, Model::Xy7025).unwrap();
    // Pin the encoded oah/owh register pair layout (low, high).
    assert_eq!(regs[8..12], [500, 2, 12_345, 0]);
    let decoded = decode_group(&regs, Model::Xy7025, group_addr(0)).unwrap();
    assert_eq!(decoded.setpoints, p.setpoints);
    assert_eq!(decoded.safety_limits, p.safety_limits);
    assert_eq!(decoded.s_opp_w, p.s_opp_w);
    assert_eq!(decoded.s_ohp_h, p.s_ohp_h);
    assert_eq!(decoded.s_ohp_m, p.s_ohp_m);
    assert_eq!(decoded.s_oah_ah, p.s_oah_ah);
    assert_eq!(decoded.s_owh_wh, p.s_owh_wh);
    assert_eq!(decoded.s_otp, p.s_otp);
    assert_eq!(decoded.power_on_output, p.power_on_output);
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
            let mut xy = Xy::new(mock, Model::Xy7025);
            $action(&mut xy).unwrap();
        }};
    }
    check!(REG_V_SET, 1440, |x: &mut Xy<_>| x.set_voltage(14.40));
    check!(REG_I_SET, 500, |x: &mut Xy<_>| x.set_current_limit(5.00));
    check!(REG_OUTPUT_EN, 1, |x: &mut Xy<_>| x.set_output(true));
    check!(REG_OUTPUT_EN, 0, |x: &mut Xy<_>| x.set_output(false));
    check!(REG_PROTECT, 0, |x: &mut Xy<_>| x.clear_protection_status());
    check!(REG_LOCK, 1, |x: &mut Xy<_>| x.set_lock(true));
    check!(REG_BACKLIGHT, 3, |x: &mut Xy<_>| x.set_backlight(3));
    check!(REG_SLEEP, 9, |x: &mut Xy<_>| x.set_sleep_minutes(9));
    check!(REG_BUZZER, 1, |x: &mut Xy<_>| x.set_buzzer(true));
    check!(REG_SLAVE_ADDR, 7, |x: &mut Xy<_>| x.set_slave_address(7));
    check!(REG_S_INI, 1, |x: &mut Xy<_>| x.set_power_on_output(true));
    check!(REG_EXTRACT_M, 3, |x: &mut Xy<_>| x.recall_group(3));
    check!(REG_BAUD_CODE, 6, |x: &mut Xy<_>| x
        .set_baud_rate(BaudRate::B115200));
    check!(REG_TEMP_UNIT, 1, |x: &mut Xy<_>| x
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
            let mut xy = Xy::new(mock, Model::Xy7025);
            assert_eq!($action(&mut xy).unwrap(), $expected);
        }};
    }
    check!(REG_V_OUT, 1234, |x: &mut Xy<_>| x.read_voltage_out(), 12.34);
    check!(REG_I_OUT, 500, |x: &mut Xy<_>| x.read_current_out(), 5.00);
    check!(REG_P_OUT, 675, |x: &mut Xy<_>| x.read_power_out(), 67.5);
    check!(REG_V_IN, 2400, |x: &mut Xy<_>| x.read_voltage_in(), 24.00);
    check!(REG_OUTPUT_EN, 1, |x: &mut Xy<_>| x.read_output(), true);
    check!(REG_LOCK, 1, |x: &mut Xy<_>| x.read_lock(), true);
    check!(REG_BACKLIGHT, 4, |x: &mut Xy<_>| x.read_backlight(), 4u8);
    check!(REG_SLEEP, 9, |x: &mut Xy<_>| x.read_sleep_minutes(), 9u16);
    check!(REG_BUZZER, 0, |x: &mut Xy<_>| x.read_buzzer(), false);
    check!(
        REG_SLAVE_ADDR,
        7,
        |x: &mut Xy<_>| x.read_slave_address(),
        7u8
    );
    check!(REG_S_INI, 1, |x: &mut Xy<_>| x.read_power_on_output(), true);
    check!(REG_MODEL, 0x6500, |x: &mut Xy<_>| x.read_model(), 0x6500u16);
    check!(REG_VERSION, 0x71, |x: &mut Xy<_>| x.read_version(), 0x71u16);
    check!(
        REG_T_IN_OFFSET,
        15,
        |x: &mut Xy<_>| x.read_temp_offset_internal(),
        1.5
    );
    // 0xFFE7 = -25 as i16 → -2.5 °C; signed decoding must not read 6551.1.
    check!(
        REG_T_IN_OFFSET,
        0xFFE7,
        |x: &mut Xy<_>| x.read_temp_offset_internal(),
        -2.5
    );
    check!(
        REG_T_EX_OFFSET,
        20,
        |x: &mut Xy<_>| x.read_temp_offset_external(),
        2.0
    );
    check!(
        REG_CVCC,
        0,
        |x: &mut Xy<_>| x.read_reg_mode(),
        RegMode::ConstantVoltage
    );
    check!(
        REG_CVCC,
        1,
        |x: &mut Xy<_>| x.read_reg_mode(),
        RegMode::ConstantCurrent
    );
    check!(
        REG_BAUD_CODE,
        6,
        |x: &mut Xy<_>| x.read_baud_rate(),
        BaudRate::B115200
    );

    // 2-reg bulk reads.
    let mut xy = Xy::new(
        MockTransport::new(vec![Op::Read {
            addr: REG_V_SET,
            values: vec![1440, 1000],
        }]),
        Model::Xy7025,
    );
    let s = xy.read_setpoints().unwrap();
    assert_eq!((s.v_set, s.i_set), (14.40, 10.00));

    let mut xy = Xy::new(
        MockTransport::new(vec![Op::Read {
            addr: REG_T_IN,
            values: vec![345, 256],
        }]),
        Model::Xy7025,
    );
    let t = xy.read_temperatures().unwrap();
    assert_eq!(t.internal, 34.5);
    assert_eq!(t.external, Some(25.6));

    let mut xy = Xy::new(
        MockTransport::new(vec![Op::Read {
            addr: REG_T_IN,
            values: vec![345, 8888],
        }]),
        Model::Xy7025,
    );
    assert_eq!(xy.read_temperatures().unwrap().external, None);
}

#[test]
fn invalid_register_values_are_not_normalized() {
    let mut group = vec![0; GROUP_LEN as usize];
    group[13] = 2;
    let mut invalid_output_status = status_fixture([0; 6]);
    invalid_output_status[REG_OUTPUT_EN as usize] = 2;
    let mut invalid_protection_status = status_fixture([0; 6]);
    invalid_protection_status[REG_PROTECT as usize] = 11;
    let mut invalid_reg_mode_status = status_fixture([0; 6]);
    invalid_reg_mode_status[REG_CVCC as usize] = 2;
    let mut xy = Xy::new(
        MockTransport::new(vec![
            Op::Read {
                addr: REG_OUTPUT_EN,
                values: vec![2],
            },
            Op::Read {
                addr: REG_BACKLIGHT,
                values: vec![6],
            },
            Op::Read {
                addr: REG_SLEEP,
                values: vec![10],
            },
            Op::Read {
                addr: REG_PROTECT,
                values: vec![11],
            },
            Op::Read {
                addr: REG_CVCC,
                values: vec![2],
            },
            Op::Read {
                addr: REG_TEMP_UNIT,
                values: vec![2],
            },
            Op::Read {
                addr: REG_BAUD_CODE,
                values: vec![9],
            },
            Op::Read {
                addr: group_addr(1),
                values: group,
            },
            Op::Read {
                addr: REG_V_SET,
                values: invalid_output_status,
            },
            Op::Read {
                addr: REG_V_SET,
                values: invalid_protection_status,
            },
            Op::Read {
                addr: REG_V_SET,
                values: invalid_reg_mode_status,
            },
        ]),
        Model::Xy7025,
    );

    for (result, register, value) in [
        (xy.read_output().map(|_| ()), REG_OUTPUT_EN, 2),
        (xy.read_backlight().map(|_| ()), REG_BACKLIGHT, 6),
        (xy.read_sleep_minutes().map(|_| ()), REG_SLEEP, 10),
        (xy.read_protection_status().map(|_| ()), REG_PROTECT, 11),
        (xy.read_reg_mode().map(|_| ()), REG_CVCC, 2),
        (xy.read_temp_unit().map(|_| ()), REG_TEMP_UNIT, 2),
        (xy.read_baud_rate().map(|_| ()), REG_BAUD_CODE, 9),
        (xy.read_group(1).map(|_| ()), group_addr(1) + 13, 2),
        (xy.read_status().map(|_| ()), REG_OUTPUT_EN, 2),
        (xy.read_status().map(|_| ()), REG_PROTECT, 11),
        (xy.read_status().map(|_| ()), REG_CVCC, 2),
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
            addr: REG_TEMP_UNIT,
            value: 0,
        },
        Op::Read {
            addr: REG_TEMP_UNIT,
            values: vec![0],
        },
        Op::WriteOne {
            addr: REG_TEMP_UNIT,
            value: 1,
        },
        Op::Read {
            addr: REG_TEMP_UNIT,
            values: vec![1],
        },
    ]);
    let mut xy = Xy::new(mock, Model::Xy7025);
    xy.set_temp_unit(TempUnit::Celsius).unwrap();
    assert_eq!(xy.read_temp_unit().unwrap(), TempUnit::Celsius);
    xy.set_temp_unit(TempUnit::Fahrenheit).unwrap();
    assert_eq!(xy.read_temp_unit().unwrap(), TempUnit::Fahrenheit);
}

/// XY7025 must report the documented family scales (DATASHEET §3).
/// `Custom` is verified separately in `custom_model_routes_user_supplied_scales`.
#[test]
fn preset_models_match_datasheet_scales() {
    let m = Model::Xy7025;
    assert_eq!(m.current_scale(), 100.0);
    assert_eq!(m.power_scale(), 10.0);
    assert_eq!(m.opp_scale(), 1.0);
}

/// `Custom` with SK-style scales must use `opp_scale=10` (S-OPP raw stores
/// 0.1 W units), distinct from XY7025 which stores whole watts.
#[test]
fn group_encode_under_custom_sk_scales_uses_opp_scale_10() {
    // 18.0 W S-OPP → raw 180 with opp_scale=10, would be 18 on XY7025.
    let p = GroupParams {
        setpoints: Setpoints {
            v_set: 5.0,
            i_set: 1.0,
        },
        safety_limits: SafetyLimits {
            lvp_v: 0.0,
            ovp_v: 0.0,
            ocp_a: 0.0,
        },
        s_opp_w: 18.0,
        s_ohp_h: 0,
        s_ohp_m: 0,
        s_oah_ah: 0.0,
        s_owh_wh: 0.0,
        s_otp: 0.0,
        power_on_output: false,
    };
    let mock = MockTransport::new(vec![Op::WriteMany {
        addr: group_addr(0),
        // v_set=500, i_set=1000 (current_scale=1000), s_opp=180, s_otp=0.
        values: vec![500, 1000, 0, 0, 0, 180, 0, 0, 0, 0, 0, 0, 0, 0],
    }]);
    let mut xy = Xy::new(mock, custom_model(1000, 100, 10));
    xy.write_group(0, &p).unwrap();
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
    let mut xy = Xy::new(FailRead, Model::Xy7025);
    assert!(matches!(
        xy.read_voltage_out(),
        Err(XyError::Rtu(RtuError::Modbus(ModbusError::BadCrc)))
    ));
}

#[test]
fn invalid_inputs_fail_before_transport_io() {
    assert_eq!(
        Xy::with_slave(MockTransport::new(vec![]), Model::Xy7025, 0).unwrap_err(),
        InputError::InvalidSlaveAddress { address: 0 }
    );

    let mut xy = Xy::new(MockTransport::new(vec![]), Model::Xy7025);
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
fn paired_register_conversions_round_trip_every_precision_boundary() {
    for scale in [100.0, 1000.0] {
        for raw in [0, 1, 16_777_217, 420_000_001, u32::MAX] {
            let value = from_reg_u32(raw as u16, (raw >> 16) as u16, scale);
            let words = to_reg_u32(value, scale, u32::MAX as f64 / scale, "fixture").unwrap();
            let round_trip = ((words.high as u32) << 16) | words.low as u32;
            assert_eq!(round_trip, raw, "scale={scale}, raw={raw}");
        }
    }
}

#[test]
fn verify_model_match_for_xy7025() {
    let mut xy = Xy::new(
        MockTransport::new(vec![
            Op::Read {
                addr: REG_MODEL,
                values: vec![0x6500],
            },
            Op::Read {
                addr: REG_MODEL,
                values: vec![0x6100],
            },
        ]),
        Model::Xy7025,
    );
    assert_eq!(
        xy.verify_model().unwrap(),
        ModelCheck::Match {
            device_code: 0x6500
        }
    );
    assert_eq!(
        xy.verify_model().unwrap(),
        ModelCheck::Match {
            device_code: 0x6100
        }
    );
}

#[test]
fn verify_model_is_inconclusive_for_unknown_code() {
    let mut xy = Xy::new(
        MockTransport::new(vec![Op::Read {
            addr: REG_MODEL,
            values: vec![0x7700],
        }]),
        Model::Xy7025,
    );
    assert_eq!(
        xy.verify_model().unwrap(),
        ModelCheck::Inconclusive {
            device_code: 0x7700
        }
    );
}

#[test]
fn verify_model_inconclusive_for_custom() {
    let mut xy = Xy::new(
        MockTransport::new(vec![Op::Read {
            addr: REG_MODEL,
            values: vec![0x6500],
        }]),
        custom_model(100, 10, 1),
    );
    assert_eq!(
        xy.verify_model().unwrap(),
        ModelCheck::Inconclusive {
            device_code: 0x6500
        }
    );
}
