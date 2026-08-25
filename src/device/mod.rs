//! High-level device API. One method per logical operation; all reads
//! and writes go through the [`crate::transport::ModbusTransport`].

use crate::error::{InputError, InputField, XyError};
use crate::framing;
use crate::regs;
use crate::regs::{DEFAULT_SLAVE, Encoding, Group, Reg};
use crate::transport::ModbusTransport;
use crate::types::enums::{BaudRate, ProtectionStatus, RegMode, TempUnit};
use crate::types::group::GroupParams;
use crate::types::status::{OnTime, SafetyLimits, Setpoints, Status, Temperature, Totals};

/// Where an input's accepted bounds come from.
#[derive(Copy, Clone, Debug)]
enum InputLimit {
    /// Fixed physical bounds in the register's own unit.
    Scalar { min: f32, max: f32 },
    /// Fixed bounds for a 32-bit pair, whose range exceeds exact `f32`.
    Wide { min: f64, max: f64 },
    /// The register's own [`Encoding::Int`] range.
    FromEncoding,
    /// Bounds vary at runtime, so the caller supplies them — S-OTP's ceiling
    /// depends on the displayed temperature unit.
    Dynamic,
}

/// What a register accepts on the way in, and the field a rejection names.
#[derive(Copy, Clone, Debug)]
struct InputSpec {
    limit: InputLimit,
    field: InputField,
}

/// Physical write limits, one statement per register. Scale and signedness
/// come from [`Reg::encoding`], so a limit can never disagree with the scale
/// it is expressed in.
const fn input_spec(reg: Reg) -> InputSpec {
    let (limit, field) = match reg {
        Reg::VSet => (
            InputLimit::Scalar {
                min: 0.0,
                max: 70.0,
            },
            InputField::VoltageSetpoint,
        ),
        Reg::ISet => (
            InputLimit::Scalar {
                min: 0.0,
                max: 25.0,
            },
            InputField::CurrentSetpoint,
        ),
        Reg::SLvp => (
            InputLimit::Scalar {
                min: 10.0,
                max: 95.0,
            },
            InputField::LowVoltageProtection,
        ),
        Reg::SOvp => (
            InputLimit::Scalar {
                min: 0.0,
                max: 72.0,
            },
            InputField::OverVoltageProtection,
        ),
        Reg::SOcp => (
            InputLimit::Scalar {
                min: 0.0,
                max: 27.0,
            },
            InputField::OverCurrentProtection,
        ),
        Reg::SOpp => (
            InputLimit::Scalar {
                min: 0.0,
                max: 2000.0,
            },
            InputField::OverPowerProtection,
        ),
        Reg::SOahLow => (
            InputLimit::Wide {
                min: 0.0,
                max: 9999.0,
            },
            InputField::ChargeLimit,
        ),
        Reg::SOwhLow => (
            InputLimit::Wide {
                min: 0.0,
                max: 4_200_000.0,
            },
            InputField::EnergyLimit,
        ),
        Reg::SOtp => (InputLimit::Dynamic, InputField::OverTemperatureProtection),
        Reg::SOhpH => (InputLimit::FromEncoding, InputField::OutputTimeHours),
        Reg::SOhpM => (InputLimit::FromEncoding, InputField::OutputTimeMinutes),
        Reg::Backlight => (InputLimit::FromEncoding, InputField::Backlight),
        Reg::Sleep => (InputLimit::FromEncoding, InputField::SleepTimeout),
        _ => panic!("register is not a validated write target"),
    };
    InputSpec { limit, field }
}

/// Divisor for a fixed-point register. Panics for a register that carries no
/// scale — a wiring mistake in this crate, never bad device data.
const fn scale_of(reg: Reg) -> f32 {
    match reg.encoding() {
        Encoding::Unsigned { scale } | Encoding::Signed { scale } => scale,
        _ => panic!("register is not fixed-point"),
    }
}

/// Divisor for a 32-bit pair, taken from its low word.
const fn wide_scale_of(low: Reg) -> f64 {
    match low.encoding() {
        Encoding::WideLow { scale } => scale,
        _ => panic!("register is not the low word of a 32-bit pair"),
    }
}

/// Decode a fixed-point word, honouring the register's own signedness. The
/// signed case is why `T-IN` no longer reports 6548.6 °C at −5 °C.
fn decode_scaled(reg: Reg, raw: u16) -> f32 {
    match reg.encoding() {
        Encoding::Unsigned { scale } => raw as f32 / scale,
        Encoding::Signed { scale } => raw as i16 as f32 / scale,
        _ => panic!("register is not fixed-point"),
    }
}

fn decode_wide(low: Reg, low_word: u16, high_word: u16) -> f64 {
    let raw = ((high_word as u32) << 16) | low_word as u32;
    raw as f64 / wide_scale_of(low)
}

fn decode_setpoints(v_set: u16, i_set: u16) -> Setpoints {
    Setpoints {
        v_set: decode_scaled(Reg::VSet, v_set),
        i_set: decode_scaled(Reg::ISet, i_set),
    }
}

/// Decode a word that came from `addr`. Group copies of a register live at a
/// different address than the M0 original, so the reported address is separate
/// from the register whose encoding is applied.
fn decode_flag_at(reg: Reg, addr: u16, raw: u16) -> Result<bool, XyError> {
    debug_assert!(matches!(reg.encoding(), Encoding::Flag));
    match raw {
        0 => Ok(false),
        1 => Ok(true),
        value => Err(XyError::InvalidRegisterValue {
            register: addr,
            value,
        }),
    }
}

fn decode_flag(reg: Reg, raw: u16) -> Result<bool, XyError> {
    decode_flag_at(reg, reg.addr(), raw)
}

fn decode_int(reg: Reg, raw: u16) -> Result<u16, XyError> {
    decode_int_at(reg, reg.addr(), raw)
}

fn decode_int_at(reg: Reg, addr: u16, raw: u16) -> Result<u16, XyError> {
    let Encoding::Int { min, max } = reg.encoding() else {
        panic!("register is not an integer");
    };
    if raw >= min && raw <= max {
        Ok(raw)
    } else {
        Err(XyError::InvalidRegisterValue {
            register: addr,
            value: raw,
        })
    }
}

fn decode_code<V>(
    reg: Reg,
    raw: u16,
    decode: impl FnOnce(u16) -> Result<V, u16>,
) -> Result<V, XyError> {
    debug_assert!(matches!(reg.encoding(), Encoding::Code));
    decode(raw).map_err(|value| XyError::InvalidRegisterValue {
        register: reg.addr(),
        value,
    })
}

#[derive(Debug)]
struct RegisterWords {
    low: u16,
    high: u16,
}

/// Encode a physical value for `reg` against its declared limit.
fn to_reg(reg: Reg, v: f32) -> Result<u16, InputError> {
    let InputLimit::Scalar { min, max } = input_spec(reg).limit else {
        panic!("register has no fixed scalar limit");
    };
    to_reg_within(reg, v, min, max)
}

/// Encode against caller-supplied bounds, for the registers whose ceiling is
/// not fixed. The rejected-field name still comes from the register.
fn to_reg_within(reg: Reg, v: f32, min: f32, max: f32) -> Result<u16, InputError> {
    let field = input_spec(reg).field;
    if !v.is_finite() {
        return Err(InputError::NonFinite { field });
    }
    if v < min || v > max {
        return Err(InputError::OutOfRange { field });
    }
    let scale = scale_of(reg);
    // A limit that could overflow the register is a table error, not a caller
    // error, so it asserts rather than returning.
    debug_assert!(
        max * scale <= u16::MAX as f32,
        "write limit overflows the register"
    );
    let raw = (v * scale + 0.5) as u16;
    // Rounding to the raw grid can cross a bound that does not itself sit on
    // that grid. Every limit in `input_spec` is currently grid-aligned, so
    // this is insurance against a future edit rather than a live path.
    let encoded = raw as f32 / scale;
    if encoded < min || encoded > max {
        return Err(InputError::OutOfRange { field });
    }
    Ok(raw)
}

fn to_reg_wide(low: Reg, v: f64) -> Result<RegisterWords, InputError> {
    let spec = input_spec(low);
    let InputLimit::Wide { min, max } = spec.limit else {
        panic!("register has no fixed wide limit");
    };
    let field = spec.field;
    if !v.is_finite() {
        return Err(InputError::NonFinite { field });
    }
    if v < min || v > max {
        return Err(InputError::OutOfRange { field });
    }
    let scale = wide_scale_of(low);
    debug_assert!(
        max * scale <= u32::MAX as f64,
        "write limit overflows the register pair"
    );
    let raw = (v * scale + 0.5) as u32;
    let encoded = raw as f64 / scale;
    if encoded < min || encoded > max {
        return Err(InputError::OutOfRange { field });
    }
    Ok(RegisterWords {
        low: raw as u16,
        high: (raw >> 16) as u16,
    })
}

/// Reject an integer input outside the register's own accepted range.
fn validate_int(reg: Reg, value: u16) -> Result<(), InputError> {
    let Encoding::Int { min, max } = reg.encoding() else {
        panic!("register is not an integer");
    };
    if value >= min && value <= max {
        Ok(())
    } else {
        Err(InputError::OutOfRange {
            field: input_spec(reg).field,
        })
    }
}

/// The device latches OVP the moment a V-SET above the S-OVP threshold
/// applies, so every path that moves either register checks the pair first.
fn check_v_set_under_ovp(v_set: u16, ovp: u16) -> Result<(), InputError> {
    if v_set > ovp {
        return Err(InputError::VoltageSetpointAboveProtection);
    }
    Ok(())
}

/// Result of checking whether the device's `MODEL` register belongs to the
/// scale family supported by [`Xy`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScaleCheck {
    Compatible { device_code: u16 },
    Inconclusive { device_code: u16 },
}

/// High-level driver for the XY7025.
///
/// Construct with [`Xy::new`] (default slave `0x01`) or
/// [`Xy::with_slave`]. Call [`Self::verify_scale_family`] during bring-up to
/// catch devices whose fixed-point scales are not known to be compatible.
#[derive(Debug)]
pub struct Xy<T: ModbusTransport> {
    transport: T,
    slave: u8,
}

impl<T: ModbusTransport> Xy<T> {
    /// Wrap a transport using the default slave address (`0x01`).
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            slave: DEFAULT_SLAVE,
        }
    }

    pub fn with_slave(transport: T, slave: u8) -> Result<Self, XyError> {
        framing::validate_read_slave(slave)?;
        Ok(Self { transport, slave })
    }

    /// Read holding registers through the configured transport and slave.
    /// Empty destinations and slices longer than
    /// [`crate::framing::MAX_READ_REGS`] return [`crate::RtuError::InvalidQuantity`].
    /// Addresses come from [`Reg::addr`].
    pub fn read_raw_holding(&mut self, addr: u16, dst: &mut [u16]) -> Result<(), XyError> {
        self.transport.read_holding(self.slave, addr, dst)?;
        Ok(())
    }

    /// Write one holding register through the configured transport and slave.
    pub fn write_raw_holding(&mut self, addr: u16, value: u16) -> Result<(), XyError> {
        self.transport
            .write_single_holding(self.slave, addr, value)?;
        Ok(())
    }

    /// Consume the device and return the inner transport.
    pub fn into_transport(self) -> T {
        self.transport
    }

    /// Read one register's raw word, with no decoding and no range check.
    ///
    /// Use this to observe exactly what the device stores: the typed getters
    /// apply the register's [`Encoding`] and reject a word outside it, which
    /// is the wrong behaviour when the point is to watch firmware clamp.
    /// [`Self::read_raw_holding`] remains for block reads and for addresses
    /// this crate does not model.
    pub fn read_word(&mut self, reg: Reg) -> Result<u16, XyError> {
        let mut r = [0u16; 1];
        self.transport
            .read_holding(self.slave, reg.addr(), &mut r)?;
        Ok(r[0])
    }

    /// Write one register's raw word, bypassing this crate's input validation
    /// and the register's scale. Firmware clamping still applies.
    pub fn write_word(&mut self, reg: Reg, value: u16) -> Result<(), XyError> {
        self.transport
            .write_single_holding(self.slave, reg.addr(), value)?;
        Ok(())
    }

    /// Read setpoints (V-SET, I-SET) — registers 0x0000–0x0001.
    pub fn read_setpoints(&mut self) -> Result<Setpoints, XyError> {
        const BASE: Reg = Reg::VSet;
        const V_SET: usize = regs::offset_in(BASE, Reg::VSet);
        const I_SET: usize = regs::offset_in(BASE, Reg::ISet);

        let mut r = [0u16; regs::span(BASE, Reg::ISet)];
        self.read_block(BASE, &mut r)?;
        Ok(decode_setpoints(r[V_SET], r[I_SET]))
    }

    /// Read the live + control snapshot — everything a supervisor needs each
    /// tick: live readings, regulation mode, latched protection cause, and the
    /// output-enable flag.
    ///
    /// **One transaction**, covering registers 0x0000–0x0012.
    ///
    /// Nine of those nineteen registers are used. 0x0006–0x000F — the
    /// cumulative counters, on-time, temperatures and the key lock — are read
    /// and discarded, because the fields wanted here sit in two spans
    /// (0x0000–0x0005 and 0x0010–0x0012) and splitting the read costs a second
    /// pre-transmit quiet gap. That gap dominates the twenty wasted bytes at
    /// every baud rate the device supports: with the bundled UART's 50 ms gap,
    /// one read against two is roughly 54 ms against 104 ms at 115200, and
    /// 263 ms against 283 ms even at 2400. The split would only pay off below
    /// about 1400 baud.
    ///
    /// The single transaction also keeps the snapshot coherent. Two reads
    /// could pair a `v_out` sampled before a trip with an `output_on` sampled
    /// after it.
    pub fn read_status(&mut self) -> Result<Status, XyError> {
        const BASE: Reg = Reg::VSet;
        const V_SET: usize = regs::offset_in(BASE, Reg::VSet);
        const I_SET: usize = regs::offset_in(BASE, Reg::ISet);
        const V_OUT: usize = regs::offset_in(BASE, Reg::VOut);
        const I_OUT: usize = regs::offset_in(BASE, Reg::IOut);
        const P_OUT: usize = regs::offset_in(BASE, Reg::POut);
        const V_IN: usize = regs::offset_in(BASE, Reg::VIn);
        const PROTECT: usize = regs::offset_in(BASE, Reg::Protect);
        const CVCC: usize = regs::offset_in(BASE, Reg::Cvcc);
        const OUTPUT_EN: usize = regs::offset_in(BASE, Reg::OutputEn);

        let mut r = [0u16; regs::span(BASE, Reg::OutputEn)];
        self.read_block(BASE, &mut r)?;
        Ok(Status {
            setpoints: decode_setpoints(r[V_SET], r[I_SET]),
            v_out: decode_scaled(Reg::VOut, r[V_OUT]),
            i_out: decode_scaled(Reg::IOut, r[I_OUT]),
            p_out: decode_scaled(Reg::POut, r[P_OUT]),
            v_in: decode_scaled(Reg::VIn, r[V_IN]),
            protection: decode_code(Reg::Protect, r[PROTECT], ProtectionStatus::from_reg)?,
            reg_mode: decode_code(Reg::Cvcc, r[CVCC], RegMode::from_reg)?,
            output_on: decode_flag(Reg::OutputEn, r[OUTPUT_EN])?,
        })
    }

    pub fn read_voltage_out(&mut self) -> Result<f32, XyError> {
        self.read_scaled(Reg::VOut)
    }
    pub fn read_current_out(&mut self) -> Result<f32, XyError> {
        self.read_scaled(Reg::IOut)
    }
    pub fn read_power_out(&mut self) -> Result<f32, XyError> {
        self.read_scaled(Reg::POut)
    }
    pub fn read_voltage_in(&mut self) -> Result<f32, XyError> {
        self.read_scaled(Reg::VIn)
    }

    /// Read cumulative output charge, energy, and on-time (registers
    /// 0x0006–0x000C, one transaction).
    ///
    /// XY7025 firmware has not been verified to snapshot the LOW/HIGH counter
    /// pairs atomically, so a read concurrent with low-word rollover may tear.
    pub fn read_totals(&mut self) -> Result<Totals, XyError> {
        const BASE: Reg = Reg::AhLow;
        const AH_LOW: usize = regs::offset_in(BASE, Reg::AhLow);
        const AH_HIGH: usize = regs::offset_in(BASE, Reg::AhHigh);
        const WH_LOW: usize = regs::offset_in(BASE, Reg::WhLow);
        const WH_HIGH: usize = regs::offset_in(BASE, Reg::WhHigh);
        const OUT_H: usize = regs::offset_in(BASE, Reg::OutH);
        const OUT_M: usize = regs::offset_in(BASE, Reg::OutM);
        const OUT_S: usize = regs::offset_in(BASE, Reg::OutS);

        let mut r = [0u16; regs::span(BASE, Reg::OutS)];
        self.read_block(BASE, &mut r)?;
        Ok(Totals {
            charge_ah: decode_wide(Reg::AhLow, r[AH_LOW], r[AH_HIGH]),
            energy_wh: decode_wide(Reg::WhLow, r[WH_LOW], r[WH_HIGH]),
            on_time: OnTime {
                hours: decode_int(Reg::OutH, r[OUT_H])?,
                minutes: decode_int(Reg::OutM, r[OUT_M])?,
                seconds: decode_int(Reg::OutS, r[OUT_S])?,
            },
        })
    }

    /// Set output voltage (V-SET, register 0x0000).
    ///
    /// **Two transactions**: the live S-OVP, then the write.
    ///
    /// Writing a V-SET above the live S-OVP latches OVP on the device, so the
    /// threshold is read first and such a setpoint is rejected with
    /// [`InputError::VoltageSetpointAboveProtection`] rather than sent. Raise
    /// protection with [`Self::set_protection`] before raising the setpoint.
    /// The read and the write are separate transactions, so a concurrent
    /// controller can still lower S-OVP in between.
    pub fn set_voltage(&mut self, volts: f32) -> Result<(), XyError> {
        let raw = to_reg(Reg::VSet, volts)?;
        check_v_set_under_ovp(raw, self.read_word(Reg::SOvp)?)?;
        self.write_word(Reg::VSet, raw)
    }

    pub fn set_current_limit(&mut self, amps: f32) -> Result<(), XyError> {
        let raw = to_reg(Reg::ISet, amps)?;
        self.write_word(Reg::ISet, raw)
    }

    /// Program LVP / OVP / OCP into the active group's protection
    /// registers (0x0052–0x0054) in one bulk write.
    ///
    /// **Two transactions**: the live V-SET, then the write.
    ///
    /// An S-OVP below the live V-SET would latch OVP the moment it applied, so
    /// the setpoint is read first and such a threshold is rejected with
    /// [`InputError::VoltageSetpointAboveProtection`] rather than sent. Lower
    /// V-SET with [`Self::set_voltage`] first. The read and the write are
    /// separate transactions, so a concurrent controller can still raise V-SET
    /// in between.
    pub fn set_protection(&mut self, l: SafetyLimits) -> Result<(), XyError> {
        let lvp = to_reg(Reg::SLvp, l.lvp_v)?;
        let ovp = to_reg(Reg::SOvp, l.ovp_v)?;
        let ocp = to_reg(Reg::SOcp, l.ocp_a)?;
        check_v_set_under_ovp(self.read_word(Reg::VSet)?, ovp)?;
        self.transport
            .write_multiple_holdings(self.slave, Reg::SLvp.addr(), &[lvp, ovp, ocp])?;
        Ok(())
    }

    /// Read LVP / OVP / OCP from the active group (0x0052–0x0054).
    pub fn read_protection(&mut self) -> Result<SafetyLimits, XyError> {
        const BASE: Reg = Reg::SLvp;
        const LVP: usize = regs::offset_in(BASE, Reg::SLvp);
        const OVP: usize = regs::offset_in(BASE, Reg::SOvp);
        const OCP: usize = regs::offset_in(BASE, Reg::SOcp);

        let mut r = [0u16; regs::span(BASE, Reg::SOcp)];
        self.read_block(BASE, &mut r)?;
        Ok(SafetyLimits {
            lvp_v: decode_scaled(Reg::SLvp, r[LVP]),
            ovp_v: decode_scaled(Reg::SOvp, r[OVP]),
            ocp_a: decode_scaled(Reg::SOcp, r[OCP]),
        })
    }

    /// Power-on output state (S-INI, register 0x005D). `false` = OFF
    /// at boot, `true` = ON. Persists in EEPROM. `false` is the safe
    /// default after an unexpected power loss — the buck stays disabled
    /// until explicitly re-enabled.
    pub fn set_power_on_output(&mut self, on: bool) -> Result<(), XyError> {
        self.write_word(Reg::SIni, on as u16)
    }

    pub fn read_power_on_output(&mut self) -> Result<bool, XyError> {
        self.read_flag(Reg::SIni)
    }

    /// Read the output-enable register (ONOFF, 0x0012).
    pub fn read_output(&mut self) -> Result<bool, XyError> {
        self.read_flag(Reg::OutputEn)
    }

    pub fn set_output(&mut self, on: bool) -> Result<(), XyError> {
        self.write_word(Reg::OutputEn, on as u16)
    }

    /// Read the latched protection cause (PROTECT, 0x0010). While the
    /// output is on, this register is necessarily `Normal` — only worth
    /// reading after observing OUTPUT_EN go low.
    pub fn read_protection_status(&mut self) -> Result<ProtectionStatus, XyError> {
        self.read_code(Reg::Protect, ProtectionStatus::from_reg)
    }

    /// Clear a latched protection cause (write 0 to PROTECT). This
    /// stops the front-panel blink but does **not** re-enable the
    /// output — call [`Self::set_output`] separately.
    pub fn clear_protection_status(&mut self) -> Result<(), XyError> {
        self.write_word(Reg::Protect, ProtectionStatus::Normal.code())
    }

    pub fn read_reg_mode(&mut self) -> Result<RegMode, XyError> {
        self.read_code(Reg::Cvcc, RegMode::from_reg)
    }

    /// Read the verified internal sensor and its unit in one register snapshot.
    /// [`Reg::TIn`] is two's-complement signed, so sub-zero ambients decode
    /// correctly.
    ///
    /// The external-probe register remains available through
    /// [`Self::read_raw_holding`], but is not exposed at the high level because
    /// its connected-probe scale has not been verified on hardware. [`Reg::TEx`]
    /// records that it is signed on the same basis as this one.
    pub fn read_temperature_internal(&mut self) -> Result<Temperature, XyError> {
        const BASE: Reg = Reg::TIn;
        const T_IN: usize = regs::offset_in(BASE, Reg::TIn);
        const TEMP_UNIT: usize = regs::offset_in(BASE, Reg::TempUnit);

        let mut r = [0u16; regs::span(BASE, Reg::TempUnit)];
        self.read_block(BASE, &mut r)?;
        Ok(Temperature {
            value: decode_scaled(Reg::TIn, r[T_IN]),
            unit: decode_code(Reg::TempUnit, r[TEMP_UNIT], TempUnit::from_reg)?,
        })
    }

    pub fn read_temp_unit(&mut self) -> Result<TempUnit, XyError> {
        self.read_code(Reg::TempUnit, TempUnit::from_reg)
    }
    pub fn set_temp_unit(&mut self, unit: TempUnit) -> Result<(), XyError> {
        self.write_word(Reg::TempUnit, unit.code())
    }

    pub fn read_temp_offset_internal(&mut self) -> Result<f32, XyError> {
        self.read_scaled(Reg::TInOffset)
    }
    pub fn read_temp_offset_external(&mut self) -> Result<f32, XyError> {
        self.read_scaled(Reg::TExOffset)
    }
    // Setters intentionally absent: XY7025 firmware silently ignores
    // Modbus writes to T-IN/T-EX offset (verified empirically — every
    // non-zero raw write reads back as 0). The offset can only be
    // changed from the front-panel calibration menu, so a Modbus setter
    // would lie about success. Use the front panel instead.

    pub fn read_lock(&mut self) -> Result<bool, XyError> {
        self.read_flag(Reg::Lock)
    }
    pub fn set_lock(&mut self, locked: bool) -> Result<(), XyError> {
        self.write_word(Reg::Lock, locked as u16)
    }

    /// Backlight brightness (1–5).
    pub fn read_backlight(&mut self) -> Result<u8, XyError> {
        Ok(self.read_int(Reg::Backlight)? as u8)
    }
    /// Set backlight brightness. Documented range is 0–5, but XY7025
    /// firmware floors writes at 1 (writing 0 reads back as 1) — the
    /// display can't be fully extinguished via Modbus, so 0 is rejected.
    pub fn set_backlight(&mut self, level: u8) -> Result<(), XyError> {
        validate_int(Reg::Backlight, level as u16)?;
        self.write_word(Reg::Backlight, level as u16)
    }

    /// Off-screen timeout in minutes.
    pub fn read_sleep_minutes(&mut self) -> Result<u16, XyError> {
        self.read_int(Reg::Sleep)
    }
    /// Set off-screen timeout in minutes. XY7025 firmware caps the
    /// stored value at 9; any write ≥10 reads back as 9. Pass 0 to
    /// disable auto-off.
    pub fn set_sleep_minutes(&mut self, minutes: u16) -> Result<(), XyError> {
        validate_int(Reg::Sleep, minutes)?;
        self.write_word(Reg::Sleep, minutes)
    }

    /// Buzzer enable. Often unimplemented in firmware.
    pub fn read_buzzer(&mut self) -> Result<bool, XyError> {
        self.read_flag(Reg::Buzzer)
    }
    pub fn set_buzzer(&mut self, on: bool) -> Result<(), XyError> {
        self.write_word(Reg::Buzzer, on as u16)
    }

    /// Product number (e.g. `0x6500` on XY7025).
    pub fn read_model(&mut self) -> Result<u16, XyError> {
        self.read_word(Reg::Model)
    }

    /// Read the device's `MODEL` register and check whether it belongs to the
    /// XY7025 scale family. This catches the footgun where `read_status().i_out`
    /// silently comes back 10× off, but it does not prove exact hardware
    /// identity or physical limits.
    ///
    /// Returns [`ScaleCheck::Inconclusive`] for unknown device codes.
    pub fn verify_scale_family(&mut self) -> Result<ScaleCheck, XyError> {
        let device_code = self.read_model()?;
        if matches!(device_code, 0x6100 | 0x6500) {
            Ok(ScaleCheck::Compatible { device_code })
        } else {
            Ok(ScaleCheck::Inconclusive { device_code })
        }
    }

    /// Firmware version (e.g. `0x0071`).
    pub fn read_version(&mut self) -> Result<u16, XyError> {
        self.read_word(Reg::Version)
    }

    /// Read the device's currently configured Modbus slave address.
    /// It may briefly differ from the address used by this driver while
    /// reconfiguring.
    pub fn read_slave_address(&mut self) -> Result<u8, XyError> {
        // `Reg::SlaveAddr` declares 1..=247, so the cast cannot truncate.
        Ok(self.read_int(Reg::SlaveAddr)? as u8)
    }
    /// Write a new slave address. Takes effect after the device resets.
    pub fn set_slave_address(&mut self, addr: u8) -> Result<(), XyError> {
        framing::validate_read_slave(addr)?;
        self.write_word(Reg::SlaveAddr, addr as u16)
    }

    pub fn read_baud_rate(&mut self) -> Result<BaudRate, XyError> {
        self.read_code(Reg::BaudCode, BaudRate::from_code)
    }
    /// Write a new baud-rate code. Takes effect after the device resets.
    pub fn set_baud_rate(&mut self, baud: BaudRate) -> Result<(), XyError> {
        self.write_word(Reg::BaudCode, baud.code())
    }

    /// Recall a stored memory group into the live operating set.
    /// [`Group::M0`] is a no-op on the device, which is already using it.
    ///
    /// One transaction.
    pub fn recall_group(&mut self, group: Group) -> Result<(), XyError> {
        self.write_word(Reg::ExtractM, group.index() as u16)
    }

    /// Read all 14 registers of a memory group.
    ///
    /// **Two transactions**: the displayed temperature unit, then the group
    /// itself. S-OTP is stored in whichever unit `F-C` selects, so the unit has
    /// to be known to label the reading. Sweeping several groups is cheaper
    /// through [`Self::read_group_in_unit`].
    ///
    /// F-C must not be changed concurrently through another controller or the
    /// front panel while this operation is in progress.
    pub fn read_group(&mut self, group: Group) -> Result<GroupParams, XyError> {
        let unit = self.read_temp_unit()?;
        self.read_group_in_unit(group, unit)
    }

    /// [`Self::read_group`] with the displayed unit already in hand, costing
    /// **one transaction** instead of two.
    ///
    /// `unit` must be what the device is currently displaying — take it from
    /// [`Self::read_temp_unit`]. Reusing one value across a sweep of several
    /// groups widens the window in which a concurrent `F-C` change would
    /// mislabel S-OTP; that is the trade for the saved round trips.
    pub fn read_group_in_unit(
        &mut self,
        group: Group,
        unit: TempUnit,
    ) -> Result<GroupParams, XyError> {
        let mut r = [0u16; Group::LEN];
        self.transport
            .read_holding(self.slave, group.addr(), &mut r)?;
        decode_group(&r, group, unit)
    }

    /// Write all 14 registers of a memory group, then return the values the
    /// device stored after temperature conversion and rounding.
    ///
    /// **Four transactions for [`Group::M0`], three otherwise**: the displayed
    /// temperature unit, the V-SET/S-OVP guard read (M0 only), the bulk write,
    /// and the read-back. At the bundled UART's 50 ms inter-frame gap that is
    /// roughly a fifth of a second. [`Self::write_group_in_unit`] drops the
    /// first of them.
    ///
    /// Because the unit is read first, invalid parameters cost that one
    /// transaction before being rejected. [`Self::write_group_in_unit`]
    /// rejects them with no I/O at all.
    ///
    /// For M0 this updates the live operating set, moving V-SET and S-OVP in
    /// one frame. The order in which the device applies the registers of an
    /// FC16 is unverified, so both are read first and the write is rejected
    /// with [`InputError::VoltageSetpointAboveProtection`] unless every
    /// possible ordering stays clear of an OVP trip: the new V-SET must sit at
    /// or below the live S-OVP, and the new S-OVP at or above the live V-SET.
    /// (Encoding has already checked the new V-SET and S-OVP against each
    /// other.)
    /// F-C must not be changed concurrently through another controller or the
    /// front panel while this operation is in progress.
    pub fn write_group(&mut self, group: Group, p: &GroupParams) -> Result<GroupParams, XyError> {
        let unit = self.read_temp_unit()?;
        self.write_group_in_unit(group, p, unit)
    }

    /// [`Self::write_group`] with the displayed unit already in hand, saving
    /// the `F-C` read. The same caveat applies: `unit` must be what the device
    /// is currently displaying.
    pub fn write_group_in_unit(
        &mut self,
        group: Group,
        p: &GroupParams,
        unit: TempUnit,
    ) -> Result<GroupParams, XyError> {
        const V_SET: usize = Group::word_offset(Reg::VSet);
        const S_OVP: usize = Group::word_offset(Reg::SOvp);

        let words = encode_group(p, unit)?;
        if group == Group::M0 {
            // V-SET and S-OVP are three registers apart, so one read of the
            // span costs less than two single-register transactions.
            let mut current = [0u16; S_OVP + 1];
            self.transport
                .read_holding(self.slave, Group::BASE, &mut current)?;
            // Either register may land first, so both pairings must clear.
            check_v_set_under_ovp(words[V_SET], current[S_OVP])?;
            check_v_set_under_ovp(current[V_SET], words[S_OVP])?;
        }
        self.transport
            .write_multiple_holdings(self.slave, group.addr(), &words)?;
        self.read_group_in_unit(group, unit)
    }

    fn read_block(&mut self, base: Reg, dst: &mut [u16]) -> Result<(), XyError> {
        self.transport.read_holding(self.slave, base.addr(), dst)?;
        Ok(())
    }

    fn read_scaled(&mut self, reg: Reg) -> Result<f32, XyError> {
        Ok(decode_scaled(reg, self.read_word(reg)?))
    }

    fn read_flag(&mut self, reg: Reg) -> Result<bool, XyError> {
        decode_flag(reg, self.read_word(reg)?)
    }

    fn read_int(&mut self, reg: Reg) -> Result<u16, XyError> {
        decode_int(reg, self.read_word(reg)?)
    }

    fn read_code<V>(
        &mut self,
        reg: Reg,
        decode: impl FnOnce(u16) -> Result<V, u16>,
    ) -> Result<V, XyError> {
        decode_code(reg, self.read_word(reg)?, decode)
    }
}

fn decode_group(
    r: &[u16; Group::LEN],
    group: Group,
    unit: TempUnit,
) -> Result<GroupParams, XyError> {
    let word = |reg: Reg| r[Group::word_offset(reg)];
    let at = |reg: Reg| group.word_addr(reg);
    Ok(GroupParams {
        setpoints: decode_setpoints(word(Reg::VSet), word(Reg::ISet)),
        safety_limits: SafetyLimits {
            lvp_v: decode_scaled(Reg::SLvp, word(Reg::SLvp)),
            ovp_v: decode_scaled(Reg::SOvp, word(Reg::SOvp)),
            ocp_a: decode_scaled(Reg::SOcp, word(Reg::SOcp)),
        },
        s_opp_w: decode_scaled(Reg::SOpp, word(Reg::SOpp)),
        s_ohp_h: decode_int_at(Reg::SOhpH, at(Reg::SOhpH), word(Reg::SOhpH))?,
        s_ohp_m: decode_int_at(Reg::SOhpM, at(Reg::SOhpM), word(Reg::SOhpM))?,
        s_oah_ah: decode_wide(Reg::SOahLow, word(Reg::SOahLow), word(Reg::SOahHigh)),
        s_owh_wh: decode_wide(Reg::SOwhLow, word(Reg::SOwhLow), word(Reg::SOwhHigh)),
        s_otp: Temperature {
            value: decode_scaled(Reg::SOtp, word(Reg::SOtp)),
            unit,
        },
        power_on_output: decode_flag_at(Reg::SIni, at(Reg::SIni), word(Reg::SIni))?,
    })
}

fn encode_group(p: &GroupParams, unit: TempUnit) -> Result<[u16; Group::LEN], InputError> {
    if !p.s_otp.value.is_finite() {
        return Err(InputError::NonFinite {
            field: InputField::OverTemperatureProtection,
        });
    }
    validate_int(Reg::SOhpH, p.s_ohp_h)?;
    validate_int(Reg::SOhpM, p.s_ohp_m)?;
    let oah = to_reg_wide(Reg::SOahLow, p.s_oah_ah)?;
    let owh = to_reg_wide(Reg::SOwhLow, p.s_owh_wh)?;

    // Assigned by register rather than as a positional literal, so wire order
    // is stated once, in `Group::WORDS`.
    let mut values = [0u16; Group::LEN];
    values[Group::word_offset(Reg::VSet)] = to_reg(Reg::VSet, p.setpoints.v_set)?;
    values[Group::word_offset(Reg::ISet)] = to_reg(Reg::ISet, p.setpoints.i_set)?;
    values[Group::word_offset(Reg::SLvp)] = to_reg(Reg::SLvp, p.safety_limits.lvp_v)?;
    values[Group::word_offset(Reg::SOvp)] = to_reg(Reg::SOvp, p.safety_limits.ovp_v)?;
    values[Group::word_offset(Reg::SOcp)] = to_reg(Reg::SOcp, p.safety_limits.ocp_a)?;
    values[Group::word_offset(Reg::SOpp)] = to_reg(Reg::SOpp, p.s_opp_w)?;
    values[Group::word_offset(Reg::SOhpH)] = p.s_ohp_h;
    values[Group::word_offset(Reg::SOhpM)] = p.s_ohp_m;
    values[Group::word_offset(Reg::SOahLow)] = oah.low;
    values[Group::word_offset(Reg::SOahHigh)] = oah.high;
    values[Group::word_offset(Reg::SOwhLow)] = owh.low;
    values[Group::word_offset(Reg::SOwhHigh)] = owh.high;
    values[Group::word_offset(Reg::SOtp)] = encode_group_otp(p.s_otp, unit)?;
    values[Group::word_offset(Reg::SIni)] = p.power_on_output as u16;

    check_v_set_under_ovp(
        values[Group::word_offset(Reg::VSet)],
        values[Group::word_offset(Reg::SOvp)],
    )?;
    Ok(values)
}

/// Group writes route S-OTP through firmware unit conversion, which clamps at
/// 110 °C / 230 °F. The ceiling therefore depends on the displayed unit.
fn encode_group_otp(temperature: Temperature, unit: TempUnit) -> Result<u16, InputError> {
    let temperature = temperature.convert_to(unit);
    let max = match unit {
        TempUnit::Celsius => 110.0,
        TempUnit::Fahrenheit => 230.0,
    };
    to_reg_within(Reg::SOtp, temperature.value, 0.0, max)
}

#[cfg(test)]
mod tests;
