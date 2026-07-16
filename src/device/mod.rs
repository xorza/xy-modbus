//! High-level device API. One method per logical operation; all reads
//! and writes go through the [`crate::transport::ModbusTransport`].

pub(crate) mod error;

use crate::device::error::{InputError, InputField, XyError};
use crate::regs::*;
use crate::transport::{ModbusTransport, RtuError};
use crate::types::enums::{BaudRate, ProtectionStatus, RegMode, TempUnit};
use crate::types::group::GroupParams;
use crate::types::status::{OnTime, SafetyLimits, Setpoints, Status, Temperature, Totals};

const CURRENT_SCALE: f32 = 100.0;
const POWER_SCALE: f32 = 10.0;
const OPP_SCALE: f32 = 1.0;

#[derive(Copy, Clone, Debug)]
struct ValueRange {
    min: f64,
    max: f64,
}

impl ValueRange {
    fn contains(self, value: f64) -> bool {
        self.min <= value && value <= self.max
    }
}

const VOLTAGE_SET_RANGE: ValueRange = ValueRange {
    min: 0.0,
    max: 70.0,
};
const CURRENT_SET_RANGE: ValueRange = ValueRange {
    min: 0.0,
    max: 25.0,
};
const LVP_RANGE: ValueRange = ValueRange {
    min: 10.0,
    max: 95.0,
};
const OVP_RANGE: ValueRange = ValueRange {
    min: 0.0,
    max: 72.0,
};
const OCP_RANGE: ValueRange = ValueRange {
    min: 0.0,
    max: 27.0,
};
const OPP_RANGE: ValueRange = ValueRange {
    min: 0.0,
    max: 2000.0,
};
const CHARGE_LIMIT_RANGE: ValueRange = ValueRange {
    min: 0.0,
    max: 9999.0,
};
const ENERGY_LIMIT_RANGE: ValueRange = ValueRange {
    min: 0.0,
    max: 4_200_000.0,
};

fn to_reg_u16(v: f32, scale: f32, range: ValueRange, field: InputField) -> Result<u16, InputError> {
    if !v.is_finite() {
        return Err(InputError::NonFinite { field });
    }
    let scaled = v * scale;
    if !range.contains(v as f64) || scaled > u16::MAX as f32 + 0.5 {
        return Err(InputError::OutOfRange { field });
    }
    let raw = (scaled + 0.5) as u16;
    if !range.contains(raw as f64 / scale as f64) {
        return Err(InputError::OutOfRange { field });
    }
    Ok(raw)
}

fn from_reg_u16(raw: u16, scale: f32) -> f32 {
    raw as f32 / scale
}

fn decode_setpoints(v_set: u16, i_set: u16, current_scale: f32) -> Setpoints {
    Setpoints {
        v_set: from_reg_u16(v_set, 100.0),
        i_set: from_reg_u16(i_set, current_scale),
    }
}

fn from_reg_i16(raw: u16, scale: f32) -> f32 {
    raw as i16 as f32 / scale
}

fn from_reg_u32(low: u16, high: u16, scale: f64) -> f64 {
    let raw = ((high as u32) << 16) | low as u32;
    raw as f64 / scale
}

#[derive(Debug)]
struct RegisterWords {
    low: u16,
    high: u16,
}

fn to_reg_u32(
    v: f64,
    scale: f64,
    range: ValueRange,
    field: InputField,
) -> Result<RegisterWords, InputError> {
    if !v.is_finite() {
        return Err(InputError::NonFinite { field });
    }
    let scaled = v * scale;
    if !range.contains(v) || scaled > u32::MAX as f64 + 0.5 {
        return Err(InputError::OutOfRange { field });
    }
    let raw = (scaled + 0.5) as u32;
    if !range.contains(raw as f64 / scale) {
        return Err(InputError::OutOfRange { field });
    }
    Ok(RegisterWords {
        low: raw as u16,
        high: (raw >> 16) as u16,
    })
}

fn validate_slave(slave: u8) -> Result<(), InputError> {
    if (1..=247).contains(&slave) {
        Ok(())
    } else {
        Err(InputError::InvalidSlaveAddress { address: slave })
    }
}

fn validate_group(group: u8) -> Result<(), InputError> {
    if group < GROUP_COUNT {
        Ok(())
    } else {
        Err(InputError::InvalidGroup { group })
    }
}

fn validate_u16_max(value: u16, max: u16, field: InputField) -> Result<(), InputError> {
    if value <= max {
        Ok(())
    } else {
        Err(InputError::OutOfRange { field })
    }
}

fn decode_bool(register: u16, value: u16) -> Result<bool, XyError> {
    decode_register(register, value, |raw| match raw {
        0 => Ok(false),
        1 => Ok(true),
        invalid => Err(invalid),
    })
}

fn decode_register<T>(
    register: u16,
    value: u16,
    decode: impl FnOnce(u16) -> Result<T, u16>,
) -> Result<T, XyError> {
    decode(value).map_err(|value| XyError::InvalidRegisterValue { register, value })
}

fn decode_u16_max(register: u16, value: u16, max: u16) -> Result<u16, XyError> {
    if value <= max {
        Ok(value)
    } else {
        Err(XyError::InvalidRegisterValue { register, value })
    }
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

    pub fn with_slave(transport: T, slave: u8) -> Result<Self, InputError> {
        validate_slave(slave)?;
        Ok(Self { transport, slave })
    }

    /// Read holding registers through the configured transport and slave.
    /// Empty destinations and slices longer than
    /// [`crate::framing::MAX_READ_REGS`] return [`RtuError::InvalidQuantity`].
    pub fn read_raw_holding(&mut self, addr: u16, dst: &mut [u16]) -> Result<(), RtuError> {
        self.transport.read_holding(self.slave, addr, dst)
    }

    /// Write one holding register through the configured transport and slave.
    pub fn write_raw_holding(&mut self, addr: u16, value: u16) -> Result<(), RtuError> {
        self.transport.write_single_holding(self.slave, addr, value)
    }

    /// Consume the device and return the inner transport.
    pub fn into_transport(self) -> T {
        self.transport
    }

    /// Read setpoints (V-SET, I-SET) — registers 0x0000–0x0001.
    pub fn read_setpoints(&mut self) -> Result<Setpoints, XyError> {
        let mut r = [0u16; 2];
        self.transport.read_holding(self.slave, REG_V_SET, &mut r)?;
        Ok(decode_setpoints(r[0], r[1], CURRENT_SCALE))
    }

    /// Read the live + control snapshot (registers 0x0000–0x0012) in
    /// a single 19-register transaction. Returns everything a supervisor
    /// needs each tick (live readings, regulation mode, latched
    /// protection cause, output-enable flag) in one Modbus round-trip.
    pub fn read_status(&mut self) -> Result<Status, XyError> {
        // Indexing below uses absolute register addresses as array offsets;
        // adjacency + base-0 are pinned by the asserts in `regs.rs`.
        const LEN: usize = REG_OUTPUT_EN as usize + 1;
        let mut r = [0u16; LEN];
        self.transport.read_holding(self.slave, REG_V_SET, &mut r)?;
        Ok(Status {
            setpoints: decode_setpoints(
                r[REG_V_SET as usize],
                r[REG_I_SET as usize],
                CURRENT_SCALE,
            ),
            v_out: from_reg_u16(r[REG_V_OUT as usize], 100.0),
            i_out: from_reg_u16(r[REG_I_OUT as usize], CURRENT_SCALE),
            p_out: from_reg_u16(r[REG_P_OUT as usize], POWER_SCALE),
            v_in: from_reg_u16(r[REG_V_IN as usize], 100.0),
            protection: decode_register(
                REG_PROTECT,
                r[REG_PROTECT as usize],
                ProtectionStatus::from_reg,
            )?,
            reg_mode: decode_register(REG_CVCC, r[REG_CVCC as usize], RegMode::from_reg)?,
            output_on: decode_bool(REG_OUTPUT_EN, r[REG_OUTPUT_EN as usize])?,
        })
    }

    pub fn read_voltage_out(&mut self) -> Result<f32, XyError> {
        Ok(from_reg_u16(self.read_one(REG_V_OUT)?, 100.0))
    }
    pub fn read_current_out(&mut self) -> Result<f32, XyError> {
        Ok(from_reg_u16(self.read_one(REG_I_OUT)?, CURRENT_SCALE))
    }
    pub fn read_power_out(&mut self) -> Result<f32, XyError> {
        Ok(from_reg_u16(self.read_one(REG_P_OUT)?, POWER_SCALE))
    }
    pub fn read_voltage_in(&mut self) -> Result<f32, XyError> {
        Ok(from_reg_u16(self.read_one(REG_V_IN)?, 100.0))
    }

    /// Read cumulative output charge, energy, and on-time (registers
    /// 0x0006–0x000C, one transaction).
    ///
    /// XY7025 firmware has not been verified to snapshot the LOW/HIGH counter
    /// pairs atomically, so a read concurrent with low-word rollover may tear.
    pub fn read_totals(&mut self) -> Result<Totals, XyError> {
        let mut r = [0u16; 7];
        self.transport
            .read_holding(self.slave, REG_AH_LOW, &mut r)?;
        Ok(Totals {
            charge_ah: from_reg_u32(r[0], r[1], 1000.0),
            energy_wh: from_reg_u32(r[2], r[3], 1000.0),
            on_time: OnTime {
                hours: r[4],
                minutes: decode_u16_max(REG_OUT_M, r[5], 59)?,
                seconds: decode_u16_max(REG_OUT_S, r[6], 59)?,
            },
        })
    }

    /// Set output voltage (V-SET, register 0x0000). Note: writing a
    /// V-SET higher than the current S-OVP latches OVP immediately —
    /// program protection (see [`Self::set_protection`]) first.
    pub fn set_voltage(&mut self, volts: f32) -> Result<(), XyError> {
        let raw = to_reg_u16(volts, 100.0, VOLTAGE_SET_RANGE, InputField::VoltageSetpoint)?;
        self.write_one(REG_V_SET, raw)
    }

    pub fn set_current_limit(&mut self, amps: f32) -> Result<(), XyError> {
        let raw = to_reg_u16(
            amps,
            CURRENT_SCALE,
            CURRENT_SET_RANGE,
            InputField::CurrentSetpoint,
        )?;
        self.write_one(REG_I_SET, raw)
    }

    /// Program LVP / OVP / OCP into the active group's protection
    /// registers (0x0052–0x0054) in one bulk write.
    pub fn set_protection(&mut self, l: SafetyLimits) -> Result<(), XyError> {
        let values = [
            to_reg_u16(l.lvp_v, 100.0, LVP_RANGE, InputField::LowVoltageProtection)?,
            to_reg_u16(l.ovp_v, 100.0, OVP_RANGE, InputField::OverVoltageProtection)?,
            to_reg_u16(
                l.ocp_a,
                CURRENT_SCALE,
                OCP_RANGE,
                InputField::OverCurrentProtection,
            )?,
        ];
        self.transport
            .write_multiple_holdings(self.slave, REG_S_LVP, &values)?;
        Ok(())
    }

    /// Read LVP / OVP / OCP from the active group (0x0052–0x0054).
    pub fn read_protection(&mut self) -> Result<SafetyLimits, XyError> {
        let mut r = [0u16; 3];
        self.transport.read_holding(self.slave, REG_S_LVP, &mut r)?;
        Ok(SafetyLimits {
            lvp_v: from_reg_u16(r[0], 100.0),
            ovp_v: from_reg_u16(r[1], 100.0),
            ocp_a: from_reg_u16(r[2], CURRENT_SCALE),
        })
    }

    /// Power-on output state (S-INI, register 0x005D). `false` = OFF
    /// at boot, `true` = ON. Persists in EEPROM. `false` is the safe
    /// default after an unexpected power loss — the buck stays disabled
    /// until explicitly re-enabled.
    pub fn set_power_on_output(&mut self, on: bool) -> Result<(), XyError> {
        self.write_one(REG_S_INI, on as u16)
    }

    pub fn read_power_on_output(&mut self) -> Result<bool, XyError> {
        let value = self.read_one(REG_S_INI)?;
        decode_bool(REG_S_INI, value)
    }

    /// Read the output-enable register (ONOFF, 0x0012).
    pub fn read_output(&mut self) -> Result<bool, XyError> {
        let value = self.read_one(REG_OUTPUT_EN)?;
        decode_bool(REG_OUTPUT_EN, value)
    }

    pub fn set_output(&mut self, on: bool) -> Result<(), XyError> {
        self.write_one(REG_OUTPUT_EN, on as u16)
    }

    /// Read the latched protection cause (PROTECT, 0x0010). While the
    /// output is on, this register is necessarily `Normal` — only worth
    /// reading after observing OUTPUT_EN go low.
    pub fn read_protection_status(&mut self) -> Result<ProtectionStatus, XyError> {
        let value = self.read_one(REG_PROTECT)?;
        decode_register(REG_PROTECT, value, ProtectionStatus::from_reg)
    }

    /// Clear a latched protection cause (write 0 to PROTECT). This
    /// stops the front-panel blink but does **not** re-enable the
    /// output — call [`Self::set_output`] separately.
    pub fn clear_protection_status(&mut self) -> Result<(), XyError> {
        self.write_one(REG_PROTECT, 0)
    }

    pub fn read_reg_mode(&mut self) -> Result<RegMode, XyError> {
        let value = self.read_one(REG_CVCC)?;
        decode_register(REG_CVCC, value, RegMode::from_reg)
    }

    /// Read the verified internal sensor and its unit in one register snapshot.
    /// The external-probe register remains available through
    /// [`Self::read_raw_holding`], but is not exposed at the high level because
    /// its connected-probe scale has not been verified on hardware.
    pub fn read_temperature_internal(&mut self) -> Result<Temperature, XyError> {
        const LEN: usize = (REG_TEMP_UNIT - REG_T_IN + 1) as usize;
        let mut r = [0u16; LEN];
        self.transport.read_holding(self.slave, REG_T_IN, &mut r)?;
        let unit = decode_register(
            REG_TEMP_UNIT,
            r[(REG_TEMP_UNIT - REG_T_IN) as usize],
            TempUnit::from_reg,
        )?;
        Ok(Temperature {
            value: from_reg_u16(r[0], 10.0),
            unit,
        })
    }

    pub fn read_temp_unit(&mut self) -> Result<TempUnit, XyError> {
        let value = self.read_one(REG_TEMP_UNIT)?;
        decode_register(REG_TEMP_UNIT, value, TempUnit::from_reg)
    }
    pub fn set_temp_unit(&mut self, unit: TempUnit) -> Result<(), XyError> {
        self.write_one(REG_TEMP_UNIT, unit.code())
    }

    pub fn read_temp_offset_internal(&mut self) -> Result<f32, XyError> {
        Ok(from_reg_i16(self.read_one(REG_T_IN_OFFSET)?, 10.0))
    }
    pub fn read_temp_offset_external(&mut self) -> Result<f32, XyError> {
        Ok(from_reg_i16(self.read_one(REG_T_EX_OFFSET)?, 10.0))
    }
    // Setters intentionally absent: XY7025 firmware silently ignores
    // Modbus writes to T-IN/T-EX offset (verified empirically — every
    // non-zero raw write reads back as 0). The offset can only be
    // changed from the front-panel calibration menu, so a Modbus setter
    // would lie about success. Use the front panel instead.

    pub fn read_lock(&mut self) -> Result<bool, XyError> {
        let value = self.read_one(REG_LOCK)?;
        decode_bool(REG_LOCK, value)
    }
    pub fn set_lock(&mut self, locked: bool) -> Result<(), XyError> {
        self.write_one(REG_LOCK, locked as u16)
    }

    /// Backlight brightness (1–5).
    pub fn read_backlight(&mut self) -> Result<u8, XyError> {
        let value = self.read_one(REG_BACKLIGHT)?;
        if (1..=5).contains(&value) {
            Ok(value as u8)
        } else {
            Err(XyError::InvalidRegisterValue {
                register: REG_BACKLIGHT,
                value,
            })
        }
    }
    /// Set backlight brightness. Documented range is 0–5, but XY7025
    /// firmware floors writes at 1 (writing 0 reads back as 1) — the
    /// display can't be fully extinguished via Modbus.
    pub fn set_backlight(&mut self, level: u8) -> Result<(), XyError> {
        if !(1..=5).contains(&level) {
            return Err(InputError::OutOfRange {
                field: InputField::Backlight,
            }
            .into());
        }
        self.write_one(REG_BACKLIGHT, level as u16)
    }

    /// Off-screen timeout in minutes.
    pub fn read_sleep_minutes(&mut self) -> Result<u16, XyError> {
        let value = self.read_one(REG_SLEEP)?;
        decode_u16_max(REG_SLEEP, value, 9)
    }
    /// Set off-screen timeout in minutes. XY7025 firmware caps the
    /// stored value at 9; any write ≥10 reads back as 9. Pass 0 to
    /// disable auto-off.
    pub fn set_sleep_minutes(&mut self, minutes: u16) -> Result<(), XyError> {
        validate_u16_max(minutes, 9, InputField::SleepTimeout)?;
        self.write_one(REG_SLEEP, minutes)
    }

    /// Buzzer enable. Often unimplemented in firmware.
    pub fn read_buzzer(&mut self) -> Result<bool, XyError> {
        let value = self.read_one(REG_BUZZER)?;
        decode_bool(REG_BUZZER, value)
    }
    pub fn set_buzzer(&mut self, on: bool) -> Result<(), XyError> {
        self.write_one(REG_BUZZER, on as u16)
    }

    /// Product number (e.g. `0x6500` on XY7025).
    pub fn read_model(&mut self) -> Result<u16, XyError> {
        self.read_one(REG_MODEL)
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
        self.read_one(REG_VERSION)
    }

    /// Read the device's currently configured Modbus slave address.
    /// It may briefly differ from the address used by this driver while
    /// reconfiguring.
    pub fn read_slave_address(&mut self) -> Result<u8, XyError> {
        let value = self.read_one(REG_SLAVE_ADDR)?;
        let address = u8::try_from(value).map_err(|_| XyError::InvalidRegisterValue {
            register: REG_SLAVE_ADDR,
            value,
        })?;
        validate_slave(address).map_err(|_| XyError::InvalidRegisterValue {
            register: REG_SLAVE_ADDR,
            value,
        })?;
        Ok(address)
    }
    /// Write a new slave address. Takes effect after the device resets.
    pub fn set_slave_address(&mut self, addr: u8) -> Result<(), XyError> {
        validate_slave(addr)?;
        self.write_one(REG_SLAVE_ADDR, addr as u16)
    }

    pub fn read_baud_rate(&mut self) -> Result<BaudRate, XyError> {
        let value = self.read_one(REG_BAUD_CODE)?;
        decode_register(REG_BAUD_CODE, value, BaudRate::from_code)
    }
    /// Write a new baud-rate code. Takes effect after the device resets.
    pub fn set_baud_rate(&mut self, baud: BaudRate) -> Result<(), XyError> {
        self.write_one(REG_BAUD_CODE, baud.code())
    }

    /// Recall a stored memory group (M0–M9) into the live operating set.
    /// `n == 0` is a no-op on the device (M0 is already current).
    pub fn recall_group(&mut self, n: u8) -> Result<(), XyError> {
        validate_group(n)?;
        self.write_one(REG_EXTRACT_M, n as u16)
    }

    /// Read all 14 registers of memory group `n` (0–9).
    ///
    /// F-C must not be changed concurrently through another controller or the
    /// front panel while this operation is in progress.
    pub fn read_group(&mut self, n: u8) -> Result<GroupParams, XyError> {
        validate_group(n)?;
        let unit = self.read_temp_unit()?;
        self.read_group_in_unit(n, unit)
    }

    fn read_group_in_unit(&mut self, n: u8, unit: TempUnit) -> Result<GroupParams, XyError> {
        let mut r = [0u16; GROUP_LEN as usize];
        let addr = group_addr(n);
        self.transport.read_holding(self.slave, addr, &mut r)?;
        decode_group(&r, addr, unit)
    }

    /// Write all 14 registers of memory group `n` (0–9), then return the values
    /// stored by the device after temperature conversion and rounding.
    ///
    /// For M0 this updates the live operating set. Valid V-SET/S-OVP crossings
    /// are staged safely while the FC10 application order remains unverified.
    /// F-C must not be changed concurrently through another controller or the
    /// front panel while this operation is in progress.
    pub fn write_group(&mut self, n: u8, p: &GroupParams) -> Result<GroupParams, XyError> {
        validate_group(n)?;
        let encoding = encode_group(p)?;
        let unit = self.read_temp_unit()?;
        let regs = encoding.for_unit(unit)?;
        if n == 0 {
            const V_SET_OFFSET: usize = 0;
            const S_OVP_OFFSET: usize = (REG_S_OVP - GROUP_BASE) as usize;
            let mut current = [0u16; S_OVP_OFFSET + 1];
            self.transport
                .read_holding(self.slave, GROUP_BASE, &mut current)?;
            if regs[V_SET_OFFSET] > current[S_OVP_OFFSET] {
                self.write_one(REG_S_OVP, regs[S_OVP_OFFSET])?;
            }
            if regs[S_OVP_OFFSET] < current[V_SET_OFFSET] {
                self.write_one(REG_V_SET, regs[V_SET_OFFSET])?;
            }
        }
        self.transport
            .write_multiple_holdings(self.slave, group_addr(n), &regs)?;
        self.read_group_in_unit(n, unit)
    }

    fn read_one(&mut self, addr: u16) -> Result<u16, XyError> {
        let mut r = [0u16; 1];
        self.transport.read_holding(self.slave, addr, &mut r)?;
        Ok(r[0])
    }

    fn write_one(&mut self, addr: u16, value: u16) -> Result<(), XyError> {
        self.transport
            .write_single_holding(self.slave, addr, value)?;
        Ok(())
    }
}

fn decode_group(
    r: &[u16; GROUP_LEN as usize],
    addr: u16,
    unit: TempUnit,
) -> Result<GroupParams, XyError> {
    let [
        v_set,
        i_set,
        s_lvp,
        s_ovp,
        s_ocp,
        s_opp,
        s_ohp_h,
        s_ohp_m,
        s_oah_low,
        s_oah_high,
        s_owh_low,
        s_owh_high,
        s_otp,
        s_ini,
    ] = *r;
    Ok(GroupParams {
        setpoints: decode_setpoints(v_set, i_set, CURRENT_SCALE),
        safety_limits: SafetyLimits {
            lvp_v: from_reg_u16(s_lvp, 100.0),
            ovp_v: from_reg_u16(s_ovp, 100.0),
            ocp_a: from_reg_u16(s_ocp, CURRENT_SCALE),
        },
        s_opp_w: from_reg_u16(s_opp, OPP_SCALE),
        s_ohp_h: decode_u16_max(addr + 6, s_ohp_h, 99)?,
        s_ohp_m: decode_u16_max(addr + 7, s_ohp_m, 59)?,
        s_oah_ah: from_reg_u32(s_oah_low, s_oah_high, 1000.0),
        s_owh_wh: from_reg_u32(s_owh_low, s_owh_high, 100.0),
        s_otp: Temperature {
            value: from_reg_u16(s_otp, 1.0),
            unit,
        },
        power_on_output: decode_bool(addr + 13, s_ini)?,
    })
}

#[derive(Debug)]
struct GroupEncoding {
    values: [u16; GROUP_LEN as usize],
    s_otp: Temperature,
}

impl GroupEncoding {
    fn for_unit(mut self, unit: TempUnit) -> Result<[u16; GROUP_LEN as usize], InputError> {
        self.values[12] = encode_group_otp(self.s_otp, unit)?;
        Ok(self.values)
    }
}

fn encode_group(p: &GroupParams) -> Result<GroupEncoding, InputError> {
    if !p.s_otp.value.is_finite() {
        return Err(InputError::NonFinite {
            field: InputField::OverTemperatureProtection,
        });
    }
    validate_u16_max(p.s_ohp_h, 99, InputField::OutputTimeHours)?;
    validate_u16_max(p.s_ohp_m, 59, InputField::OutputTimeMinutes)?;
    let oah = to_reg_u32(
        p.s_oah_ah,
        1000.0,
        CHARGE_LIMIT_RANGE,
        InputField::ChargeLimit,
    )?;
    let owh = to_reg_u32(
        p.s_owh_wh,
        100.0,
        ENERGY_LIMIT_RANGE,
        InputField::EnergyLimit,
    )?;
    let values = [
        to_reg_u16(
            p.setpoints.v_set,
            100.0,
            VOLTAGE_SET_RANGE,
            InputField::VoltageSetpoint,
        )?,
        to_reg_u16(
            p.setpoints.i_set,
            CURRENT_SCALE,
            CURRENT_SET_RANGE,
            InputField::CurrentSetpoint,
        )?,
        to_reg_u16(
            p.safety_limits.lvp_v,
            100.0,
            LVP_RANGE,
            InputField::LowVoltageProtection,
        )?,
        to_reg_u16(
            p.safety_limits.ovp_v,
            100.0,
            OVP_RANGE,
            InputField::OverVoltageProtection,
        )?,
        to_reg_u16(
            p.safety_limits.ocp_a,
            CURRENT_SCALE,
            OCP_RANGE,
            InputField::OverCurrentProtection,
        )?,
        to_reg_u16(
            p.s_opp_w,
            OPP_SCALE,
            OPP_RANGE,
            InputField::OverPowerProtection,
        )?,
        p.s_ohp_h,
        p.s_ohp_m,
        oah.low,
        oah.high,
        owh.low,
        owh.high,
        0,
        p.power_on_output as u16,
    ];
    if values[0] > values[3] {
        return Err(InputError::VoltageSetpointAboveProtection);
    }
    Ok(GroupEncoding {
        values,
        s_otp: p.s_otp,
    })
}

fn encode_group_otp(temperature: Temperature, unit: TempUnit) -> Result<u16, InputError> {
    let temperature = temperature.convert_to(unit);
    let max = match unit {
        TempUnit::Celsius => 110.0,
        TempUnit::Fahrenheit => 230.0,
    };
    to_reg_u16(
        temperature.value,
        1.0,
        ValueRange { min: 0.0, max },
        InputField::OverTemperatureProtection,
    )
}

#[cfg(test)]
mod tests;
