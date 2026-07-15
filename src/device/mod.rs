//! High-level device API. One method per logical operation; all reads
//! and writes go through the [`crate::ModbusTransport`].

pub(crate) mod error;

use crate::device::error::{InputError, InputField, XyError};
use crate::regs::*;
use crate::transport::{ModbusTransport, RtuError};
use crate::types::enums::{BaudRate, ProtectionStatus, RegMode, TempUnit};
use crate::types::group::GroupParams;
use crate::types::model::{Model, ModelRange, ScaleCheck};
use crate::types::status::{OnTime, SafetyLimits, Setpoints, Status, Temperatures, Totals};

fn to_reg_u16(v: f32, scale: f32, range: ModelRange, field: InputField) -> Result<u16, InputError> {
    if !v.is_finite() {
        return Err(InputError::NonFinite { field });
    }
    let scaled = v * scale;
    if !range.contains(v as f64) || scaled > u16::MAX as f32 + 0.5 {
        return Err(InputError::OutOfRange { field });
    }
    Ok((scaled + 0.5) as u16)
}

fn from_reg_u16(raw: u16, scale: f32) -> f32 {
    raw as f32 / scale
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
    range: ModelRange,
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

/// High-level driver for XY7025 and compatible 14-register profiles.
///
/// Construct with [`Xy::new`] (default slave `0x01`) or
/// [`Xy::with_slave`]. The [`Model`] supplies fixed-point scales and physical
/// write limits. Passing the wrong scale family silently yields readings off by
/// 10×, so call [`Self::verify_scale_family`] during bring-up.
#[derive(Debug)]
pub struct Xy<T: ModbusTransport> {
    transport: T,
    slave: u8,
    model: Model,
}

impl<T: ModbusTransport> Xy<T> {
    /// Wrap a transport using the default slave address (`0x01`).
    pub fn new(transport: T, model: Model) -> Self {
        model.assert_valid();
        Self {
            transport,
            slave: DEFAULT_SLAVE,
            model,
        }
    }

    pub fn with_slave(transport: T, model: Model, slave: u8) -> Result<Self, InputError> {
        model.assert_valid();
        validate_slave(slave)?;
        Ok(Self {
            transport,
            slave,
            model,
        })
    }

    /// Read holding registers through the configured transport and slave.
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
        Ok(Setpoints {
            v_set: from_reg_u16(r[0], 100.0),
            i_set: from_reg_u16(r[1], self.model.current_scale()),
        })
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
        let i_scale = self.model.current_scale();
        let p_scale = self.model.power_scale();
        Ok(Status {
            v_set: from_reg_u16(r[REG_V_SET as usize], 100.0),
            i_set: from_reg_u16(r[REG_I_SET as usize], i_scale),
            v_out: from_reg_u16(r[REG_V_OUT as usize], 100.0),
            i_out: from_reg_u16(r[REG_I_OUT as usize], i_scale),
            p_out: from_reg_u16(r[REG_P_OUT as usize], p_scale),
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
        Ok(from_reg_u16(
            self.read_one(REG_I_OUT)?,
            self.model.current_scale(),
        ))
    }
    pub fn read_power_out(&mut self) -> Result<f32, XyError> {
        Ok(from_reg_u16(
            self.read_one(REG_P_OUT)?,
            self.model.power_scale(),
        ))
    }
    pub fn read_voltage_in(&mut self) -> Result<f32, XyError> {
        Ok(from_reg_u16(self.read_one(REG_V_IN)?, 100.0))
    }

    /// Read cumulative output charge, energy, and on-time (registers
    /// 0x0006–0x000C, one transaction).
    pub fn read_totals(&mut self) -> Result<Totals, XyError> {
        let mut r = [0u16; 7];
        self.transport
            .read_holding(self.slave, REG_AH_LOW, &mut r)?;
        Ok(Totals {
            charge_ah: from_reg_u32(r[0], r[1], 1000.0),
            energy_wh: from_reg_u32(r[2], r[3], 1000.0),
            on_time: OnTime {
                hours: r[4],
                minutes: r[5],
                seconds: r[6],
            },
        })
    }

    /// Set output voltage (V-SET, register 0x0000). Note: writing a
    /// V-SET higher than the current S-OVP latches OVP immediately —
    /// program protection (see [`Self::set_protection`]) first.
    pub fn set_voltage(&mut self, volts: f32) -> Result<(), XyError> {
        let raw = to_reg_u16(
            volts,
            100.0,
            self.model.limits().voltage_set_v,
            InputField::VoltageSetpoint,
        )?;
        self.write_one(REG_V_SET, raw)
    }

    pub fn set_current_limit(&mut self, amps: f32) -> Result<(), XyError> {
        let raw = to_reg_u16(
            amps,
            self.model.current_scale(),
            self.model.limits().current_set_a,
            InputField::CurrentSetpoint,
        )?;
        self.write_one(REG_I_SET, raw)
    }

    /// Program LVP / OVP / OCP into the active group's protection
    /// registers (0x0052–0x0054) in one bulk write.
    pub fn set_protection(&mut self, l: SafetyLimits) -> Result<(), XyError> {
        let limits = self.model.limits();
        let values = [
            to_reg_u16(
                l.lvp_v,
                100.0,
                limits.lvp_v,
                InputField::LowVoltageProtection,
            )?,
            to_reg_u16(
                l.ovp_v,
                100.0,
                limits.ovp_v,
                InputField::OverVoltageProtection,
            )?,
            to_reg_u16(
                l.ocp_a,
                self.model.current_scale(),
                limits.ocp_a,
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
            ocp_a: from_reg_u16(r[2], self.model.current_scale()),
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

    /// Both fields are in the unit selected by [`Self::read_temp_unit`].
    /// See [`Temperatures`] for caveats on the external field — its
    /// decoding scale is unverified on real hardware.
    pub fn read_temperatures(&mut self) -> Result<Temperatures, XyError> {
        let mut r = [0u16; 2];
        self.transport.read_holding(self.slave, REG_T_IN, &mut r)?;
        Ok(Temperatures {
            internal: from_reg_u16(r[0], 10.0),
            external: (r[1] != 8888).then(|| from_reg_u16(r[1], 10.0)),
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
        validate_u16_max(value, 9, InputField::SleepTimeout).map_err(|_| {
            XyError::InvalidRegisterValue {
                register: REG_SLEEP,
                value,
            }
        })?;
        Ok(value)
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
    /// configured [`Model`]'s scale family. This catches the footgun where
    /// `read_status().i_out` silently comes back 10× off, but it does not prove
    /// exact hardware identity or physical limits.
    ///
    /// Returns [`ScaleCheck::Inconclusive`] for unknown device codes or custom
    /// profiles without a canonical code.
    pub fn verify_scale_family(&mut self) -> Result<ScaleCheck, XyError> {
        let device_code = self.read_model()?;
        if self.model.recognizes_scale_code(device_code) {
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
    pub fn read_group(&mut self, n: u8) -> Result<GroupParams, XyError> {
        validate_group(n)?;
        let mut r = [0u16; GROUP_LEN as usize];
        let addr = group_addr(n);
        self.transport.read_holding(self.slave, addr, &mut r)?;
        decode_group(&r, self.model, addr)
    }

    /// Write all 14 registers of memory group `n` (0–9) in one bulk
    /// transaction. For M0 this updates the live operating set;
    /// otherwise it programs EEPROM and takes effect on
    /// [`Self::recall_group`].
    pub fn write_group(&mut self, n: u8, p: &GroupParams) -> Result<(), XyError> {
        validate_group(n)?;
        let regs = encode_group(p, self.model)?;
        self.transport
            .write_multiple_holdings(self.slave, group_addr(n), &regs)?;
        Ok(())
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
    model: Model,
    addr: u16,
) -> Result<GroupParams, XyError> {
    let i_scale = model.current_scale();
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
        setpoints: Setpoints {
            v_set: from_reg_u16(v_set, 100.0),
            i_set: from_reg_u16(i_set, i_scale),
        },
        safety_limits: SafetyLimits {
            lvp_v: from_reg_u16(s_lvp, 100.0),
            ovp_v: from_reg_u16(s_ovp, 100.0),
            ocp_a: from_reg_u16(s_ocp, i_scale),
        },
        s_opp_w: from_reg_u16(s_opp, model.opp_scale()),
        s_ohp_h,
        s_ohp_m,
        s_oah_ah: from_reg_u32(s_oah_low, s_oah_high, 1000.0),
        s_owh_wh: from_reg_u32(s_owh_low, s_owh_high, 100.0),
        // XY7025 stores S-OTP in displayed degrees rather than tenths.
        s_otp: from_reg_u16(s_otp, 1.0),
        power_on_output: decode_bool(addr + 13, s_ini)?,
    })
}

fn encode_group(p: &GroupParams, model: Model) -> Result<[u16; GROUP_LEN as usize], InputError> {
    let i_scale = model.current_scale();
    let limits = model.limits();
    validate_u16_max(p.s_ohp_h, 99, InputField::OutputTimeHours)?;
    validate_u16_max(p.s_ohp_m, 59, InputField::OutputTimeMinutes)?;
    let oah = to_reg_u32(
        p.s_oah_ah,
        1000.0,
        limits.charge_limit_ah,
        InputField::ChargeLimit,
    )?;
    let owh = to_reg_u32(
        p.s_owh_wh,
        100.0,
        limits.energy_limit_wh,
        InputField::EnergyLimit,
    )?;
    Ok([
        to_reg_u16(
            p.setpoints.v_set,
            100.0,
            limits.voltage_set_v,
            InputField::VoltageSetpoint,
        )?,
        to_reg_u16(
            p.setpoints.i_set,
            i_scale,
            limits.current_set_a,
            InputField::CurrentSetpoint,
        )?,
        to_reg_u16(
            p.safety_limits.lvp_v,
            100.0,
            limits.lvp_v,
            InputField::LowVoltageProtection,
        )?,
        to_reg_u16(
            p.safety_limits.ovp_v,
            100.0,
            limits.ovp_v,
            InputField::OverVoltageProtection,
        )?,
        to_reg_u16(
            p.safety_limits.ocp_a,
            i_scale,
            limits.ocp_a,
            InputField::OverCurrentProtection,
        )?,
        to_reg_u16(
            p.s_opp_w,
            model.opp_scale(),
            limits.opp_w,
            InputField::OverPowerProtection,
        )?,
        p.s_ohp_h,
        p.s_ohp_m,
        oah.low,
        oah.high,
        owh.low,
        owh.high,
        to_reg_u16(
            p.s_otp,
            1.0,
            ModelRange {
                min: 0.0,
                max: 230.0,
            },
            InputField::OverTemperatureProtection,
        )?,
        p.power_on_output as u16,
    ])
}

#[cfg(test)]
mod tests;
