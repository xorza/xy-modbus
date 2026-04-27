//! High-level device API. One method per logical operation; all reads
//! and writes go through the [`crate::ModbusTransport`].

use crate::regs::*;
use crate::transport::{ModbusTransport, RtuError};
use crate::types::{
    BaudRate, GroupParams, Model, ModelCheck, OnTime, ProtectionStatus, RegMode, SafetyLimits,
    Setpoints, Status, TempUnit, Totals,
};

// Fixed-point conversion. Inputs are clamped to u16 — caller is responsible
// for staying within the device's documented ranges (per-model V/A/W limits).
// Negative or NaN inputs are a logic error: the device's V/A/W are all
// non-negative, and silently saturating NaN to 0 would mask bad math upstream.
fn to_reg_u16(v: f32, scale: f32) -> u16 {
    assert!(v >= 0.0, "to_reg_u16: negative or NaN input ({v})");
    let r = (v * scale + 0.5) as i32;
    r.clamp(0, u16::MAX as i32) as u16
}

fn from_reg_u16(raw: u16, scale: f32) -> f32 {
    raw as f32 / scale
}

// Signed variants for registers the firmware encodes as i16 two's
// complement (currently only the temperature calibration offsets at
// 0x001A / 0x001B — both can legitimately be negative).
fn to_reg_i16(v: f32, scale: f32) -> u16 {
    assert!(!v.is_nan(), "to_reg_i16: NaN input");
    let scaled = v * scale;
    // Round-half-away-from-zero without pulling in libm's `round`.
    let r = if scaled >= 0.0 {
        scaled + 0.5
    } else {
        scaled - 0.5
    } as i32;
    r.clamp(i16::MIN as i32, i16::MAX as i32) as i16 as u16
}

fn from_reg_i16(raw: u16, scale: f32) -> f32 {
    raw as i16 as f32 / scale
}

/// `low` / `high` match the on-wire register pair order (low word at the
/// lower address — see `REG_AH_LOW` / `REG_AH_HIGH`).
fn from_reg_u32(low: u16, high: u16, scale: f32) -> f32 {
    let raw = ((high as u32) << 16) | low as u32;
    raw as f32 / scale
}

/// Returns `(low, high)` words, matching the on-wire register pair order.
fn to_reg_u32(v: f32, scale: f32) -> (u16, u16) {
    assert!(v >= 0.0, "to_reg_u32: negative or NaN input ({v})");
    let r = (v * scale + 0.5) as i64;
    let r = r.clamp(0, u32::MAX as i64) as u32;
    (r as u16, (r >> 16) as u16)
}

/// Driver for the XY-series buck converter.
///
/// Construct with [`Xy::new`] (default slave `0x01`) or
/// [`Xy::with_slave`]. The [`Model`] selects per-variant scales for
/// I-SET / IOUT / S-OCP / POWER / S-OPP — passing the wrong model
/// silently yields readings off by 10×, so cross-check
/// [`Self::read_model`] against your hardware.
#[derive(Debug)]
pub struct Xy<T: ModbusTransport> {
    transport: T,
    slave: u8,
    model: Model,
}

impl<T: ModbusTransport> Xy<T> {
    /// Wrap a transport using the default slave address (`0x01`).
    pub fn new(transport: T, model: Model) -> Self {
        Self::with_slave(transport, model, DEFAULT_SLAVE)
    }

    pub fn with_slave(transport: T, model: Model, slave: u8) -> Self {
        Self {
            transport,
            slave,
            model,
        }
    }

    pub fn slave(&self) -> u8 {
        self.slave
    }

    pub fn model(&self) -> Model {
        self.model
    }

    /// Borrow the underlying transport.
    pub fn transport(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Consume the device and return the inner transport.
    pub fn into_transport(self) -> T {
        self.transport
    }

    // ─── Status & live readings ──────────────────────────────────────────────

    /// Read setpoints (V-SET, I-SET) — registers 0x0000–0x0001.
    pub fn read_setpoints(&mut self) -> Result<Setpoints, RtuError> {
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
    pub fn read_status(&mut self) -> Result<Status, RtuError> {
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
            protection: ProtectionStatus::from_reg(r[REG_PROTECT as usize]),
            reg_mode: RegMode::from_reg(r[REG_CVCC as usize]),
            output_on: r[REG_OUTPUT_EN as usize] != 0,
        })
    }

    pub fn read_voltage_out(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg_u16(self.read_one(REG_V_OUT)?, 100.0))
    }
    pub fn read_current_out(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg_u16(
            self.read_one(REG_I_OUT)?,
            self.model.current_scale(),
        ))
    }
    pub fn read_power_out(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg_u16(
            self.read_one(REG_P_OUT)?,
            self.model.power_scale(),
        ))
    }
    pub fn read_voltage_in(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg_u16(self.read_one(REG_V_IN)?, 100.0))
    }

    // ─── Cumulative totals ───────────────────────────────────────────────────

    /// Read cumulative output charge, energy, and on-time (registers
    /// 0x0006–0x000C, one transaction).
    pub fn read_totals(&mut self) -> Result<Totals, RtuError> {
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

    // ─── Setpoint shortcuts ──────────────────────────────────────────────────

    /// Set output voltage (V-SET, register 0x0000). Note: writing a
    /// V-SET higher than the current S-OVP latches OVP immediately —
    /// program protection (see [`Self::set_protection`]) first.
    pub fn set_voltage(&mut self, volts: f32) -> Result<(), RtuError> {
        self.write_one(REG_V_SET, to_reg_u16(volts, 100.0))
    }

    pub fn set_current_limit(&mut self, amps: f32) -> Result<(), RtuError> {
        self.write_one(REG_I_SET, to_reg_u16(amps, self.model.current_scale()))
    }

    /// Program LVP / OVP / OCP into the active group's protection
    /// registers (0x0052–0x0054) in one bulk write.
    pub fn set_protection(&mut self, l: SafetyLimits) -> Result<(), RtuError> {
        let values = [
            to_reg_u16(l.lvp_v, 100.0),
            to_reg_u16(l.ovp_v, 100.0),
            to_reg_u16(l.ocp_a, self.model.current_scale()),
        ];
        self.transport
            .write_multiple_holdings(self.slave, REG_S_LVP, &values)
    }

    /// Read LVP / OVP / OCP from the active group (0x0052–0x0054).
    pub fn read_protection(&mut self) -> Result<SafetyLimits, RtuError> {
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
    pub fn set_power_on_output(&mut self, on: bool) -> Result<(), RtuError> {
        self.write_one(REG_S_INI, on as u16)
    }

    pub fn read_power_on_output(&mut self) -> Result<bool, RtuError> {
        Ok(self.read_one(REG_S_INI)? != 0)
    }

    // ─── Output enable & protection status ───────────────────────────────────

    /// Read the output-enable register (ONOFF, 0x0012).
    pub fn read_output(&mut self) -> Result<bool, RtuError> {
        Ok(self.read_one(REG_OUTPUT_EN)? != 0)
    }

    pub fn set_output(&mut self, on: bool) -> Result<(), RtuError> {
        self.write_one(REG_OUTPUT_EN, on as u16)
    }

    /// Read the latched protection cause (PROTECT, 0x0010). While the
    /// output is on, this register is necessarily `Normal` — only worth
    /// reading after observing OUTPUT_EN go low.
    pub fn read_protection_status(&mut self) -> Result<ProtectionStatus, RtuError> {
        Ok(ProtectionStatus::from_reg(self.read_one(REG_PROTECT)?))
    }

    /// Clear a latched protection cause (write 0 to PROTECT). This
    /// stops the front-panel blink but does **not** re-enable the
    /// output — call [`Self::set_output`] separately.
    pub fn clear_protection_status(&mut self) -> Result<(), RtuError> {
        self.write_one(REG_PROTECT, 0)
    }

    pub fn read_reg_mode(&mut self) -> Result<RegMode, RtuError> {
        Ok(RegMode::from_reg(self.read_one(REG_CVCC)?))
    }

    // ─── Temperatures ────────────────────────────────────────────────────────

    /// Returns `(internal, external)` in the unit selected by
    /// [`Self::read_temp_unit`].
    pub fn read_temperatures(&mut self) -> Result<(f32, f32), RtuError> {
        let mut r = [0u16; 2];
        self.transport.read_holding(self.slave, REG_T_IN, &mut r)?;
        Ok((from_reg_u16(r[0], 10.0), from_reg_u16(r[1], 10.0)))
    }

    pub fn read_temp_unit(&mut self) -> Result<TempUnit, RtuError> {
        Ok(TempUnit::from_reg(self.read_one(REG_TEMP_UNIT)?))
    }
    pub fn set_temp_unit(&mut self, unit: TempUnit) -> Result<(), RtuError> {
        self.write_one(REG_TEMP_UNIT, unit.to_reg())
    }

    pub fn read_temp_offset_internal(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg_i16(self.read_one(REG_T_IN_OFFSET)?, 10.0))
    }
    pub fn set_temp_offset_internal(&mut self, offset: f32) -> Result<(), RtuError> {
        self.write_one(REG_T_IN_OFFSET, to_reg_i16(offset, 10.0))
    }
    pub fn read_temp_offset_external(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg_i16(self.read_one(REG_T_EX_OFFSET)?, 10.0))
    }
    pub fn set_temp_offset_external(&mut self, offset: f32) -> Result<(), RtuError> {
        self.write_one(REG_T_EX_OFFSET, to_reg_i16(offset, 10.0))
    }

    // ─── Front panel & misc ──────────────────────────────────────────────────

    pub fn read_lock(&mut self) -> Result<bool, RtuError> {
        Ok(self.read_one(REG_LOCK)? != 0)
    }
    pub fn set_lock(&mut self, locked: bool) -> Result<(), RtuError> {
        self.write_one(REG_LOCK, locked as u16)
    }

    /// Backlight brightness (0–5).
    pub fn read_backlight(&mut self) -> Result<u8, RtuError> {
        Ok(self.read_one(REG_BACKLIGHT)? as u8)
    }
    pub fn set_backlight(&mut self, level: u8) -> Result<(), RtuError> {
        self.write_one(REG_BACKLIGHT, level as u16)
    }

    /// Off-screen timeout in minutes.
    pub fn read_sleep_minutes(&mut self) -> Result<u16, RtuError> {
        self.read_one(REG_SLEEP)
    }
    pub fn set_sleep_minutes(&mut self, minutes: u16) -> Result<(), RtuError> {
        self.write_one(REG_SLEEP, minutes)
    }

    /// Buzzer enable. Often unimplemented in firmware.
    pub fn read_buzzer(&mut self) -> Result<bool, RtuError> {
        Ok(self.read_one(REG_BUZZER)? != 0)
    }
    pub fn set_buzzer(&mut self, on: bool) -> Result<(), RtuError> {
        self.write_one(REG_BUZZER, on as u16)
    }

    // ─── Identity & comms config ─────────────────────────────────────────────

    /// Product number (e.g. `0x6500` on XY7025).
    pub fn read_model(&mut self) -> Result<u16, RtuError> {
        self.read_one(REG_MODEL)
    }

    /// Read the device's `MODEL` register and check it against the
    /// configured [`Model`]'s expected family code. Catches the
    /// "wrong scale family" footgun where `read_status().i_out` would
    /// silently come back 10× off — a one-call sanity check at boot.
    ///
    /// Returns [`ModelCheck::Inconclusive`] when the configured model
    /// has no canonical code (SK family, `Custom`) or when the device
    /// reports a code outside the documented set; [`ModelCheck::Match`]
    /// when codes line up; [`ModelCheck::Mismatch`] when they don't —
    /// the dangerous case the caller should refuse to proceed past.
    pub fn verify_model(&mut self) -> Result<ModelCheck, RtuError> {
        let device_code = self.read_model()?;
        match self.model.expected_model_code() {
            Some(expected) if expected == device_code => Ok(ModelCheck::Match { device_code }),
            Some(expected) => Ok(ModelCheck::Mismatch {
                expected_code: expected,
                device_code,
            }),
            None => Ok(ModelCheck::Inconclusive { device_code }),
        }
    }

    /// Firmware version (e.g. `0x0071`).
    pub fn read_version(&mut self) -> Result<u16, RtuError> {
        self.read_one(REG_VERSION)
    }

    /// Read the device's currently configured Modbus slave address.
    /// Note: [`Self::slave`] is the address the *driver* is talking to;
    /// they may differ briefly while reconfiguring.
    pub fn read_slave_address(&mut self) -> Result<u8, RtuError> {
        Ok(self.read_one(REG_SLAVE_ADDR)? as u8)
    }
    /// Write a new slave address. Takes effect after the device resets.
    pub fn set_slave_address(&mut self, addr: u8) -> Result<(), RtuError> {
        self.write_one(REG_SLAVE_ADDR, addr as u16)
    }

    pub fn read_baud_rate(&mut self) -> Result<BaudRate, RtuError> {
        Ok(BaudRate::from_code(self.read_one(REG_BAUD_CODE)?))
    }
    /// Write a new baud-rate code. Takes effect after the device resets.
    pub fn set_baud_rate(&mut self, baud: BaudRate) -> Result<(), RtuError> {
        self.write_one(REG_BAUD_CODE, baud.code())
    }

    /// Recall a stored memory group (M0–M9) into the live operating set.
    /// `n == 0` is a no-op on the device (M0 is already current).
    pub fn recall_group(&mut self, n: u8) -> Result<(), RtuError> {
        assert!(
            n < GROUP_COUNT,
            "group index {n} out of range (0..{GROUP_COUNT})"
        );
        self.write_one(REG_EXTRACT_M, n as u16)
    }

    // ─── Memory groups (M0–M9) ───────────────────────────────────────────────

    /// Read all 14 registers of memory group `n` (0–9).
    pub fn read_group(&mut self, n: u8) -> Result<GroupParams, RtuError> {
        assert!(
            n < GROUP_COUNT,
            "group index {n} out of range (0..{GROUP_COUNT})"
        );
        let mut r = [0u16; GROUP_LEN as usize];
        self.transport
            .read_holding(self.slave, group_addr(n), &mut r)?;
        Ok(decode_group(&r, self.model))
    }

    /// Write all 14 registers of memory group `n` (0–9) in one bulk
    /// transaction. For M0 this updates the live operating set;
    /// otherwise it programs EEPROM and takes effect on
    /// [`Self::recall_group`].
    pub fn write_group(&mut self, n: u8, p: &GroupParams) -> Result<(), RtuError> {
        assert!(
            n < GROUP_COUNT,
            "group index {n} out of range (0..{GROUP_COUNT})"
        );
        let regs = encode_group(p, self.model);
        self.transport
            .write_multiple_holdings(self.slave, group_addr(n), &regs)
    }

    // ─── Internals ───────────────────────────────────────────────────────────

    fn read_one(&mut self, addr: u16) -> Result<u16, RtuError> {
        let mut r = [0u16; 1];
        self.transport.read_holding(self.slave, addr, &mut r)?;
        Ok(r[0])
    }

    fn write_one(&mut self, addr: u16, value: u16) -> Result<(), RtuError> {
        self.transport.write_single_holding(self.slave, addr, value)
    }
}

// ─── Group encode / decode ──────────────────────────────────────────────────

fn decode_group(r: &[u16; GROUP_LEN as usize], model: Model) -> GroupParams {
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
    GroupParams {
        v_set: from_reg_u16(v_set, 100.0),
        i_set: from_reg_u16(i_set, i_scale),
        s_lvp_v: from_reg_u16(s_lvp, 100.0),
        s_ovp_v: from_reg_u16(s_ovp, 100.0),
        s_ocp_a: from_reg_u16(s_ocp, i_scale),
        s_opp_w: from_reg_u16(s_opp, model.opp_scale()),
        s_ohp_h,
        s_ohp_m,
        s_oah_ah: from_reg_u32(s_oah_low, s_oah_high, 1000.0),
        s_owh_wh: from_reg_u32(s_owh_low, s_owh_high, 100.0),
        s_otp: from_reg_u16(s_otp, 10.0),
        power_on_output: s_ini != 0,
    }
}

fn encode_group(p: &GroupParams, model: Model) -> [u16; GROUP_LEN as usize] {
    let i_scale = model.current_scale();
    let (oah_low, oah_high) = to_reg_u32(p.s_oah_ah, 1000.0);
    let (owh_low, owh_high) = to_reg_u32(p.s_owh_wh, 100.0);
    [
        to_reg_u16(p.v_set, 100.0),
        to_reg_u16(p.i_set, i_scale),
        to_reg_u16(p.s_lvp_v, 100.0),
        to_reg_u16(p.s_ovp_v, 100.0),
        to_reg_u16(p.s_ocp_a, i_scale),
        to_reg_u16(p.s_opp_w, model.opp_scale()),
        p.s_ohp_h,
        p.s_ohp_m,
        oah_low,
        oah_high,
        owh_low,
        owh_high,
        to_reg_u16(p.s_otp, 10.0),
        p.power_on_output as u16,
    ]
}

#[cfg(test)]
mod tests;
