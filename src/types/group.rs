//! Memory-group parameters (M0–M9).

use crate::types::status::{SafetyLimits, Setpoints};

/// All 14 registers of a memory group (M0–M9). Field order matches the
/// on-wire register order.
///
/// Note the cumulative limits use *different* scales: `s_oah_ah` is the
/// charge limit in Ah (encoded raw / 1000), `s_owh_wh` the energy limit
/// in Wh (encoded raw / 100 — 10 mWh units, *not* the 1 mWh scale used
/// by the cumulative WH counters at 0x0008/0x0009). The XY firmware
/// stores the energy threshold in coarser units to extend the 32-bit
/// range to ~42.9 GWh.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GroupParams {
    pub setpoints: Setpoints,
    pub safety_limits: SafetyLimits,
    /// Over-power threshold in W. Resolution depends on model (1 W on
    /// XY7025, 0.1 W on the SK family).
    pub s_opp_w: f32,
    /// Output-on time limit, hours.
    pub s_ohp_h: u16,
    /// Output-on time limit, minutes.
    pub s_ohp_m: u16,
    /// Cumulative-charge limit in Ah.
    pub s_oah_ah: f64,
    /// Cumulative-energy limit in Wh.
    pub s_owh_wh: f64,
    /// Over-temperature threshold (°C/°F per [`crate::TempUnit`]).
    pub s_otp: f32,
    /// Power-on output state. `false` = boot with output OFF, `true` = ON.
    pub power_on_output: bool,
}
