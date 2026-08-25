//! Memory-group parameters (M0–M9).

use crate::types::status::{SafetyLimits, Setpoints, Temperature};

/// Every register of a memory group, as physical values.
///
/// Field order here is for readers, not for the wire: encoding and decoding
/// address each word by [`crate::regs::Reg`], and
/// [`crate::regs::Group::WORDS`] is the only statement of wire order. Adding
/// or reordering a field cannot silently change the frame.
///
/// The two cumulative limits are stored at *different* scales — see
/// [`crate::regs::OAH_LIMIT_SCALE`] and [`crate::regs::OWH_LIMIT_SCALE`],
/// where the reason lives.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GroupParams {
    pub setpoints: Setpoints,
    pub safety_limits: SafetyLimits,
    /// Over-power threshold in W (1 W resolution on XY7025).
    pub s_opp_w: f32,
    /// Output-on time limit, hours.
    pub s_ohp_h: u16,
    /// Output-on time limit, minutes.
    pub s_ohp_m: u16,
    /// Cumulative-charge limit in Ah.
    pub s_oah_ah: f64,
    /// Cumulative-energy limit in Wh.
    pub s_owh_wh: f64,
    /// Over-temperature threshold and the unit in which it was specified.
    pub s_otp: Temperature,
    /// Power-on output state. `false` = boot with output OFF, `true` = ON.
    pub power_on_output: bool,
}
