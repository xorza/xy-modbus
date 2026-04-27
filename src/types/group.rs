//! Memory-group parameters (M0–M9).

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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GroupParams {
    pub v_set: f32,
    pub i_set: f32,
    pub s_lvp_v: f32,
    pub s_ovp_v: f32,
    pub s_ocp_a: f32,
    /// Over-power threshold in W. Resolution depends on model (1 W on
    /// XY7025, 0.1 W on SK family) — encoded by [`super::Model::opp_scale`].
    pub s_opp_w: f32,
    /// Output-on time limit, hours.
    pub s_ohp_h: u16,
    /// Output-on time limit, minutes.
    pub s_ohp_m: u16,
    /// Cumulative-charge limit in Ah.
    pub s_oah_ah: f32,
    /// Cumulative-energy limit in Wh.
    pub s_owh_wh: f32,
    /// Over-temperature threshold (°C/°F per [`super::TempUnit`]).
    pub s_otp: f32,
    /// Power-on output state. `false` = boot with output OFF, `true` = ON.
    pub power_on_output: bool,
}
