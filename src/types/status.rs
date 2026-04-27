//! Live readings, setpoints, and cumulative counters.

use super::enums::{ProtectionStatus, RegMode};

// ─── Setpoints ───────────────────────────────────────────────────────────────

/// Output voltage / current setpoints (registers 0x0000–0x0001).
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Setpoints {
    pub v_set: f32,
    pub i_set: f32,
}

// ─── Status ──────────────────────────────────────────────────────────────────

/// Live + control snapshot covering registers 0x0000–0x0012 in a single
/// 19-register transaction. Returns everything a supervisor needs each
/// tick (live readings, regulation mode, latched protection cause,
/// output-enable flag) in one Modbus round-trip.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Status {
    pub v_set: f32,
    pub i_set: f32,
    pub v_out: f32,
    pub i_out: f32,
    pub p_out: f32,
    pub v_in: f32,
    /// `PROTECT` register (0x0010). Necessarily `Normal` while
    /// [`Self::output_on`] is true.
    pub protection: ProtectionStatus,
    /// `CVCC` register (0x0011) — current regulation mode.
    pub reg_mode: RegMode,
    /// `OUTPUT_EN` register (0x0012).
    pub output_on: bool,
}

// ─── OnTime ──────────────────────────────────────────────────────────────────

/// Output-on time as reported by the device (h/m/s).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OnTime {
    pub hours: u16,
    pub minutes: u16,
    pub seconds: u16,
}

impl OnTime {
    pub const fn total_seconds(self) -> u32 {
        self.hours as u32 * 3600 + self.minutes as u32 * 60 + self.seconds as u32
    }
}

// ─── Totals ──────────────────────────────────────────────────────────────────

/// Cumulative output counters and on-time (registers 0x0006–0x000C).
///
/// Charge and energy are composed from 32-bit low/high register pairs.
/// The high words are flagged as untested in community docs — verify
/// against your hardware before trusting them at high totals.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Totals {
    /// Cumulative output charge in Ah.
    pub charge_ah: f32,
    /// Cumulative output energy in Wh.
    pub energy_wh: f32,
    /// Output-on time, accumulated.
    pub on_time: OnTime,
}

// ─── Temperatures ────────────────────────────────────────────────────────────

/// Temperature readings from registers `0x000D` (T-IN) and `0x000E` (T-EX),
/// in the unit selected by [`super::TempUnit`].
///
/// `internal` is the on-board sensor — verified on XY7025 hardware.
///
/// `_external_unverified` is the optional external probe input. With no
/// thermistor connected the field reads `888.8` as a sentinel; the
/// decoding scale for a *connected* probe has not been verified on real
/// hardware. The leading underscore is a deliberate marker — treat the
/// value as advisory until you've cross-checked it against a known
/// reference temperature on your unit.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Temperatures {
    pub internal: f32,
    pub _external_unverified: f32,
}

// ─── SafetyLimits ────────────────────────────────────────────────────────────

/// Hard trip limits programmed into the buck's protection registers.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SafetyLimits {
    pub lvp_v: f32,
    pub ovp_v: f32,
    pub ocp_a: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn on_time_total_seconds() {
        assert_eq!(OnTime::default().total_seconds(), 0);
        assert_eq!(
            OnTime {
                hours: 0,
                minutes: 0,
                seconds: 1,
            }
            .total_seconds(),
            1
        );
        // 1h 23m 45s = 3600 + 1380 + 45.
        assert_eq!(
            OnTime {
                hours: 1,
                minutes: 23,
                seconds: 45,
            }
            .total_seconds(),
            5025
        );
        // No overflow with full u16 hours: 65535 * 3600 = 235_926_000 < u32::MAX.
        assert_eq!(
            OnTime {
                hours: u16::MAX,
                minutes: 0,
                seconds: 0,
            }
            .total_seconds(),
            65535u32 * 3600
        );
    }
}
