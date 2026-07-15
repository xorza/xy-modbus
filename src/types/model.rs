//! Hardware variant presets and per-model register scales.

use core::num::NonZeroU16;

/// Hardware variant. Selected at construction (`Xy::new`) and used to
/// scale the registers whose resolution differs across the family —
/// I-SET, IOUT, S-OCP, POWER, S-OPP. See `DATASHEET.md` §3 for the
/// scale table.
///
/// Cross-check by reading `MODEL` (`0x0016`): `0x6500` is XY7025
/// (newer firmware revision observed in 2024+ batches; older vendor
/// docs cite `0x6100` for the same protocol). The crate does not
/// probe automatically — pick the variant that matches your hardware.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Model {
    Xy7025,
    /// Escape hatch for hardware not covered by the preset variants.
    /// Each scale is the integer denominator the firmware uses on the
    /// wire — e.g. `current_scale = 100` means a raw register value of
    /// `1234` represents `12.34 A`. The XY firmware uses integer
    /// denominators on every known variant; cross-check against the
    /// vendor docs for your unit.
    Custom {
        current_scale: NonZeroU16,
        power_scale: NonZeroU16,
        opp_scale: NonZeroU16,
    },
}

impl Model {
    /// Scale for I-SET, IOUT, S-OCP. 100 on XY7025 (10 mA).
    pub(crate) const fn current_scale(self) -> f32 {
        match self {
            Self::Xy7025 => 100.0,
            Self::Custom { current_scale, .. } => current_scale.get() as f32,
        }
    }

    /// Scale for POWER (`0x0004`). 10 on XY7025 (100 mW).
    pub(crate) const fn power_scale(self) -> f32 {
        match self {
            Self::Xy7025 => 10.0,
            Self::Custom { power_scale, .. } => power_scale.get() as f32,
        }
    }

    /// Scale for S-OPP in memory groups (`0x0055`). 1 W on XY7025.
    pub(crate) const fn opp_scale(self) -> f32 {
        match self {
            Self::Xy7025 => 1.0,
            Self::Custom { opp_scale, .. } => opp_scale.get() as f32,
        }
    }

    pub(crate) const fn recognizes_model_code(self, code: u16) -> bool {
        match self {
            Self::Xy7025 => matches!(code, 0x6100 | 0x6500),
            Self::Custom { .. } => false,
        }
    }

    pub(crate) const fn max_voltage_set(self) -> f32 {
        match self {
            Self::Xy7025 => 70.0,
            Self::Custom { .. } => u16::MAX as f32 / 100.0,
        }
    }

    pub(crate) const fn max_current_set(self) -> f32 {
        match self {
            Self::Xy7025 => 25.0,
            Self::Custom { current_scale, .. } => u16::MAX as f32 / current_scale.get() as f32,
        }
    }

    pub(crate) const fn max_lvp(self) -> f32 {
        match self {
            Self::Xy7025 => 95.0,
            Self::Custom { .. } => u16::MAX as f32 / 100.0,
        }
    }

    pub(crate) const fn max_ovp(self) -> f32 {
        match self {
            Self::Xy7025 => 72.0,
            Self::Custom { .. } => u16::MAX as f32 / 100.0,
        }
    }

    pub(crate) const fn max_ocp(self) -> f32 {
        match self {
            Self::Xy7025 => 27.0,
            Self::Custom { current_scale, .. } => u16::MAX as f32 / current_scale.get() as f32,
        }
    }

    pub(crate) const fn max_opp(self) -> f32 {
        match self {
            Self::Xy7025 => 2000.0,
            Self::Custom { opp_scale, .. } => u16::MAX as f32 / opp_scale.get() as f32,
        }
    }
}

/// Outcome of [`crate::Xy::verify_model`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ModelCheck {
    /// Device's `MODEL` register matches the configured model's family.
    Match { device_code: u16 },
    /// Verification was not possible because the device returned an
    /// unknown code or the configured model is `Custom`.
    Inconclusive { device_code: u16 },
}
