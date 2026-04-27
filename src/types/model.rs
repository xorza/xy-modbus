//! Hardware variant presets and per-model register scales.

/// Hardware variant. Selected at construction (`Xy::new`) and used to
/// scale the registers whose resolution differs across the family —
/// I-SET, IOUT, S-OCP, POWER, S-OPP. See `DATASHEET.md` §3 for the
/// scale table.
///
/// Cross-check by reading `MODEL` (`0x0016`): `0x6100`-class is
/// XY6020L / XY7025. The crate does not probe automatically — pick
/// the variant that matches your hardware.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Model {
    /// Protocol-identical to [`Self::Xy7025`] (same scales, same `MODEL`
    /// register code `0x6100`). Pick whichever matches the silkscreen on
    /// your board — the crate treats them interchangeably.
    Xy6020L,
    /// Protocol-identical to [`Self::Xy6020L`].
    Xy7025,
    /// Escape hatch for hardware not covered by the preset variants.
    /// Each scale is the integer denominator the firmware uses on the
    /// wire — e.g. `current_scale = 100` means a raw register value of
    /// `1234` represents `12.34 A`. The XY firmware uses integer
    /// denominators on every known variant; cross-check against the
    /// vendor docs for your unit.
    Custom {
        current_scale: u16,
        power_scale: u16,
        opp_scale: u16,
    },
}

impl Model {
    /// Scale for I-SET, IOUT, S-OCP. 100 on XY6020L/XY7025 (10 mA).
    pub const fn current_scale(self) -> f32 {
        match self {
            Self::Xy6020L | Self::Xy7025 => 100.0,
            Self::Custom { current_scale, .. } => current_scale as f32,
        }
    }

    /// Scale for POWER (`0x0004`). 10 on XY6020L/XY7025 (100 mW).
    pub const fn power_scale(self) -> f32 {
        match self {
            Self::Xy6020L | Self::Xy7025 => 10.0,
            Self::Custom { power_scale, .. } => power_scale as f32,
        }
    }

    /// Scale for S-OPP in memory groups (`0x0055`). 1 W on XY6020L/XY7025.
    pub const fn opp_scale(self) -> f32 {
        match self {
            Self::Xy6020L | Self::Xy7025 => 1.0,
            Self::Custom { opp_scale, .. } => opp_scale as f32,
        }
    }

    /// Expected value of the device's `MODEL` register (`0x0016`) for
    /// this variant, if known. Used by [`crate::Xy::verify_model`] to
    /// catch wrong-scale-family misconfiguration.
    ///
    /// `0x6100` is shared by XY6020L and XY7025 — they have identical
    /// register scales, so the choice doesn't affect protocol behavior.
    /// `Custom` returns `None` (no canonical code).
    pub const fn expected_model_code(self) -> Option<u16> {
        match self {
            Self::Xy6020L | Self::Xy7025 => Some(0x6100),
            Self::Custom { .. } => None,
        }
    }
}

// ─── ModelCheck ──────────────────────────────────────────────────────────────

/// Outcome of [`crate::Xy::verify_model`]. `Mismatch` is the dangerous
/// case — readings WILL be off by 10× until the configured [`Model`] is
/// changed to match the hardware.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ModelCheck {
    /// Device's `MODEL` register matches the configured model's family.
    Match { device_code: u16 },
    /// Device reports a code mapped to a different scale family. The
    /// configured [`Model`] is wrong for this hardware; readings will
    /// be off until it's corrected.
    Mismatch {
        expected_code: u16,
        device_code: u16,
    },
    /// Verification was not possible: either the device returned a
    /// code outside the documented set, or the configured model is
    /// `Custom` (no canonical expected code).
    Inconclusive { device_code: u16 },
}
