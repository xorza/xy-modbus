//! Hardware profile, register scales, and physical operating limits.

use core::num::NonZeroU16;

/// Inclusive physical range accepted by a model profile.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ModelRange {
    pub min: f64,
    pub max: f64,
}

impl ModelRange {
    pub(crate) fn contains(self, value: f64) -> bool {
        self.min <= value && value <= self.max
    }
}

/// Model-specific fixed-point denominators.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ModelScales {
    /// I-SET, IOUT, and S-OCP denominator.
    pub current: NonZeroU16,
    /// POWER denominator.
    pub power: NonZeroU16,
    /// S-OPP denominator.
    pub over_power: NonZeroU16,
}

/// Physical limits enforced before writes reach the device.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ModelLimits {
    pub voltage_set_v: ModelRange,
    pub current_set_a: ModelRange,
    pub lvp_v: ModelRange,
    pub ovp_v: ModelRange,
    pub ocp_a: ModelRange,
    pub opp_w: ModelRange,
    pub charge_limit_ah: ModelRange,
    pub energy_limit_wh: ModelRange,
}

/// Hardware profile selected at [`crate::Xy`] construction.
///
/// `Xy7025` is the only profile verified on hardware. `Custom` supports
/// devices with the same 14-register memory-group layout by requiring both
/// their fixed-point scales and physical limits explicitly. SK-family devices
/// use a 15-register group layout and should use the raw framing/transport
/// layers until a dedicated high-level profile exists.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Model {
    Xy7025,
    Custom {
        scales: ModelScales,
        limits: ModelLimits,
    },
}

impl Model {
    pub(crate) const fn current_scale(self) -> f32 {
        match self {
            Self::Xy7025 => 100.0,
            Self::Custom { scales, .. } => scales.current.get() as f32,
        }
    }

    pub(crate) const fn power_scale(self) -> f32 {
        match self {
            Self::Xy7025 => 10.0,
            Self::Custom { scales, .. } => scales.power.get() as f32,
        }
    }

    pub(crate) const fn opp_scale(self) -> f32 {
        match self {
            Self::Xy7025 => 1.0,
            Self::Custom { scales, .. } => scales.over_power.get() as f32,
        }
    }

    pub(crate) const fn limits(self) -> ModelLimits {
        match self {
            Self::Xy7025 => ModelLimits {
                voltage_set_v: ModelRange {
                    min: 0.0,
                    max: 70.0,
                },
                current_set_a: ModelRange {
                    min: 0.0,
                    max: 25.0,
                },
                lvp_v: ModelRange {
                    min: 10.0,
                    max: 95.0,
                },
                ovp_v: ModelRange {
                    min: 0.0,
                    max: 72.0,
                },
                ocp_a: ModelRange {
                    min: 0.0,
                    max: 27.0,
                },
                opp_w: ModelRange {
                    min: 0.0,
                    max: 2000.0,
                },
                charge_limit_ah: ModelRange {
                    min: 0.0,
                    max: 9999.0,
                },
                energy_limit_wh: ModelRange {
                    min: 0.0,
                    max: 4_200_000.0,
                },
            },
            Self::Custom { limits, .. } => limits,
        }
    }

    pub(crate) const fn recognizes_scale_code(self, code: u16) -> bool {
        match self {
            Self::Xy7025 => matches!(code, 0x6100 | 0x6500),
            Self::Custom { .. } => false,
        }
    }

    pub(crate) fn assert_valid(self) {
        if let Self::Custom { limits, .. } = self {
            assert_range(
                limits.voltage_set_v,
                100.0,
                u16::MAX as f64,
                "voltage setpoint",
            );
            assert_range(
                limits.current_set_a,
                self.current_scale() as f64,
                u16::MAX as f64,
                "current setpoint",
            );
            assert_range(limits.lvp_v, 100.0, u16::MAX as f64, "LVP");
            assert_range(limits.ovp_v, 100.0, u16::MAX as f64, "OVP");
            assert_range(
                limits.ocp_a,
                self.current_scale() as f64,
                u16::MAX as f64,
                "OCP",
            );
            assert_range(
                limits.opp_w,
                self.opp_scale() as f64,
                u16::MAX as f64,
                "OPP",
            );
            assert_range(
                limits.charge_limit_ah,
                1000.0,
                u32::MAX as f64,
                "charge limit",
            );
            assert_range(
                limits.energy_limit_wh,
                100.0,
                u32::MAX as f64,
                "energy limit",
            );
        }
    }
}

fn assert_range(range: ModelRange, scale: f64, raw_max: f64, field: &'static str) {
    assert!(
        range.min.is_finite() && range.max.is_finite(),
        "{field} range must be finite"
    );
    assert!(
        0.0 <= range.min && range.min <= range.max,
        "invalid {field} range"
    );
    assert!(
        range.max * scale <= raw_max,
        "{field} range is not representable on the wire"
    );
}

/// Result of checking whether the configured profile uses scales compatible
/// with the device's `MODEL` register.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScaleCheck {
    Compatible { device_code: u16 },
    Inconclusive { device_code: u16 },
}
