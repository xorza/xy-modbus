//! The XY-series holding-register map.
//!
//! [`Reg`] names every register this driver models and carries how its raw
//! 16-bit word maps to a value. Keeping the address and the [`Encoding`]
//! together is deliberate: a register's scale and signedness used to live in
//! prose here while the decoding lived in `device`, and a `T-IN` read decoded
//! as unsigned reported 6548.6 °C for a −5 °C ambient.
//!
//! Registers are unsigned unless their encoding says [`Encoding::Signed`]. The
//! four temperature registers are two's-complement. That encoding is inferred
//! rather than vendor-documented — no XY datasheet or community source records
//! it — but signed is the safe reading either way: raw `0x8000..=0xFFFF`
//! decodes to −3276.8..−0.1, which a sensor can plausibly report, against
//! 3276.8..6553.5 unsigned, which it cannot. If the firmware does clamp at
//! zero those raw values never appear and both decodings agree on every value
//! that does.
//!
//! [`Reg::addr`] gives the address [`crate::Xy::read_raw_holding`] and
//! [`crate::Xy::write_raw_holding`] take. Use [`span`] to size a buffer for a
//! contiguous block read and [`offset_in`] to index it, so the block layout is
//! derived from the map rather than restated as literals.

/// Default Modbus slave address. Reconfigurable via [`Reg::SlaveAddr`]; the
/// new value only takes effect after the device resets.
pub const DEFAULT_SLAVE: u8 = 0x01;

/// Volts, in hundredths.
pub const VOLT_SCALE: f32 = 100.0;
/// Amps, in hundredths.
pub const CURRENT_SCALE: f32 = 100.0;
/// Watts, in tenths.
pub const POWER_SCALE: f32 = 10.0;
/// Degrees (°C or °F per [`Reg::TempUnit`]), in tenths.
pub const TEMP_SCALE: f32 = 10.0;
/// Over-temperature threshold: raw equals whole degrees in the displayed unit.
/// The third-party "scale 10" entry is wrong for XY7025 firmware.
pub const OTP_SCALE: f32 = 1.0;
/// Over-power threshold: raw equals whole watts.
pub const OPP_SCALE: f32 = 1.0;
/// Cumulative charge counter, in milliamp-hours.
pub const AH_COUNTER_SCALE: f64 = 1000.0;
/// Cumulative energy counter, in milliwatt-hours.
pub const WH_COUNTER_SCALE: f64 = 1000.0;
/// Charge *limit*, in milliamp-hours — same units as the counter.
pub const OAH_LIMIT_SCALE: f64 = 1000.0;
/// Energy *limit*, in 10 mWh units. Deliberately coarser than
/// [`WH_COUNTER_SCALE`]: the firmware trades resolution for a 32-bit range
/// reaching ~42.9 MWh.
pub const OWH_LIMIT_SCALE: f64 = 100.0;

/// How a register's raw word maps to a value.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Encoding {
    /// Unsigned fixed-point: `raw / scale`.
    Unsigned { scale: f32 },
    /// Two's-complement fixed-point: `(raw as i16) / scale`.
    Signed { scale: f32 },
    /// Low word of a 32-bit pair: `((high << 16) | low) / scale`. The pair's
    /// scale lives here; the high word carries [`Encoding::WideHigh`].
    WideLow { scale: f64 },
    /// High word of a 32-bit pair.
    WideHigh,
    /// Boolean flag; `0` or `1`, anything else is a bad response.
    Flag,
    /// Enumerated code with a dedicated decoder ([`crate::ProtectionStatus`],
    /// [`crate::RegMode`], [`crate::TempUnit`], [`crate::BaudRate`]).
    Code,
    /// Integer accepted within an inclusive range.
    Int { min: u16, max: u16 },
    /// Opaque word carrying no interpretable quantity.
    Opaque,
}

/// A register's address paired with how to read its word.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RegSpec {
    pub addr: u16,
    pub encoding: Encoding,
}

/// A holding register on an XY-series buck converter.
///
/// The `S*` variants are the *active* group (M0) copies, which are the live
/// operating set. Every other memory group stores the same words at
/// [`group_addr`]`(n)` — see [`GROUP_WORDS`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Reg {
    /// V-SET — output voltage setpoint.
    VSet,
    /// I-SET — output current limit setpoint.
    ISet,
    /// V-OUT — measured output voltage.
    VOut,
    /// I-OUT — measured output current.
    IOut,
    /// P-OUT — measured output power.
    POut,
    /// V-IN — measured input voltage.
    VIn,
    /// AH-LOW — cumulative output charge, low word.
    AhLow,
    /// AH-HIGH — cumulative output charge, high word. Untested in community
    /// docs; treat the 32-bit composition as best-effort.
    AhHigh,
    /// WH-LOW — cumulative output energy, low word.
    WhLow,
    /// WH-HIGH — cumulative output energy, high word.
    WhHigh,
    /// OUT-H — output-on time, hours.
    OutH,
    /// OUT-M — output-on time, minutes.
    OutM,
    /// OUT-S — output-on time, seconds.
    OutS,
    /// T-IN — internal temperature.
    TIn,
    /// T-EX — external probe temperature. The connected-probe scale is
    /// unverified, so the high-level API exposes only [`Reg::TIn`].
    TEx,
    /// LOCK — front-panel key lock.
    Lock,
    /// PROTECT — latched protection cause. Write 0 to clear.
    Protect,
    /// CVCC — regulation mode.
    Cvcc,
    /// ONOFF — output enable.
    OutputEn,
    /// F-C — temperature unit.
    TempUnit,
    /// B-LED — backlight brightness. Firmware floors a write of 0 to 1.
    Backlight,
    /// SLEEP — off-screen timeout in minutes. Firmware caps writes at 9.
    Sleep,
    /// MODEL — product number (e.g. `0x6500` on XY7025).
    Model,
    /// VERSION — firmware version (e.g. `0x0071`).
    Version,
    /// SLAVE-ADD — Modbus slave address; takes effect after device reset.
    SlaveAddr,
    /// BAUDRATE_L — baud-rate code.
    BaudCode,
    /// T-IN-OFFSET — internal-temp calibration offset. Writes are silently
    /// ignored over Modbus on XY7025; use the front-panel menu.
    TInOffset,
    /// T-EX-OFFSET — external-temp calibration offset. Same write caveat.
    TExOffset,
    /// BUZZER — buzzer enable. Often unimplemented in firmware.
    Buzzer,
    /// EXTRACT-M — recall a memory group into M0.
    ExtractM,
    /// S-LVP — low-voltage protection threshold.
    SLvp,
    /// S-OVP — over-voltage protection threshold.
    SOvp,
    /// S-OCP — over-current protection threshold.
    SOcp,
    /// S-OPP — over-power protection threshold.
    SOpp,
    /// S-OHP-H — output-on time limit, hours.
    SOhpH,
    /// S-OHP-M — output-on time limit, minutes.
    SOhpM,
    /// S-OAH-LOW — cumulative-charge limit, low word.
    SOahLow,
    /// S-OAH-HIGH — cumulative-charge limit, high word.
    SOahHigh,
    /// S-OWH-LOW — cumulative-energy limit, low word.
    SOwhLow,
    /// S-OWH-HIGH — cumulative-energy limit, high word.
    SOwhHigh,
    /// S-OTP — over-temperature threshold, in the unit [`Reg::TempUnit`]
    /// selects.
    SOtp,
    /// S-INI — power-on output state.
    SIni,
}

impl Reg {
    /// Every modelled register, for callers that want to walk the map.
    pub const ALL: [Self; 42] = [
        Self::VSet,
        Self::ISet,
        Self::VOut,
        Self::IOut,
        Self::POut,
        Self::VIn,
        Self::AhLow,
        Self::AhHigh,
        Self::WhLow,
        Self::WhHigh,
        Self::OutH,
        Self::OutM,
        Self::OutS,
        Self::TIn,
        Self::TEx,
        Self::Lock,
        Self::Protect,
        Self::Cvcc,
        Self::OutputEn,
        Self::TempUnit,
        Self::Backlight,
        Self::Sleep,
        Self::Model,
        Self::Version,
        Self::SlaveAddr,
        Self::BaudCode,
        Self::TInOffset,
        Self::TExOffset,
        Self::Buzzer,
        Self::ExtractM,
        Self::SLvp,
        Self::SOvp,
        Self::SOcp,
        Self::SOpp,
        Self::SOhpH,
        Self::SOhpM,
        Self::SOahLow,
        Self::SOahHigh,
        Self::SOwhLow,
        Self::SOwhHigh,
        Self::SOtp,
        Self::SIni,
    ];

    /// Address and encoding in one table, so a register's facts cannot drift
    /// apart across separate `match` arms.
    pub const fn spec(self) -> RegSpec {
        let (addr, encoding) = match self {
            Self::VSet => (0x0000, Encoding::Unsigned { scale: VOLT_SCALE }),
            Self::ISet => (
                0x0001,
                Encoding::Unsigned {
                    scale: CURRENT_SCALE,
                },
            ),
            Self::VOut => (0x0002, Encoding::Unsigned { scale: VOLT_SCALE }),
            Self::IOut => (
                0x0003,
                Encoding::Unsigned {
                    scale: CURRENT_SCALE,
                },
            ),
            Self::POut => (0x0004, Encoding::Unsigned { scale: POWER_SCALE }),
            Self::VIn => (0x0005, Encoding::Unsigned { scale: VOLT_SCALE }),
            Self::AhLow => (
                0x0006,
                Encoding::WideLow {
                    scale: AH_COUNTER_SCALE,
                },
            ),
            Self::AhHigh => (0x0007, Encoding::WideHigh),
            Self::WhLow => (
                0x0008,
                Encoding::WideLow {
                    scale: WH_COUNTER_SCALE,
                },
            ),
            Self::WhHigh => (0x0009, Encoding::WideHigh),
            Self::OutH => (
                0x000A,
                Encoding::Int {
                    min: 0,
                    max: u16::MAX,
                },
            ),
            Self::OutM => (0x000B, Encoding::Int { min: 0, max: 59 }),
            Self::OutS => (0x000C, Encoding::Int { min: 0, max: 59 }),
            Self::TIn => (0x000D, Encoding::Signed { scale: TEMP_SCALE }),
            Self::TEx => (0x000E, Encoding::Signed { scale: TEMP_SCALE }),
            Self::Lock => (0x000F, Encoding::Flag),
            Self::Protect => (0x0010, Encoding::Code),
            Self::Cvcc => (0x0011, Encoding::Code),
            Self::OutputEn => (0x0012, Encoding::Flag),
            Self::TempUnit => (0x0013, Encoding::Code),
            Self::Backlight => (0x0014, Encoding::Int { min: 1, max: 5 }),
            Self::Sleep => (0x0015, Encoding::Int { min: 0, max: 9 }),
            Self::Model => (0x0016, Encoding::Opaque),
            Self::Version => (0x0017, Encoding::Opaque),
            Self::SlaveAddr => (0x0018, Encoding::Int { min: 1, max: 247 }),
            Self::BaudCode => (0x0019, Encoding::Code),
            Self::TInOffset => (0x001A, Encoding::Signed { scale: TEMP_SCALE }),
            Self::TExOffset => (0x001B, Encoding::Signed { scale: TEMP_SCALE }),
            Self::Buzzer => (0x001C, Encoding::Flag),
            Self::ExtractM => (0x001D, Encoding::Int { min: 0, max: 9 }),
            Self::SLvp => (0x0052, Encoding::Unsigned { scale: VOLT_SCALE }),
            Self::SOvp => (0x0053, Encoding::Unsigned { scale: VOLT_SCALE }),
            Self::SOcp => (
                0x0054,
                Encoding::Unsigned {
                    scale: CURRENT_SCALE,
                },
            ),
            Self::SOpp => (0x0055, Encoding::Unsigned { scale: OPP_SCALE }),
            Self::SOhpH => (0x0056, Encoding::Int { min: 0, max: 99 }),
            Self::SOhpM => (0x0057, Encoding::Int { min: 0, max: 59 }),
            Self::SOahLow => (
                0x0058,
                Encoding::WideLow {
                    scale: OAH_LIMIT_SCALE,
                },
            ),
            Self::SOahHigh => (0x0059, Encoding::WideHigh),
            Self::SOwhLow => (
                0x005A,
                Encoding::WideLow {
                    scale: OWH_LIMIT_SCALE,
                },
            ),
            Self::SOwhHigh => (0x005B, Encoding::WideHigh),
            Self::SOtp => (0x005C, Encoding::Unsigned { scale: OTP_SCALE }),
            Self::SIni => (0x005D, Encoding::Flag),
        };
        RegSpec { addr, encoding }
    }

    /// Holding-register address.
    pub const fn addr(self) -> u16 {
        self.spec().addr
    }

    /// How this register's raw word maps to a value.
    pub const fn encoding(self) -> Encoding {
        self.spec().encoding
    }
}

/// Base register address of memory group M0 — the live operating set.
pub const GROUP_BASE: u16 = 0x0050;
/// Address distance between consecutive memory groups.
pub const GROUP_STRIDE: u16 = 0x0010;
/// Number of memory groups (M0 through M9).
pub const GROUP_COUNT: u8 = 10;

/// The M0 register occupying each word of a memory group, in wire order.
///
/// Indexing this is the one statement of a group's layout: the first two words
/// mirror [`Reg::VSet`] / [`Reg::ISet`] at `0x0000`, and the rest are the `S*`
/// registers at [`GROUP_BASE`] upward.
pub const GROUP_WORDS: [Reg; 14] = [
    Reg::VSet,
    Reg::ISet,
    Reg::SLvp,
    Reg::SOvp,
    Reg::SOcp,
    Reg::SOpp,
    Reg::SOhpH,
    Reg::SOhpM,
    Reg::SOahLow,
    Reg::SOahHigh,
    Reg::SOwhLow,
    Reg::SOwhHigh,
    Reg::SOtp,
    Reg::SIni,
];

/// Number of registers per memory group — the length of [`GROUP_WORDS`], so
/// the two cannot disagree.
pub const GROUP_LEN: usize = GROUP_WORDS.len();

/// Base register address of memory group `n` (0..=9).
pub const fn group_addr(n: u8) -> u16 {
    assert!(n < GROUP_COUNT, "group index out of range");
    GROUP_BASE + (n as u16) * GROUP_STRIDE
}

/// Index of `reg` within a memory group's words.
///
/// Panics for a register that is not part of a group — a wiring mistake, not
/// bad device data.
pub const fn group_offset(reg: Reg) -> usize {
    let mut i = 0;
    while i < GROUP_LEN {
        if GROUP_WORDS[i] as u8 == reg as u8 {
            return i;
        }
        i += 1;
    }
    panic!("register is not a memory-group word")
}

/// Number of registers spanned by the inclusive range `lo..=hi`. Sizes the
/// destination of a single block read.
pub const fn span(lo: Reg, hi: Reg) -> usize {
    (hi.addr() - lo.addr()) as usize + 1
}

/// Index of `reg` within a block read that started at `base`.
///
/// Bind the result to a `const` item so a register outside the block fails the
/// build — either here on the subtraction, or at the array index — instead of
/// panicking against live hardware.
pub const fn offset_in(base: Reg, reg: Reg) -> usize {
    (reg.addr() - base.addr()) as usize
}

// Words 2.. of a group sit at consecutive addresses from `GROUP_BASE`, which
// is what lets a group be read or written as one transaction. Words 0 and 1
// are the `0x0000` mirrors and are excluded.
const _: () = {
    let mut i = 2;
    while i < GROUP_LEN {
        assert!(GROUP_WORDS[i].addr() == GROUP_BASE + i as u16);
        i += 1;
    }
};

#[cfg(test)]
mod tests {
    extern crate std;
    use crate::regs::{
        Encoding, GROUP_COUNT, GROUP_LEN, GROUP_WORDS, Reg, group_addr, offset_in, span,
    };
    use std::vec::Vec;

    #[test]
    #[should_panic(expected = "group index out of range")]
    fn group_address_rejects_out_of_range_index() {
        group_addr(GROUP_COUNT);
    }

    #[test]
    fn group_addresses_are_strided() {
        // M0 at 0x0050, then 0x0010 per group up to M9 at 0x00E0.
        assert_eq!(group_addr(0), 0x0050);
        assert_eq!(group_addr(1), 0x0060);
        assert_eq!(group_addr(9), 0x00E0);
        // The live protection trio sits at M0 words 2..=4.
        assert_eq!(GROUP_WORDS[2], Reg::SLvp);
        assert_eq!(Reg::SLvp.addr(), 0x0052);
        assert_eq!(GROUP_WORDS[GROUP_LEN - 1], Reg::SIni);
    }

    #[test]
    fn span_and_offset_agree() {
        // 0x0006..=0x000C is seven registers, indexed 0..=6.
        assert_eq!(span(Reg::AhLow, Reg::OutS), 7);
        assert_eq!(offset_in(Reg::AhLow, Reg::AhLow), 0);
        assert_eq!(offset_in(Reg::AhLow, Reg::OutS), 6);
        // A single-register block spans one word at index zero.
        assert_eq!(span(Reg::TempUnit, Reg::TempUnit), 1);
        assert_eq!(offset_in(Reg::TempUnit, Reg::TempUnit), 0);
        // Every offset a block read uses must be inside the span it sizes.
        assert!(offset_in(Reg::VSet, Reg::OutputEn) < span(Reg::VSet, Reg::OutputEn));
    }

    /// Addresses must be unique — a duplicate would silently alias two
    /// registers, and the `spec` table is the only thing preventing it.
    #[test]
    fn every_register_has_a_distinct_address() {
        let mut addrs: Vec<u16> = Reg::ALL.iter().map(|r| r.addr()).collect();
        addrs.sort_unstable();
        let before = addrs.len();
        addrs.dedup();
        assert_eq!(addrs.len(), before, "duplicate register address");
        // 0x0000..=0x001D contiguous, plus the M0 block 0x0052..=0x005D.
        assert_eq!(before, 0x1E + 12);
    }

    /// Every 32-bit pair must be a `WideLow` immediately followed by its
    /// `WideHigh`; the low word is where the pair's scale lives.
    #[test]
    fn wide_pairs_are_adjacent_low_then_high() {
        for (low, high) in [
            (Reg::AhLow, Reg::AhHigh),
            (Reg::WhLow, Reg::WhHigh),
            (Reg::SOahLow, Reg::SOahHigh),
            (Reg::SOwhLow, Reg::SOwhHigh),
        ] {
            assert!(matches!(low.encoding(), Encoding::WideLow { .. }));
            assert_eq!(high.encoding(), Encoding::WideHigh);
            assert_eq!(high.addr(), low.addr() + 1);
        }
        // The energy limit is stored coarser than the energy counter.
        assert_eq!(Reg::SOwhLow.encoding(), Encoding::WideLow { scale: 100.0 });
        assert_eq!(Reg::WhLow.encoding(), Encoding::WideLow { scale: 1000.0 });
    }
}
