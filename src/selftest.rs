//! Hardware conformance suite for the XY7025, generic over the transport.
//!
//! Behind the `selftest` feature, which pulls in `std` and `log` — this is
//! bring-up equipment, not something to run in a `no_std` control loop. Point
//! it at your own [`Xy`] and it reports what the device does against what
//! `DATASHEET.md` says it should.
//!
//! Nothing here is ESP32-specific; `examples/esp32c6-test/` is only a
//! bootstrap that builds a UART and calls [`run`]. The crate's own
//! `tests/on_device_suite.rs` runs the same code against a simulated device,
//! so it stays honest as the driver changes.
//!
//! **This suite writes to the device.** It snapshots every persistent register
//! as raw words, restores them exactly at the end, and leaves the live output
//! off. It briefly enables the output at V=0 / I=0 to prove the on/off
//! plumbing, so disconnect sensitive loads first. Baud rate and slave address
//! are only ever written back to their current values — changing either would
//! orphan the device on its next boot.

// `extern crate std` in `lib.rs` links std for this module, but a `no_std`
// crate has no std prelude, so the alloc types come in by name.
use std::format;
use std::string::{String, ToString};
use std::thread;
use std::time::Duration;
use std::vec::Vec;

use log::{error, info, warn};

use crate::regs::{Group, Reg};
use crate::transport::ModbusTransport;
use crate::{
    GroupParams, InputError, ProtectionStatus, SafetyLimits, Setpoints, TempUnit, Temperature, Xy,
    XyError,
};

/// A failed check. Driver errors convert into one, so test bodies use plain
/// `?` on driver calls instead of mapping every result.
#[derive(Debug)]
pub struct Failure(String);

impl From<XyError> for Failure {
    fn from(e: XyError) -> Self {
        Self(format!("driver: {e}"))
    }
}

impl std::fmt::Display for Failure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

pub type Outcome = Result<(), Failure>;

/// Adapt a bare driver call for a helper that wants an [`Outcome`]. Test bodies
/// use `?` instead; this is for the places that collect results rather than
/// propagate them.
fn driver(r: Result<(), XyError>) -> Outcome {
    r.map_err(Failure::from)
}

fn fail(message: impl Into<String>) -> Failure {
    Failure(message.into())
}

/// Give the device time to apply a write. Zero means "no wait" — a simulated
/// device applies writes instantly, so the host test does not pay for it.
fn settle_for(delay: Duration) {
    if !delay.is_zero() {
        thread::sleep(delay);
    }
}

/// Two live reads of the same quantity are taken at different instants, so a
/// real output can drift between them.
const DRIFT: f32 = 0.02;
/// A setpoint written and read back is the same fixed-point word, so it must
/// return the same value; the epsilon only absorbs `f32` division.
const ROUND_TRIP: f32 = 0.000_1;

fn check<V: PartialEq + std::fmt::Debug>(what: &str, expected: V, actual: V) -> Outcome {
    if expected == actual {
        Ok(())
    } else {
        Err(fail(format!(
            "{what}: expected {expected:?}, got {actual:?}"
        )))
    }
}

fn close(what: &str, expected: f32, actual: f32, tolerance: f32) -> Outcome {
    if (expected - actual).abs() <= tolerance {
        Ok(())
    } else {
        Err(fail(format!(
            "{what}: expected {expected:.4} ±{tolerance}, got {actual:.4}"
        )))
    }
}

fn close64(what: &str, expected: f64, actual: f64) -> Outcome {
    if (expected - actual).abs() <= 0.000_02 {
        Ok(())
    } else {
        Err(fail(format!(
            "{what}: expected {expected:.5}, got {actual:.5}"
        )))
    }
}

/// Run `body`, then `cleanup` regardless, reporting both if both fail. Cleanup
/// runs even when the body bailed early, which is what keeps a failed probe
/// from leaving the device reconfigured.
fn with_cleanup(what: &str, body: Outcome, cleanup: Outcome) -> Outcome {
    match (body, cleanup) {
        (body, Ok(())) => body,
        (Ok(()), Err(c)) => Err(fail(format!("{what} cleanup: {c}"))),
        (Err(b), Err(c)) => Err(fail(format!("{b}; {what} cleanup also failed: {c}"))),
    }
}

/// Tally of the whole run.
#[derive(Debug, Default)]
pub struct Report {
    pub passed: u32,
    pub failed: u32,
}

impl Report {
    fn record(&mut self, name: &str, outcome: Outcome) {
        match outcome {
            Ok(()) => {
                info!("PASS  {name}");
                self.passed += 1;
            }
            Err(e) => {
                error!("FAIL  {name}: {e}");
                self.failed += 1;
            }
        }
    }
}

/// Every persistent register, captured as raw words so restoring cannot be
/// altered by firmware unit conversion.
#[derive(Debug)]
struct Snapshot {
    output_on: bool,
    temp_unit: TempUnit,
    lock: bool,
    backlight: u8,
    sleep_minutes: u16,
    buzzer: bool,
    groups: [[u16; Group::LEN]; Group::ALL.len()],
}

impl Snapshot {
    fn capture<T: ModbusTransport>(xy: &mut Xy<T>) -> Result<Self, Failure> {
        let mut groups = [[0; Group::LEN]; Group::ALL.len()];
        for (group, slot) in Group::ALL.iter().zip(groups.iter_mut()) {
            xy.read_raw_holding(group.addr(), slot)?;
        }
        Ok(Self {
            output_on: xy.read_output()?,
            temp_unit: xy.read_temp_unit()?,
            lock: xy.read_lock()?,
            backlight: xy.read_backlight()?,
            sleep_minutes: xy.read_sleep_minutes()?,
            buzzer: xy.read_buzzer()?,
            groups,
        })
    }

    /// Restore everything, collecting failures rather than stopping at the
    /// first — a partial restore is worse than a noisy one.
    fn restore<T: ModbusTransport>(&self, xy: &mut Xy<T>) -> Outcome {
        xy.set_output(false)
            .map_err(|e| fail(format!("cannot disable output before restore: {e}")))?;
        check("output before restore", false, xy.read_output()?)?;

        let mut errors = Vec::new();
        let mut step = |what: &str, r: Outcome| {
            if let Err(e) = r {
                errors.push(format!("{what}: {e}"));
            }
        };

        step("unlock", driver(xy.set_lock(false)));
        step("V_SET=0", driver(xy.set_voltage(0.0)));
        step("I_SET=0", driver(xy.set_current_limit(0.0)));
        step("temperature unit", driver(xy.set_temp_unit(self.temp_unit)));

        // Non-active groups first; M0 last so the live set lands once the
        // stored ones are already correct.
        for group in Group::ALL.iter().skip(1) {
            step(
                &format!("M{}", group.index()),
                restore_group(xy, *group, &self.groups[group.index() as usize]),
            );
        }
        step("M0", restore_group(xy, Group::M0, &self.groups[0]));

        step("backlight", driver(xy.set_backlight(self.backlight)));
        step(
            "sleep timeout",
            driver(xy.set_sleep_minutes(self.sleep_minutes)),
        );
        step("buzzer", driver(xy.set_buzzer(self.buzzer)));
        step("clear protection", driver(xy.clear_protection_status()));
        step("lock", driver(xy.set_lock(self.lock)));
        step("verify", self.verify(xy));

        if self.output_on {
            warn!("snapshot had output ON — leaving it OFF; re-enable manually if intended");
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(fail(errors.join("; ")))
        }
    }

    fn verify<T: ModbusTransport>(&self, xy: &mut Xy<T>) -> Outcome {
        check("OUTPUT_EN", false, xy.read_output()?)?;
        check("F-C", self.temp_unit, xy.read_temp_unit()?)?;
        check("LOCK", self.lock, xy.read_lock()?)?;
        check("B-LED", self.backlight, xy.read_backlight()?)?;
        check("SLEEP", self.sleep_minutes, xy.read_sleep_minutes()?)?;
        check("BUZZER", self.buzzer, xy.read_buzzer()?)?;
        for group in Group::ALL {
            let mut actual = [0; Group::LEN];
            xy.read_raw_holding(group.addr(), &mut actual)?;
            check(
                &format!("M{} raw", group.index()),
                self.groups[group.index() as usize],
                actual,
            )?;
        }
        Ok(())
    }
}

/// Write a group back word by word. For M0 the protection words go first, so
/// restoring a high V-SET cannot cross a not-yet-restored S-OVP.
fn restore_group<T: ModbusTransport>(
    xy: &mut Xy<T>,
    group: Group,
    words: &[u16; Group::LEN],
) -> Outcome {
    // The words before S-LVP are the V-SET / I-SET pair.
    const SETPOINTS: usize = Group::word_offset(Reg::SLvp);
    let write = |xy: &mut Xy<T>, offset: usize| -> Outcome {
        xy.write_raw_holding(group.addr() + offset as u16, words[offset])?;
        Ok(())
    };
    if group == Group::M0 {
        for offset in SETPOINTS..Group::LEN {
            write(xy, offset)?;
        }
        for offset in 0..SETPOINTS {
            write(xy, offset)?;
        }
    } else {
        for offset in 0..Group::LEN {
            write(xy, offset)?;
        }
    }
    let mut actual = [0; Group::LEN];
    xy.read_raw_holding(group.addr(), &mut actual)?;
    check(&format!("M{} restored", group.index()), *words, actual)
}

/// Leave the device in a state every test can assume: output off, boot-safe,
/// no latched fault, and protection wide open so sweeps are not clamped.
fn prepare<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    xy.set_output(false)?;
    check("baseline output", false, xy.read_output()?)?;
    xy.set_power_on_output(false)?;
    check(
        "baseline power-on output",
        false,
        xy.read_power_on_output()?,
    )?;
    xy.clear_protection_status()?;
    xy.set_safety_limits(HEADROOM)?;
    Ok(())
}

/// Wide-open protection so the V/I sweeps are never clamped. Restored from the
/// snapshot at the end like everything else.
const HEADROOM: SafetyLimits = SafetyLimits {
    lvp_v: 10.0,
    ovp_v: 72.0,
    ocp_a: 27.0,
};

// Hand copy of `device::input_spec`, which is private: V_SET 0–70, I_SET 0–25
// at 0.01 resolution; OVP 0–72, OCP 0–27, LVP 10–95. Update alongside that
// table — these samples probe its boundaries and would stop doing so silently.
const V_SET_SAMPLES: &[f32] = &[0.00, 0.01, 3.30, 12.34, 13.50, 19.99, 33.33, 48.00, 70.00];
const I_SET_SAMPLES: &[f32] = &[0.00, 0.01, 1.23, 5.00, 15.00, 25.00];

const PROT_SAMPLES: &[SafetyLimits] = &[
    SafetyLimits {
        lvp_v: 10.0,
        ovp_v: 5.00,
        ocp_a: 0.50,
    },
    SafetyLimits {
        lvp_v: 11.5,
        ovp_v: 14.40,
        ocp_a: 5.00,
    },
    SafetyLimits {
        lvp_v: 44.0,
        ovp_v: 56.00,
        ocp_a: 20.00,
    },
    SafetyLimits {
        lvp_v: 90.0,
        ovp_v: 70.00,
        ocp_a: 27.00,
    },
];

fn identity<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    let check_result = xy.check_model()?;
    info!(
        "  MODEL=0x{:04X} VERSION=0x{:04X} {check_result:?}",
        xy.read_model()?,
        xy.read_version()?
    );
    // Both flags matter and they differ on real hardware: 0x6100-class shares
    // the scales but not the ceilings this driver enforces.
    check("scales_match", true, check_result.scales_match)?;
    check("limits_match", true, check_result.limits_match)?;
    check("device_code", 0x6500, check_result.device_code)
}

/// The bulk snapshot must agree with the granular reads that share its
/// registers — this is what would catch a wrong offset in `read_status`.
fn status_agrees_with_granular_reads<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    let s = xy.read_status()?;
    let sp = xy.read_setpoints()?;
    close("v_set", s.setpoints.v_set, sp.v_set, DRIFT)?;
    close("i_set", s.setpoints.i_set, sp.i_set, DRIFT)?;
    close("v_out", s.v_out, xy.read_voltage_out()?, DRIFT)?;
    close("i_out", s.i_out, xy.read_current_out()?, DRIFT)?;
    close("p_out", s.p_out, xy.read_power_out()?, DRIFT)?;
    close("v_in", s.v_in, xy.read_voltage_in()?, DRIFT)?;
    check("output_on", s.output_on, xy.read_output()?)?;
    check("reg_mode", s.reg_mode, xy.read_reg_mode()?)?;
    info!(
        "  V_IN={:.2} V_OUT={:.2} I_OUT={:.3} P_OUT={:.2} prot={} reg={:?} on={}",
        s.v_in, s.v_out, s.i_out, s.p_out, s.protection, s.reg_mode, s.output_on
    );
    if s.v_in < 1.0 {
        return Err(fail(format!("V_IN={:.2} — is the buck powered?", s.v_in)));
    }
    Ok(())
}

fn totals<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    let t = xy.read_totals()?;
    // Decode the same span by hand: an independent check of the crate's own
    // scales, so deriving it from the crate would defeat the point.
    let mut raw = [0u16; 7];
    xy.read_raw_holding(Reg::AhLow.addr(), &mut raw)?;
    let compose = |low: u16, high: u16| ((high as u32) << 16 | low as u32) as f64;
    check("charge", compose(raw[0], raw[1]) / 1000.0, t.charge_ah)?;
    check("energy", compose(raw[2], raw[3]) / 1000.0, t.energy_wh)?;
    check("hours", raw[4], t.on_time.hours)?;
    check("minutes", raw[5], t.on_time.minutes)?;
    check("seconds", raw[6], t.on_time.seconds)?;
    check(
        "total seconds",
        raw[4] as u32 * 3600 + raw[5] as u32 * 60 + raw[6] as u32,
        t.on_time.total_seconds(),
    )?;
    info!(
        "  {:.3} Ah, {:.3} Wh, {}h{}m{}s",
        t.charge_ah, t.energy_wh, t.on_time.hours, t.on_time.minutes, t.on_time.seconds
    );
    Ok(())
}

fn setpoint_sweeps<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    for &v in V_SET_SAMPLES {
        xy.set_voltage(v)?;
        close(
            &format!("V_SET={v:.2}"),
            v,
            xy.read_setpoints()?.v_set,
            ROUND_TRIP,
        )?;
        close(
            &format!("V_SET={v:.2} via status"),
            v,
            xy.read_status()?.setpoints.v_set,
            ROUND_TRIP,
        )?;
    }
    for &i in I_SET_SAMPLES {
        xy.set_current_limit(i)?;
        close(
            &format!("I_SET={i:.2}"),
            i,
            xy.read_setpoints()?.i_set,
            ROUND_TRIP,
        )?;
    }
    info!(
        "  swept {} V and {} I values",
        V_SET_SAMPLES.len(),
        I_SET_SAMPLES.len()
    );
    Ok(())
}

fn protection_sweep<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    // Drop V_SET under the smallest OVP sample, or the guard rejects the write.
    xy.set_voltage(0.0)?;
    xy.set_current_limit(0.0)?;
    let body = (|| -> Outcome {
        for limits in PROT_SAMPLES {
            xy.set_safety_limits(*limits)?;
            let got = xy.read_safety_limits()?;
            close("LVP", limits.lvp_v, got.lvp_v, ROUND_TRIP)?;
            close("OVP", limits.ovp_v, got.ovp_v, ROUND_TRIP)?;
            close("OCP", limits.ocp_a, got.ocp_a, ROUND_TRIP)?;
        }
        Ok(())
    })();
    with_cleanup("headroom", body, driver(xy.set_safety_limits(HEADROOM)))
}

fn expect_crossing_rejected(what: &str, result: Result<(), XyError>) -> Outcome {
    match result {
        Err(XyError::Input(InputError::VoltageSetpointAboveProtection)) => Ok(()),
        Err(other) => Err(fail(format!(
            "{what}: expected VoltageSetpointAboveProtection, got {other}"
        ))),
        Ok(()) => Err(fail(format!("{what}: was accepted but must be rejected"))),
    }
}

/// The driver refuses writes that would put V-SET above S-OVP, which the device
/// answers by latching OVP. All three entry points must agree, and none may
/// change the device when they refuse.
fn crossing_guards<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    let body = (|| -> Outcome {
        xy.set_voltage(0.0)?;
        xy.set_safety_limits(SafetyLimits {
            lvp_v: 10.0,
            ovp_v: 20.0,
            ocp_a: 5.0,
        })?;
        xy.set_voltage(5.0)?;

        expect_crossing_rejected("set_voltage above S-OVP", xy.set_voltage(25.0))?;
        close(
            "V_SET unchanged",
            5.0,
            xy.read_setpoints()?.v_set,
            ROUND_TRIP,
        )?;

        expect_crossing_rejected(
            "set_safety_limits below V-SET",
            xy.set_safety_limits(SafetyLimits {
                lvp_v: 10.0,
                ovp_v: 4.0,
                ocp_a: 5.0,
            }),
        )?;
        close(
            "S-OVP unchanged",
            20.0,
            xy.read_safety_limits()?.ovp_v,
            ROUND_TRIP,
        )?;

        // M0 is the live set, so a group write crosses the same way. The pair
        // inside the group is self-consistent (25 <= 30); it is the *live*
        // S-OVP of 20 that must reject it.
        let mut crossing = probe_params(0);
        crossing.setpoints.v_set = 25.0;
        crossing.safety_limits.ovp_v = 30.0;
        expect_crossing_rejected(
            "write_group(M0) above live S-OVP",
            xy.write_group(Group::M0, &crossing).map(|_| ()),
        )?;
        close(
            "V_SET still unchanged",
            5.0,
            xy.read_setpoints()?.v_set,
            ROUND_TRIP,
        )
    })();
    with_cleanup(
        "guards",
        body,
        (|| -> Outcome {
            xy.set_voltage(0.0)?;
            xy.set_safety_limits(HEADROOM)?;
            Ok(())
        })(),
    )
}

fn protection_status<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    info!("  PROTECT={}", xy.read_protection_status()?);
    xy.clear_protection_status()?;
    check(
        "PROTECT after clear",
        ProtectionStatus::Normal,
        xy.read_protection_status()?,
    )
}

fn output_enable<T: ModbusTransport>(xy: &mut Xy<T>, settle: Duration) -> Outcome {
    xy.set_output(false)?;
    xy.set_voltage(0.0)?;
    xy.set_current_limit(0.0)?;
    xy.clear_protection_status()?;

    let body = (|| -> Outcome {
        for want in [true, false] {
            xy.set_output(want)?;
            settle_for(settle * 2);
            check("OUTPUT_EN", want, xy.read_output()?)?;
            check("status.output_on", want, xy.read_status()?.output_on)?;
        }
        Ok(())
    })();
    with_cleanup(
        "output",
        body,
        (|| -> Outcome {
            xy.set_output(false)?;
            check("OUTPUT_EN off", false, xy.read_output()?)
        })(),
    )
}

/// Every boolean register, round-tripped both ways through the typed API.
fn flags<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    type Set<T> = fn(&mut Xy<T>, bool) -> Result<(), XyError>;
    type Get<T> = fn(&mut Xy<T>) -> Result<bool, XyError>;
    // OUTPUT_EN is deliberately absent: enabling the output is only safe after
    // forcing V and I to zero, which `output_enable` does and this does not.
    let cases: [(&str, Set<T>, Get<T>); 3] = [
        ("LOCK", Xy::set_lock, Xy::read_lock),
        ("BUZZER", Xy::set_buzzer, Xy::read_buzzer),
        ("S_INI", Xy::set_power_on_output, Xy::read_power_on_output),
    ];
    let body = (|| -> Outcome {
        for (name, set, get) in cases {
            for want in [true, false] {
                set(xy, want)?;
                check(name, want, get(xy)?)?;
            }
        }
        Ok(())
    })();
    // Leave the one that matters for safety in its boot-safe state.
    with_cleanup(
        "flags",
        body,
        (|| -> Outcome {
            xy.set_power_on_output(false)?;
            xy.set_lock(false)?;
            Ok(())
        })(),
    )
}

fn temperature<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    for unit in [TempUnit::Celsius, TempUnit::Fahrenheit] {
        xy.set_temp_unit(unit)?;
        check("F-C", unit, xy.read_temp_unit()?)?;
        let t = xy.read_temperature_internal()?;
        check("T_INT unit follows F-C", unit, t.unit)?;
        info!("  T_INT={:.1} {:?}", t.value, t.unit);
    }

    // Offsets are read-only over Modbus; confirm the typed decode matches the
    // raw words. Both registers are two's-complement signed.
    let int_off = xy.read_temp_offset_internal()?;
    let ext_off = xy.read_temp_offset_external()?;
    let mut raw = [0u16; 2];
    xy.read_raw_holding(Reg::TInOffset.addr(), &mut raw)?;
    close(
        "T_INT_OFFSET",
        raw[0] as i16 as f32 / 10.0,
        int_off,
        ROUND_TRIP,
    )?;
    close(
        "T_EXT_OFFSET",
        raw[1] as i16 as f32 / 10.0,
        ext_off,
        ROUND_TRIP,
    )?;

    // The external probe is raw-only. Log it decoded as signed so a connected
    // thermistor can be sanity-checked by eye during bring-up.
    let t_ex = xy.read_word(Reg::TEx)?;
    info!(
        "  T_INT_OFFSET={int_off:.1} T_EXT_OFFSET={ext_off:.1} T_EX raw={t_ex} ({:.1} signed)",
        t_ex as i16 as f32 / 10.0
    );
    Ok(())
}

fn display_settings<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    for level in 1..=5u8 {
        xy.set_backlight(level)?;
        check(&format!("B-LED={level}"), level, xy.read_backlight()?)?;
    }
    for minutes in [0u16, 1, 5, 9] {
        xy.set_sleep_minutes(minutes)?;
        check(
            &format!("SLEEP={minutes}"),
            minutes,
            xy.read_sleep_minutes()?,
        )?;
    }
    // The driver rejects what firmware would silently clamp.
    check(
        "set_backlight(0) rejected",
        true,
        xy.set_backlight(0).is_err(),
    )?;
    check(
        "set_sleep_minutes(10) rejected",
        true,
        xy.set_sleep_minutes(10).is_err(),
    )
}

fn comms<T: ModbusTransport>(xy: &mut Xy<T>, expected_baud: u32) -> Outcome {
    let slave = xy.read_slave_address()?;
    let baud = xy.read_baud_rate()?;
    info!("  SLAVE=0x{slave:02X} BAUD={baud:?}");
    if slave != 1 {
        return Err(fail(format!(
            "SLAVE 0x{slave:02X} but this transport addresses 0x01"
        )));
    }
    if baud.baud() != expected_baud {
        return Err(fail(format!(
            "device reports {} baud, UART configured for {expected_baud}",
            baud.baud()
        )));
    }
    // Both registers only take effect after a device reset, so writing the
    // current value back exercises the setter without orphaning the bus.
    xy.set_slave_address(slave)?;
    check("SLAVE round-trip", slave, xy.read_slave_address()?)?;
    xy.set_baud_rate(baud)?;
    check("BAUD round-trip", baud, xy.read_baud_rate()?)
}

/// A request the driver must refuse without touching the bus, and classify as
/// the caller's fault rather than the device's.
fn request_faults<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    let mut nothing: [u16; 0] = [];
    match xy.read_raw_holding(Reg::VSet.addr(), &mut nothing) {
        Err(XyError::Rtu(e)) if e.is_request_error() => Ok(()),
        Err(other) => Err(fail(format!("empty read: unexpected {other}"))),
        Ok(()) => Err(fail("empty read was accepted".to_string())),
    }
}

/// Save `reg`, write each probe, check what actually reads back, restore.
/// Firmware quirks live here: what the device *stores* is not always what was
/// written, and `expected` is where each quirk is stated.
fn raw_probe<T: ModbusTransport>(
    xy: &mut Xy<T>,
    label: &str,
    reg: Reg,
    probes: &[u16],
    settle: Duration,
    expected: impl Fn(u16, u16) -> u16,
) -> Outcome {
    let original = xy.read_word(reg)?;
    info!("  {label} original raw = {original}");
    let body = (|| -> Outcome {
        for &written in probes {
            xy.write_word(reg, written)?;
            settle_for(settle);
            check(
                &format!("{label} wrote {written}"),
                expected(written, original),
                xy.read_word(reg)?,
            )?;
        }
        Ok(())
    })();
    let cleanup = (|| -> Outcome {
        xy.write_word(reg, original)?;
        check(&format!("{label} restored"), original, xy.read_word(reg)?)
    })();
    with_cleanup(label, body, cleanup)
}

/// S-OTP is scale 1 — the raw word is whole degrees in whatever unit `F-C`
/// selects — and single-register writes bypass the conversion and clamping a
/// group write goes through, so they round-trip exactly in both units.
fn s_otp_scale<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    let original_unit = xy.read_temp_unit()?;
    let probes = &[10, 50, 95, 110, 150, 230, 500, 1100];
    let body = (|| -> Outcome {
        for unit in [TempUnit::Celsius, TempUnit::Fahrenheit] {
            xy.set_temp_unit(unit)?;
            raw_probe(
                xy,
                &format!("S-OTP {unit:?}"),
                Reg::SOtp,
                probes,
                Duration::ZERO,
                |written, _| written,
            )?;
        }
        Ok(())
    })();
    with_cleanup("S-OTP unit", body, driver(xy.set_temp_unit(original_unit)))
}

/// Firmware silently ignores Modbus writes to the calibration offsets, which
/// is why the driver exposes no setter for them.
fn temp_offset_writes_ignored<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    raw_probe(
        xy,
        "T-IN-OFFSET",
        Reg::TInOffset,
        &[1, 2, 5, 10, 50, 100],
        Duration::ZERO,
        |_, original| original,
    )
}

/// Firmware caps SLEEP at nine minutes; anything above reads back as 9.
fn sleep_ceiling<T: ModbusTransport>(xy: &mut Xy<T>, settle: Duration) -> Outcome {
    raw_probe(
        xy,
        "SLEEP",
        Reg::Sleep,
        &[0, 1, 8, 9, 10, 11, 30, 60, 999],
        settle,
        |written, _| written.min(9),
    )
}

/// Firmware floors the backlight at 1; writing 0 reads back as 1.
fn backlight_floor<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    raw_probe(
        xy,
        "B-LED",
        Reg::Backlight,
        &[0, 1, 3, 5],
        Duration::ZERO,
        |written, _| written.max(1),
    )
}

/// A distinct, in-range parameter set per group, so a mixed-up group index
/// shows as a value mismatch rather than passing by accident.
fn probe_params(n: u8) -> GroupParams {
    GroupParams {
        setpoints: Setpoints {
            v_set: 5.0 + f32::from(n) * 0.5,
            i_set: 0.10 + f32::from(n) * 0.05,
        },
        safety_limits: SafetyLimits {
            lvp_v: 10.0 + f32::from(n),
            ovp_v: 20.0 + f32::from(n),
            ocp_a: 1.0 + f32::from(n) * 0.25,
        },
        s_opp_w: 50.0 + f32::from(n) * 5.0,
        s_ohp_h: u16::from(n),
        s_ohp_m: u16::from(n) * 5,
        s_oah_ah: 1.0 + f64::from(n),
        s_owh_wh: 10.0 + f64::from(n) * 2.0,
        s_otp: Temperature {
            value: 50.0 + f32::from(n),
            unit: TempUnit::Celsius,
        },
        // M0 is the live set, so keep it boot-safe; the others cover both.
        power_on_output: n != 0 && n.is_multiple_of(2),
    }
}

fn compare_group(what: &str, want: &GroupParams, got: &GroupParams) -> Outcome {
    close(
        &format!("{what} v_set"),
        want.setpoints.v_set,
        got.setpoints.v_set,
        ROUND_TRIP,
    )?;
    close(
        &format!("{what} i_set"),
        want.setpoints.i_set,
        got.setpoints.i_set,
        ROUND_TRIP,
    )?;
    close(
        &format!("{what} lvp"),
        want.safety_limits.lvp_v,
        got.safety_limits.lvp_v,
        ROUND_TRIP,
    )?;
    close(
        &format!("{what} ovp"),
        want.safety_limits.ovp_v,
        got.safety_limits.ovp_v,
        ROUND_TRIP,
    )?;
    close(
        &format!("{what} ocp"),
        want.safety_limits.ocp_a,
        got.safety_limits.ocp_a,
        ROUND_TRIP,
    )?;
    close(
        &format!("{what} opp"),
        want.s_opp_w,
        got.s_opp_w,
        ROUND_TRIP,
    )?;
    check(&format!("{what} ohp_h"), want.s_ohp_h, got.s_ohp_h)?;
    check(&format!("{what} ohp_m"), want.s_ohp_m, got.s_ohp_m)?;
    close64(&format!("{what} oah"), want.s_oah_ah, got.s_oah_ah)?;
    close64(&format!("{what} owh"), want.s_owh_wh, got.s_owh_wh)?;
    check(&format!("{what} otp unit"), want.s_otp.unit, got.s_otp.unit)?;
    // A group write routes S-OTP through firmware unit conversion, which
    // rounds by up to a degree. `s_otp_scale` covers the exact path.
    close(
        &format!("{what} otp"),
        want.s_otp.value,
        got.s_otp.value,
        1.0,
    )
}

/// Round-trip every memory group through all four group entry points, then put
/// each one back exactly as it was found.
fn group_round_trip<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    xy.set_temp_unit(TempUnit::Celsius)?;
    let unit = xy.read_temp_unit()?;
    for group in Group::ALL {
        let n = group.index();
        let mut original = [0; Group::LEN];
        xy.read_raw_holding(group.addr(), &mut original)?;
        let probe = probe_params(n);

        let body = (|| -> Outcome {
            // Alternate the two write paths so both are exercised across the
            // sweep; `write_group` reads F-C itself, the other is handed it.
            let stored = if n.is_multiple_of(2) {
                xy.write_group(group, &probe)?
            } else {
                xy.write_group_in_unit(group, &probe, unit)?
            };
            compare_group(&format!("M{n} write"), &probe, &stored)?;
            // The typed read path is separate code from the write's read-back.
            compare_group(&format!("M{n} read"), &probe, &xy.read_group(group)?)?;
            compare_group(
                &format!("M{n} read_in_unit"),
                &probe,
                &xy.read_group_in_unit(group, unit)?,
            )
        })();
        with_cleanup(&format!("M{n}"), body, restore_group(xy, group, &original))?;
    }
    info!("  round-tripped all {} groups", Group::ALL.len());
    Ok(())
}

/// Recalling a group copies its words into the live M0 set.
fn recall<T: ModbusTransport>(xy: &mut Xy<T>) -> Outcome {
    xy.set_output(false)?;
    for group in Group::ALL {
        let n = group.index();
        // Drop V-SET first: recall replaces the live protection wholesale.
        xy.set_voltage(0.0)?;
        let mut expected = [0; Group::LEN];
        xy.read_raw_holding(group.addr(), &mut expected)?;

        let body = (|| -> Outcome {
            xy.recall_group(group)?;
            let mut actual = [0; Group::LEN];
            xy.read_raw_holding(Group::M0.addr(), &mut actual)?;
            check(&format!("M{n} recalled into M0"), expected, actual)
        })();
        with_cleanup(
            &format!("recall M{n}"),
            body,
            (|| -> Outcome {
                xy.set_power_on_output(false)?;
                check("S_INI boot-safe", false, xy.read_power_on_output()?)
            })(),
        )?;
    }
    info!("  recalled all {} groups", Group::ALL.len());
    Ok(())
}

/// Take the transport back out and rebuild around it. Consumes `xy`, so this
/// runs last.
fn lifecycle<T: ModbusTransport>(xy: Xy<T>) -> Outcome {
    let mut rebuilt = Xy::with_slave(xy.into_transport(), 1)?;
    check(
        "rebuilt talks to the device",
        true,
        rebuilt.check_model()?.limits_match,
    )
}

/// Run the whole suite against `xy`, leaving the device as it was found with
/// the output off. `expected_baud` is what the caller configured its UART for.
pub fn run<T: ModbusTransport>(mut xy: Xy<T>, expected_baud: u32, settle: Duration) -> Report {
    let mut report = Report::default();

    let snapshot = match Snapshot::capture(&mut xy) {
        Ok(s) => s,
        Err(e) => {
            error!("FATAL: snapshot failed ({e}) — nothing was changed");
            report.failed += 1;
            return report;
        }
    };
    info!("snapshot: {snapshot:#?}");

    if let Err(e) = prepare(&mut xy) {
        error!("FATAL: safe baseline failed ({e})");
        if let Err(restore) = snapshot.restore(&mut xy) {
            error!("restore after baseline failure also failed: {restore}");
        }
        report.failed += 1;
        return report;
    }

    type Test<T> = fn(&mut Xy<T>) -> Outcome;
    let tests: [(&str, Test<T>); 13] = [
        ("identity", identity),
        (
            "status_agrees_with_granular_reads",
            status_agrees_with_granular_reads,
        ),
        ("totals", totals),
        ("setpoint_sweeps", setpoint_sweeps),
        ("protection_sweep", protection_sweep),
        ("crossing_guards", crossing_guards),
        ("protection_status", protection_status),
        ("flags", flags),
        ("temperature", temperature),
        ("display_settings", display_settings),
        ("request_faults", request_faults),
        ("s_otp_scale", s_otp_scale),
        ("temp_offset_writes_ignored", temp_offset_writes_ignored),
    ];
    for (name, test) in tests {
        report.record(name, test(&mut xy));
    }
    // Kept out of the table because they take an extra argument, or must run
    // in a particular order relative to the restore below.
    report.record("output_enable", output_enable(&mut xy, settle));
    report.record("sleep_ceiling", sleep_ceiling(&mut xy, settle));
    report.record("backlight_floor", backlight_floor(&mut xy));
    report.record("comms", comms(&mut xy, expected_baud));
    report.record("group_round_trip", group_round_trip(&mut xy));
    report.record("recall", recall(&mut xy));

    info!("--- restoring snapshot ---");
    report.record("snapshot restore", snapshot.restore(&mut xy));
    report.record("lifecycle", lifecycle(xy));

    info!(
        "=== xy-modbus on-device suite complete: {} passed, {} failed ===",
        report.passed, report.failed
    );
    report
}
