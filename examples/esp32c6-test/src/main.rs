//! On-device exerciser for every public XY7025 API in `xy-modbus`.
//!
//! Wiring (matches esp32-battery board, UART1 @ 115200 8N1):
//!   ESP GPIO16 (TX) -> XY RX
//!   ESP GPIO17 (RX) -> XY TX
//!   common GND.
//!
//! WARNING: this test enables the buck output briefly with V_SET=0 and
//! I_SET=0 to verify the on/off plumbing. Disconnect any sensitive load
//! before running. Baud rate and slave address are read but never
//! written — changing them would orphan the device on next boot.
//!
//! Every other writable register is sweep-tested across its documented
//! range. Original values are snapshotted at start and restored at end.

use std::thread;
use std::time::Duration;

use log::{error, info, warn};

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::uart::UartDriver;
use esp_idf_hal::uart::config::Config;
use esp_idf_hal::units::Hertz;

use xy_modbus::{
    BaudRate, GroupParams, Model, ModelCheck, ProtectionStatus, RegMode, RtuError, SafetyLimits,
    Setpoints, TempUnit, Xy,
};

const BAUD: u32 = 115200;
const PACK_MODEL: Model = Model::Xy7025;

// ─── XY7025 documented ranges (DATASHEET.md §1) ─────────────────────────────
// V_OUT 0–70 V, I_OUT 0–25 A, resolution 0.01.
// OVP 0–72, OCP 0–27, OPP 0–2000, LVP 10–95, OTP 0–110.

const V_SET_SAMPLES: &[f32] = &[
    0.00, 0.01, 0.05, 0.10, 0.50, 1.00, 3.30, 5.00, 7.50, 10.00, 12.00, 12.34, 13.50, 14.40, 19.99,
    25.00, 33.33, 48.00, 60.00, 70.00,
];
const I_SET_SAMPLES: &[f32] = &[
    0.00, 0.01, 0.05, 0.10, 0.25, 0.50, 1.00, 1.23, 2.50, 5.00, 10.00, 15.00, 20.00, 25.00,
];

// Headroom protection while sweeping V/I. Restored after the sweep.
const HEADROOM_SAFETY: SafetyLimits = SafetyLimits {
    lvp_v: 10.0,
    ovp_v: 72.0,
    ocp_a: 27.0,
};

const PROT_SAMPLES: &[SafetyLimits] = &[
    SafetyLimits { lvp_v: 10.0, ovp_v: 5.00, ocp_a: 0.50 },
    SafetyLimits { lvp_v: 11.5, ovp_v: 14.40, ocp_a: 5.00 },
    SafetyLimits { lvp_v: 22.0, ovp_v: 28.80, ocp_a: 10.00 },
    SafetyLimits { lvp_v: 44.0, ovp_v: 56.00, ocp_a: 20.00 },
    SafetyLimits { lvp_v: 90.0, ovp_v: 70.00, ocp_a: 27.00 },
    SafetyLimits { lvp_v: 10.0, ovp_v: 72.00, ocp_a: 27.00 },
];

const TEMP_OFFSET_SAMPLES: &[f32] = &[-9.9, -5.0, -1.5, -0.1, 0.0, 0.1, 1.5, 5.0, 9.9];

const SLEEP_SAMPLES: &[u16] = &[0, 1, 2, 5, 10, 15, 30, 60];

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("xy-modbus ESP32-C6 on-device test suite starting");

    let peripherals = Peripherals::take().expect("peripherals");
    let uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio16,
        peripherals.pins.gpio17,
        None::<esp_idf_hal::gpio::AnyIOPin>,
        None::<esp_idf_hal::gpio::AnyIOPin>,
        &Config::new().baudrate(Hertz(BAUD)),
    )
    .expect("UART1 init");

    let mut xy = Xy::from_esp_uart(uart, PACK_MODEL);

    // Snapshot every writable register before we touch anything, so a
    // panic mid-suite leaves a clean log of what to manually restore.
    let snapshot = match snapshot_all(&mut xy) {
        Ok(s) => s,
        Err(e) => {
            error!("FATAL: snapshot failed: {e} — aborting");
            park();
        }
    };
    info!("snapshot taken: {snapshot:#?}");

    // Force a known-safe baseline: output OFF, protection cleared, OVP/OCP
    // wide open so the V/I sweeps don't trip mid-write.
    if let Err(e) = xy.set_output(false).and_then(|_| xy.clear_protection_status()) {
        warn!("baseline disable/clear failed: {e}");
    }
    if let Err(e) = xy.set_protection(HEADROOM_SAFETY) {
        warn!("baseline set_protection failed: {e}");
    }

    let mut pass = 0u32;
    let mut fail = 0u32;
    let mut run = |name: &str, r: Result<(), String>| match r {
        Ok(()) => {
            info!("PASS  {name}");
            pass += 1;
        }
        Err(e) => {
            error!("FAIL  {name}: {e}");
            fail += 1;
        }
    };

    run("identity", test_identity(&mut xy));
    run("status_consistency", test_status_consistency(&mut xy));
    run("live_readings", test_live_readings(&mut xy));
    run("totals", test_totals(&mut xy));
    run("voltage_sweep", test_voltage_sweep(&mut xy));
    run("current_sweep", test_current_sweep(&mut xy));
    run("protection_sweep", test_protection_sweep(&mut xy));
    run("protection_status_clear", test_protection_status_clear(&mut xy));
    run("output_enable_disable", test_output_enable_disable(&mut xy));
    run("power_on_output", test_power_on_output(&mut xy));
    run("reg_mode", test_reg_mode(&mut xy));
    run("temperatures", test_temperatures(&mut xy));
    run("temp_unit", test_temp_unit(&mut xy));
    run("temp_offsets", test_temp_offsets(&mut xy));
    run("lock", test_lock(&mut xy));
    run("backlight_full_range", test_backlight_full_range(&mut xy));
    run("sleep_minutes_sweep", test_sleep_minutes_sweep(&mut xy));
    run("buzzer", test_buzzer(&mut xy));
    run("comms_settings_read_only", test_comms_settings(&mut xy));
    run("group_full_round_trip_all", test_group_full_round_trip_all(&mut xy));
    run("recall_each_group", test_recall_each_group(&mut xy));

    info!("--- restoring snapshot ---");
    match restore_all(&mut xy, &snapshot) {
        Ok(()) => info!("snapshot restored"),
        Err(e) => error!("snapshot restore FAILED: {e}"),
    }

    info!("=== xy-modbus on-device test complete: {pass} passed, {fail} failed ===");
    park();
}

fn park() -> ! {
    loop {
        thread::sleep(Duration::from_secs(60));
    }
}

// ─── Snapshot ───────────────────────────────────────────────────────────────

#[derive(Debug)]
struct Snapshot {
    setpoints: Setpoints,
    safety: SafetyLimits,
    power_on_output: bool,
    output_on: bool,
    temp_unit: TempUnit,
    temp_offset_internal: f32,
    temp_offset_external: f32,
    lock: bool,
    backlight: u8,
    sleep_minutes: u16,
    buzzer: bool,
    groups: [GroupParams; 10],
}

fn snapshot_all<'d>(xy: &mut T<'d>) -> Result<Snapshot, String> {
    let mut groups = core::array::from_fn(|_| GroupParams {
        v_set: 0.0,
        i_set: 0.0,
        s_lvp_v: 0.0,
        s_ovp_v: 0.0,
        s_ocp_a: 0.0,
        s_opp_w: 0.0,
        s_ohp_h: 0,
        s_ohp_m: 0,
        s_oah_ah: 0.0,
        s_owh_wh: 0.0,
        s_otp: 0.0,
        power_on_output: false,
    });
    for (n, slot) in groups.iter_mut().enumerate() {
        *slot = xy.read_group(n as u8).map_err(rtu)?;
    }
    Ok(Snapshot {
        setpoints: xy.read_setpoints().map_err(rtu)?,
        safety: xy.read_protection().map_err(rtu)?,
        power_on_output: xy.read_power_on_output().map_err(rtu)?,
        output_on: xy.read_output().map_err(rtu)?,
        temp_unit: xy.read_temp_unit().map_err(rtu)?,
        temp_offset_internal: xy.read_temp_offset_internal().map_err(rtu)?,
        temp_offset_external: xy.read_temp_offset_external().map_err(rtu)?,
        lock: xy.read_lock().map_err(rtu)?,
        backlight: xy.read_backlight().map_err(rtu)?,
        sleep_minutes: xy.read_sleep_minutes().map_err(rtu)?,
        buzzer: xy.read_buzzer().map_err(rtu)?,
        groups,
    })
}

fn restore_all<'d>(xy: &mut T<'d>, s: &Snapshot) -> Result<(), String> {
    // Output OFF before touching setpoints. Caller can re-enable manually.
    xy.set_output(false).map_err(rtu)?;
    // Drop V_SET first so re-applying the original protection (which may
    // have an OVP below the post-sweep V_SET) isn't rejected.
    xy.set_voltage(0.0).map_err(rtu)?;
    xy.set_current_limit(0.0).map_err(rtu)?;
    for (n, g) in s.groups.iter().enumerate() {
        xy.write_group(n as u8, g).map_err(rtu)?;
    }
    xy.set_protection(s.safety).map_err(rtu)?;
    xy.set_voltage(s.setpoints.v_set).map_err(rtu)?;
    xy.set_current_limit(s.setpoints.i_set).map_err(rtu)?;
    xy.set_power_on_output(s.power_on_output).map_err(rtu)?;
    xy.set_temp_unit(s.temp_unit).map_err(rtu)?;
    xy.set_temp_offset_internal(s.temp_offset_internal).map_err(rtu)?;
    xy.set_temp_offset_external(s.temp_offset_external).map_err(rtu)?;
    xy.set_lock(s.lock).map_err(rtu)?;
    xy.set_backlight(s.backlight).map_err(rtu)?;
    xy.set_sleep_minutes(s.sleep_minutes).map_err(rtu)?;
    xy.set_buzzer(s.buzzer).map_err(rtu)?;
    xy.clear_protection_status().map_err(rtu)?;
    if s.output_on {
        warn!("snapshot had output ON — leaving it OFF; re-enable manually if intended");
    }
    Ok(())
}

// ─── Helpers ────────────────────────────────────────────────────────────────

type T<'d> = Xy<xy_modbus::esp_idf::EspIdfTransport<'d>>;

fn rtu(e: RtuError) -> String {
    format!("rtu: {e}")
}

fn approx(a: f32, b: f32) -> bool {
    (a - b).abs() < 0.02
}

fn expect_eq<X: PartialEq + core::fmt::Debug>(name: &str, expected: X, actual: X) -> Result<(), String> {
    if expected == actual {
        Ok(())
    } else {
        Err(format!("{name}: expected {expected:?}, got {actual:?}"))
    }
}

fn expect_approx(name: &str, expected: f32, actual: f32) -> Result<(), String> {
    if approx(expected, actual) {
        Ok(())
    } else {
        Err(format!("{name}: expected {expected:.3}, got {actual:.3}"))
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────────

fn test_identity(xy: &mut T) -> Result<(), String> {
    let model_raw = xy.read_model().map_err(rtu)?;
    let version = xy.read_version().map_err(rtu)?;
    let check = xy.verify_model().map_err(rtu)?;
    info!("  MODEL=0x{model_raw:04X} VERSION=0x{version:04X} check={check:?}");
    match check {
        ModelCheck::Match { device_code } => expect_eq("model_code", 0x6500u16, device_code),
        ModelCheck::Mismatch { expected_code, device_code } => Err(format!(
            "MODEL mismatch: configured 0x{expected_code:04X}, device 0x{device_code:04X}"
        )),
        ModelCheck::Inconclusive { device_code } => Err(format!(
            "MODEL inconclusive (device 0x{device_code:04X}); XY7025 expected 0x6500"
        )),
    }
}

fn test_status_consistency(xy: &mut T) -> Result<(), String> {
    // status read must agree with the granular reads that share registers.
    let s = xy.read_status().map_err(rtu)?;
    let sp = xy.read_setpoints().map_err(rtu)?;
    let v_out = xy.read_voltage_out().map_err(rtu)?;
    let i_out = xy.read_current_out().map_err(rtu)?;
    let p_out = xy.read_power_out().map_err(rtu)?;
    let v_in = xy.read_voltage_in().map_err(rtu)?;
    let on = xy.read_output().map_err(rtu)?;
    let mode = xy.read_reg_mode().map_err(rtu)?;
    expect_approx("status.v_set vs read_setpoints", s.v_set, sp.v_set)?;
    expect_approx("status.i_set vs read_setpoints", s.i_set, sp.i_set)?;
    expect_approx("status.v_out vs read_voltage_out", s.v_out, v_out)?;
    expect_approx("status.i_out vs read_current_out", s.i_out, i_out)?;
    expect_approx("status.p_out vs read_power_out", s.p_out, p_out)?;
    expect_approx("status.v_in vs read_voltage_in", s.v_in, v_in)?;
    expect_eq("status.output_on vs read_output", s.output_on, on)?;
    expect_eq("status.reg_mode vs read_reg_mode", s.reg_mode, mode)?;
    info!(
        "  status: V_SET={:.2} I_SET={:.2} V_OUT={:.2} I_OUT={:.3} P_OUT={:.2} V_IN={:.2} prot={} reg={:?} on={}",
        s.v_set, s.i_set, s.v_out, s.i_out, s.p_out, s.v_in, s.protection, s.reg_mode, s.output_on,
    );
    Ok(())
}

fn test_live_readings(xy: &mut T) -> Result<(), String> {
    let v_in = xy.read_voltage_in().map_err(rtu)?;
    info!("  V_IN={v_in:.2}");
    if v_in < 1.0 {
        return Err(format!("V_IN={v_in:.2} — is the buck powered?"));
    }
    Ok(())
}

fn test_totals(xy: &mut T) -> Result<(), String> {
    let t = xy.read_totals().map_err(rtu)?;
    info!(
        "  totals: charge={:.3} Ah, energy={:.3} Wh, on_time={}h{}m{}s ({}s)",
        t.charge_ah,
        t.energy_wh,
        t.on_time.hours,
        t.on_time.minutes,
        t.on_time.seconds,
        t.on_time.total_seconds(),
    );
    Ok(())
}

fn test_voltage_sweep(xy: &mut T) -> Result<(), String> {
    for &v in V_SET_SAMPLES {
        xy.set_voltage(v).map_err(rtu)?;
        let got = xy.read_setpoints().map_err(rtu)?.v_set;
        expect_approx(&format!("V_SET={v:.2}"), v, got)?;
        // Also verify status agrees.
        let s = xy.read_status().map_err(rtu)?;
        expect_approx(&format!("V_SET={v:.2} via status"), v, s.v_set)?;
    }
    info!("  swept {} V values", V_SET_SAMPLES.len());
    Ok(())
}

fn test_current_sweep(xy: &mut T) -> Result<(), String> {
    for &i in I_SET_SAMPLES {
        xy.set_current_limit(i).map_err(rtu)?;
        let got = xy.read_setpoints().map_err(rtu)?.i_set;
        expect_approx(&format!("I_SET={i:.2}"), i, got)?;
    }
    info!("  swept {} I values", I_SET_SAMPLES.len());
    Ok(())
}

fn test_protection_sweep(xy: &mut T) -> Result<(), String> {
    // Drop V_SET below the smallest OVP sample so set_protection isn't
    // silently rejected for OVP < V_SET.
    xy.set_voltage(0.0).map_err(rtu)?;
    xy.set_current_limit(0.0).map_err(rtu)?;
    for s in PROT_SAMPLES {
        xy.set_protection(*s).map_err(rtu)?;
        let got = xy.read_protection().map_err(rtu)?;
        expect_approx(&format!("LVP@{:?}", s), s.lvp_v, got.lvp_v)?;
        expect_approx(&format!("OVP@{:?}", s), s.ovp_v, got.ovp_v)?;
        expect_approx(&format!("OCP@{:?}", s), s.ocp_a, got.ocp_a)?;
    }
    // Restore a wide-open headroom for downstream tests.
    xy.set_protection(HEADROOM_SAFETY).map_err(rtu)?;
    info!("  swept {} protection sets", PROT_SAMPLES.len());
    Ok(())
}

fn test_protection_status_clear(xy: &mut T) -> Result<(), String> {
    let before = xy.read_protection_status().map_err(rtu)?;
    info!("  PROTECT={before}");
    if matches!(before, ProtectionStatus::Unknown(_)) {
        return Err(format!("PROTECT decoded as {before}"));
    }
    xy.clear_protection_status().map_err(rtu)?;
    let after = xy.read_protection_status().map_err(rtu)?;
    expect_eq("PROTECT after clear", ProtectionStatus::Normal, after)
}

fn test_output_enable_disable(xy: &mut T) -> Result<(), String> {
    // Force a no-load-friendly condition first.
    xy.set_output(false).map_err(rtu)?;
    xy.set_voltage(0.0).map_err(rtu)?;
    xy.set_current_limit(0.0).map_err(rtu)?;
    xy.clear_protection_status().map_err(rtu)?;

    xy.set_output(true).map_err(rtu)?;
    thread::sleep(Duration::from_millis(100));
    let on = xy.read_output().map_err(rtu)?;
    let s_on = xy.read_status().map_err(rtu)?;
    expect_eq("OUTPUT_EN after enable", true, on)?;
    expect_eq("status.output_on after enable", true, s_on.output_on)?;
    info!("  output ON: V_OUT={:.2} I_OUT={:.3}", s_on.v_out, s_on.i_out);

    xy.set_output(false).map_err(rtu)?;
    thread::sleep(Duration::from_millis(100));
    let off = xy.read_output().map_err(rtu)?;
    let s_off = xy.read_status().map_err(rtu)?;
    expect_eq("OUTPUT_EN after disable", false, off)?;
    expect_eq("status.output_on after disable", false, s_off.output_on)
}

fn test_power_on_output(xy: &mut T) -> Result<(), String> {
    let original = xy.read_power_on_output().map_err(rtu)?;
    for v in [!original, original, true, false, true, false] {
        xy.set_power_on_output(v).map_err(rtu)?;
        expect_eq("S_INI round-trip", v, xy.read_power_on_output().map_err(rtu)?)?;
    }
    Ok(())
}

fn test_reg_mode(xy: &mut T) -> Result<(), String> {
    let m = xy.read_reg_mode().map_err(rtu)?;
    info!("  CVCC={m:?}");
    matches!(m, RegMode::ConstantVoltage | RegMode::ConstantCurrent)
        .then_some(())
        .ok_or_else(|| format!("unexpected reg mode {m:?}"))
}

fn test_temperatures(xy: &mut T) -> Result<(), String> {
    let (internal, external) = xy.read_temperatures().map_err(rtu)?;
    info!("  T_INT={internal:.1} T_EXT={external:.1}");
    Ok(())
}

fn test_temp_unit(xy: &mut T) -> Result<(), String> {
    for u in [TempUnit::Celsius, TempUnit::Fahrenheit, TempUnit::Celsius, TempUnit::Fahrenheit] {
        xy.set_temp_unit(u).map_err(rtu)?;
        expect_eq("F-C", u, xy.read_temp_unit().map_err(rtu)?)?;
    }
    Ok(())
}

fn test_temp_offsets(xy: &mut T) -> Result<(), String> {
    for &v in TEMP_OFFSET_SAMPLES {
        xy.set_temp_offset_internal(v).map_err(rtu)?;
        expect_approx(
            &format!("T_INT_OFFSET={v:.1}"),
            v,
            xy.read_temp_offset_internal().map_err(rtu)?,
        )?;
        xy.set_temp_offset_external(v).map_err(rtu)?;
        expect_approx(
            &format!("T_EXT_OFFSET={v:.1}"),
            v,
            xy.read_temp_offset_external().map_err(rtu)?,
        )?;
    }
    info!("  swept {} temp offsets", TEMP_OFFSET_SAMPLES.len());
    Ok(())
}

fn test_lock(xy: &mut T) -> Result<(), String> {
    for v in [true, false, true, false] {
        xy.set_lock(v).map_err(rtu)?;
        expect_eq("LOCK", v, xy.read_lock().map_err(rtu)?)?;
    }
    Ok(())
}

fn test_backlight_full_range(xy: &mut T) -> Result<(), String> {
    // B-LED documented range 0..=5.
    for level in 0u8..=5 {
        xy.set_backlight(level).map_err(rtu)?;
        expect_eq(&format!("BL={level}"), level, xy.read_backlight().map_err(rtu)?)?;
    }
    Ok(())
}

fn test_sleep_minutes_sweep(xy: &mut T) -> Result<(), String> {
    for &m in SLEEP_SAMPLES {
        xy.set_sleep_minutes(m).map_err(rtu)?;
        expect_eq(&format!("SLEEP={m}"), m, xy.read_sleep_minutes().map_err(rtu)?)?;
    }
    info!("  swept {} sleep values", SLEEP_SAMPLES.len());
    Ok(())
}

fn test_buzzer(xy: &mut T) -> Result<(), String> {
    for v in [true, false, true, false] {
        xy.set_buzzer(v).map_err(rtu)?;
        expect_eq("BUZZER", v, xy.read_buzzer().map_err(rtu)?)?;
    }
    Ok(())
}

fn test_comms_settings(xy: &mut T) -> Result<(), String> {
    // Read-only — writing slave address or baud rate would orphan the
    // device on the next cold boot.
    let slave = xy.read_slave_address().map_err(rtu)?;
    let baud = xy.read_baud_rate().map_err(rtu)?;
    info!("  SLAVE=0x{slave:02X} BAUD={baud:?}");
    if slave != xy.slave() {
        return Err(format!(
            "SLAVE mismatch: device 0x{slave:02X}, transport assumes 0x{:02X}",
            xy.slave()
        ));
    }
    if !matches!(baud, BaudRate::B115200) {
        warn!("  device baud is {baud:?}, not B115200 — UART would normally be misconfigured");
    }
    Ok(())
}

fn test_group_full_round_trip_all(xy: &mut T) -> Result<(), String> {
    // Write a unique probe to every M0..=M9, read back, verify, restore.
    for n in 0u8..=9 {
        let original = xy.read_group(n).map_err(rtu)?;
        let probe = GroupParams {
            v_set: 5.0 + n as f32 * 0.5,
            i_set: 0.10 + n as f32 * 0.05,
            s_lvp_v: 10.0 + n as f32,
            s_ovp_v: 20.0 + n as f32,
            s_ocp_a: 1.0 + n as f32 * 0.25,
            s_opp_w: 50.0 + n as f32 * 5.0,
            s_ohp_h: n as u16,
            s_ohp_m: n as u16 * 5,
            s_oah_ah: 1.0 + n as f32,
            s_owh_wh: 10.0 + n as f32 * 2.0,
            s_otp: 50.0 + n as f32,
            power_on_output: n % 2 == 0,
        };
        xy.write_group(n, &probe).map_err(rtu)?;
        let r = xy.read_group(n).map_err(rtu)?;
        expect_approx(&format!("M{n} v_set"), probe.v_set, r.v_set)?;
        expect_approx(&format!("M{n} i_set"), probe.i_set, r.i_set)?;
        expect_approx(&format!("M{n} s_lvp_v"), probe.s_lvp_v, r.s_lvp_v)?;
        expect_approx(&format!("M{n} s_ovp_v"), probe.s_ovp_v, r.s_ovp_v)?;
        expect_approx(&format!("M{n} s_ocp_a"), probe.s_ocp_a, r.s_ocp_a)?;
        expect_approx(&format!("M{n} s_opp_w"), probe.s_opp_w, r.s_opp_w)?;
        expect_eq(&format!("M{n} s_ohp_h"), probe.s_ohp_h, r.s_ohp_h)?;
        expect_eq(&format!("M{n} s_ohp_m"), probe.s_ohp_m, r.s_ohp_m)?;
        expect_approx(&format!("M{n} s_oah_ah"), probe.s_oah_ah, r.s_oah_ah)?;
        expect_approx(&format!("M{n} s_owh_wh"), probe.s_owh_wh, r.s_owh_wh)?;
        expect_approx(&format!("M{n} s_otp"), probe.s_otp, r.s_otp)?;
        expect_eq(&format!("M{n} power_on_output"), probe.power_on_output, r.power_on_output)?;
        // Restore immediately so a later failure doesn't leave M0..M9 trashed.
        xy.write_group(n, &original).map_err(rtu)?;
    }
    info!("  exercised all 10 memory groups");
    Ok(())
}

fn test_recall_each_group(xy: &mut T) -> Result<(), String> {
    // Recall overwrites V_SET / I_SET / protection. Keep output OFF and
    // re-apply HEADROOM_SAFETY between recalls so a group with a tight
    // OVP doesn't break the next iteration's V sweep prerequisites.
    xy.set_output(false).map_err(rtu)?;
    for n in 0u8..=9 {
        // Lower V_SET first so the recalled group's OVP can't be below
        // the live setpoint and reject the write.
        xy.set_voltage(0.0).map_err(rtu)?;
        xy.recall_group(n).map_err(rtu)?;
        let _ = xy.read_setpoints().map_err(rtu)?;
        xy.set_protection(HEADROOM_SAFETY).map_err(rtu)?;
    }
    info!("  recalled all 10 memory groups");
    Ok(())
}
