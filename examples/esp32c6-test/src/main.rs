//! On-device exerciser for every public XY7025 API in `xy-modbus`.
//!
//! Wiring (matches esp32-battery board, UART1 @ 115200 8N1):
//!   ESP GPIO16 (TX) -> XY RX
//!   ESP GPIO17 (RX) -> XY TX
//!   common GND.
//!
//! WARNING: this test enables the buck output briefly with V_SET=0 and
//! I_SET=0 to verify the on/off plumbing. Disconnect any sensitive load
//! before running. Baud rate and slave address are never changed —
//! changing them would orphan the device on next boot.
//!
//! Persistent configuration is snapshotted at start and restored at end.
//! The live output and any latched protection cause are left safely off
//! and cleared rather than restored.

use std::thread;
use std::time::Duration;

use log::{error, info, warn};

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::uart::UartDriver;
use esp_idf_hal::uart::config::Config;
use esp_idf_hal::units::Hertz;

use xy_modbus::{
    GroupParams, ProtectionStatus, SafetyLimits, ScaleCheck, Setpoints, TempUnit, Temperature, Xy,
};

const BAUD: u32 = 115200;
const GROUP_BASE: u16 = 0x0050;
const GROUP_STRIDE: u16 = 0x0010;
const GROUP_COUNT: usize = 10;
const GROUP_REGISTER_COUNT: usize = 14;

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
        lvp_v: 22.0,
        ovp_v: 28.80,
        ocp_a: 10.00,
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
    SafetyLimits {
        lvp_v: 10.0,
        ovp_v: 72.00,
        ocp_a: 27.00,
    },
];

// Firmware caps SLEEP at 9 minutes max (raw probe: any write ≥10 reads
// back as 9). 0 = disabled.
const SLEEP_SAMPLES: &[u16] = &[0, 1, 2, 5, 8, 9];

// Backlight: firmware clamps 0 → 1 (display always at least dim). Sweep
// the accepted range only.
const BACKLIGHT_SAMPLES: core::ops::RangeInclusive<u8> = 1..=5;

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

    let mut xy = Xy::from_esp_uart(uart);

    // Capture exact register words so firmware conversion cannot alter
    // temperatures while restoring a non-Celsius snapshot.
    let snapshot = match snapshot_all(&mut xy) {
        Ok(s) => s,
        Err(e) => {
            error!("FATAL: snapshot failed: {e} — aborting");
            park();
        }
    };
    info!("snapshot taken: {snapshot:#?}");

    if let Err(e) = prepare_safe_baseline(&mut xy) {
        error!("FATAL: safe baseline failed: {e} — aborting");
        match restore_all(&mut xy, &snapshot) {
            Ok(()) => info!("snapshot restored after baseline failure"),
            Err(restore_error) => {
                error!("snapshot restore after baseline failure FAILED: {restore_error}")
            }
        }
        park();
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
    run("totals", test_totals(&mut xy));
    run("voltage_sweep", test_voltage_sweep(&mut xy));
    run("current_sweep", test_current_sweep(&mut xy));
    run("protection_sweep", test_protection_sweep(&mut xy));
    run(
        "protection_status_clear",
        test_protection_status_clear(&mut xy),
    );
    run("output_enable_disable", test_output_enable_disable(&mut xy));
    run("power_on_output", test_power_on_output(&mut xy));
    run("reg_mode", test_reg_mode(&mut xy));
    run("internal temperature", test_internal_temperature(&mut xy));
    run("temp_unit", test_temp_unit(&mut xy));
    run(
        "temp_offsets_read_only",
        test_temp_offsets_read_only(&mut xy),
    );
    run("lock", test_lock(&mut xy));
    run("backlight_full_range", test_backlight_full_range(&mut xy));
    run("sleep_minutes_sweep", test_sleep_minutes_sweep(&mut xy));
    run("buzzer", test_buzzer(&mut xy));
    run("comms_settings_read_only", test_comms_settings(&mut xy));
    run(
        "comms_setters_idempotent",
        test_comms_setters_idempotent(&mut xy),
    );
    run("s_otp_raw_probe", test_s_otp_raw_probe(&mut xy));
    run("temp_offset_raw_probe", test_temp_offset_raw_probe(&mut xy));
    run("sleep_raw_probe", test_sleep_raw_probe(&mut xy));
    run(
        "group_full_round_trip_all",
        test_group_full_round_trip_all(&mut xy),
    );
    run("recall_each_group", test_recall_each_group(&mut xy));

    info!("--- restoring snapshot ---");
    match restore_all(&mut xy, &snapshot) {
        Ok(()) => info!("snapshot restored"),
        Err(e) => {
            error!("snapshot restore FAILED: {e}");
            fail += 1;
        }
    }

    // Terminal test: exercises the constructor/destructor lifecycle —
    // `into_transport` (consumes `xy`) followed by `with_slave` to
    // rebuild on the same UART. Must run last because it moves `xy`.
    let lifecycle = test_lifecycle(xy);
    match &lifecycle {
        Ok(()) => {
            info!("PASS  lifecycle");
            pass += 1;
        }
        Err(e) => {
            error!("FAIL  lifecycle: {e}");
            fail += 1;
        }
    }

    info!("=== xy-modbus on-device test complete: {pass} passed, {fail} failed ===");
    park();
}

fn park() -> ! {
    loop {
        thread::sleep(Duration::from_secs(60));
    }
}

#[derive(Debug)]
struct Snapshot {
    output_on: bool,
    temp_unit: TempUnit,
    lock: bool,
    backlight: u8,
    sleep_minutes: u16,
    buzzer: bool,
    groups: [[u16; GROUP_REGISTER_COUNT]; GROUP_COUNT],
}

fn snapshot_all<'d>(xy: &mut T<'d>) -> Result<Snapshot, String> {
    let mut groups = [[0; GROUP_REGISTER_COUNT]; GROUP_COUNT];
    for (n, slot) in groups.iter_mut().enumerate() {
        xy.read_raw_holding(group_address(n), slot)
            .map_err(driver_error)?;
    }
    Ok(Snapshot {
        output_on: xy.read_output().map_err(driver_error)?,
        temp_unit: xy.read_temp_unit().map_err(driver_error)?,
        lock: xy.read_lock().map_err(driver_error)?,
        backlight: xy.read_backlight().map_err(driver_error)?,
        sleep_minutes: xy.read_sleep_minutes().map_err(driver_error)?,
        buzzer: xy.read_buzzer().map_err(driver_error)?,
        groups,
    })
}

fn restore_all<'d>(xy: &mut T<'d>, s: &Snapshot) -> Result<(), String> {
    xy.set_output(false)
        .map_err(|e| format!("cannot disable output before restore: {e}"))?;
    expect_eq(
        "output before restore",
        false,
        xy.read_output().map_err(driver_error)?,
    )?;

    let mut errors = Vec::new();
    record_restore(
        &mut errors,
        "unlock front panel",
        xy.set_lock(false).map_err(driver_error),
    );
    record_restore(
        &mut errors,
        "set V_SET=0",
        xy.set_voltage(0.0).map_err(driver_error),
    );
    record_restore(
        &mut errors,
        "set I_SET=0",
        xy.set_current_limit(0.0).map_err(driver_error),
    );
    record_restore(
        &mut errors,
        "restore temperature unit",
        xy.set_temp_unit(s.temp_unit).map_err(driver_error),
    );

    for n in 1..GROUP_COUNT {
        restore_group(xy, n, &s.groups[n], &mut errors);
    }
    restore_group(xy, 0, &s.groups[0], &mut errors);

    record_restore(
        &mut errors,
        "restore backlight",
        xy.set_backlight(s.backlight).map_err(driver_error),
    );
    record_restore(
        &mut errors,
        "restore sleep timeout",
        xy.set_sleep_minutes(s.sleep_minutes).map_err(driver_error),
    );
    record_restore(
        &mut errors,
        "restore buzzer",
        xy.set_buzzer(s.buzzer).map_err(driver_error),
    );
    record_restore(
        &mut errors,
        "clear protection status",
        xy.clear_protection_status().map_err(driver_error),
    );
    record_restore(
        &mut errors,
        "restore lock",
        xy.set_lock(s.lock).map_err(driver_error),
    );

    verify_restoration(xy, s, &mut errors);
    if s.output_on {
        warn!("snapshot had output ON — leaving it OFF; re-enable manually if intended");
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

fn prepare_safe_baseline(xy: &mut T) -> Result<(), String> {
    xy.set_output(false).map_err(driver_error)?;
    expect_eq(
        "safe baseline output",
        false,
        xy.read_output().map_err(driver_error)?,
    )?;
    xy.set_power_on_output(false).map_err(driver_error)?;
    expect_eq(
        "safe baseline power-on output",
        false,
        xy.read_power_on_output().map_err(driver_error)?,
    )?;
    xy.clear_protection_status().map_err(driver_error)?;
    xy.set_protection(HEADROOM_SAFETY).map_err(driver_error)
}

fn group_address(n: usize) -> u16 {
    debug_assert!(n < GROUP_COUNT);
    GROUP_BASE + n as u16 * GROUP_STRIDE
}

fn restore_group(
    xy: &mut T,
    n: usize,
    words: &[u16; GROUP_REGISTER_COUNT],
    errors: &mut Vec<String>,
) {
    if n == 0 {
        // Restore live protection before V_SET so an OVP crossing cannot trip.
        for (offset, &word) in words.iter().enumerate().skip(2) {
            restore_group_word(xy, n, offset, word, errors);
        }
        for (offset, &word) in words.iter().enumerate().take(2) {
            restore_group_word(xy, n, offset, word, errors);
        }
    } else {
        for (offset, &word) in words.iter().enumerate() {
            restore_group_word(xy, n, offset, word, errors);
        }
    }
}

fn restore_group_exact(
    xy: &mut T,
    n: usize,
    words: &[u16; GROUP_REGISTER_COUNT],
) -> Result<(), String> {
    let mut errors = Vec::new();
    restore_group(xy, n, words, &mut errors);
    let mut actual = [0; GROUP_REGISTER_COUNT];
    record_restore(
        &mut errors,
        &format!("verify restored M{n}"),
        xy.read_raw_holding(group_address(n), &mut actual)
            .map_err(driver_error)
            .and_then(|()| expect_eq(&format!("M{n} raw restore"), *words, actual)),
    );
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

fn restore_group_word(xy: &mut T, n: usize, offset: usize, word: u16, errors: &mut Vec<String>) {
    let label = format!("restore M{n} register +{offset}");
    record_restore(
        errors,
        &label,
        xy.write_raw_holding(group_address(n) + offset as u16, word)
            .map_err(driver_error),
    );
}

fn verify_restoration(xy: &mut T, s: &Snapshot, errors: &mut Vec<String>) {
    record_restore(
        errors,
        "verify output remains off",
        xy.read_output()
            .map_err(driver_error)
            .and_then(|actual| expect_eq("OUTPUT_EN", false, actual)),
    );
    record_restore(
        errors,
        "verify temperature unit",
        xy.read_temp_unit()
            .map_err(driver_error)
            .and_then(|actual| expect_eq("F-C", s.temp_unit, actual)),
    );
    record_restore(
        errors,
        "verify lock",
        xy.read_lock()
            .map_err(driver_error)
            .and_then(|actual| expect_eq("LOCK", s.lock, actual)),
    );
    record_restore(
        errors,
        "verify backlight",
        xy.read_backlight()
            .map_err(driver_error)
            .and_then(|actual| expect_eq("B-LED", s.backlight, actual)),
    );
    record_restore(
        errors,
        "verify sleep timeout",
        xy.read_sleep_minutes()
            .map_err(driver_error)
            .and_then(|actual| expect_eq("SLEEP", s.sleep_minutes, actual)),
    );
    record_restore(
        errors,
        "verify buzzer",
        xy.read_buzzer()
            .map_err(driver_error)
            .and_then(|actual| expect_eq("BUZZER", s.buzzer, actual)),
    );
    for n in 0..GROUP_COUNT {
        let mut actual = [0; GROUP_REGISTER_COUNT];
        let result = xy
            .read_raw_holding(group_address(n), &mut actual)
            .map_err(driver_error)
            .and_then(|()| expect_eq(&format!("M{n} raw snapshot"), s.groups[n], actual));
        record_restore(errors, &format!("verify M{n}"), result);
    }
}

fn record_restore(errors: &mut Vec<String>, label: &str, result: Result<(), String>) {
    if let Err(error) = result {
        errors.push(format!("{label}: {error}"));
    }
}

type T<'d> = Xy<xy_modbus::esp_idf::EspIdfTransport<'d>>;

fn driver_error(e: impl core::fmt::Display) -> String {
    format!("driver: {e}")
}

fn expect_eq<X: PartialEq + core::fmt::Debug>(
    name: &str,
    expected: X,
    actual: X,
) -> Result<(), String> {
    if expected == actual {
        Ok(())
    } else {
        Err(format!("{name}: expected {expected:?}, got {actual:?}"))
    }
}

fn expect_approx(name: &str, expected: f32, actual: f32) -> Result<(), String> {
    if (expected - actual).abs() < 0.02 {
        Ok(())
    } else {
        Err(format!("{name}: expected {expected:.3}, got {actual:.3}"))
    }
}

fn expect_precise(name: &str, expected: f32, actual: f32) -> Result<(), String> {
    if (expected - actual).abs() < 0.000_1 {
        Ok(())
    } else {
        Err(format!("{name}: expected {expected:.3}, got {actual:.3}"))
    }
}

fn expect_approx64(name: &str, expected: f64, actual: f64) -> Result<(), String> {
    if (expected - actual).abs() < 0.000_02 {
        Ok(())
    } else {
        Err(format!("{name}: expected {expected:.5}, got {actual:.5}"))
    }
}

fn finish_with_cleanup(
    result: Result<(), String>,
    cleanup: Result<(), String>,
    cleanup_name: &str,
) -> Result<(), String> {
    if let Err(cleanup_error) = cleanup {
        return match result {
            Ok(()) => Err(format!("{cleanup_name}: {cleanup_error}")),
            Err(test_error) => Err(format!(
                "{test_error}; {cleanup_name} also failed: {cleanup_error}"
            )),
        };
    }
    result
}

fn test_identity(xy: &mut T) -> Result<(), String> {
    let model_raw = xy.read_model().map_err(driver_error)?;
    let version = xy.read_version().map_err(driver_error)?;
    let check = xy.verify_scale_family().map_err(driver_error)?;
    info!("  MODEL=0x{model_raw:04X} VERSION=0x{version:04X} check={check:?}");
    match check {
        ScaleCheck::Compatible { .. } => Ok(()),
        ScaleCheck::Inconclusive { device_code } => Err(format!(
            "MODEL inconclusive (device 0x{device_code:04X}); XY7025 expected 0x6500"
        )),
    }
}

fn test_status_consistency(xy: &mut T) -> Result<(), String> {
    // status read must agree with the granular reads that share registers.
    let s = xy.read_status().map_err(driver_error)?;
    let sp = xy.read_setpoints().map_err(driver_error)?;
    let v_out = xy.read_voltage_out().map_err(driver_error)?;
    let i_out = xy.read_current_out().map_err(driver_error)?;
    let p_out = xy.read_power_out().map_err(driver_error)?;
    let v_in = xy.read_voltage_in().map_err(driver_error)?;
    let on = xy.read_output().map_err(driver_error)?;
    let mode = xy.read_reg_mode().map_err(driver_error)?;
    expect_approx(
        "status.v_set vs read_setpoints",
        s.setpoints.v_set,
        sp.v_set,
    )?;
    expect_approx(
        "status.i_set vs read_setpoints",
        s.setpoints.i_set,
        sp.i_set,
    )?;
    expect_approx("status.v_out vs read_voltage_out", s.v_out, v_out)?;
    expect_approx("status.i_out vs read_current_out", s.i_out, i_out)?;
    expect_approx("status.p_out vs read_power_out", s.p_out, p_out)?;
    expect_approx("status.v_in vs read_voltage_in", s.v_in, v_in)?;
    expect_eq("status.output_on vs read_output", s.output_on, on)?;
    expect_eq("status.reg_mode vs read_reg_mode", s.reg_mode, mode)?;
    info!(
        "  status: V_SET={:.2} I_SET={:.2} V_OUT={:.2} I_OUT={:.3} P_OUT={:.2} V_IN={:.2} prot={} reg={:?} on={}",
        s.setpoints.v_set,
        s.setpoints.i_set,
        s.v_out,
        s.i_out,
        s.p_out,
        s.v_in,
        s.protection,
        s.reg_mode,
        s.output_on,
    );
    if s.v_in < 1.0 {
        return Err(format!("V_IN={:.2} — is the buck powered?", s.v_in));
    }
    Ok(())
}

fn test_totals(xy: &mut T) -> Result<(), String> {
    let t = xy.read_totals().map_err(driver_error)?;
    let mut raw = [0u16; 7];
    xy.read_raw_holding(0x0006, &mut raw)
        .map_err(driver_error)?;
    let charge_raw = ((raw[1] as u32) << 16) | raw[0] as u32;
    let energy_raw = ((raw[3] as u32) << 16) | raw[2] as u32;
    expect_eq(
        "total charge decode",
        charge_raw as f64 / 1000.0,
        t.charge_ah,
    )?;
    expect_eq(
        "total energy decode",
        energy_raw as f64 / 1000.0,
        t.energy_wh,
    )?;
    expect_eq("on-time hours", raw[4], t.on_time.hours)?;
    expect_eq("on-time minutes", raw[5], t.on_time.minutes)?;
    expect_eq("on-time seconds", raw[6], t.on_time.seconds)?;
    let expected_seconds = raw[4] as u32 * 3600 + raw[5] as u32 * 60 + raw[6] as u32;
    expect_eq(
        "on-time total seconds",
        expected_seconds,
        t.on_time.total_seconds(),
    )?;
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
        xy.set_voltage(v).map_err(driver_error)?;
        let got = xy.read_setpoints().map_err(driver_error)?.v_set;
        expect_precise(&format!("V_SET={v:.2}"), v, got)?;
        // Also verify status agrees.
        let s = xy.read_status().map_err(driver_error)?;
        expect_precise(&format!("V_SET={v:.2} via status"), v, s.setpoints.v_set)?;
    }
    info!("  swept {} V values", V_SET_SAMPLES.len());
    Ok(())
}

fn test_current_sweep(xy: &mut T) -> Result<(), String> {
    for &i in I_SET_SAMPLES {
        xy.set_current_limit(i).map_err(driver_error)?;
        let got = xy.read_setpoints().map_err(driver_error)?.i_set;
        expect_precise(&format!("I_SET={i:.2}"), i, got)?;
    }
    info!("  swept {} I values", I_SET_SAMPLES.len());
    Ok(())
}

fn test_protection_sweep(xy: &mut T) -> Result<(), String> {
    // Drop V_SET below the smallest OVP sample so set_protection isn't
    // silently rejected for OVP < V_SET.
    xy.set_voltage(0.0).map_err(driver_error)?;
    xy.set_current_limit(0.0).map_err(driver_error)?;
    for s in PROT_SAMPLES {
        xy.set_protection(*s).map_err(driver_error)?;
        let got = xy.read_protection().map_err(driver_error)?;
        expect_precise(&format!("LVP@{:?}", s), s.lvp_v, got.lvp_v)?;
        expect_precise(&format!("OVP@{:?}", s), s.ovp_v, got.ovp_v)?;
        expect_precise(&format!("OCP@{:?}", s), s.ocp_a, got.ocp_a)?;
    }
    // Restore a wide-open headroom for downstream tests.
    xy.set_protection(HEADROOM_SAFETY).map_err(driver_error)?;
    info!("  swept {} protection sets", PROT_SAMPLES.len());
    Ok(())
}

fn test_protection_status_clear(xy: &mut T) -> Result<(), String> {
    let before = xy.read_protection_status().map_err(driver_error)?;
    info!("  PROTECT={before}");
    xy.clear_protection_status().map_err(driver_error)?;
    let after = xy.read_protection_status().map_err(driver_error)?;
    expect_eq("PROTECT after clear", ProtectionStatus::Normal, after)
}

fn test_output_enable_disable(xy: &mut T) -> Result<(), String> {
    // Force a no-load-friendly condition first.
    xy.set_output(false).map_err(driver_error)?;
    xy.set_voltage(0.0).map_err(driver_error)?;
    xy.set_current_limit(0.0).map_err(driver_error)?;
    xy.clear_protection_status().map_err(driver_error)?;

    let result = (|| {
        xy.set_output(true).map_err(driver_error)?;
        thread::sleep(Duration::from_millis(100));
        let on = xy.read_output().map_err(driver_error)?;
        let s_on = xy.read_status().map_err(driver_error)?;
        expect_eq("OUTPUT_EN after enable", true, on)?;
        expect_eq("status.output_on after enable", true, s_on.output_on)?;
        info!(
            "  output ON: V_OUT={:.2} I_OUT={:.3}",
            s_on.v_out, s_on.i_out
        );

        xy.set_output(false).map_err(driver_error)?;
        thread::sleep(Duration::from_millis(100));
        let off = xy.read_output().map_err(driver_error)?;
        let s_off = xy.read_status().map_err(driver_error)?;
        expect_eq("OUTPUT_EN after disable", false, off)?;
        expect_eq("status.output_on after disable", false, s_off.output_on)
    })();
    let cleanup = xy
        .set_output(false)
        .map_err(driver_error)
        .and_then(|()| xy.read_output().map_err(driver_error))
        .and_then(|actual| expect_eq("OUTPUT_EN cleanup", false, actual));
    finish_with_cleanup(result, cleanup, "output cleanup")
}

fn test_power_on_output(xy: &mut T) -> Result<(), String> {
    let result = (|| {
        for value in [true, false] {
            xy.set_power_on_output(value).map_err(driver_error)?;
            expect_eq(
                "S_INI round-trip",
                value,
                xy.read_power_on_output().map_err(driver_error)?,
            )?;
        }
        Ok(())
    })();
    let cleanup = xy
        .set_power_on_output(false)
        .map_err(driver_error)
        .and_then(|()| xy.read_power_on_output().map_err(driver_error))
        .and_then(|actual| expect_eq("S_INI cleanup", false, actual));
    finish_with_cleanup(result, cleanup, "power-on-output cleanup")
}

fn test_reg_mode(xy: &mut T) -> Result<(), String> {
    let m = xy.read_reg_mode().map_err(driver_error)?;
    info!("  CVCC={m:?}");
    Ok(())
}

fn test_internal_temperature(xy: &mut T) -> Result<(), String> {
    let t = xy.read_temperature_internal().map_err(driver_error)?;
    expect_eq(
        "T_INT unit",
        xy.read_temp_unit().map_err(driver_error)?,
        t.unit,
    )?;
    info!("  T_INT={:.1} {:?}", t.value, t.unit);
    Ok(())
}

fn test_temp_unit(xy: &mut T) -> Result<(), String> {
    for u in [TempUnit::Celsius, TempUnit::Fahrenheit] {
        xy.set_temp_unit(u).map_err(driver_error)?;
        expect_eq("F-C", u, xy.read_temp_unit().map_err(driver_error)?)?;
    }
    Ok(())
}

fn test_temp_offsets_read_only(xy: &mut T) -> Result<(), String> {
    // Driver no longer exposes setters (firmware silently ignores them);
    // confirm the read path works for both offset registers. The raw
    // probe below covers the firmware no-op behavior at the wire level.
    let int_off = xy.read_temp_offset_internal().map_err(driver_error)?;
    let ext_off = xy.read_temp_offset_external().map_err(driver_error)?;
    let mut raw = [0u16; 2];
    xy.read_raw_holding(0x001A, &mut raw)
        .map_err(driver_error)?;
    expect_precise(
        "T_INT_OFFSET raw decode",
        raw[0] as i16 as f32 / 10.0,
        int_off,
    )?;
    expect_precise(
        "T_EXT_OFFSET raw decode",
        raw[1] as i16 as f32 / 10.0,
        ext_off,
    )?;
    info!("  T_INT_OFFSET={int_off:.1} T_EXT_OFFSET={ext_off:.1}");
    Ok(())
}

fn test_lock(xy: &mut T) -> Result<(), String> {
    for v in [true, false] {
        xy.set_lock(v).map_err(driver_error)?;
        expect_eq("LOCK", v, xy.read_lock().map_err(driver_error)?)?;
    }
    Ok(())
}

fn test_backlight_full_range(xy: &mut T) -> Result<(), String> {
    // Firmware clamps 0 → 1, so the accepted range is 1..=5.
    for level in BACKLIGHT_SAMPLES {
        xy.set_backlight(level).map_err(driver_error)?;
        expect_eq(
            &format!("BL={level}"),
            level,
            xy.read_backlight().map_err(driver_error)?,
        )?;
    }
    xy.write_raw_holding(0x0014, 0).map_err(driver_error)?;
    let after_zero = xy.read_backlight().map_err(driver_error)?;
    if after_zero != 1 {
        return Err(format!(
            "BL=0 expected firmware-clamp to 1, got {after_zero}"
        ));
    }
    Ok(())
}

fn test_sleep_minutes_sweep(xy: &mut T) -> Result<(), String> {
    for &m in SLEEP_SAMPLES {
        xy.set_sleep_minutes(m).map_err(driver_error)?;
        expect_eq(
            &format!("SLEEP={m}"),
            m,
            xy.read_sleep_minutes().map_err(driver_error)?,
        )?;
    }
    xy.write_raw_holding(0x0015, 60).map_err(driver_error)?;
    let clamped = xy.read_sleep_minutes().map_err(driver_error)?;
    if clamped != 9 {
        return Err(format!(
            "SLEEP=60 expected firmware-clamp to 9, got {clamped}"
        ));
    }
    info!(
        "  swept {} sleep values + verified 9-min ceiling",
        SLEEP_SAMPLES.len()
    );
    Ok(())
}

fn test_buzzer(xy: &mut T) -> Result<(), String> {
    for v in [true, false] {
        xy.set_buzzer(v).map_err(driver_error)?;
        expect_eq("BUZZER", v, xy.read_buzzer().map_err(driver_error)?)?;
    }
    Ok(())
}

fn test_comms_settings(xy: &mut T) -> Result<(), String> {
    // Read-only — writing slave address or baud rate would orphan the
    // device on the next cold boot.
    let slave = xy.read_slave_address().map_err(driver_error)?;
    let baud = xy.read_baud_rate().map_err(driver_error)?;
    info!("  SLAVE=0x{slave:02X} BAUD={baud:?}");
    if slave != 1 {
        return Err(format!(
            "SLAVE mismatch: device 0x{slave:02X}, transport assumes 0x01"
        ));
    }
    if baud.baud() != BAUD {
        return Err(format!(
            "BAUD mismatch: register reports {} but UART is configured for {BAUD}",
            baud.baud()
        ));
    }
    Ok(())
}

/// Exercises `set_slave_address` and `set_baud_rate` codepaths safely
/// by writing the current value back. Both registers only take effect
/// after device reset, so a same-value write is fully idempotent and
/// can't orphan the bus mid-test.
fn test_comms_setters_idempotent(xy: &mut T) -> Result<(), String> {
    let slave = xy.read_slave_address().map_err(driver_error)?;
    xy.set_slave_address(slave).map_err(driver_error)?;
    expect_eq(
        "SLAVE same-value round-trip",
        slave,
        xy.read_slave_address().map_err(driver_error)?,
    )?;

    let baud = xy.read_baud_rate().map_err(driver_error)?;
    xy.set_baud_rate(baud).map_err(driver_error)?;
    expect_eq(
        "BAUD same-value round-trip",
        baud,
        xy.read_baud_rate().map_err(driver_error)?,
    )?;
    Ok(())
}

/// Drains `xy` via `into_transport`, rebuilds via `with_slave` on the
/// same UART transport, and confirms the rebuilt instance can still
/// talk to the device. Consumes `xy` — must run last.
fn test_lifecycle(xy: T<'static>) -> Result<(), String> {
    let transport = xy.into_transport();
    let mut rebuilt: T<'static> = Xy::with_slave(transport, 1).map_err(driver_error)?;

    // Final round-trip on the rebuilt instance.
    let check = rebuilt.verify_scale_family().map_err(driver_error)?;
    if !matches!(check, ScaleCheck::Compatible { .. }) {
        return Err(format!("rebuilt scale-family check: {check:?}"));
    }
    Ok(())
}

/// Verifies the empirically established scale-1 S-OTP behavior through
/// single-register writes, which bypass group-write conversion and clamps.
fn test_s_otp_raw_probe(xy: &mut T) -> Result<(), String> {
    const REG_S_OTP_M0: u16 = 0x005C;

    let original_unit = xy.read_temp_unit().map_err(driver_error)?;
    info!("  device unit at probe start: {original_unit:?}");

    let mut original = [0u16; 1];
    xy.read_raw_holding(REG_S_OTP_M0, &mut original)
        .map_err(driver_error)?;
    info!(
        "  S-OTP M0 raw original = {} (in {original_unit:?})",
        original[0]
    );

    let probes: &[u16] = &[10, 50, 80, 95, 100, 110, 150, 200, 230, 500, 950, 1100];

    let result = (|| {
        for unit in [TempUnit::Celsius, TempUnit::Fahrenheit] {
            xy.set_temp_unit(unit).map_err(driver_error)?;
            info!("  --- probing in {unit:?} ---");
            for &raw in probes {
                xy.write_raw_holding(REG_S_OTP_M0, raw)
                    .map_err(driver_error)?;
                let mut got = [0u16; 1];
                xy.read_raw_holding(REG_S_OTP_M0, &mut got)
                    .map_err(driver_error)?;
                expect_eq(&format!("S-OTP {unit:?} raw {raw}"), raw, got[0])?;
                info!("    S-OTP write raw {raw:>4} -> read raw {}", got[0]);
            }
        }
        Ok(())
    })();

    let cleanup = (|| {
        xy.set_temp_unit(original_unit).map_err(driver_error)?;
        xy.write_raw_holding(REG_S_OTP_M0, original[0])
            .map_err(driver_error)?;
        let mut verify = [0u16; 1];
        xy.read_raw_holding(REG_S_OTP_M0, &mut verify)
            .map_err(driver_error)?;
        expect_eq("S-OTP restore", original[0], verify[0])
    })();
    finish_with_cleanup(result, cleanup, "S-OTP cleanup")
}

/// Verifies that XY7025 firmware ignores Modbus writes to T-IN-OFFSET.
fn test_temp_offset_raw_probe(xy: &mut T) -> Result<(), String> {
    const REG_T_IN_OFFSET: u16 = 0x001A;
    let mut original = [0u16; 1];
    xy.read_raw_holding(REG_T_IN_OFFSET, &mut original)
        .map_err(driver_error)?;
    info!("  T-IN-OFFSET raw original = {}", original[0]);

    let probes: &[u16] = &[0, 1, 2, 5, 10, 50, 100];
    let result = (|| {
        for &raw in probes {
            xy.write_raw_holding(REG_T_IN_OFFSET, raw)
                .map_err(driver_error)?;
            let mut got = [0u16; 1];
            xy.read_raw_holding(REG_T_IN_OFFSET, &mut got)
                .map_err(driver_error)?;
            expect_eq(
                &format!("T-IN-OFFSET ignored raw {raw}"),
                original[0],
                got[0],
            )?;
            info!("    T-IN-OFFSET write raw {raw:>3} -> read raw {}", got[0]);
        }
        Ok(())
    })();

    let cleanup = (|| {
        xy.write_raw_holding(REG_T_IN_OFFSET, original[0])
            .map_err(driver_error)?;
        let mut verify = [0u16; 1];
        xy.read_raw_holding(REG_T_IN_OFFSET, &mut verify)
            .map_err(driver_error)?;
        expect_eq("T-IN-OFFSET restore", original[0], verify[0])
    })();
    finish_with_cleanup(result, cleanup, "T-IN-OFFSET cleanup")
}

/// Verifies the empirically established nine-minute SLEEP ceiling.
fn test_sleep_raw_probe(xy: &mut T) -> Result<(), String> {
    const REG_SLEEP: u16 = 0x0015;
    let mut original = [0u16; 1];
    xy.read_raw_holding(REG_SLEEP, &mut original)
        .map_err(driver_error)?;
    info!("  SLEEP raw original = {}", original[0]);

    let probes: &[u16] = &[0, 1, 5, 8, 9, 10, 11, 15, 30, 60, 100, 999];
    let result = (|| {
        for &raw in probes {
            xy.write_raw_holding(REG_SLEEP, raw).map_err(driver_error)?;
            thread::sleep(Duration::from_millis(50));
            let mut got = [0u16; 1];
            xy.read_raw_holding(REG_SLEEP, &mut got)
                .map_err(driver_error)?;
            expect_eq(&format!("SLEEP raw {raw}"), raw.min(9), got[0])?;
            info!("    SLEEP write raw {raw:>3} -> read raw {}", got[0]);
        }
        Ok(())
    })();

    let cleanup = (|| {
        xy.write_raw_holding(REG_SLEEP, original[0])
            .map_err(driver_error)?;
        let mut verify = [0u16; 1];
        xy.read_raw_holding(REG_SLEEP, &mut verify)
            .map_err(driver_error)?;
        expect_eq("SLEEP restore", original[0], verify[0])
    })();
    finish_with_cleanup(result, cleanup, "SLEEP cleanup")
}

fn test_group_full_round_trip_all(xy: &mut T) -> Result<(), String> {
    // Force Celsius so any temp-unit-dependent encoding doesn't
    // contaminate the s_otp readback comparison.
    xy.set_temp_unit(TempUnit::Celsius).map_err(driver_error)?;
    // Write a unique probe to every M0..=M9, read back, verify, restore.
    for n in 0u8..=9 {
        let mut original = [0; GROUP_REGISTER_COUNT];
        xy.read_raw_holding(group_address(n as usize), &mut original)
            .map_err(driver_error)?;
        let probe = GroupParams {
            setpoints: Setpoints {
                v_set: 5.0 + n as f32 * 0.5,
                i_set: 0.10 + n as f32 * 0.05,
            },
            safety_limits: SafetyLimits {
                lvp_v: 10.0 + n as f32,
                ovp_v: 20.0 + n as f32,
                ocp_a: 1.0 + n as f32 * 0.25,
            },
            s_opp_w: 50.0 + n as f32 * 5.0,
            s_ohp_h: n as u16,
            s_ohp_m: n as u16 * 5,
            s_oah_ah: 1.0 + n as f64,
            s_owh_wh: 10.0 + n as f64 * 2.0,
            s_otp: Temperature {
                value: 50.0 + n as f32,
                unit: TempUnit::Celsius,
            },
            // M0 remains boot-safe; non-active groups still cover both values.
            power_on_output: n != 0 && n % 2 == 0,
        };
        let result = (|| {
            let r = xy.write_group(n, &probe).map_err(driver_error)?;
            expect_precise(
                &format!("M{n} v_set"),
                probe.setpoints.v_set,
                r.setpoints.v_set,
            )?;
            expect_precise(
                &format!("M{n} i_set"),
                probe.setpoints.i_set,
                r.setpoints.i_set,
            )?;
            expect_precise(
                &format!("M{n} s_lvp_v"),
                probe.safety_limits.lvp_v,
                r.safety_limits.lvp_v,
            )?;
            expect_precise(
                &format!("M{n} s_ovp_v"),
                probe.safety_limits.ovp_v,
                r.safety_limits.ovp_v,
            )?;
            expect_precise(
                &format!("M{n} s_ocp_a"),
                probe.safety_limits.ocp_a,
                r.safety_limits.ocp_a,
            )?;
            expect_precise(&format!("M{n} s_opp_w"), probe.s_opp_w, r.s_opp_w)?;
            expect_eq(&format!("M{n} s_ohp_h"), probe.s_ohp_h, r.s_ohp_h)?;
            expect_eq(&format!("M{n} s_ohp_m"), probe.s_ohp_m, r.s_ohp_m)?;
            expect_approx64(&format!("M{n} s_oah_ah"), probe.s_oah_ah, r.s_oah_ah)?;
            expect_approx64(&format!("M{n} s_owh_wh"), probe.s_owh_wh, r.s_owh_wh)?;
            // Group writes route through firmware unit conversion, which
            // introduces ±1° rounding. Single-register writes round-trip
            // exactly (see s_otp_raw_probe).
            expect_eq(&format!("M{n} s_otp unit"), probe.s_otp.unit, r.s_otp.unit)?;
            if (probe.s_otp.value - r.s_otp.value).abs() > 1.0 {
                return Err(format!(
                    "M{n} s_otp: expected {:.1} ±1, got {:.1}",
                    probe.s_otp.value, r.s_otp.value
                ));
            }
            expect_eq(
                &format!("M{n} power_on_output"),
                probe.power_on_output,
                r.power_on_output,
            )
        })();
        let cleanup = restore_group_exact(xy, n as usize, &original);
        finish_with_cleanup(result, cleanup, &format!("M{n} cleanup"))?;
    }
    info!("  exercised all 10 memory groups");
    Ok(())
}

fn test_recall_each_group(xy: &mut T) -> Result<(), String> {
    // Recall overwrites the live M0 set, so keep the output off and lower
    // V_SET before comparing the recalled raw words.
    xy.set_output(false).map_err(driver_error)?;
    for n in 0u8..=9 {
        xy.set_voltage(0.0).map_err(driver_error)?;
        let mut expected = [0; GROUP_REGISTER_COUNT];
        xy.read_raw_holding(group_address(n as usize), &mut expected)
            .map_err(driver_error)?;
        let result = (|| {
            xy.recall_group(n).map_err(driver_error)?;
            let mut actual = [0; GROUP_REGISTER_COUNT];
            xy.read_raw_holding(GROUP_BASE, &mut actual)
                .map_err(driver_error)?;
            expect_eq(&format!("recall M{n} into M0"), expected, actual)
        })();
        let cleanup = xy
            .set_power_on_output(false)
            .map_err(driver_error)
            .and_then(|()| xy.read_power_on_output().map_err(driver_error))
            .and_then(|actual| expect_eq("recalled S_INI cleanup", false, actual));
        finish_with_cleanup(result, cleanup, &format!("recall M{n} cleanup"))?;
    }
    info!("  recalled all 10 memory groups");
    Ok(())
}
