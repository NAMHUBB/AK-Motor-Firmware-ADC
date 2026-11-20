//! AK60-39 V3.0 KV80 (settings motor model 3) next to the AK40-10 boot model.
use crate::{
    advanced::{self, Mode, Settings},
    bridge::{Bridge, Drive, Output},
    config,
    control::Command,
    cubemars::{self, AK40_10, AK60_39, Feedback},
    robotiq,
};

fn fb(now: u64, position: f32, iq: f32) -> Feedback {
    Feedback {
        position_rad: position,
        velocity_rad_s: 0.0,
        iq_a: iq,
        temperature_c: 25,
        error: 0,
        received_ms: now,
    }
}
fn word(bytes: &[u8], i: usize) -> u16 {
    u16::from_be_bytes([bytes[i * 2], bytes[i * 2 + 1]])
}
fn fault(out: &Output) -> (u16, u16) {
    (word(&out.status1000, 3), word(&out.status1000, 12))
}
fn write(settings: Settings) -> std::vec::Vec<u8> {
    let mut bytes = std::vec![2, 0x10, 0x12, 0, 0, 16, 32];
    bytes.extend(settings.words().iter().flat_map(|w| w.to_be_bytes()));
    bytes.extend(robotiq::crc16(&bytes).to_le_bytes());
    bytes
}
fn ak60(mode: Mode) -> Settings {
    Settings {
        mode,
        motor_model: AK60_39,
        ..Settings::default()
    }
}
/// Motor ID 2, enabled (rACT) with the given settings from OFF.
fn enabled(s: Settings, force: u16) -> (Bridge, Command) {
    let mut b = Bridge::for_motor(config::MOTION, 2);
    let c = Command {
        action: 1,
        position: 1000,
        speed: 1000,
        force,
    };
    b.step_with_settings(0, Command::default(), Some(0), None, s);
    b.step_with_settings(1, c, Some(1), Some(fb(1, 0.0, 0.0)), s);
    (b, c)
}

#[test]
fn ak60_39_settings_round_trip_with_its_70_rpm_velocity_limit() {
    let s = Settings {
        velocity_mrad: 7331,
        ..ak60(Mode::Velocity)
    };
    assert!(s.valid());
    assert_eq!(s.words()[12], AK60_39);
    assert_eq!(Settings::decode(s.words()), Some(s));
    // 7332 mrad/s is past 70 RPM for the AK60-39 but well inside the AK40-10 370 RPM.
    let fast = Settings {
        velocity_mrad: 7332,
        ..s
    };
    assert!(!fast.valid());
    assert!(
        Settings {
            motor_model: AK40_10,
            ..fast
        }
        .valid()
    );
    // Torque settings keep the 9 N*m bound for both motors.
    assert!(
        Settings {
            torque_mnm: 9000,
            ..s
        }
        .valid()
    );
    assert!(
        !Settings {
            torque_mnm: 9001,
            ..s
        }
        .valid()
    );
}

#[test]
fn the_motor_model_changes_only_while_the_motor_is_off() {
    let boot = Settings::default();
    assert_eq!(boot.motor_model, AK40_10);
    let next = ak60(Mode::Position);
    let mut tx = [0; robotiq::MAX_ADU];
    for action in [1, 9] {
        let mut current = boot;
        let on = Command {
            action,
            ..Command::default()
        };
        assert!(advanced::process(2, &write(next), &mut tx, &mut current, on, &[0; 32]).is_none());
        assert_eq!(current, boot, "action {action}");
    }
    let mut current = boot;
    let reply = advanced::process(
        2,
        &write(next),
        &mut tx,
        &mut current,
        Command::default(),
        &[0; 32],
    );
    assert!(reply.is_some_and(|r| r.written));
    assert_eq!(current, next);
    // Same model, new mode: a held (enabled, stopped) motor still takes it.
    let velocity = ak60(Mode::Velocity);
    let holding = Command {
        action: 1,
        ..Command::default()
    };
    let reply = advanced::process(
        2,
        &write(velocity),
        &mut tx,
        &mut current,
        holding,
        &[0; 32],
    );
    assert!(reply.is_some_and(|r| r.written));
    assert_eq!(current, velocity);
}

#[test]
fn a_model_change_reaching_an_enabled_bridge_trips_and_applies_after_off() {
    let (mut b, c) = enabled(Settings::default(), 1000);
    assert_eq!(b.motor_model(), AK40_10);
    let next = ak60(Mode::Position);
    let out = b.step_with_settings(2, c, Some(2), Some(fb(2, 0.0, 0.0)), next);
    assert_eq!(fault(&out), (0x0d, 12));
    assert_eq!(out.drive, Drive::Disabled);
    assert_eq!(b.motor_model(), AK40_10, "the old model stays while on");
    let off = b.step_with_settings(3, Command::default(), Some(3), Some(fb(3, 0.0, 0.0)), next);
    assert_eq!(b.motor_model(), AK60_39);
    assert_eq!(off.drive, Drive::Disabled);
    b.step_with_settings(4, c, Some(4), Some(fb(4, 0.0, 0.0)), next);
    let out = b.step_with_settings(5, c, Some(5), Some(fb(5, 0.0, 0.0)), next);
    assert_eq!(fault(&out).0, 0);
    assert_eq!((word(&out.advanced_status, 14) >> 8) & 3, AK60_39);
}

#[test]
fn ak60_39_position_mode_reaches_9_nm_and_ramps_at_70_rpm() {
    let s = Settings {
        kp_centi: 10_000,
        kd_milli: 0,
        ..ak60(Mode::Position)
    };
    let (mut b, mut c) = enabled(s, 1000);
    c.action = 9;
    let out = b.step_with_settings(2, c, Some(2), Some(fb(2, 0.0, 0.0)), s);
    let Drive::Torque(t) = out.drive else {
        panic!("{:?}", out.drive)
    };
    // Speed 1000 = 70 RPM: one 1 ms step of the ramp at kp 100.
    let step = 70.0 * core::f32::consts::TAU / 60.0 * 0.001;
    assert!((t - 100.0 * step).abs() < 0.001, "{t}");
    // Far from the target the whole 9 N*m force scale is available (the AK40-10 stops at
    // 4.1 N*m), and it is a 1.69 A current command on the AK60-39.
    let out = b.step_with_settings(3, c, Some(3), Some(fb(3, -20.0, 0.0)), s);
    assert_eq!(out.drive, Drive::Torque(9.0));
    let wire = cubemars::torque_command(b.motor_model(), 2, 9.0).unwrap();
    assert_eq!(wire.id, 0x102);
    assert_eq!(i32::from_be_bytes(wire.data[..4].try_into().unwrap()), 1687);
    // Status: model 3 in advanced word 14, torque estimate with Kt 5.33 (1 A = 5.333 N*m).
    let out = b.step_with_settings(4, c, Some(4), Some(fb(4, -20.0, 1.0)), s);
    assert_eq!((word(&out.advanced_status, 14) >> 8) & 3, AK60_39);
    let torque = (u32::from(word(&out.advanced_status, 10)) << 16)
        | u32::from(word(&out.advanced_status, 11));
    assert!((torque as i32 - 5333).abs() <= 1, "{torque}");
}

#[test]
fn ak60_39_velocity_and_profile_packets_use_14_pole_pairs_and_39_to_1() {
    // Velocity 7.331 rad/s is clamped to speed 1000 = 70 RPM = 70 x 546 ERPM.
    let s = Settings {
        velocity_mrad: 7331,
        ..ak60(Mode::Velocity)
    };
    let (mut b, mut c) = enabled(s, 1000);
    c.action = 9;
    let out = b.step_with_settings(2, c, Some(2), Some(fb(2, 0.0, 0.0)), s);
    let Drive::Can(p) = out.drive else {
        panic!("{:?}", out.drive)
    };
    assert_eq!(p.id, 0x302);
    let erpm = i32::from_be_bytes(p.data[..4].try_into().unwrap());
    assert!((erpm - 38220).abs() <= 1, "{erpm}");
    // Profile mode: speed and acceleration fields are ERPM / 10.
    let s = Settings {
        acceleration_mrad: 1000,
        position_mdeg: 90_000,
        ..ak60(Mode::Profile)
    };
    let (mut b, mut c) = enabled(s, 1000);
    c.action = 9;
    let out = b.step_with_settings(2, c, Some(2), Some(fb(2, 0.0, 0.0)), s);
    let Drive::Can(p) = out.drive else {
        panic!("{:?}", out.drive)
    };
    assert_eq!(p.id, 0x602);
    assert_eq!(i32::from_be_bytes(p.data[..4].try_into().unwrap()), 900_000);
    let speed = i16::from_be_bytes([p.data[4], p.data[5]]);
    assert!((speed - 3822).abs() <= 1, "{speed}");
    // 1 rad/s^2 = 9.55 RPM/s x 546 / 10.
    assert_eq!(i16::from_be_bytes([p.data[6], p.data[7]]), 521);
}

#[test]
fn ak60_39_over_limit_current_trips_with_its_own_torque_constant() {
    // Force 100 = 0.9 N*m. 0.2 A is 1.07 N*m on the AK60-39 (over 0.9 + 0.1) but only
    // 0.096 N*m with the AK40-10 constant.
    for (model, tripped) in [(AK60_39, true), (AK40_10, false)] {
        let s = Settings {
            torque_mnm: 500,
            motor_model: model,
            ..Settings::default()
        };
        let s = Settings {
            mode: Mode::Cst,
            ..s
        };
        let (mut b, mut c) = enabled(s, 100);
        c.action = 9;
        let out = b.step_with_settings(2, c, Some(2), Some(fb(2, 0.0, 0.2)), s);
        assert_eq!(fault(&out).0 == 0x0e, tripped, "model {model}");
    }
}

#[test]
fn ak60_39_feedback_is_parsed_at_its_scale_on_its_own_id() {
    // 5460 ERPM = 10 output RPM on the AK60-39 (39 output RPM on the AK40-10).
    let data = [0x00, 0x00, 0x02, 0x22, 0x00, 0x64, 30, 0];
    let v = |model| {
        cubemars::parse_feedback(model, 2, 0x2902, &data, 0)
            .unwrap()
            .velocity_rad_s
    };
    let rpm = |rad_s: f32| rad_s * 60.0 / core::f32::consts::TAU;
    assert!((rpm(v(AK60_39)) - 10.0).abs() < 1e-3);
    assert!((rpm(v(AK40_10)) - 39.0).abs() < 1e-3);
    assert!(cubemars::parse_feedback(AK60_39, 1, 0x2902, &data, 0).is_none());
}
