use crate::{
    advanced::{self, Mode, Settings},
    bridge::{Bridge, Drive},
    config,
    control::Command,
    cubemars::{self, Feedback},
    robotiq,
};

#[test]
fn rated_speed_wire_range_and_native_modes_reach_the_motor_limit() {
    // Direct velocity is bounded by the rated speed or the signed i16 mrad/s wire field:
    // the AK40-10 370 RPM is beyond the field, so 32.767 rad/s (312.9 RPM).
    let profile = cubemars::AK40_10_PROFILE;
    let (limit, rpm_max) = (32767, 312.9);
    assert_eq!(profile.velocity_limit_mrad(), limit);
    {
        for signed in [-(limit as i16), limit as i16] {
            let settings = Settings {
                mode: Mode::Velocity,
                velocity_mrad: signed,
                ..Settings::default()
            };
            assert!(settings.valid());
            assert_eq!(Settings::decode(settings.words()), Some(settings));
            let (mut b, mut c) = armed(settings);
            c.action = 9;
            c.speed = 1000;
            let Drive::Can(p) = b
                .step_with_settings(2, c, Some(2), Some(feedback(2)), settings)
                .drive
            else {
                panic!()
            };
            assert_eq!(p.id, 0x368);
            let erpm = i32::from_be_bytes(p.data[..4].try_into().unwrap());
            let rpm = erpm as f32 / profile.electrical_ratio;
            // Integer ERPM: within one electrical RPM of the limit.
            assert!(
                (rpm.abs() - rpm_max).abs() <= 1.0 / profile.electrical_ratio + 0.001,
                "{rpm}"
            );
            assert!(rpm.abs() <= profile.rated_speed_rpm);
            assert_eq!(rpm.is_sign_negative(), signed < 0);
            if limit < 32767 {
                assert!(
                    !Settings {
                        velocity_mrad: if signed < 0 { signed - 1 } else { signed + 1 },
                        ..settings
                    }
                    .valid()
                );
            }
        }
    }
    let settings = Settings {
        mode: Mode::Profile,
        position_mdeg: 180000,
        ..Settings::default()
    };
    let (mut b, mut c) = armed(settings);
    c.action = 9;
    let Drive::Can(p) = b
        .step_with_settings(2, c, Some(2), Some(feedback(2)), settings)
        .drive
    else {
        panic!()
    };
    assert_eq!(p.id, 0x668);
    // Speed 1000 is the AK40-10 rated 370 RPM: 370 * 140 / 10 ERPM units.
    let speed = i16::from_be_bytes([p.data[4], p.data[5]]);
    assert!((speed - 5180).abs() <= 1, "{speed}");
}

fn feedback(now: u64) -> Feedback {
    Feedback {
        position_rad: 0.0,
        velocity_rad_s: 0.0,
        iq_a: 0.0,
        temperature_c: -5,
        error: 0,
        received_ms: now,
    }
}
fn armed(settings: Settings) -> (Bridge, Command) {
    let mut b = Bridge::new(config::MOTION);
    let c = Command {
        action: 1,
        position: 500,
        speed: 1000,
        force: 1000,
    };
    b.step_with_settings(0, Command::default(), Some(0), None, settings);
    b.step_with_settings(1, c, Some(1), Some(feedback(1)), settings);
    (b, c)
}
fn word(bytes: &[u8], i: usize) -> u16 {
    u16::from_be_bytes([bytes[i * 2], bytes[i * 2 + 1]])
}
fn write(settings: Settings) -> std::vec::Vec<u8> {
    let mut bytes = std::vec![9, 0x10, 0x12, 0, 0, 16, 32];
    bytes.extend(settings.words().iter().flat_map(|w| w.to_be_bytes()));
    bytes.extend(robotiq::crc16(&bytes).to_le_bytes());
    bytes
}
#[test]
fn settings_are_atomic_and_mode_cannot_change_during_a_run() {
    let mut settings = Settings::default();
    let before = settings;
    let next = Settings {
        mode: Mode::Cst,
        torque_mnm: -2500,
        ..settings
    };
    let request = write(next);
    let mut tx = [0; robotiq::MAX_ADU];
    for length in 0..request.len() {
        assert!(
            advanced::process(
                9,
                &request[..length],
                &mut tx,
                &mut settings,
                Command::default(),
                &[0; 32]
            )
            .is_none()
        );
        assert_eq!(settings, before);
    }
    assert!(
        advanced::process(
            9,
            &request,
            &mut tx,
            &mut settings,
            Command {
                action: 9,
                ..Command::default()
            },
            &[0; 32]
        )
        .is_none()
    );
    assert_eq!(settings, before);
    let mut invalid = next;
    invalid.torque_mnm = 9001;
    assert!(
        advanced::process(
            9,
            &write(invalid),
            &mut tx,
            &mut settings,
            Command::default(),
            &[0; 32]
        )
        .is_none()
    );
    assert_eq!(settings, before);
    let r = advanced::process(
        9,
        &request,
        &mut tx,
        &mut settings,
        Command::default(),
        &[0; 32],
    )
    .unwrap();
    assert!(r.written);
    assert_eq!(r.len, 8);
    assert_eq!(settings, next);
    assert_eq!(robotiq::crc16(&tx[..6]), u16::from_le_bytes([tx[6], tx[7]]));
}
#[test]
fn runtime_gain_change_preserves_mode_and_motion_target() {
    let initial = Settings {
        kp_centi: 100,
        kd_milli: 0,
        ..Settings::default()
    };
    let (mut b, mut c) = armed(initial);
    c.action = 9;
    c.position = 1000;
    let first = b.step_with_settings(2, c, Some(2), Some(feedback(2)), initial);
    let Drive::Torque(first) = first.drive else {
        panic!()
    };
    let changed = Settings {
        kp_centi: 200,
        ..initial
    };
    let second = b.step_with_settings(3, c, Some(3), Some(feedback(3)), changed);
    let Drive::Torque(second_torque) = second.drive else {
        panic!()
    };
    assert!(second_torque > first * 3.9 && second_torque < first * 4.1);
    assert_eq!(word(&second.advanced_status, 3), 200);
    assert_eq!(word(&second.status1000, 4), 1000);
}
#[test]
fn cst_sends_signed_current_and_honors_torque_zero_stop_and_watchdog() {
    for requested in [-9000, -2500, 0, 2500, 9000] {
        let settings = Settings {
            mode: Mode::Cst,
            torque_mnm: requested,
            ..Settings::default()
        };
        let (mut b, mut c) = armed(settings);
        c.action = 9;
        c.force = 500;
        c.speed = 0;
        let out = b.step_with_settings(2, c, Some(2), Some(feedback(2)), settings);
        let Drive::Can(p) = out.drive else { panic!() };
        assert_eq!(p.id, 0x168);
        assert_eq!(p.len, 4);
        let iq = i32::from_be_bytes(p.data[..4].try_into().unwrap());
        let t = iq as f32 / 1000.0 * cubemars::AK40_10_PROFILE.kt;
        // Force 500 asks 4.5 N*m; the AK40-10 peak is 4.1.
        assert!(t.abs() <= 4.1 + 1e-3);
        if requested != 0 {
            assert_eq!(t.is_sign_negative(), requested < 0);
        }
        c.force = 0;
        assert_eq!(
            b.step_with_settings(3, c, Some(3), Some(feedback(3)), settings)
                .drive,
            Drive::Torque(0.0)
        );
        c.action = 1;
        c.force = 500;
        assert_eq!(
            b.step_with_settings(4, c, Some(4), Some(feedback(4)), settings)
                .drive,
            Drive::Torque(0.0)
        );
        c.action = 9;
        let stale = b.step_with_settings(105, c, Some(105), Some(feedback(4)), settings);
        assert_eq!(stale.drive, Drive::Disabled);
        assert_eq!(word(&stale.status1000, 12), 3);
    }
}
#[test]
fn signed_telemetry_and_no_feedback_temperature_are_unambiguous() {
    let settings = Settings::default();
    let (mut b, c) = armed(settings);
    let mut f = feedback(2);
    f.iq_a = -10.0;
    f.velocity_rad_s = -1.234;
    let out = b.step_with_settings(2, c, Some(2), Some(f), settings);
    assert_eq!(word(&out.advanced_status, 7) as i16, -5);
    assert_eq!(
        i32::from_be_bytes(out.advanced_status[16..20].try_into().unwrap()),
        -10000
    );
    assert_eq!(
        i32::from_be_bytes(out.advanced_status[20..24].try_into().unwrap()),
        -4814
    );
    assert_eq!(
        i32::from_be_bytes(out.advanced_status[24..28].try_into().unwrap()),
        -1234
    );
    let missing = Bridge::new(config::MOTION).step_with_settings(
        0,
        Command::default(),
        Some(0),
        None,
        settings,
    );
    assert_eq!(word(&missing.advanced_status, 7), 0x8000);
}
#[test]
fn native_mode_packets_and_boundary_recovery_have_correct_units() {
    let v = cubemars::velocity_command(cubemars::AK40_10, 104, 1.0);
    assert_eq!(v.id, 0x368);
    // 1 rad/s = 9.55 output RPM x 140 (14 pole pairs x 10:1).
    assert_eq!(i32::from_be_bytes(v.data[..4].try_into().unwrap()), 1336);
    let profile =
        cubemars::profile_command(cubemars::AK40_10, 104, core::f32::consts::PI, 2.0, 1.0);
    assert_eq!(profile.id, 0x668);
    assert_eq!(
        i32::from_be_bytes(profile.data[..4].try_into().unwrap()),
        1800000
    );
    assert_eq!(i16::from_be_bytes([profile.data[4], profile.data[5]]), 267);
    assert_eq!(i16::from_be_bytes([profile.data[6], profile.data[7]]), 133);
    for torque in [-2000, 2000] {
        let settings = Settings {
            mode: Mode::Cst,
            torque_mnm: torque,
            ..Settings::default()
        };
        let (mut b, mut c) = armed(settings);
        c.action = 9;
        let mut f = feedback(2);
        // Beyond the +8 turn endpoint: only inward (negative) torque is allowed.
        f.position_rad = 51.0;
        let Drive::Can(p) = b.step_with_settings(2, c, Some(2), Some(f), settings).drive else {
            panic!()
        };
        let iq = i32::from_be_bytes(p.data[..4].try_into().unwrap());
        if torque > 0 {
            assert_eq!(iq, 0);
        } else {
            assert!(iq < 0);
        }
    }
}
#[test]
fn measured_overlimit_and_a_mode_change_during_a_run_latch_until_off() {
    let settings = Settings {
        mode: Mode::Cst,
        ..Settings::default()
    };
    let (mut b, mut c) = armed(settings);
    c.action = 9;
    c.force = 100;
    let mut f = feedback(2);
    // 3 A is 1.44 N*m on the AK40-10, over the 0.9 N*m limit plus its 0.1 margin.
    f.iq_a = 3.0;
    let out = b.step_with_settings(2, c, Some(2), Some(f), settings);
    assert_eq!(out.drive, Drive::Disabled);
    assert_eq!(word(&out.status1000, 12), 11);
    let reset = b.step_with_settings(3, Command::default(), Some(3), Some(feedback(3)), settings);
    assert_eq!(word(&reset.status1000, 3), 0);
    let (mut b, mut c) = armed(settings);
    c.action = 9;
    b.step_with_settings(2, c, Some(2), Some(feedback(2)), settings);
    let changed = Settings {
        mode: Mode::Velocity,
        ..settings
    };
    let out = b.step_with_settings(3, c, Some(3), Some(feedback(3)), changed);
    assert_eq!(out.drive, Drive::Disabled);
    assert_eq!(word(&out.status1000, 12), 12);
}

#[test]
fn a_stopped_motor_takes_any_mode_and_runs_it_on_the_next_go() {
    let position = Settings {
        mode: Mode::Position,
        ..Settings::default()
    };
    // Active and holding (rACT, no rGTO): the board accepts a settings write in any mode.
    let mut tx = [0; robotiq::MAX_ADU];
    let stopped = Command {
        action: 1,
        ..Command::default()
    };
    for mode in [
        Mode::Velocity,
        Mode::Cst,
        Mode::Brake,
        Mode::Profile,
        Mode::Hybrid,
    ] {
        let mut current = position;
        let next = Settings {
            mode,
            velocity_mrad: 1000,
            torque_mnm: 500,
            ..position
        };
        let reply = advanced::process(9, &write(next), &mut tx, &mut current, stopped, &[0; 32]);
        assert!(reply.is_some_and(|r| r.written), "{mode:?}");
        assert_eq!(current, next);
    }
    // The bridge switches while holding, then runs the new mode on the go edge.
    let (mut b, mut c) = armed(position);
    let held = b.step_with_settings(2, c, Some(2), Some(feedback(2)), position);
    assert_eq!(word(&held.status1000, 3), 0);
    let velocity = Settings {
        mode: Mode::Velocity,
        velocity_mrad: 1000,
        ..position
    };
    let switched = b.step_with_settings(3, c, Some(3), Some(feedback(3)), velocity);
    assert_eq!(word(&switched.status1000, 3), 0, "no fault while stopped");
    assert_eq!(word(&switched.advanced_status, 2), Mode::Velocity as u16);
    c.action = 9;
    let run = b.step_with_settings(4, c, Some(4), Some(feedback(4)), velocity);
    let Drive::Can(p) = run.drive else {
        panic!("{:?}", run.drive)
    };
    assert_eq!(p.id, 0x368, "velocity command");
    assert_eq!(word(&run.status1000, 3), 0);
}

#[test]
fn a_run_sent_straight_from_off_activates_then_starts_as_a_go_edge() {
    // No separate activation: rACT + rGTO together. CSP+CST still starts its CSP phase.
    let hybrid = Settings {
        mode: Mode::Hybrid,
        threshold_mnm: 300,
        torque_mnm: 250,
        ..Settings::default()
    };
    let mut b = Bridge::new(config::MOTION);
    let run = Command {
        action: 9,
        position: 600,
        speed: 500,
        force: 500,
    };
    b.step_with_settings(0, Command::default(), Some(0), None, hybrid);
    let activating = b.step_with_settings(1, run, Some(1), None, hybrid);
    assert_eq!(activating.drive, Drive::Torque(0.0));
    let activated = b.step_with_settings(2, run, Some(2), Some(feedback(2)), hybrid);
    assert_eq!(activated.drive, Drive::Torque(0.0));
    assert_eq!(word(&activated.status1000, 3), 0);
    let moving = b.step_with_settings(3, run, Some(3), Some(feedback(3)), hybrid);
    assert!(
        matches!(moving.drive, Drive::Torque(t) if t > 0.0),
        "{:?}",
        moving.drive
    );
    // Advanced status word 14 bits 1-2: CSP phase.
    assert_eq!((word(&moving.advanced_status, 14) >> 1) & 3, 1);
}
