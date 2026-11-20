use crate::{
    advanced::{self, Mode, Settings},
    bridge::{Bridge, Drive, Output},
    config,
    control::Command,
    cubemars::{self, Feedback},
    robotiq,
};

fn settings() -> Settings {
    Settings {
        mode: Mode::Hybrid,
        torque_mnm: -400,
        threshold_mnm: 600,
        dwell_ms: 60,
        kp_centi: 100,
        kd_milli: 0,
        ..Settings::default()
    }
}
fn feedback(now: u64, torque: f32, position: f32) -> Feedback {
    Feedback {
        position_rad: position,
        velocity_rad_s: 0.0,
        iq_a: torque / cubemars::AK40_10_PROFILE.kt,
        temperature_c: 30,
        error: 0,
        received_ms: now,
    }
}
fn phase(out: &Output) -> u16 {
    (u16::from_be_bytes([out.advanced_status[28], out.advanced_status[29]]) >> 1) & 3
}
fn armed(s: Settings) -> (Bridge, Command) {
    let mut b = Bridge::new(config::MotionConfig {
        max_speed_rad_s: 2.0,
        ..config::MOTION
    });
    let mut c = Command {
        action: 0,
        position: 750,
        speed: 500,
        force: 250,
    };
    b.step_with_settings(0, c, Some(0), None, s);
    c.action = 1;
    b.step_with_settings(1, c, Some(1), Some(feedback(1, 0.0, 0.0)), s);
    c.action = 9;
    (b, c)
}
fn step(b: &mut Bridge, c: Command, s: Settings, now: u64, torque: f32) -> Output {
    b.step_with_settings(now, c, Some(now), Some(feedback(now, torque, 0.0)), s)
}
/// A held grip pushes with the CST torque through the CSP loop's torque command and
/// reports the held object (gOBJ 2 closing, 1 opening).
fn assert_current(out: Output, target: f32) {
    assert_eq!(phase(&out), 2);
    let Drive::Torque(t) = out.drive else {
        panic!("CST torque command required: {:?}", out.drive)
    };
    assert!((t - target).abs() < 0.001, "{t}");
    assert_eq!(out.status[0] >> 6, if target > 0.0 { 2 } else { 1 });
}

#[test]
fn threshold_dwell_uses_distinct_fresh_feedback_and_latches_cst() {
    let s = settings();
    let (mut b, c) = armed(s);
    let out = step(&mut b, c, s, 2, 0.7);
    assert_eq!(phase(&out), 1);
    assert!(matches!(out.drive, Drive::Torque(_)));
    // Replaying a still-fresh sample for 60 controller ticks cannot satisfy dwell.
    for now in 3..=62 {
        let out = b.step_with_settings(now, c, Some(now), Some(feedback(2, 0.7, 0.0)), s);
        assert_eq!(phase(&out), 1);
    }
    assert_current(step(&mut b, c, s, 63, 0.7), 0.4);
    // While the fingers stay short of the target the grip holds, whatever the torque reads.
    assert_current(step(&mut b, c, s, 64, 0.0), 0.4);
    assert_current(step(&mut b, c, s, 65, -0.1), 0.4);
}

#[test]
fn cst_keeps_pushing_toward_the_target_whatever_the_reported_current_sign() {
    // Neither the CST setting's sign nor the sign the motor reports for its current
    // decides the grip: CST keeps pushing the way CSP drove, toward the target.
    for (position, expected) in [(750, 0.6), (100, -0.6)] {
        for setting in [-600, 600] {
            for contact in [-0.7f32, 0.7] {
                let s = Settings {
                    torque_mnm: setting,
                    ..settings()
                };
                let (mut b, mut c) = armed(s);
                c.position = position;
                assert_eq!(phase(&step(&mut b, c, s, 2, contact)), 1);
                assert_current(step(&mut b, c, s, 62, contact), expected);
            }
        }
    }
}

#[test]
fn at_the_target_cst_never_joins_and_csp_holds_it() {
    // Nothing is gripped at the target: whatever the current reads, CSP alone holds it.
    let s = settings();
    let (mut b, c) = armed(s);
    let m = config::MOTION;
    let target = m.open_rad + (m.closed_rad - m.open_rad) * 750.0 / 1024.0;
    for now in (2..2000).step_by(10) {
        let torque = if now < 1000 { 0.7 } else { -0.7 };
        let out = b.step_with_settings(now, c, Some(now), Some(feedback(now, torque, target)), s);
        assert_eq!(phase(&out), 1);
        assert_eq!(out.status[0] >> 6, 3);
    }
}

#[test]
fn transient_overload_resets_dwell_and_below_threshold_never_switches() {
    let s = settings();
    let (mut b, c) = armed(s);
    for (now, torque) in [(2, 0.7), (22, 0.7), (42, 0.59), (62, 0.7), (102, 0.7)] {
        assert_eq!(phase(&step(&mut b, c, s, now, torque)), 1);
    }
    assert_current(step(&mut b, c, s, 122, 0.7), 0.4);
    let (mut b, c) = armed(s);
    for now in (2..500).step_by(20) {
        assert_eq!(phase(&step(&mut b, c, s, now, 0.59)), 1);
    }
}

fn moving(now: u64, torque: f32, velocity: f32) -> Feedback {
    Feedback {
        velocity_rad_s: velocity,
        ..feedback(now, torque, 0.0)
    }
}

#[test]
fn accelerating_shaft_never_switches_and_motion_resets_dwell() {
    let s = settings();
    let (mut b, c) = armed(s);
    // CSP saturates while it accelerates a free load: torque alone is not contact.
    for now in (2..500).step_by(20) {
        let out = b.step_with_settings(now, c, Some(now), Some(moving(now, 0.7, 0.5)), s);
        assert_eq!(phase(&out), 1);
    }
    // Held means at most a quarter of the commanded 1 rad/s; faster restarts the dwell.
    for (now, velocity) in [(502, 0.0), (522, -0.24), (542, 0.3), (562, 0.0), (602, 0.2)] {
        let out = b.step_with_settings(now, c, Some(now), Some(moving(now, 0.7, velocity)), s);
        assert_eq!(phase(&out), 1);
    }
    let out = b.step_with_settings(622, c, Some(622), Some(moving(622, 0.7, 0.05)), s);
    assert_current(out, 0.4);
}

#[test]
fn a_grip_that_keeps_creeping_below_a_quarter_of_the_speed_switches_to_cst() {
    let s = settings();
    let (mut b, c) = armed(s);
    // A yielding object: saturated at 0.7 N*m while still closing at 0.2 rad/s, twice the
    // old fixed 0.1 rad/s and a fifth of the commanded 1 rad/s.
    for now in (2..62).step_by(10) {
        let out = b.step_with_settings(now, c, Some(now), Some(moving(now, 0.7, 0.2)), s);
        assert_eq!(phase(&out), 1);
    }
    let out = b.step_with_settings(62, c, Some(62), Some(moving(62, 0.7, 0.2)), s);
    assert_current(out, 0.4);
    // At a slower command the floor of 0.1 rad/s still applies.
    let slow = Command { speed: 100, ..c };
    let (mut b, _) = armed(s);
    for now in (2..200).step_by(10) {
        let out = b.step_with_settings(now, slow, Some(now), Some(moving(now, 0.7, 0.15)), s);
        assert_eq!(phase(&out), 1);
    }
}

#[test]
fn after_cruise_a_torque_at_the_threshold_switches_while_the_fingers_still_close() {
    let s = settings();
    let (mut b, c) = armed(s);
    // Free closing at the commanded 1 rad/s with little torque: under way.
    for now in (2..102).step_by(10) {
        let out = b.step_with_settings(now, c, Some(now), Some(moving(now, 0.15, 1.0)), s);
        assert_eq!(phase(&out), 1);
    }
    // A yielding object: still closing at 0.6 rad/s (past a quarter of the speed), but
    // the torque stays at the threshold for the dwell.
    for now in (102..162).step_by(10) {
        let out = b.step_with_settings(now, c, Some(now), Some(moving(now, 0.65, 0.6)), s);
        assert_eq!(phase(&out), 1);
    }
    let out = b.step_with_settings(162, c, Some(162), Some(moving(162, 0.65, 0.6)), s);
    assert_current(out, 0.4);
    // Below the threshold it never switches, however long it closes.
    let (mut b, c) = armed(s);
    for now in (2..102).step_by(10) {
        b.step_with_settings(now, c, Some(now), Some(moving(now, 0.15, 1.0)), s);
    }
    for now in (102..1102).step_by(10) {
        let out = b.step_with_settings(now, c, Some(now), Some(moving(now, 0.4, 0.6)), s);
        assert_eq!(phase(&out), 1);
    }
}

fn sample(now: u64, torque: f32, position: f32, velocity: f32) -> Feedback {
    Feedback {
        velocity_rad_s: velocity,
        ..feedback(now, torque, position)
    }
}
fn detail(out: &Output) -> u16 {
    u16::from_be_bytes([out.status1000[24], out.status1000[25]])
}
fn assert_brake(out: &Output, limit: f32) {
    let Drive::Can(p) = out.drive else {
        panic!("current brake packet required: {:?}", out.drive)
    };
    assert_eq!(p.id, 0x268);
    let current = i32::from_be_bytes(p.data[..4].try_into().unwrap()) as f32 / 1000.0;
    assert!(
        (current * cubemars::AK40_10_PROFILE.kt - limit).abs() < 0.001,
        "{current}"
    );
}

#[test]
fn a_grip_keeps_its_push_at_zero_speed_and_a_new_target_is_a_new_csp_move() {
    let s = settings();
    let (mut b, mut c) = armed(s);
    step(&mut b, c, s, 2, 0.7);
    assert_current(step(&mut b, c, s, 62, 0.7), 0.4);
    // Without a ramp the reference keeps its lead, so the grip still pushes with CST.
    c.speed = 0;
    assert_current(step(&mut b, c, s, 63, 0.4), 0.4);
    assert_current(step(&mut b, c, s, 64, 0.4), 0.4);
    // A new target while executing starts CSP from the shaft: the grip ends there.
    c.speed = 500;
    c.position = 100;
    let out = step(&mut b, c, s, 65, 0.4);
    assert_eq!(phase(&out), 1);
    assert_eq!(out.status[0] >> 6, 0);
    let Drive::Torque(t) = out.drive else {
        panic!()
    };
    assert!((-0.01..0.0).contains(&t), "{t}");
}

#[test]
fn a_slipping_grip_is_pulled_back_to_the_csp_reference_and_ends_at_the_target() {
    let s = settings();
    let (mut b, mut c) = armed(s);
    c.position = 1024;
    let end = config::MOTION.closed_rad;
    let mut at = |now: u64, torque: f32, position: f32, velocity: f32| {
        let fb = sample(now, torque, position, velocity);
        b.step_with_settings(now, c, Some(now), Some(fb), s)
    };
    at(2, 0.7, end - 3.0, 0.0);
    assert_current(at(62, 0.7, end - 3.0, 0.0), 0.4);
    // The object slips and the shaft runs ahead: the reference does not run with it, so
    // the loop pulls it back with no more than the CST torque instead of pushing on.
    let out = at(63, 0.4, end - 2.0, 0.9);
    assert_eq!(phase(&out), 2);
    assert_eq!(out.drive, Drive::Torque(-0.4));
    // It took 1 rad from rest to reach 0.9 rad/s, so 1 rad (plus the feedback lead) from
    // the travel end the CSP guard brakes at the full 2.25 N*m limit that may have sped
    // it up, and stopped inside it resumes the ramp from there without a trip.
    assert_brake(&at(64, 0.4, end - 1.0, 0.9), 2.25);
    assert_brake(&at(65, 0.4, end - 0.5, 0.3), 2.25);
    let out = at(66, 0.4, end - 0.4, 0.0);
    assert_eq!(
        (out.drive, out.status[2], phase(&out)),
        (Drive::Torque(0.0), 0, 2)
    );
    let Drive::Torque(t) = at(67, 0.0, end - 0.4, 0.0).drive else {
        panic!()
    };
    assert!((0.0..0.01).contains(&t), "{t}");
    // At the target nothing holds the fingers any more: CSP takes it back (gOBJ 3).
    let out = at(68, 0.0, end, 0.0);
    assert_eq!((phase(&out), out.status[0] >> 6, out.status[2]), (1, 3, 0));
    let out = at(200, 0.7, end, 0.0);
    assert_eq!((phase(&out), out.status[0] >> 6), (1, 3));
    assert_eq!(detail(&out), 0);
}

#[test]
fn csp_brakes_for_a_travel_end_target_then_resumes_from_the_stop() {
    let s = settings();
    let (mut b, mut c) = armed(s);
    c.position = 1024;
    let end = config::MOTION.closed_rad;
    let start = end - 3.0;
    let mut at = |now: u64, position: f32, velocity: f32| {
        let fb = sample(now, 0.0, position, velocity);
        b.step_with_settings(now, c, Some(now), Some(fb), s)
    };
    // Held at rest, the ramp leads the shaft by the following lag (1.1 * 2.25 / kp 1).
    for now in 2..2600 {
        at(now, start, 0.0);
    }
    // It reached 2 rad/s 1 rad from rest; 2 rad short of the end it keeps driving.
    let out = at(2600, start + 1.0, 2.0);
    assert!(
        matches!(out.drive, Drive::Torque(t) if t > 1.0),
        "{:?}",
        out.drive
    );
    let out = at(2601, end - 1.0, 2.0);
    assert_brake(&out, 2.25);
    assert_eq!(phase(&out), 1);
    // Stopped inside: the ramp resumes toward the end from there, not from behind.
    let Drive::Torque(t) = at(2602, end - 1.0, 0.0).drive else {
        panic!()
    };
    assert!((0.0..0.01).contains(&t), "{t}");
}

#[test]
fn stop_holds_its_position_off_clears_and_identical_execute_rearms() {
    let s = settings();
    let (mut b, mut c) = armed(s);
    step(&mut b, c, s, 2, 0.7);
    assert_current(step(&mut b, c, s, 62, 0.7), 0.4);
    c.action = 1;
    let out = b.step_with_settings(63, c, Some(63), Some(feedback(63, 0.0, 1.0)), s);
    assert_eq!(phase(&out), 0);
    assert_eq!(out.drive, Drive::Torque(0.0));
    let held = b.step_with_settings(64, c, Some(64), Some(feedback(64, 0.0, 1.1)), s);
    let Drive::Torque(t) = held.drive else {
        panic!()
    };
    assert!(t < -0.09 && t > -0.11);
    c.action = 9;
    let restarted = step(&mut b, c, s, 65, 0.0);
    assert_eq!(phase(&restarted), 1);
    let Drive::Torque(t) = restarted.drive else {
        panic!()
    };
    assert!(t.abs() < 0.01); // Fresh ramp, no accumulated old position error.
    c.action = 0;
    let off = step(&mut b, c, s, 66, 0.7);
    assert_eq!(off.drive, Drive::Disabled);
    assert_eq!(phase(&off), 0);
}

#[test]
fn zeros_cannot_trigger_and_latched_cst_honors_zero_limit() {
    let s = settings();
    for (speed, force) in [(0, 250), (500, 0)] {
        let (mut b, mut c) = armed(s);
        c.speed = speed;
        c.force = force;
        for now in (2..200).step_by(20) {
            assert_eq!(phase(&step(&mut b, c, s, now, 0.7)), 1);
        }
    }
    let (mut b, mut c) = armed(s);
    step(&mut b, c, s, 2, 0.7);
    assert_current(step(&mut b, c, s, 62, 0.7), 0.4);
    c.force = 0;
    assert_eq!(step(&mut b, c, s, 63, 0.7).drive, Drive::Torque(0.0));
}

#[test]
fn stale_feedback_and_host_timeout_disable_in_both_hybrid_phases() {
    let s = settings();
    for latched in [false, true] {
        let (mut b, c) = armed(s);
        step(&mut b, c, s, 2, 0.7);
        if latched {
            step(&mut b, c, s, 62, 0.7);
        }
        let out = b.step_with_settings(200, c, Some(200), Some(feedback(2, 0.7, 0.0)), s);
        assert_eq!(out.drive, Drive::Disabled);
        assert_eq!(out.status[2], 9);
        assert_eq!(phase(&out), 0);
        let (mut b, c) = armed(s);
        step(&mut b, c, s, 2, 0.7);
        if latched {
            step(&mut b, c, s, 62, 0.7);
        }
        let out = b.step_with_settings(2000, c, Some(1), Some(feedback(2000, 0.7, 0.0)), s);
        assert_eq!(out.drive, Drive::Disabled);
        assert_eq!(out.status[2], 9);
    }
}

#[test]
fn hybrid_target_changes_require_stop_but_gain_tuning_does_not_rearm_cst() {
    let s = settings();
    let (mut b, c) = armed(s);
    step(&mut b, c, s, 2, 0.7);
    step(&mut b, c, s, 62, 0.7);
    let tuned = Settings { kp_centi: 200, ..s };
    assert_current(step(&mut b, c, tuned, 63, 0.0), 0.4);
    let changed = Settings {
        torque_mnm: 800,
        ..tuned
    };
    assert_eq!(step(&mut b, c, changed, 64, 0.0).drive, Drive::Disabled);
}

#[test]
fn hybrid_packets_are_atomic_and_invalid_profiles_or_thresholds_are_rejected() {
    let s = settings();
    let words = s.words();
    assert_eq!(Settings::decode(words), Some(s));
    for (index, bad) in [
        (0, 7),
        (10, 0),
        (10, 9001),
        (11, 9),
        (11, 1001),
        (12, 1),
        (12, 4),
        (13, 1),
    ] {
        let mut w = words;
        w[index] = bad;
        assert!(Settings::decode(w).is_none());
    }
    let changed = Settings {
        torque_mnm: 800,
        ..s
    };
    let mut req = std::vec![9, 16, 0x12, 0, 0, 16, 32];
    req.extend(changed.words().into_iter().flat_map(u16::to_be_bytes));
    req.extend(robotiq::crc16(&req).to_le_bytes());
    let mut actual = s;
    let mut tx = [0; robotiq::MAX_ADU];
    let cmd = Command {
        action: 9,
        ..Command::default()
    };
    assert!(advanced::process(9, &req, &mut tx, &mut actual, cmd, &[0; 32]).is_none());
    assert_eq!(actual, s);
    let (mut b, mut c) = armed(s);
    c.force = 50;
    assert_eq!(step(&mut b, c, s, 2, 0.0).drive, Drive::Disabled);
}

#[test]
fn ak40_10_and_ak60_39_are_the_only_motors_and_mit_mode_is_gone() {
    let profile = cubemars::AK40_10_PROFILE;
    // Torque requests become servo current commands clamped at the 4.1 N*m peak.
    for (requested, expected) in [(1.0, 1.0), (-2.5, -2.5), (9.0, 4.1), (-9.0, -4.1)] {
        let p = cubemars::torque_command(cubemars::AK40_10, 104, requested).unwrap();
        assert_eq!(p.id, 0x168);
        let iq = i32::from_be_bytes(p.data[..4].try_into().unwrap()) as f32 / 1000.0;
        assert!(
            (iq * profile.kt - expected).abs() < 0.001,
            "{requested}: {iq}"
        );
    }
    assert_eq!(
        cubemars::torque_command(cubemars::AK40_10, 104, 0.0),
        Some(cubemars::zero_current(104))
    );
    // Velocity and feedback use 14 pole pairs * 10:1.
    let p = cubemars::velocity_command(cubemars::AK40_10, 104, core::f32::consts::TAU);
    assert!((i32::from_be_bytes(p.data[..4].try_into().unwrap()) as f32 - 8400.0).abs() < 1.0);
    // Settings word 12 must be 2 (AK40-10) or 3 (AK60-39): 0 (AK80-9), 1 (AK60-6) and 4 are
    // refused, and so is mode 1 (the MIT mode those motors had).
    let s = Settings {
        mode: Mode::Position,
        ..Settings::default()
    };
    let words = s.words();
    assert_eq!(words[12], cubemars::AK40_10);
    assert_eq!(Settings::decode(words), Some(s));
    for model in [0, 1, 4, 0xffff] {
        let mut w = words;
        w[12] = model;
        assert_eq!(Settings::decode(w), None, "model {model}");
    }
    let mut w = words;
    w[12] = cubemars::AK60_39;
    assert_eq!(
        Settings::decode(w),
        Some(Settings {
            motor_model: cubemars::AK60_39,
            ..s
        })
    );
    let mut w = words;
    w[0] = 1;
    assert_eq!(Settings::decode(w), None, "MIT mode");
}

#[test]
fn ak40_10_position_mode_caps_torque_scales_speed_and_reports_model() {
    let s = Settings {
        mode: Mode::Position,
        kp_centi: 10_000,
        kd_milli: 0,
        ..Settings::default()
    };
    let mut b = Bridge::new(config::MOTION);
    let mut c = Command {
        action: 1,
        position: 1000,
        speed: 1000,
        force: 1000,
    };
    b.step_with_settings(0, c, Some(0), None, s);
    b.step_with_settings(1, c, Some(1), Some(feedback(1, 0.0, 0.0)), s);
    c.action = 9;
    let out = b.step_with_settings(2, c, Some(2), Some(feedback(2, 0.0, 0.0)), s);
    let Drive::Torque(t) = out.drive else {
        panic!()
    };
    // Speed 1000 = 370 RPM: one 1 ms step of the ramp at kp 100.
    let step = 370.0 * core::f32::consts::TAU / 60.0 * 0.001;
    assert!((t - 100.0 * step).abs() < 0.001, "{t}");
    // Far from the target, the 9 N*m scale is capped at the 4.1 N*m peak.
    let mut far = feedback(3, 0.0, -20.0);
    far.iq_a = 0.0;
    let out = b.step_with_settings(3, c, Some(3), Some(far), s);
    assert_eq!(out.drive, Drive::Torque(4.1));
    let status = u16::from_be_bytes([out.advanced_status[28], out.advanced_status[29]]);
    assert_eq!((status >> 8) & 3, 2);
    assert_eq!(
        u16::from_be_bytes([out.advanced_status[2], out.advanced_status[3]]),
        3
    );
}

#[test]
fn csp_phase_banks_no_reference_while_held_below_the_threshold() {
    let s = settings();
    let (mut b, c) = armed(s);
    // Held below the switch threshold for 3 s: the 1 rad/s ramp alone would bank 3 rad.
    for now in 2..=3001 {
        assert_eq!(phase(&step(&mut b, c, s, now, 0.5)), 1);
    }
    let released = b.step_with_settings(3002, c, Some(3002), Some(feedback(3002, 0.0, 1.0)), s);
    assert_eq!(phase(&released), 1);
    let Drive::Torque(t) = released.drive else {
        panic!()
    };
    // kp = 1 and a 2.25 N*m limit: the reference leads the shaft by at most 2.475 rad.
    assert!((t - 1.476).abs() < 0.001, "{t}");
}

#[test]
fn boot_gains_are_the_ak40_10_ones() {
    // Kp 1 / Kd 0.05; the host overwrites them with every settings write.
    let s = Settings::default();
    assert_eq!((s.kp_centi, s.kd_milli), (100, 50));
    assert!(s.valid());
}

#[test]
fn build_time_motor_id_defaults_to_104_and_accepts_only_1_to_127() {
    assert_eq!(config::parse_motor_id(None), 0x68);
    for (text, id) in [("1", 1), ("2", 2), ("104", 104), ("127", 127)] {
        assert_eq!(config::parse_motor_id(Some(text)), id);
    }
    for bad in ["", "0", "128", "1000", "a", "-1", " 2"] {
        assert!(
            std::panic::catch_unwind(|| config::parse_motor_id(Some(bad))).is_err(),
            "{bad}"
        );
    }
}

#[test]
fn build_time_slave_id_defaults_to_nine_and_accepts_only_1_to_247() {
    assert_eq!(config::parse_slave_id(None), 9);
    for (text, id) in [("1", 1), ("2", 2), ("09", 9), ("247", 247)] {
        assert_eq!(config::parse_slave_id(Some(text)), id);
    }
    for bad in ["", "0", "248", "1000", "a", "-1", " 1"] {
        assert!(
            std::panic::catch_unwind(|| config::parse_slave_id(Some(bad))).is_err(),
            "{bad}"
        );
    }
}
