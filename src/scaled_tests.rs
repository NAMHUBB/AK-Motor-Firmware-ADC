use crate::{
    bridge::{Bridge, Drive, FaultDetail, Output},
    config::{self, MOTION, MotionConfig},
    control::Command,
    cubemars::{self, Feedback, MotorProfile},
};

/// The only motor profile: AK40-10.
fn boot() -> MotorProfile {
    cubemars::AK40_10_PROFILE
}

fn config() -> MotionConfig {
    MotionConfig {
        kp: 1.0,
        kd: 0.0,
        // Non-speed regression fixtures retain their independent 2 rad/s setup.
        max_speed_rad_s: 2.0,
        ..MOTION
    }
}

fn command() -> Command {
    Command {
        action: 9,
        position: 1000,
        speed: 1000,
        force: 1000,
    }
}

fn feedback(now: u64, position: f32) -> Feedback {
    Feedback {
        position_rad: position,
        velocity_rad_s: 0.0,
        iq_a: 0.0,
        temperature_c: 25,
        error: 0,
        received_ms: now,
    }
}

fn activate(bridge: &mut Bridge, command: Command, position: f32) -> Output {
    let first = bridge.step_scaled(0, command, Some(0), None);
    assert_eq!(first.drive, Drive::Torque(0.0));
    let ready = bridge.step_scaled(1, command, Some(1), Some(feedback(1, position)));
    assert_eq!(ready.status[2], 0);
    assert_eq!(ready.status[0] & 0x30, 0x30);
    assert_eq!(ready.drive, Drive::Torque(0.0));
    ready
}

fn torque(output: Output) -> f32 {
    assert_eq!(output.status[2], 0);
    let Drive::Torque(value) = output.drive else {
        panic!("unexpected disabled drive")
    };
    value
}

fn close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 0.000_01,
        "expected {expected}, got {actual}"
    );
}

fn word(output: &Output, index: usize) -> u16 {
    u16::from_be_bytes(
        output.status1000[index * 2..index * 2 + 2]
            .try_into()
            .unwrap(),
    )
}

fn millidegrees(output: &Output) -> i32 {
    ((u32::from(word(output, 6)) << 16) | u32::from(word(output, 7))) as i32
}

#[test]
fn scaled_position_midpoint_is_exact_zero_and_endpoints_remain_exact() {
    let end = config::POSITION_TURNS * core::f32::consts::TAU;
    for (units, position) in [
        (0, -end),
        (511, -end / 512.0),
        (512, 0.0),
        (513, end / 512.0),
        (1024, end),
    ] {
        let command = Command {
            position: units,
            ..command()
        };
        let mut bridge = Bridge::new(config());
        activate(&mut bridge, command, position);
        let output = bridge.step_scaled(2, command, Some(2), Some(feedback(2, position)));
        close(torque(output), 0.0);
        assert_eq!(output.status[0] >> 6, 3);
        assert_eq!(word(&output, 4), units);
        assert_eq!(word(&output, 5), units);
    }
}

#[test]
fn scaled_speed_controls_the_actual_ramp_from_zero_to_rated_rpm() {
    // AK40-10 370 RPM: 0.37, 185, 370.
    for units in [0, 1, 500, 1000] {
        let rpm = boot().rated_speed_rpm * units as f32 / 1000.0;
        let speed = rpm * core::f32::consts::TAU / 60.0;
        let command = Command {
            speed: units,
            ..command()
        };
        let mut bridge = Bridge::new(MotionConfig {
            max_speed_rad_s: MOTION.max_speed_rad_s,
            ..config()
        });
        activate(&mut bridge, command, 0.0);
        for now in 2..=101 {
            // With kp=1, kd=0 and a stationary measured shaft, torque reveals
            // the actual reference position without reaching the torque cap.
            let output = bridge.step_scaled(now, command, Some(now), Some(feedback(now, 0.0)));
            close(torque(output), speed * (now - 1) as f32 / 1000.0);
        }
    }
}

#[test]
fn scaled_torque_requests_nine_and_four_point_five_newton_metres_capped_at_the_peak() {
    for (units, limit) in [(1000, 9.0f32), (500, 4.5), (0, 0.0)] {
        let command = Command {
            force: units,
            ..command()
        };
        let mut bridge = Bridge::new(MotionConfig {
            kp: 10_000.0,
            ..config()
        });
        activate(&mut bridge, command, 0.0);
        let output = bridge.step_scaled(2, command, Some(2), Some(feedback(2, 0.0)));
        // Never above the motor's own peak (AK40-10: 4.1 N*m).
        close(torque(output), limit.min(boot().max_torque));
        assert_eq!(word(&output, 10), units);
    }
}

#[test]
fn zero_torque_tracks_external_motion_without_contact_or_reference_backlog() {
    let mut bridge = Bridge::new(config());
    let command = command();
    activate(&mut bridge, command, 0.0);
    let muted = Command {
        force: 0,
        ..command
    };
    for now in 2..=202 {
        let position = (now - 2) as f32 / 200.0;
        let output = bridge.step_scaled(now, muted, Some(now), Some(feedback(now, position)));
        assert_eq!(torque(output), 0.0);
        assert_eq!(output.status[0] >> 6, 0);
    }
    let resumed = bridge.step_scaled(203, command, Some(203), Some(feedback(203, 1.0)));
    close(torque(resumed), 0.002);
    assert_eq!(resumed.status[0] >> 6, 0);
}

#[test]
fn zero_speed_stops_at_current_position_and_clears_a_latched_contact() {
    let mut bridge = Bridge::new(MotionConfig {
        kp: 100.0,
        contact_dwell_ms: 2,
        ..config()
    });
    let command = command();
    activate(&mut bridge, command, 0.0);
    let mut last = None;
    for now in 2..=50 {
        let mut sample = feedback(now, 0.0);
        sample.iq_a = 14.0;
        last = Some(bridge.step_scaled(now, command, Some(now), Some(sample)));
    }
    assert_eq!(last.unwrap().status[0] >> 6, 2);
    let paused = Command {
        speed: 0,
        ..command
    };
    let stopped = bridge.step_scaled(51, paused, Some(51), Some(feedback(51, 0.1)));
    close(torque(stopped), 0.0);
    assert_eq!(stopped.status[0] >> 6, 0);
    for now in 52..=55 {
        let mut displaced = feedback(now, 0.15);
        displaced.iq_a = 14.0;
        let holding = bridge.step_scaled(now, paused, Some(now), Some(displaced));
        close(torque(holding), -(5.0f32).min(boot().max_torque));
        assert_eq!(holding.status[0] >> 6, 0);
    }
}

#[test]
fn positions_outside_slider_activate_and_return_without_a_boundary_snap() {
    // Outside the +-8 turn slider but inside the +-3200 deg feedback range.
    for position in [-52.0, 52.0] {
        let command = Command {
            position: 500,
            ..command()
        };
        let mut bridge = Bridge::new(config());
        let ready = activate(&mut bridge, command, position);
        assert_eq!(
            millidegrees(&ready),
            if position > 0.0 {
                2_979_381
            } else {
                -2_979_381
            }
        );
        assert_eq!(word(&ready, 5), if position > 0.0 { 1024 } else { 0 });
        let inward = bridge.step_scaled(2, command, Some(2), Some(feedback(2, position)));
        close(torque(inward), if position > 0.0 { -0.002 } else { 0.002 });
        assert_eq!(word(&inward, 4), 500);
    }
}

#[test]
fn extended_status_preserves_signed_angle_full_current_and_native_echoes() {
    let command = Command {
        action: 0,
        position: 333,
        speed: 456,
        force: 789,
    };
    let mut sample = feedback(150, -90.6 * core::f32::consts::PI / 180.0);
    sample.iq_a = -12.5;
    let output = Bridge::new(config()).step_scaled(200, command, Some(200), Some(sample));
    assert_eq!(word(&output, 0), 0x414b);
    assert_eq!(word(&output, 1), 8);
    assert_eq!(word(&output, 4), 333);
    assert_eq!(word(&output, 5), 496);
    assert_eq!(millidegrees(&output), -90_600);
    assert_eq!(word(&output, 8), 12_500);
    assert_eq!(word(&output, 9), 456);
    assert_eq!(word(&output, 10), 789);
    assert_eq!(word(&output, 11), 50);
    assert_eq!(word(&output, 12), 0);
    assert_eq!(word(&output, 14), 104);
    assert_eq!(word(&output, 15), 1000);
    assert_eq!(output.status[5], 255);
}

#[test]
fn timeout_diagnostics_distinguish_host_activation_and_running_feedback() {
    let command = command();
    let mut host = Bridge::new(config());
    let now = MOTION.host_timeout_ms + 1;
    let output = host.step_scaled(now, command, Some(0), Some(feedback(now, 0.0)));
    assert_eq!(output.drive, Drive::Disabled);
    assert_eq!(word(&output, 3), 9);
    assert_eq!(word(&output, 12), 1);

    let mut activating = Bridge::new(config());
    activating.step_scaled(0, command, Some(0), None);
    let now = MOTION.activation_timeout_ms + 1;
    let output = activating.step_scaled(now, command, Some(now), None);
    assert_eq!(output.drive, Drive::Disabled);
    assert_eq!(word(&output, 3), 9);
    assert_eq!(word(&output, 11), u16::MAX);
    assert_eq!(word(&output, 12), 2);

    let mut running = Bridge::new(config());
    activate(&mut running, command, 0.0);
    let now = MOTION.feedback_timeout_ms + 2;
    let output = running.step_scaled(now, command, Some(now), Some(feedback(1, 0.0)));
    assert_eq!(output.drive, Drive::Disabled);
    assert_eq!(word(&output, 3), 9);
    assert_eq!(word(&output, 12), 3);
}

#[test]
fn can_tick_and_tx_fault_details_latch_until_an_explicit_reset() {
    for (detail, code) in [
        (FaultDetail::CanBus, 4),
        (FaultDetail::TickLate, 5),
        (FaultDetail::TxQueue, 6),
    ] {
        let mut bridge = Bridge::new(config());
        let command = command();
        activate(&mut bridge, command, 0.0);
        bridge.trip_with_detail(0x09, detail);
        let fault = bridge.step_scaled(2, command, Some(2), Some(feedback(2, 0.0)));
        assert_eq!(fault.drive, Drive::Disabled);
        assert_eq!(word(&fault, 3), 9);
        assert_eq!(word(&fault, 12), code);
        let reset = bridge.step_scaled(3, Command::default(), Some(3), Some(feedback(3, 0.0)));
        assert_eq!(reset.drive, Drive::Disabled);
        assert_eq!(word(&reset, 3), 0);
        assert_eq!(word(&reset, 12), 0);
    }
}

#[test]
fn reset_does_not_retrip_on_a_motor_error_from_the_previous_activation() {
    let command = command();
    let mut bridge = Bridge::new(config());
    activate(&mut bridge, command, 0.0);
    let mut old_error = feedback(2, 0.0);
    old_error.error = 2;
    let fault = bridge.step_scaled(2, command, Some(2), Some(old_error));
    assert_eq!(word(&fault, 3), 0x0e);
    assert_eq!(word(&fault, 12), 8);
    let reset = bridge.step_scaled(3, Command::default(), Some(3), Some(old_error));
    assert_eq!(word(&reset, 3), 0);
    let waiting = bridge.step_scaled(4, command, Some(4), Some(old_error));
    assert_eq!(waiting.drive, Drive::Torque(0.0));
    assert_eq!(word(&waiting, 3), 0);
    assert_eq!(waiting.status[0] & 0x30, 0x10);
    let ready = bridge.step_scaled(5, command, Some(5), Some(feedback(5, 0.0)));
    assert_eq!(word(&ready, 3), 0);
    assert_eq!(word(&ready, 12), 0);
    assert_eq!(ready.status[0] & 0x30, 0x30);
}

#[test]
fn a_held_shaft_banks_no_reference_and_resumes_at_the_commanded_speed() {
    // kp = 1 and a 0.9 N*m limit saturate at 0.9 rad; the reference may lead by 0.99 rad.
    let held = Command {
        force: 100,
        ..command()
    };
    let mut bridge = Bridge::new(config());
    activate(&mut bridge, held, 0.0);
    let mut blocked = None;
    // One second blocked at 0 rad: the 2 rad/s ramp alone would bank 2 rad.
    for now in 2..=1001 {
        blocked = Some(bridge.step_scaled(now, held, Some(now), Some(feedback(now, 0.0))));
    }
    close(torque(blocked.unwrap()), 0.9);
    // Released to 0.5 rad: the reference is 0.99 rad plus one 2 mrad ramp step.
    let released = bridge.step_scaled(1002, held, Some(1002), Some(feedback(1002, 0.5)));
    close(torque(released), 0.492);

    // Pushed 5 rad away from a settled target, then released 4.5 rad away.
    let parked = Command {
        position: 512,
        ..held
    };
    let mut bridge = Bridge::new(config());
    activate(&mut bridge, parked, 0.0);
    let settled = bridge.step_scaled(2, parked, Some(2), Some(feedback(2, 0.0)));
    assert_eq!(settled.status[0] >> 6, 3);
    for now in 3..=102 {
        let pushed = bridge.step_scaled(now, parked, Some(now), Some(feedback(now, -5.0)));
        close(torque(pushed), 0.9);
    }
    let released = bridge.step_scaled(103, parked, Some(103), Some(feedback(103, -4.5)));
    close(torque(released), 0.492);
}
