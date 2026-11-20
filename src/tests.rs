use crate::{
    bridge::{Bridge, Drive},
    config::{MOTION, MotionConfig},
    cubemars::{self, Feedback},
    robotiq::{self, Command, MAX_ADU, process},
};
use std::vec::Vec;

fn adu(bytes: &[u8]) -> Vec<u8> {
    let mut b = bytes.to_vec();
    b.extend_from_slice(&robotiq::crc16(bytes).to_le_bytes());
    b
}
fn fb(t: u64, p: f32) -> Feedback {
    Feedback {
        position_rad: p,
        velocity_rad_s: 0.0,
        iq_a: 0.0,
        temperature_c: 25,
        error: 0,
        received_ms: t,
    }
}
fn cfg() -> MotionConfig {
    // Contact/stop fixtures use a fixed 0..1 rad, low-speed, low-torque test plant.
    // The configured motor ratings are exercised separately below.
    MotionConfig {
        commissioned: true,
        open_rad: 0.0,
        closed_rad: 1.0,
        max_speed_rad_s: 0.5,
        max_torque_nm: 1.0,
        ..MOTION
    }
}
fn cmd() -> Command {
    Command {
        action: 9,
        position: 255,
        speed: 255,
        force: 255,
    }
}
fn activate(b: &mut Bridge, c: Command, p: f32) {
    assert_eq!(b.step(0, c, Some(0), None).drive, Drive::Torque(0.0));
    let o = b.step(1, c, Some(1), Some(fb(1, p)));
    assert_eq!(o.drive, Drive::Torque(0.0));
    assert_eq!(o.status[0] & 0x31, 0x31);
}

#[test]
fn documented_robotiq_crc_and_write_bytes() {
    let reset = adu(&[9, 0x10, 3, 0xe8, 0, 3, 6, 0, 0, 0, 0, 0, 0]);
    assert_eq!(&reset[13..], &[0x73, 0x30]);
    let mut regs = [0; 16];
    let mut tx = [0; MAX_ADU];
    let r = process(
        9,
        &adu(&[9, 0x10, 3, 0xe8, 0, 3, 6, 9, 0, 0, 255, 255, 255]),
        &mut tx,
        &mut regs,
        &[0; 16],
    )
    .unwrap();
    assert_eq!(&tx[..r.len], &[9, 0x10, 3, 0xe8, 0, 3, 1, 0x30]);
    assert_eq!(Command::from_bytes(&regs), cmd());
    assert_eq!(&regs[6..], &[0; 10]);
}
#[test]
fn documented_status_read_and_fc03_alias() {
    let status = [0xf9, 0, 0, 0, 0x0d, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    let mut regs = [0; 16];
    let mut tx = [0; MAX_ADU];
    let request = [9, 3, 7, 0xd0, 0, 3, 4, 0x0e];
    let r = process(9, &request, &mut tx, &mut regs, &status).unwrap();
    assert_eq!(&tx[..r.len], &[9, 3, 6, 0xf9, 0, 0, 0, 0x0d, 0, 0x56, 0x4c]);
    let r = process(9, &adu(&[9, 4, 7, 0xd0, 0, 3]), &mut tx, &mut regs, &status).unwrap();
    assert_eq!(&tx[..r.len - 2], &[9, 4, 6, 0xf9, 0, 0, 0, 0x0d, 0]);
}
#[test]
fn fc23_validates_both_ranges_before_committing() {
    let mut regs = [0; 16];
    let mut tx = [0; MAX_ADU];
    let mut status = [0; 16];
    status[0] = 0x31;
    let req = adu(&[
        9, 0x17, 7, 0xd0, 0, 3, 3, 0xe8, 0, 3, 6, 9, 0, 0, 127, 128, 129,
    ]);
    let r = process(9, &req, &mut tx, &mut regs, &status).unwrap();
    assert!(r.written);
    assert_eq!(r.len, 11);
    assert_eq!(tx[3], 0x31);
    assert_eq!(regs[3], 127);
    let before = regs;
    let bad = adu(&[9, 0x17, 7, 0xd7, 0, 3, 3, 0xe8, 0, 3, 6, 0, 0, 0, 0, 0, 0]);
    assert!(process(9, &bad, &mut tx, &mut regs, &status).is_none());
    assert_eq!(regs, before);
}
#[test]
fn invalid_frames_never_modify_registers() {
    let mut tx = [0; MAX_ADU];
    let mut regs = [0; 16];
    for header in [
        &[8, 0x10, 3, 0xe8, 0, 1, 2, 1, 0][..], // foreign address
        &[9, 0x10, 3, 0xe8, 0, 0, 0][..],       // zero count
        &[9, 0x10, 3, 0xef, 0, 2, 4, 1, 0, 0, 0][..], // beyond range
        &[9, 0x10, 3, 0xe8, 0xff, 0xff, 2, 1, 0][..], // overflow count
        &[9, 0x10, 3, 0xe8, 0, 3, 2, 1, 0][..], // count mismatch
        &[9, 4, 7, 0xd0, 0, 9][..],             // excessive read
        &[9, 0x17][..],                         // truncated
    ] {
        assert!(process(9, &adu(header), &mut tx, &mut regs, &[0; 16]).is_none());
        assert_eq!(regs, [0; 16]);
    }
    let mut bad = adu(&[9, 0x10, 3, 0xe8, 0, 1, 2, 1, 0]);
    bad[7] ^= 1;
    assert!(process(9, &bad, &mut tx, &mut regs, &[0; 16]).is_none());
    assert_eq!(regs, [0; 16]);
}
#[test]
fn broadcast_write_is_silent_and_reserved_bytes_are_zeroed() {
    let mut tx = [0; MAX_ADU];
    let mut regs = [0; 16];
    let r = process(
        9,
        &adu(&[0, 0x10, 3, 0xe8, 0, 4, 8, 255, 255, 255, 2, 3, 4, 255, 255]),
        &mut tx,
        &mut regs,
        &[0; 16],
    )
    .unwrap();
    assert!(r.written);
    assert_eq!(r.len, 0);
    assert_eq!(regs, [0x39, 0, 0, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    assert!(
        process(
            9,
            &adu(&[0, 4, 7, 0xd0, 0, 3]),
            &mut tx,
            &mut regs,
            &[0; 16]
        )
        .is_none()
    );
}
#[test]
fn random_and_truncated_requests_do_not_panic_or_overrun() {
    let mut seed = 7u32;
    let mut tx = [0; MAX_ADU];
    let mut regs = [0; 16];
    for length in 0..=MAX_ADU {
        let mut bytes = [0; MAX_ADU];
        for b in bytes.iter_mut().take(length) {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            *b = (seed >> 24) as u8;
        }
        if length >= 4 {
            bytes[0] = 9;
            let c = robotiq::crc16(&bytes[..length - 2]).to_le_bytes();
            bytes[length - 2..length].copy_from_slice(&c);
        }
        let _ = process(9, &bytes[..length], &mut tx, &mut regs, &[0; 16]);
    }
}
#[test]
fn disable_and_temporary_origin_frame_layout() {
    assert_eq!(cubemars::disable(1).id, 0xf01);
    assert_eq!(cubemars::disable(1).len, 0);
    let origin = cubemars::set_origin(2);
    assert_eq!((origin.id, origin.len, origin.data[0]), (0x502, 1, 0));
}
#[test]
fn feedback_id_length_sign_and_physical_units() {
    // 1400 electrical RPM: 10 output RPM through the AK40-10 14 pole pairs x 10:1.
    let bytes = [3, 0x84, 0, 140, 0xff, 0x9c, 0xfb, 0];
    let f = cubemars::parse_feedback(cubemars::AK40_10, 1, 0x2901, &bytes, 42).unwrap();
    assert!((f.position_rad - core::f32::consts::FRAC_PI_2).abs() < 0.00001);
    assert!((f.velocity_rad_s - core::f32::consts::TAU / 6.0).abs() < 0.00001);
    assert_eq!(f.iq_a, -1.0);
    assert_eq!(f.temperature_c, -5);
    assert_eq!(f.received_ms, 42);
    assert!(cubemars::parse_feedback(cubemars::AK40_10, 1, 0x2902, &bytes, 0).is_none());
    assert!(cubemars::parse_feedback(cubemars::AK40_10, 1, 0x2901, &bytes[..7], 0).is_none());
    let mut ack = bytes;
    ack[7] = 0x77;
    assert!(cubemars::parse_feedback(cubemars::AK40_10, 1, 0x2901, &ack, 0).is_none());
}
#[test]
fn commissioning_lock_and_invalid_geometry_never_drive() {
    for c in [
        MotionConfig {
            commissioned: false,
            ..cfg()
        },
        MotionConfig {
            closed_rad: 0.0,
            ..cfg()
        },
        MotionConfig {
            kp: f32::NAN,
            ..cfg()
        },
        MotionConfig {
            max_torque_nm: 26.0,
            ..cfg()
        },
    ] {
        let o = Bridge::new(c).step(0, cmd(), Some(0), Some(fb(0, 0.0)));
        assert_eq!(o.drive, Drive::Disabled);
        assert_eq!(o.status[2], 13);
    }
}
#[test]
fn activation_waits_for_feedback_then_trajectory_is_bounded() {
    let mut b = Bridge::new(MOTION);
    activate(&mut b, cmd(), 0.3);
    // The AK40-10 peak (4.1 N*m) is below the 9 N*m force scale.
    let cap = MOTION
        .max_torque_nm
        .min(cubemars::AK40_10_PROFILE.max_torque);
    let mut saturated = false;
    for t in 2..1000 {
        let o = b.step(t, cmd(), Some(t), Some(fb(t, 0.3)));
        assert_eq!(o.status[2], 0);
        let Drive::Torque(torque) = o.drive else {
            panic!("unexpected disable")
        };
        assert!((0.0..=cap).contains(&torque));
        saturated |= torque == cap;
    }
    assert!(
        saturated,
        "Exercise the configured torque cap during the rated-speed ramp"
    );
}
#[test]
fn host_and_motor_timeouts_latch_until_reset() {
    for host_timeout in [true, false] {
        let mut b = Bridge::new(cfg());
        activate(&mut b, cmd(), 0.0);
        let t = 1200;
        let o = b.step(
            t,
            cmd(),
            Some(if host_timeout { 1 } else { t }),
            Some(fb(if host_timeout { t } else { 1 }, 0.0)),
        );
        assert_eq!(o.drive, Drive::Disabled);
        assert_eq!(o.status[2], 9);
        assert_eq!(
            b.step(t + 1, cmd(), Some(t + 1), Some(fb(t + 1, 0.0)))
                .drive,
            Drive::Disabled
        );
        assert_eq!(
            b.step(t + 2, Command::default(), Some(t + 2), None).status[2],
            0
        );
        assert_eq!(
            b.step(t + 3, cmd(), Some(t + 3), Some(fb(t + 3, 0.0)))
                .drive,
            Drive::Torque(0.0)
        );
    }
}
#[test]
fn no_feedback_activation_times_out_without_position_command() {
    let mut b = Bridge::new(cfg());
    assert_eq!(b.step(0, cmd(), Some(0), None).drive, Drive::Torque(0.0));
    let o = b.step(1001, cmd(), Some(1001), None);
    assert_eq!(o.drive, Drive::Disabled);
    assert_eq!(o.status[2], 9);
}
#[test]
fn motor_faults_disable() {
    for (error, expected) in [(1, 8), (2, 14), (3, 12), (4, 10), (5, 12), (6, 8), (7, 13)] {
        let mut b = Bridge::new(cfg());
        activate(&mut b, cmd(), 0.0);
        let mut f = fb(2, 0.0);
        f.error = error;
        let o = b.step(2, cmd(), Some(2), Some(f));
        assert_eq!(o.drive, Drive::Disabled);
        assert_eq!(o.status[2], expected);
    }
}
#[test]
fn contact_dwell_latches_closing_and_stop_holds_actual_position() {
    let mut b = Bridge::new(cfg());
    activate(&mut b, cmd(), 0.0);
    let mut o = b.step(2, cmd(), Some(2), Some(fb(2, 0.0)));
    for t in 3..800 {
        let mut f = fb(t, 0.0);
        f.iq_a = 2.0;
        o = b.step(t, cmd(), Some(t), Some(f));
    }
    assert_eq!(o.status[0] >> 6, 2);
    assert_eq!(o.drive, Drive::Torque(1.0));
    let stop = Command { action: 1, ..cmd() };
    let o = b.step(801, stop, Some(801), Some(fb(801, 0.0)));
    assert_eq!(o.status[0] & 0xc8, 0);
    assert_eq!(o.drive, Drive::Torque(0.0));
}
#[test]
fn reverse_geometry_maps_position_and_opening_contact() {
    let mut b = Bridge::new(MotionConfig {
        open_rad: 1.0,
        closed_rad: 0.0,
        ..cfg()
    });
    let c = Command {
        position: 0,
        ..cmd()
    };
    activate(&mut b, c, 0.0);
    let mut o = b.step(2, c, Some(2), Some(fb(2, 0.0)));
    assert_eq!(o.status[4], 255);
    for t in 3..800 {
        let mut f = fb(t, 0.0);
        f.iq_a = 2.0;
        o = b.step(t, c, Some(t), Some(f));
    }
    assert_eq!(o.status[0] >> 6, 1);
    assert_eq!(o.drive, Drive::Torque(1.0));
}
#[test]
fn release_direction_latches_and_completion_requires_reset() {
    let mut b = Bridge::new(cfg());
    let c = Command { action: 1, ..cmd() };
    activate(&mut b, c, 0.5);
    let release = Command { action: 0x31, ..c };
    let progress = b.step(2, release, Some(2), Some(fb(2, 0.5)));
    assert_eq!(progress.status[2], 0x0B);
    assert_eq!(progress.status[0] & 0x39, 1);
    let o = b.step(3, Command { action: 0x11, ..c }, Some(3), Some(fb(3, 0.0)));
    assert_eq!(o.status[2], 15);
    assert_eq!(o.drive, Drive::Disabled);
    assert_eq!(
        b.step(4, c, Some(4), Some(fb(4, 0.0))).drive,
        Drive::Disabled
    );
}
#[test]
fn release_timeout_and_requested_position_status() {
    let mut b = Bridge::new(cfg());
    let c = Command {
        action: 0x31,
        ..cmd()
    };
    activate(&mut b, c, 0.5);
    b.step(2, c, Some(2), Some(fb(2, 0.5)));
    let o = b.step(5003, c, Some(5003), Some(fb(5003, 0.5)));
    assert_eq!(o.status[2], 13);
    assert_eq!(o.drive, Drive::Disabled);
    let mut b = Bridge::new(cfg());
    activate(&mut b, cmd(), 1.0);
    let o = b.step(2, cmd(), Some(2), Some(fb(2, 1.0)));
    assert_eq!(o.status[0], 0xf9);
    assert_eq!(o.status[3], 255);
    assert_eq!(o.status[4], 255);
}

#[test]
fn complete_rtu_command_mit_feedback_rtu_status_roundtrip() {
    let mut registers = [0; 16];
    let mut tx = [0; MAX_ADU];
    let write = adu(&[9, 0x10, 3, 0xe8, 0, 3, 6, 9, 0, 0, 255, 255, 255]);
    assert!(
        process(9, &write, &mut tx, &mut registers, &[0; 16])
            .unwrap()
            .written
    );
    let c = Command::from_bytes(&registers);
    let mut b = Bridge::new(cfg());
    activate(&mut b, c, 0.0);
    let moving = b.step(2, c, Some(2), Some(fb(2, 0.0)));
    let Drive::Torque(t) = moving.drive else {
        panic!("No torque request")
    };
    let wire = cubemars::torque_command(cubemars::AK40_10, 1, t).unwrap();
    assert_eq!(wire.id, 0x101, "servo current command");
    assert!(t > 0.0 && t <= 1.0);
    // Mock motor reports the target ahead of the internal trajectory ramp. rPR 255 is
    // one 1/256 step short of the closed end: 0.996 rad, about 57.1 deg.
    let motor = cubemars::parse_feedback(
        cubemars::AK40_10,
        1,
        0x2901,
        &[2, 0x3b, 0, 0, 0, 0, 25, 0],
        3,
    )
    .unwrap();
    let arrived = b.step(3, c, Some(3), Some(motor));
    assert_eq!(arrived.status[0], 0xf9);
    let Drive::Torque(t) = arrived.drive else {
        panic!("Target hold missing")
    };
    assert!(
        t.abs() < 0.01,
        "Do not pull back toward a stale ramp position"
    );
    let reply = process(
        9,
        &adu(&[9, 4, 7, 0xd0, 0, 3]),
        &mut tx,
        &mut registers,
        &arrived.status,
    )
    .unwrap();
    assert_eq!(&tx[3..9], &[0xf9, 0, 0, 255, 255, 0]);
    assert_eq!(
        robotiq::crc16(&tx[..reply.len - 2]),
        u16::from_le_bytes([tx[reply.len - 2], tx[reply.len - 1]])
    );
}

#[test]
fn action_without_activation_reports_precondition_fault() {
    let c = Command { action: 8, ..cmd() };
    let o = Bridge::new(cfg()).step(0, c, Some(0), None);
    assert_eq!(o.drive, Drive::Disabled);
    assert_eq!(o.status[2], 7);
}
