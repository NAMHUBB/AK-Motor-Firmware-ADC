//! Activation homing of the ball-screw gripper against a simulated shaft.
use crate::{
    advanced::{Mode, Settings},
    bridge::{Bridge, Drive, FaultDetail, Output},
    config::{self, GRIPPER_LEFT, GRIPPER_RIGHT, GRIPPERS, Gripper, MOTION},
    control::Command,
    cubemars::{self, Feedback},
};

/// Output shaft with inertia and viscous friction between two hard ends. The motor draws
/// the commanded torque, so the measured current follows it, and reports `position`
/// from its own origin, which the temporary-origin command moves (unless `deaf`).
struct Shaft {
    position: f32,
    origin: f32,
    deaf: bool,
    velocity: f32,
    torque: f32,
    low: f32,
    high: f32,
    /// Fastest arrival at the low end, and the most torque pushed into it.
    low_impact: f32,
    low_torque: f32,
    /// Coulomb friction: the shaft stays put while the torque is below it.
    friction: f32,
    /// A yielding object: from this position on it resists this many N*m per rad pressed
    /// in (toward the high end).
    spring: Option<(f32, f32)>,
}
impl Shaft {
    fn new(position: f32, low: f32, high: f32) -> Self {
        Self {
            position,
            origin: 0.0,
            deaf: false,
            velocity: 0.0,
            torque: 0.0,
            low,
            high,
            low_impact: 0.0,
            low_torque: 0.0,
            friction: 0.0,
            spring: None,
        }
    }
    /// What the motor reports: the angle from its origin.
    fn reported(&self) -> f32 {
        self.position - self.origin
    }
    fn feedback(&self, now: u64) -> Feedback {
        Feedback {
            position_rad: self.reported(),
            velocity_rad_s: self.velocity,
            iq_a: self.torque / cubemars::AK40_10_PROFILE.kt,
            temperature_c: 25,
            error: 0,
            received_ms: now,
        }
    }
    fn apply(&mut self, drive: Drive) {
        if matches!(drive, Drive::Can(p) if p.id >> 8 == 5 && p.len == 1 && p.data[0] == 0)
            && !self.deaf
        {
            self.origin = self.position;
        }
        self.torque = match drive {
            Drive::Torque(t) => t,
            _ => 0.0,
        };
        let load = self
            .spring
            .map_or(0.0, |(at, k)| -k * (self.position - at).max(0.0));
        let push = self.torque + load;
        let drive = if self.velocity == 0.0 && push.abs() <= self.friction {
            0.0
        } else {
            push - self.friction
                * if self.velocity != 0.0 {
                    self.velocity.signum()
                } else {
                    push.signum()
                }
        };
        let before = self.velocity;
        self.velocity += (drive - 0.005 * self.velocity) / 0.001 * 0.001;
        // Friction stops the shaft rather than reversing it.
        if before != 0.0 && self.velocity * before < 0.0 && push.abs() <= self.friction {
            self.velocity = 0.0;
        }
        self.position += self.velocity * 0.001;
        if self.position >= self.high {
            self.position = self.high;
            self.velocity = self.velocity.min(0.0);
        }
        if self.position <= self.low {
            self.low_impact = self.low_impact.max(-self.velocity);
            self.low_torque = self.low_torque.max(-self.torque);
            self.position = self.low;
            self.velocity = self.velocity.max(0.0);
        }
    }
}

fn word(bytes: &[u8], index: usize) -> u16 {
    u16::from_be_bytes([bytes[index * 2], bytes[index * 2 + 1]])
}
fn activation(o: &Output) -> u8 {
    (o.status[0] >> 4) & 3
}
fn homing(o: &Output) -> u16 {
    (word(&o.advanced_status, 14) >> 4) & 3
}
fn detail(o: &Output) -> u16 {
    word(&o.status1000, 12)
}
fn angle_deg(o: &Output) -> f32 {
    ((u32::from(word(&o.status1000, 6)) << 16) | u32::from(word(&o.status1000, 7))) as i32 as f32
        / 1000.0
}
fn command(action: u8, position: u16) -> Command {
    Command {
        action,
        position,
        speed: 1000,
        force: 100,
    }
}

/// The frame the CAN task sends for one output of the bridge driving motor `id` (can.rs).
fn frame(id: u8, drive: Drive) -> cubemars::Packet {
    match drive {
        Drive::Can(packet) => packet,
        Drive::Torque(torque) => cubemars::torque_command(cubemars::AK40_10, id, torque).unwrap(),
        Drive::Disabled => cubemars::disable(id),
    }
}

/// Steps 1 ms at a time until `done` holds (or `limit` ms pass); returns the last output.
/// Every frame goes to the bridge's own motor ID, and never as MIT (the AK40-10 has none).
fn run(
    b: &mut Bridge,
    shaft: &mut Shaft,
    now: &mut u64,
    cmd: Command,
    settings: Settings,
    limit: u64,
    done: impl Fn(&Output) -> bool,
) -> Output {
    let end = *now + limit;
    loop {
        let o = b.step_with_settings(*now, cmd, Some(*now), Some(shaft.feedback(*now)), settings);
        let sent = frame(b.motor_id(), o.drive);
        assert_eq!(sent.id & 0xff, u32::from(b.motor_id()), "{sent:?}");
        assert_ne!(sent.id >> 8, 8, "MIT frame {sent:?}");
        shaft.apply(o.drive);
        *now += 1;
        if done(&o) || *now >= end {
            return o;
        }
    }
}
/// The gains the host should write (Kp 2.00 / Kd 0.100, as the AK60-6 grippers ran): the
/// boot Kp 1 lets friction hold the fingers twice as far off a target.
fn host_gains() -> Settings {
    Settings {
        kp_centi: 200,
        kd_milli: 100,
        ..Settings::default()
    }
}

/// 2 mm lead with positive rotation closing, so the walls below read naturally.
const TEST: Gripper = Gripper {
    close_sign: 1.0,
    ..GRIPPER_LEFT
};
/// Motor CAN ID 1, as on the dual-motor Left channel.
fn gripper() -> Bridge {
    Bridge::for_motor(MOTION, 1).with_gripper(TEST)
}

#[test]
fn computed_travel_follows_the_paper_geometry_and_each_side_has_its_lead() {
    // 50 mm per finger * 0.298 = 14.9 mm of nut travel: 7.46 turns with a 2 mm lead.
    assert!((config::NUT_PER_FINGER_MM - 0.2983).abs() < 1e-3);
    assert!((GRIPPER_LEFT.nut_travel_mm - 14.92).abs() < 0.01);
    assert!((GRIPPER_LEFT.travel_rad().to_degrees() - 2685.0).abs() < 1.0);
    // Right: 3 mm lead, opened 11.25 mm as set on it (900 deg on its AK60-6). The AK40-10
    // reports its true output angle: 1350 deg.
    let right_turn = 3.0;
    assert!((GRIPPER_RIGHT.travel_rad().to_degrees() - 1350.0).abs() < 0.01);
    assert!((GRIPPER_RIGHT.homing_speed_rad_s() - 6.70).abs() < 0.01);
    assert!((GRIPPER_RIGHT.nut_travel_mm - 11.25).abs() < 1e-4);
    // Both fit the +-3200 deg servo feedback range from the closed end at 0 deg.
    assert!(GRIPPER_LEFT.travel_rad() < config::FEEDBACK_POSITION_RAD);
    // The installed grippers open on positive rotation.
    let close = if cfg!(feature = "close-positive") {
        1.0
    } else {
        -1.0
    };
    assert_eq!(GRIPPER_LEFT.close_sign, close);
    assert_eq!(GRIPPER_RIGHT.close_sign, close);
    // Nut travel per reported turn: Left 2 mm, Right 3 mm.
    let per_turn: std::vec::Vec<f32> = GRIPPERS
        .iter()
        .map(|g| g.lead_mm * g.rotation_scale)
        .collect();
    let right = config::MODBUS_SLAVE_ID == 2;
    if cfg!(feature = "dual-motor") {
        assert_eq!(per_turn, [2.0, right_turn]);
    } else {
        assert_eq!(per_turn, [if right { right_turn } else { 2.0 }]);
    }
}

#[test]
fn activation_closes_onto_the_closed_end_then_opens_by_the_travel() {
    let travel = TEST.travel_rad();
    let mut shaft = Shaft::new(0.0, -100.0, 5.0);
    let mut b = gripper();
    let mut now = 0;
    let settings = Settings::default();
    let closing = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        200,
        |_| false,
    );
    assert_eq!(activation(&closing), 1);
    assert_eq!(homing(&closing), 1);
    assert!(shaft.velocity > 9.0, "closes at the homing speed");
    // The stall on the closed end starts the opening.
    let opening = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        3000,
        |o| homing(o) == 2,
    );
    assert_eq!(homing(&opening), 2);
    assert_eq!(activation(&opening), 1);
    assert!((shaft.position - 5.0).abs() < 0.01);
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        10_000,
        |o| activation(o) == 3,
    );
    assert_eq!(activation(&done), 3, "activation completes at the open end");
    assert_eq!(homing(&done), 3);
    assert!((shaft.position - (5.0 - travel)).abs() < 0.05);
    assert_eq!(done.status[2], 0);
    assert_eq!(done.status[4], 0, "gPO 0 = fully open");
    assert!(
        angle_deg(&done).abs() < 3.0,
        "angle is measured from the open end"
    );
    // Close 5 rad, open 60 % at 10 rad/s and the last 40 % at a third of it.
    assert!(now < 10_000, "homing took {now} ms");
    // 255 drives to just short of the closed stall, toward positive angles.
    let closed = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(9, 1024),
        settings,
        10_000,
        |o| o.status[4] == 255 && o.status[0] >> 6 == 3,
    );
    assert_eq!(closed.status[4], 255);
    assert!((shaft.position - (5.0 - TEST.rad(TEST.closed_margin_mm))).abs() < 0.02);
    let expected = (travel - TEST.rad(TEST.closed_margin_mm)).to_degrees();
    assert!((angle_deg(&closed) - expected).abs() < 1.5);
}

#[test]
fn reset_during_homing_stops_and_the_next_activation_homes_again() {
    let mut shaft = Shaft::new(0.0, -100.0, 5.0);
    let mut b = gripper();
    let mut now = 0;
    let settings = Settings::default();
    run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        2000,
        |o| homing(o) == 2,
    );
    let off = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(0, 0),
        settings,
        1,
        |_| true,
    );
    assert_eq!(off.drive, Drive::Disabled);
    assert_eq!(activation(&off), 0);
    assert_eq!(homing(&off), 0);
    let again = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        10,
        |_| false,
    );
    assert_eq!(homing(&again), 1, "a new activation starts by closing");
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        12_000,
        |o| activation(o) == 3,
    );
    assert_eq!(activation(&done), 3);
    assert!((shaft.position - (5.0 - TEST.travel_rad())).abs() < 0.05);
}

#[test]
fn an_opening_stall_past_half_the_travel_becomes_the_open_end() {
    let mut shaft = Shaft::new(0.0, -25.0, 5.0);
    let mut b = gripper();
    let mut now = 0;
    let settings = Settings::default();
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        12_000,
        |o| activation(o) == 3 || o.status[2] != 0,
    );
    assert_eq!(activation(&done), 3);
    // Fully open stays 0.5 mm of nut travel off the stall.
    assert!((shaft.position - (-25.0 + TEST.rad(TEST.open_margin_mm))).abs() < 0.02);
    assert_eq!(done.status[4], 0);
}

#[test]
fn homing_without_an_end_or_with_a_short_stroke_trips() {
    let settings = Settings::default();
    // Nothing stops the closing: it gives up before the feedback range ends.
    let mut shaft = Shaft::new(0.0, -100.0, 100.0);
    let mut b = gripper();
    let mut now = 0;
    let o = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        12_000,
        |o| o.status[2] != 0,
    );
    assert_eq!(
        (o.status[2], detail(&o)),
        (0x0d, FaultDetail::HomingNoEnd as u16)
    );
    assert_eq!(o.drive, Drive::Disabled);
    // Twice the travel, zeroing on the way so the feedback never leaves its range.
    assert!(shaft.position > 1.95 * TEST.travel_rad());
    assert!(shaft.reported().abs() < config::FEEDBACK_POSITION_RAD);
    // An object between the fingers stops the opening early.
    let mut shaft = Shaft::new(0.0, -10.0, 5.0);
    let mut b = gripper();
    let mut now = 0;
    let o = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        12_000,
        |o| o.status[2] != 0,
    );
    assert_eq!(
        (o.status[2], detail(&o)),
        (0x0d, FaultDetail::HomingShortStroke as u16)
    );
    assert_eq!(activation(&o), 0);
    // The homing timeout bounds a shaft that never settles.
    let mut shaft = Shaft::new(0.0, -100.0, 5.0);
    let mut b = Bridge::new(MOTION).with_gripper(Gripper {
        homing_timeout_ms: 1000,
        ..TEST
    });
    let mut now = 0;
    let o = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        12_000,
        |o| o.status[2] != 0,
    );
    assert_eq!(
        (o.status[2], detail(&o)),
        (0x0d, FaultDetail::HomingNoEnd as u16)
    );
    assert!((1000..1100).contains(&now));
}

#[test]
fn negative_closing_rotation_maps_255_to_the_negative_end() {
    let mut shaft = Shaft::new(0.0, -5.0, 100.0);
    let mut b = Bridge::new(MOTION).with_gripper(Gripper {
        close_sign: -1.0,
        ..TEST
    });
    let mut now = 0;
    let settings = Settings::default();
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        12_000,
        |o| activation(o) == 3,
    );
    assert_eq!(activation(&done), 3);
    assert!((shaft.position - (-5.0 + TEST.travel_rad())).abs() < 0.05);
    assert_eq!(done.status[4], 0);
    let closed = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(9, 1024),
        settings,
        10_000,
        |o| o.status[0] >> 6 == 3,
    );
    assert!((shaft.position - (-5.0 + TEST.rad(TEST.closed_margin_mm))).abs() < 0.02);
    assert_eq!(closed.status[4], 255);
    assert!(
        angle_deg(&closed) > 2000.0,
        "the angle from open grows toward closed"
    );
}

#[test]
fn a_go_held_through_activation_is_refused_until_cleared_and_csp_cst_can_release() {
    let settings = Settings {
        mode: Mode::Hybrid,
        ..Settings::default()
    };
    let mut shaft = Shaft::new(0.0, -100.0, 5.0);
    let mut b = gripper();
    let mut now = 0;
    // Go set together with rACT: homing runs as usual and only rACT acts meanwhile.
    let homing = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(9, 512),
        settings,
        200,
        |_| false,
    );
    assert_eq!(
        (activation(&homing), homing.status[0] & 8),
        (1, 0),
        "gGTO 0"
    );
    assert_eq!(
        word(&homing.advanced_status, 14) >> 7 & 1,
        1,
        "action refused"
    );
    let homed = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(9, 512),
        settings,
        12_000,
        |o| activation(o) == 3,
    );
    assert_eq!(activation(&homed), 3);
    let open = shaft.position;
    // Still held after homing: no move, the torque stays off fully open.
    let held = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(9, 512),
        settings,
        300,
        |_| false,
    );
    assert_eq!(held.drive, Drive::Torque(0.0));
    assert_eq!(held.status[0] & 8, 0, "gGTO 0 while refused");
    assert_eq!(
        word(&held.advanced_status, 14) >> 6 & 3,
        3,
        "rest off, refused"
    );
    assert!((shaft.position - open).abs() < 0.01);
    // Clearing go releases the lock; a new go moves from there.
    run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 512),
        settings,
        20,
        |_| false,
    );
    let moving = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(9, 512),
        settings,
        300,
        |_| false,
    );
    assert_eq!(
        word(&moving.advanced_status, 14) >> 1 & 3,
        1,
        "CSP phase started"
    );
    assert_eq!(word(&moving.advanced_status, 14) >> 6 & 3, 0);
    assert!(shaft.position > open + 1.0);
    // Auto release is allowed in CSP+CST and crosses the travel at the homing speed.
    let released = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(0x11, 512),
        settings,
        10_000,
        |o| o.status[2] != 0x0b,
    );
    assert_eq!(
        released.status[2], 0x0f,
        "release completed at the closed end"
    );
    assert!((shaft.position - (5.0 - TEST.rad(TEST.closed_margin_mm))).abs() < 0.02);
}

#[test]
fn a_bare_motor_still_activates_on_the_first_feedback() {
    let mut b = Bridge::new(MOTION);
    let mut shaft = Shaft::new(1.0, -100.0, 100.0);
    let mut now = 0;
    let o = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        Settings::default(),
        3,
        |_| false,
    );
    assert_eq!(activation(&o), 3);
    assert_eq!(homing(&o), 0);
    assert!(
        (angle_deg(&o) - 1.0f32.to_degrees()).abs() < 0.01,
        "raw angle"
    );
}

#[test]
fn reactivating_far_from_zero_zeroes_on_the_way_and_homes_closed_at_zero() {
    // The field case: the motor already reads 2899 deg (it kept its count after a failed
    // homing) and the closed end lies 3151 deg away, beyond the feedback range.
    let gripper = Gripper {
        close_sign: -1.0,
        ..GRIPPER_LEFT
    };
    let mut shaft = Shaft::new(0.0, -55.0, 100.0);
    shaft.origin = -2899f32.to_radians();
    let mut b = Bridge::for_motor(MOTION, 1).with_gripper(gripper);
    let mut now = 0;
    let settings = Settings::default();
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        20_000,
        |o| activation(o) == 3 || o.status[2] != 0,
    );
    assert_eq!((done.status[2], activation(&done)), (0, 3));
    // The last origin is the closed end; fully open is +travel from it.
    assert_eq!(shaft.origin, -55.0);
    assert!((shaft.reported() - gripper.travel_rad()).abs() < 0.05);
    assert_eq!(done.status[4], 0);
    assert!(angle_deg(&done).abs() < 3.0);
    let closed = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(9, 1024),
        settings,
        10_000,
        |o| o.status[0] >> 6 == 3,
    );
    assert_eq!(closed.status[4], 255);
    assert!((shaft.reported() - gripper.rad(gripper.closed_margin_mm)).abs() < 0.02);
}

#[test]
fn a_motor_that_ignores_the_origin_command_trips_before_moving() {
    let mut shaft = Shaft::new(2899f32.to_radians(), -100.0, 100.0);
    shaft.deaf = true;
    let mut b = gripper();
    let mut now = 0;
    let o = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        Settings::default(),
        2000,
        |o| o.status[2] != 0,
    );
    assert_eq!(
        (o.status[2], detail(&o)),
        (0x0d, FaultDetail::OriginNotSet as u16)
    );
    assert!(now < 250);
    assert!((shaft.position - 2899f32.to_radians()).abs() < 1e-3);
}

#[test]
fn right_homes_at_left_finger_speed_and_meets_a_near_open_end_at_half_speed() {
    // Right's nut moves 3 mm per AK40-10 output turn, so 6.70 rad/s gives Left's 3.2 mm/s.
    let gripper = Gripper {
        close_sign: 1.0,
        ..GRIPPER_RIGHT
    };
    let speed = gripper.homing_speed_rad_s();
    assert!((speed - 6.70).abs() < 0.01);
    assert!((GRIPPER_LEFT.homing_speed_rad_s() - 10.05).abs() < 0.01);
    // An open end at 90 % of the computed travel, inside the slow approach.
    let open_end = 5.0 - 0.9 * gripper.travel_rad();
    let mut shaft = Shaft::new(0.0, open_end, 5.0);
    let mut b = Bridge::for_motor(MOTION, 2).with_gripper(gripper);
    let mut now = 0;
    let settings = host_gains();
    run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        300,
        |_| false,
    );
    assert!(
        (shaft.velocity - speed).abs() < 0.2,
        "closing at {}",
        shaft.velocity
    );
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        20_000,
        |o| activation(o) == 3 || o.status[2] != 0,
    );
    assert_eq!((done.status[2], activation(&done)), (0, 3));
    // Met at about half the homing speed, with no more than the homing torque.
    assert!(
        shaft.low_impact <= 0.6 * speed,
        "impact {}",
        shaft.low_impact
    );
    assert!(
        shaft.low_torque <= 0.3 + 1e-4,
        "pushed {}",
        shaft.low_torque
    );
    assert!((shaft.position - (open_end + gripper.rad(gripper.open_margin_mm))).abs() < 0.02);
    assert_eq!(done.status[4], 0);
}

#[test]
fn the_approach_keeps_full_torque_so_friction_does_not_end_the_opening_early() {
    // 0.2 N*m of friction stopped Left in the half-torque approach; it now opens fully.
    let mut shaft = Shaft::new(0.0, -100.0, 5.0);
    shaft.friction = 0.2;
    let mut b = gripper();
    let mut now = 0;
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        host_gains(),
        20_000,
        |o| activation(o) == 3 || o.status[2] != 0,
    );
    assert_eq!((done.status[2], activation(&done)), (0, 3));
    assert!((shaft.position - (5.0 - TEST.travel_rad())).abs() < 0.1);
    // The previous half-torque approach stalls on that friction and opens short.
    let mut shaft = Shaft::new(0.0, -100.0, 5.0);
    shaft.friction = 0.2;
    let mut b = Bridge::for_motor(MOTION, 1).with_gripper(Gripper {
        approach_from: 0.6,
        approach_speed: 1.0 / 3.0,
        approach_torque: 0.5,
        ..TEST
    });
    let mut now = 0;
    run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        host_gains(),
        20_000,
        |o| activation(o) == 3 || o.status[2] != 0,
    );
    assert!(
        shaft.position > 5.0 - 0.7 * TEST.travel_rad(),
        "opened to {}",
        shaft.position
    );
}

#[test]
fn a_yielding_object_switches_to_cst_near_the_threshold_and_cst_holds_it() {
    // Right on 2026-10-30: closing a yielding object it stayed at 0.4 N*m and kept
    // moving, so CSP never handed over. Threshold 0.3, CST 0.25 N*m, Force 0.954 N*m,
    // speed 98 (2.39 rad/s).
    let settings = Settings {
        mode: Mode::Hybrid,
        threshold_mnm: 300,
        torque_mnm: 250,
        dwell_ms: 60,
        ..Settings::default()
    };
    let mut shaft = Shaft::new(0.0, -100.0, 5.0);
    shaft.friction = 0.05;
    let mut b = gripper();
    let mut now = 0;
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        20_000,
        |o| activation(o) == 3 || o.status[2] != 0,
    );
    assert_eq!((done.status[2], activation(&done)), (0, 3));
    // The object sits 30 % of the way closed and resists 0.05 N*m per rad pressed in.
    let contact = shaft.position + 0.3 * TEST.travel_rad();
    shaft.spring = Some((contact, 0.05));
    let close = Command {
        action: 9,
        position: 1024,
        speed: 98,
        force: 106,
    };
    let phase = |o: &Output| (word(&o.advanced_status, 14) >> 1) & 3;
    let held = run(&mut b, &mut shaft, &mut now, close, settings, 20_000, |o| {
        phase(o) == 2
    });
    assert_eq!(phase(&held), 2, "CST took over");
    // At the 0.3 N*m threshold (object plus friction: about 5 rad in), long before the
    // Force limit the old rule waited for (about 18 rad in).
    let pressed = shaft.position - contact;
    assert!(
        (4.0..7.0).contains(&pressed),
        "switched {pressed} rad into the object"
    );
    assert_eq!(held.status[0] >> 6, 2, "object while closing");
    // CST's 0.25 N*m and friction balance the object: the fingers stop.
    let out = run(&mut b, &mut shaft, &mut now, close, settings, 2000, |_| {
        false
    });
    assert_eq!(phase(&out), 2);
    assert!(
        shaft.velocity.abs() < 0.05,
        "still moving at {}",
        shaft.velocity
    );
    assert!((shaft.position - contact - pressed).abs() < 1.5);
}

#[test]
fn an_object_that_falls_out_ends_the_grip_at_the_target_at_the_csp_speed() {
    // CSP+CST keeps the position loop through the grip: without the object the fingers
    // close on only at the CSP speed and only to the target, where CSP holds again (the
    // former pure CST sped up into the travel end and tripped 0x0D detail 15). With the gains
    // the host should write: the boot Kp 1 leaves friction / Kp = 0.05 rad short of the
    // target, outside the 0.005 rad tolerance of an uncalibrated arrival.
    let settings = Settings {
        mode: Mode::Hybrid,
        threshold_mnm: 300,
        torque_mnm: 250,
        dwell_ms: 60,
        ..host_gains()
    };
    let mut shaft = Shaft::new(0.0, -100.0, 5.0);
    shaft.friction = 0.05;
    let mut b = gripper();
    let mut now = 0;
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        20_000,
        |o| activation(o) == 3 || o.status[2] != 0,
    );
    assert_eq!((done.status[2], activation(&done)), (0, 3));
    shaft.spring = Some((shaft.position + 0.3 * TEST.travel_rad(), 0.05));
    // Speed 98: 3.80 rad/s on the AK40-10.
    let speed = cubemars::AK40_10_PROFILE
        .rated_speed_rad_s()
        .min(MOTION.max_speed_rad_s)
        * 98.0
        / 1000.0;
    let close = Command {
        action: 9,
        position: 1024,
        speed: 98,
        force: 106,
    };
    let phase = |o: &Output| (word(&o.advanced_status, 14) >> 1) & 3;
    let held = run(&mut b, &mut shaft, &mut now, close, settings, 20_000, |o| {
        phase(o) == 2
    });
    assert_eq!(phase(&held), 2);
    let out = run(&mut b, &mut shaft, &mut now, close, settings, 1000, |_| {
        false
    });
    assert_eq!((phase(&out), out.status[0] >> 6), (2, 2), "still gripping");
    assert!(shaft.velocity.abs() < 0.05);
    shaft.spring = None;
    let dropped = shaft.position;
    let (mut catch_up, mut cruise) = (0.0f32, 0.0f32);
    let mut out = out;
    for _ in 0..30_000 {
        out = run(&mut b, &mut shaft, &mut now, close, settings, 1, |_| true);
        let peak = if shaft.position - dropped < 2.0 {
            &mut catch_up
        } else {
            &mut cruise
        };
        *peak = peak.max(shaft.velocity.abs());
        if phase(&out) != 2 || out.status[2] != 0 {
            break;
        }
    }
    assert_eq!(
        (out.status[2], phase(&out)),
        (0, 1),
        "CSP took the target back"
    );
    // Only the CST lag (1.1 * 0.25 / kp 2 = 0.14 rad) is caught up faster,
    // ringing out within 2 rad; then the ramp.
    assert!(catch_up < 2.0 * speed, "caught up at {catch_up} rad/s");
    assert!(cruise < 1.1 * speed, "ran at {cruise} rad/s");
    let out = run(&mut b, &mut shaft, &mut now, close, settings, 2000, |o| {
        o.status[2] != 0
    });
    assert_eq!((out.status[2], phase(&out)), (0, 1));
    // CSP holds the target (gOBJ 3).
    assert_eq!(out.status[0] >> 6, 3);
    assert_eq!(out.status[4], 255);
    assert!(
        shaft.position < shaft.high,
        "stopped short of the closed end"
    );
}

#[test]
fn activation_closes_on_the_csp_ramp_and_takes_100_percent_after_a_100_ms_stall() {
    let mut shaft = Shaft::new(0.0, -100.0, 5.0);
    shaft.friction = 0.05;
    let mut b = gripper();
    let mut now = 0;
    let settings = Settings::default();
    let (mut touched, mut opening) = (None, None);
    for _ in 0..12_000 {
        let o = run(
            &mut b,
            &mut shaft,
            &mut now,
            command(1, 0),
            settings,
            1,
            |_| true,
        );
        if homing(&o) == 1 && touched.is_none() {
            // The position loop, within the homing torque either way.
            if let Drive::Torque(t) = o.drive {
                assert!(t.abs() <= TEST.homing_torque_nm + 1e-6, "{t}");
            }
        }
        if touched.is_none() && shaft.position >= shaft.high {
            touched = Some(now);
        }
        if homing(&o) == 2 {
            opening = Some(now);
            break;
        }
    }
    // A 100 ms stall on the closed end is taken as 100 %.
    let held = opening.unwrap() - touched.unwrap();
    assert!(
        (TEST.homing_dwell_ms..TEST.homing_dwell_ms + 50).contains(&held),
        "held {held} ms"
    );
    // Then CSP opens fully (0 %), where activation ends with the torque off.
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        20_000,
        |o| activation(o) == 3 || o.status[2] != 0,
    );
    assert_eq!(
        (done.status[2], activation(&done), done.status[4]),
        (0, 3, 0)
    );
    assert_eq!(done.drive, Drive::Torque(0.0));
    assert_eq!(word(&done.advanced_status, 14) >> 6 & 1, 1);
}

#[test]
fn an_arrived_move_rests_with_the_torque_off_but_a_dwelled_grip_keeps_pushing() {
    let settings = Settings {
        mode: Mode::Hybrid,
        threshold_mnm: 300,
        torque_mnm: 250,
        dwell_ms: 60,
        ..Settings::default()
    };
    let mut shaft = Shaft::new(0.0, -100.0, 5.0);
    shaft.friction = 0.1;
    let mut b = gripper();
    let mut now = 0;
    let done = run(
        &mut b,
        &mut shaft,
        &mut now,
        command(1, 0),
        settings,
        20_000,
        |o| activation(o) == 3 || o.status[2] != 0,
    );
    assert_eq!((done.status[2], activation(&done)), (0, 3));
    let close = Command {
        action: 9,
        position: 1024,
        speed: 98,
        force: 106,
    };
    let stop = Command { action: 1, ..close };
    let open = Command {
        position: 0,
        ..close
    };
    let rest = |o: &Output| word(&o.advanced_status, 14) >> 6 & 1;
    // Below the torque threshold all the way: arrival (gOBJ 3) frees the next action, even
    // where friction / kp (0.05 rad) keeps the fingers outside the strict 0.005 rad, and
    // the torque goes off there.
    let arrived = run(&mut b, &mut shaft, &mut now, close, settings, 20_000, |o| {
        o.status[0] >> 6 == 3 || o.status[2] != 0
    });
    assert_eq!((arrived.status[2], arrived.status[0] >> 6), (0, 3));
    assert_eq!((arrived.drive, rest(&arrived)), (Drive::Torque(0.0), 1));
    let short = 5.0 - TEST.rad(TEST.closed_margin_mm) - shaft.position;
    assert!(
        (0.005..TEST.travel_rad() / 512.0).contains(&short),
        "{short}"
    );
    assert_eq!(arrived.status[4], 255);
    // Over the threshold: gOBJ 2 only once it has lasted the dwell.
    run(&mut b, &mut shaft, &mut now, stop, settings, 20, |_| false);
    run(&mut b, &mut shaft, &mut now, open, settings, 20_000, |o| {
        o.status[0] >> 6 == 3
    });
    run(&mut b, &mut shaft, &mut now, stop, settings, 20, |_| false);
    shaft.spring = Some((shaft.position + 0.3 * TEST.travel_rad(), 0.2));
    let (mut over, mut gripped) = (None, None);
    for _ in 0..20_000 {
        let o = run(&mut b, &mut shaft, &mut now, close, settings, 1, |_| true);
        if over.is_none() && shaft.torque.abs() >= 0.3 {
            over = Some(now);
        }
        if o.status[0] >> 6 == 2 {
            gripped = Some(now);
            break;
        }
    }
    let waited = gripped.unwrap() - over.unwrap();
    assert!(
        (60..200).contains(&waited),
        "gripped {waited} ms after the threshold"
    );
    // The grip keeps pushing with the CST torque (0.25 N*m), never with the torque off.
    let grip = run(&mut b, &mut shaft, &mut now, close, settings, 1000, |_| {
        false
    });
    assert_eq!((grip.status[0] >> 6, rest(&grip)), (2, 0));
    assert!(
        matches!(grip.drive, Drive::Torque(t) if (t - 0.25).abs() < 0.01),
        "{:?}",
        grip.drive
    );
    // Stop, then open fully: the torque goes off there and the gripper stays activated.
    run(&mut b, &mut shaft, &mut now, stop, settings, 20, |_| false);
    let opened = run(&mut b, &mut shaft, &mut now, open, settings, 20_000, |o| {
        o.status[0] >> 6 == 3 || o.status[2] != 0
    });
    assert_eq!(
        (opened.status[2], opened.status[4], activation(&opened)),
        (0, 0, 3)
    );
    assert_eq!((opened.drive, rest(&opened)), (Drive::Torque(0.0), 1));
    // It stays off while open is held, and through a stop.
    let held = run(&mut b, &mut shaft, &mut now, open, settings, 500, |_| false);
    assert_eq!((held.drive, held.status[0] >> 6), (Drive::Torque(0.0), 3));
    let stopped = run(&mut b, &mut shaft, &mut now, stop, settings, 100, |_| false);
    assert_eq!((stopped.drive, rest(&stopped)), (Drive::Torque(0.0), 1));
    assert_eq!((activation(&stopped), homing(&stopped)), (3, 3));
    // A partial close to the middle rests with the torque off there too.
    shaft.spring = None;
    let middle = Command {
        position: 512,
        ..close
    };
    let half = run(
        &mut b,
        &mut shaft,
        &mut now,
        middle,
        settings,
        20_000,
        |o| o.status[0] >> 6 == 3 || o.status[2] != 0,
    );
    assert_eq!(
        (half.status[2], half.status[0] >> 6, rest(&half)),
        (0, 3, 1)
    );
    assert_eq!(half.drive, Drive::Torque(0.0));
    assert!((126..=130).contains(&half.status[4]), "{}", half.status[4]);
    // The next close starts at once, without homing again.
    let moving = run(&mut b, &mut shaft, &mut now, close, settings, 300, |_| {
        false
    });
    assert_eq!((rest(&moving), homing(&moving)), (0, 3));
    assert!(matches!(moving.drive, Drive::Torque(t) if t > 0.0));
    let closed = run(&mut b, &mut shaft, &mut now, close, settings, 20_000, |o| {
        o.status[0] >> 6 == 3 || o.status[2] != 0
    });
    assert_eq!(
        (closed.status[2], closed.status[0] >> 6, closed.status[4]),
        (0, 3, 255)
    );
    assert_eq!((closed.drive, rest(&closed)), (Drive::Torque(0.0), 1));
}

/// AK40-10 CSP+CST with the gains the host should write (Kp 2.00 / Kd 0.100, as the AK60-6
/// grippers ran). The ak40-10 boot gains (Kp 1.00) double the friction / Kp a
/// stopped move keeps from its target: with 0.05 N*m of friction Right (3 mm lead, half a
/// step = 0.046 rad) then stops 0.05 rad off and never reports arrival.
fn ak40_10_settings() -> Settings {
    Settings {
        mode: Mode::Hybrid,
        kp_centi: 200,
        kd_milli: 100,
        threshold_mnm: 300,
        torque_mnm: 250,
        dwell_ms: 60,
        ..Settings::default()
    }
}

/// Left and Right as the dual-motor board drives them (slave 1 -> CAN ID 1, slave 2 -> CAN
/// ID 2), Right reporting the AK40-10 true output angle, both closing on positive rotation
/// so the walls below read naturally.
fn ak40_10_dual() -> [(u8, Gripper); 2] {
    let left = Gripper {
        close_sign: 1.0,
        ..GRIPPER_LEFT
    };
    let right = Gripper {
        close_sign: 1.0,
        ..GRIPPER_RIGHT
    };
    [
        (config::DUAL_MOTOR[0].motor_id, left),
        (config::DUAL_MOTOR[1].motor_id, right),
    ]
}

/// Activates (homes) one AK40-10 gripper; returns it fully open with the torque off.
fn ak40_10_activated(id: u8, gripper: Gripper, shaft: &mut Shaft, now: &mut u64) -> Bridge {
    let mut b = Bridge::for_motor(MOTION, id).with_gripper(gripper);
    let done = run(
        &mut b,
        shaft,
        now,
        command(1, 0),
        ak40_10_settings(),
        30_000,
        |o| activation(o) == 3 || o.status[2] != 0,
    );
    assert_eq!((done.status[2], activation(&done)), (0, 3), "ID {id}");
    assert_eq!(
        done.drive,
        Drive::Torque(0.0),
        "ID {id} rests open, torque off"
    );
    assert_eq!(done.status[4], 0, "ID {id} gPO 0 = fully open");
    assert_eq!(word(&done.status1000, 14), u16::from(id), "status CAN ID");
    // Friction / Kp may keep the fingers that much short of the computed open end.
    let open = 5.0 - gripper.travel_rad();
    assert!(
        (shaft.position - open).abs() < 0.1,
        "ID {id} at {}",
        shaft.position
    );
    b
}

#[test]
fn ak40_10_dual_motor_left_id1_and_right_id2_activate_position_speed_and_force() {
    assert_eq!(
        config::DUAL_MOTOR.map(|c| (c.slave, c.motor_id)),
        [(1, 1), (2, 2)]
    );
    let settings = ak40_10_settings();
    let rated = cubemars::AK40_10_PROFILE.rated_speed_rad_s();
    for (id, gripper) in ak40_10_dual() {
        let mut shaft = Shaft::new(0.0, -100.0, 5.0);
        shaft.friction = 0.05;
        let mut now = 0;
        let mut b = ak40_10_activated(id, gripper, &mut shaft, &mut now);
        // Position 128 (rPR, x 4 = 512) at speed 500 = half the AK40-10 rated 370 RPM.
        let half = Command {
            action: 9,
            position: 512,
            speed: 500,
            force: 106,
        };
        let mut peak: f32 = 0.0;
        let mut arrived = None;
        for _ in 0..20_000 {
            let o = run(&mut b, &mut shaft, &mut now, half, settings, 1, |_| true);
            peak = peak.max(shaft.velocity.abs());
            if o.status[0] >> 6 == 3 || o.status[2] != 0 {
                arrived = Some(o);
                break;
            }
        }
        let arrived = arrived.expect("move to 128 finished");
        assert_eq!(
            (arrived.status[2], arrived.status[0] >> 6),
            (0, 3),
            "ID {id}"
        );
        assert_eq!(arrived.status[4], 128, "ID {id} gPO");
        assert_eq!(
            arrived.drive,
            Drive::Torque(0.0),
            "ID {id} rests, torque off"
        );
        // A little overshoot while the loop catches up its following lag; AK60-6's 233 RPM
        // scale would cruise at 12.2 rad/s.
        assert!(
            (peak - 0.5 * rated).abs() < 0.15 * 0.5 * rated,
            "ID {id} peak {peak} rad/s, commanded {}",
            0.5 * rated
        );
        // Force 106 = 0.954 N*m torque limit. An object 40 % further in: CSP never pushes
        // past that limit, CST (0.25 N*m) takes over and holds it (gOBJ 2).
        let stop = Command { action: 1, ..half };
        run(&mut b, &mut shaft, &mut now, stop, settings, 20, |_| false);
        shaft.spring = Some((shaft.position + 0.2 * gripper.travel_rad(), 0.2));
        let close = Command {
            position: 1024,
            speed: 98,
            ..half
        };
        let limit = 9.0 * 106.0 / 1000.0;
        let mut pushed: f32 = 0.0;
        let mut gripped = None;
        for _ in 0..20_000 {
            let o = run(&mut b, &mut shaft, &mut now, close, settings, 1, |_| true);
            pushed = pushed.max(shaft.torque.abs());
            if o.status[0] >> 6 == 2 || o.status[2] != 0 {
                gripped = Some(o);
                break;
            }
        }
        let gripped = gripped.expect("closed on the object");
        assert_eq!(
            (gripped.status[2], gripped.status[0] >> 6),
            (0, 2),
            "ID {id}"
        );
        assert!(pushed <= limit + 1e-4, "ID {id} pushed {pushed} N*m");
        let held = run(&mut b, &mut shaft, &mut now, close, settings, 1000, |_| {
            false
        });
        assert_eq!(held.status[0] >> 6, 2, "ID {id} still holding");
        assert!(
            matches!(held.drive, Drive::Torque(t) if (t - 0.25).abs() < 0.01),
            "ID {id} {:?}",
            held.drive
        );
        // Open fully again: arrives at 0 and rests with the torque off, still activated.
        run(&mut b, &mut shaft, &mut now, stop, settings, 20, |_| false);
        shaft.spring = None;
        let open = Command {
            position: 0,
            ..half
        };
        let opened = run(&mut b, &mut shaft, &mut now, open, settings, 20_000, |o| {
            o.status[0] >> 6 == 3 || o.status[2] != 0
        });
        assert_eq!((opened.status[2], opened.status[0] >> 6), (0, 3), "ID {id}");
        assert_eq!((opened.status[4], activation(&opened)), (0, 3), "ID {id}");
        assert_eq!(opened.drive, Drive::Torque(0.0));
    }
}

#[test]
fn ak40_10_force_1000_is_capped_at_its_4_1_nm_peak_on_both_motors() {
    // Force 1000 asks for 9 N*m; an AK40-10 never gets more than its 4.1 N*m peak, and the
    // current frame it gets says so (servo current, mode 1, at that motor's ID).
    let settings = Settings {
        threshold_mnm: 4000,
        dwell_ms: 1000,
        ..ak40_10_settings()
    };
    let peak = cubemars::AK40_10_PROFILE.max_torque;
    assert_eq!(peak, 4.1);
    for (id, gripper) in ak40_10_dual() {
        let mut shaft = Shaft::new(0.0, -100.0, 5.0);
        let mut now = 0;
        let mut b = ak40_10_activated(id, gripper, &mut shaft, &mut now);
        // A rigid object 30 % in: the shaft's hard end.
        shaft.high = shaft.position + 0.3 * gripper.travel_rad();
        let close = Command {
            action: 9,
            position: 1024,
            speed: 500,
            force: 1000,
        };
        let mut pushed: f32 = 0.0;
        for _ in 0..3000 {
            let o = run(&mut b, &mut shaft, &mut now, close, settings, 1, |_| true);
            assert_eq!(o.status[2], 0, "ID {id}");
            pushed = pushed.max(shaft.torque.abs());
        }
        assert!((pushed - peak).abs() < 1e-4, "ID {id} pushed {pushed} N*m");
        let sent = frame(id, Drive::Torque(9.0));
        assert_eq!(sent.id, 0x100 | u32::from(id));
        let milliamps = i32::from_be_bytes(sent.data[..4].try_into().unwrap());
        assert_eq!(
            milliamps,
            (peak / cubemars::AK40_10_PROFILE.kt * 1000.0) as i32
        );
    }
}
