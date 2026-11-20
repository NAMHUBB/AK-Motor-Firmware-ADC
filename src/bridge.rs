//! Hardware-independent motion state machine; caller supplies monotonic milliseconds.
use crate::{
    advanced::{self, Mode, Settings},
    config::{self, Gripper, MotionConfig},
    control::{self, Command},
    cubemars::{self, Feedback},
    extended,
};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(u16)]
pub enum FaultDetail {
    #[default]
    None = 0,
    HostTimeout = 1,
    ActivationFeedbackTimeout = 2,
    FeedbackTimeout = 3,
    CanBus = 4,
    TickLate = 5,
    TxQueue = 6,
    InvalidConfig = 7,
    Motor = 8,
    InvalidFeedback = 9,
    ReleaseTimeout = 10,
    TorqueLimit = 11,
    ModeChange = 12,
    // 13 was MitRange: MIT mode, removed with the AK80-9 / AK60-6 profiles.
    // 14 was a CST overspeed trip and 15 a CST stop at a travel end. CSP+CST keeps the
    // position loop through the grip, so the shaft never outruns the CSP ramp or target.
    /// Activation homing found no closed end: no stall within HOMING_MAX_CLOSE x the
    /// travel, or the homing timeout passed.
    HomingNoEnd = 16,
    /// The homing opening stalled before half the travel (object between the fingers?).
    HomingShortStroke = 17,
    /// The motor's feedback never showed the temporary origin homing set.
    OriginNotSet = 18,
}

/// Activation homing (Robotiq-style auto-calibration, see config::Gripper).
#[derive(Clone, Copy, Debug, PartialEq)]
enum Homing {
    /// Closing at the homing speed until the shaft stalls on the closed end.
    Closing,
    /// Opening from the closed stall `closed` to `target`: the computed open end, or just
    /// short of an earlier stall once `blocked`.
    Opening {
        closed: f32,
        target: f32,
        blocked: bool,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Drive {
    Disabled,
    Torque(f32),
    Can(crate::cubemars::Packet),
}

/// A CSP+CST stop with the motor's own current brake at the torque limit. The driver
/// closes that loop itself and holds once stopped, so a late feedback sample never
/// drives the shaft back the other way.
#[derive(Clone, Copy, Debug)]
struct Brake {
    /// Sign of the motion being braked.
    direction: f32,
}

/// Reference lead over the measured shaft, as a multiple of the error that saturates
/// the position loop. Beyond it a held shaft only banks error, which the loop would
/// close at the torque limit (far above the commanded speed) once the load lets go.
const FOLLOWING_LAG_MARGIN: f32 = 1.1;

/// Output speed at or below which a shaft counts as stopped: the floor of the CSP hold
/// speed below, and the rest speed of the travel guard and homing.
const HYBRID_STOPPED_SPEED_RAD_S: f32 = 0.1;

/// CSP counts a saturated shaft as held at no more than this share of its commanded
/// speed. CSP also saturates while it accelerates a free load, so torque alone must not
/// switch to CST, but a free load passes this speed within a few ms of the dwell. A grip
/// on a yielding object keeps creeping well below it: with a fixed 0.1 rad/s the Right
/// gripper (2026-10-29) crept past it, never reached CST and CSP kept squeezing.
const HYBRID_HELD_SPEED_SHARE: f32 = 0.25;

/// Once a CSP move has run at this share of its commanded speed it is under way, and a
/// torque at the threshold is contact even while the fingers still close: on a yielding
/// object Right (2026-10-30) kept closing at a quarter of its speed or more, so only a
/// held shaft would never switch. A free load accelerating from rest reaches this speed
/// before its torque leaves saturation, which the stopped rule still covers.
const HYBRID_CRUISE_SPEED_SHARE: f32 = 0.8;

/// Homing closes at most this many times the gripper travel before it gives up on an end.
const HOMING_MAX_CLOSE: f32 = 2.0;
/// A homed gripper's move has arrived within half a Robotiq step of its target (gPO then
/// reads the requested rPR): friction / Kp can hold the fingers farther short than the
/// strict position tolerance, which never reported arrival.
const ARRIVAL_STEPS: f32 = 2.0 * control::POSITION_STEPS as f32;
/// The first sample this long after the origin command that reads within the tolerance
/// of zero confirms it; none within the timeout trips.
const ORIGIN_SETTLE_MS: u64 = 5;
const ORIGIN_TOLERANCE_RAD: f32 = 0.5;
const ORIGIN_TIMEOUT_MS: u64 = 200;

/// Feedback and current-loop delay covered when braking for a travel end.
const TRAVEL_BRAKE_LEAD_S: f32 = 0.01;

/// Keep a ramped reference within the saturating lag of the measured shaft. A held
/// shaft still sees the saturated output, so holding torque, contact and CST
/// detection are unchanged; once released it resumes the ramp from where it is.
fn follow(setpoint: f32, position: f32, torque_limit: f32, kp: f32) -> f32 {
    if kp <= 0.0 {
        return setpoint;
    }
    let lag = FOLLOWING_LAG_MARGIN * torque_limit / kp;
    setpoint.clamp(position - lag, position + lag)
}
#[derive(Clone, Copy, Debug)]
pub struct Output {
    pub drive: Drive,
    pub status: [u8; 16],
    pub status1000: [u8; extended::STATUS_BYTES],
    pub advanced_status: [u8; advanced::STATUS_BYTES],
}

pub struct Bridge {
    settings: Settings,
    cfg: MotionConfig,
    previous: Command,
    activated: bool,
    activation_started: Option<u64>,
    fault: u8,
    fault_detail: FaultDetail,
    setpoint: f32,
    contact_since: Option<u64>,
    object: u8,
    contact_direction: f32,
    release_started: Option<u64>,
    release_open: bool,
    previous_tick: Option<u64>,
    hybrid_phase: u16,
    hybrid_since: Option<u64>,
    hybrid_last_sample: Option<u64>,
    /// The current CSP move has reached its cruise speed (HYBRID_CRUISE_SPEED_SHARE).
    hybrid_cruised: bool,
    /// Sign of the way from the grip to the target, which CSP+CST pushes.
    hybrid_direction: f32,
    hybrid_brake: Option<Brake>,
    /// Where the shaft last rested, its peak speed since, and how far it had
    /// travelled from rest when it reached that peak.
    travel_rest: f32,
    travel_peak: f32,
    travel_reach: f32,
    /// CAN ID of the motor this bridge drives; every frame and status word 14 carry it.
    motor_id: u8,
    /// Attached gripper: every activation homes it before accepting motion. None drives
    /// the bare motor between the fixed MotionConfig endpoints.
    gripper: Option<Gripper>,
    homing: Option<Homing>,
    /// Shaft position at the last temporary origin, and the closing done before it.
    homing_origin: f32,
    homing_closed: f32,
    /// When the temporary origin command went out, until feedback shows it.
    zeroing: Option<u64>,
    /// First fresh sample of the current homing stall or settle, and the last one counted.
    homing_since: Option<u64>,
    homing_sample: Option<u64>,
    /// open_rad / closed_rad come from homing; status then reports the angle from open.
    calibrated: bool,
    /// MotionConfig endpoints, restored while a new homing runs.
    endpoints: (f32, f32),
    /// A homed gripper resting with the torque off where its move arrived (gOBJ 3, any
    /// target, and the open end where activation ends). It stays activated, so the next
    /// move needs no new homing. A grip (gOBJ 1 / 2) keeps pushing instead.
    rest_torque_off: bool,
    /// Go or auto release was set while the gripper was not activated (homing): it stays
    /// refused, reported as gGTO 0, until the master clears it.
    action_locked: bool,
}

impl Bridge {
    pub fn new(cfg: MotionConfig) -> Self {
        Self::for_motor(cfg, config::MOTOR_ID)
    }

    pub fn for_motor(cfg: MotionConfig, motor_id: u8) -> Self {
        Self {
            motor_id,
            settings: Settings {
                kp_centi: (cfg.kp * 100.0) as u16,
                kd_milli: (cfg.kd * 1000.0) as u16,
                ..Settings::default()
            },
            cfg,
            previous: Command::default(),
            activated: false,
            activation_started: None,
            fault: 0,
            fault_detail: FaultDetail::None,
            setpoint: 0.0,
            contact_since: None,
            object: 0,
            contact_direction: 0.0,
            release_started: None,
            release_open: false,
            previous_tick: None,
            hybrid_phase: 0,
            hybrid_since: None,
            hybrid_last_sample: None,
            hybrid_cruised: false,
            hybrid_direction: 1.0,
            hybrid_brake: None,
            travel_rest: 0.0,
            travel_peak: 0.0,
            travel_reach: 0.0,
            gripper: None,
            homing: None,
            homing_origin: 0.0,
            homing_closed: 0.0,
            zeroing: None,
            homing_since: None,
            homing_sample: None,
            calibrated: false,
            endpoints: (cfg.open_rad, cfg.closed_rad),
            rest_torque_off: false,
            action_locked: false,
        }
    }

    /// Home `gripper` on every activation (config::Gripper).
    pub fn with_gripper(mut self, gripper: Gripper) -> Self {
        self.gripper = Some(gripper);
        self
    }

    pub fn motor_id(&self) -> u8 {
        self.motor_id
    }

    /// Faults latch until the host clears rACT. No automatic restart on link recovery.
    pub fn trip(&mut self, fault: u8) {
        self.trip_with_detail(fault, FaultDetail::None);
    }

    pub fn trip_with_detail(&mut self, fault: u8, detail: FaultDetail) {
        self.fault = fault;
        self.fault_detail = detail;
        self.activated = false;
        self.homing = None;
        self.hybrid_phase = 0;
        self.hybrid_since = None;
        self.hybrid_brake = None;
        self.rest_torque_off = false;
    }

    fn track_travel(&mut self, fb: Feedback) {
        let speed = fb.velocity_rad_s.abs();
        if speed <= HYBRID_STOPPED_SPEED_RAD_S {
            self.travel_rest = fb.position_rad;
            self.travel_peak = 0.0;
            self.travel_reach = 0.0;
        } else if speed > self.travel_peak {
            self.travel_peak = speed;
            self.travel_reach = (fb.position_rad - self.travel_rest).abs();
        }
    }

    /// True once a travel end is no farther than the distance the shaft took to reach
    /// its peak speed since it last rested. That speed was gained with at most the
    /// torque limit against friction; braking with the limit plus friction is at least
    /// as strong, so the shaft stops within that distance (external loads excepted).
    fn travel_end_near(&self, fb: Feedback) -> bool {
        let speed = fb.velocity_rad_s.abs();
        if speed <= HYBRID_STOPPED_SPEED_RAD_S {
            return false;
        }
        let remaining = if fb.velocity_rad_s > 0.0 {
            self.cfg.open_rad.max(self.cfg.closed_rad) - fb.position_rad
        } else {
            fb.position_rad - self.cfg.open_rad.min(self.cfg.closed_rad)
        };
        remaining <= self.travel_reach + speed * TRAVEL_BRAKE_LEAD_S
    }

    fn start_brake(&mut self, fb: Feedback) {
        self.hybrid_brake = Some(Brake {
            direction: fb.velocity_rad_s.signum(),
        });
    }

    /// The latched brake ends once the braked motion has stopped (or reversed).
    fn still_braking(&self, fb: Feedback) -> bool {
        self.hybrid_brake
            .is_some_and(|brake| fb.velocity_rad_s * brake.direction > HYBRID_STOPPED_SPEED_RAD_S)
    }

    fn brake_packet(&self, limit: f32) -> cubemars::Packet {
        cubemars::current_command(self.settings.motor_model, self.motor_id, limit, true)
    }

    /// The selected motor (advanced settings word 12).
    fn motor(&self) -> cubemars::MotorProfile {
        cubemars::profile(self.settings.motor_model)
    }
    /// Motor model the bridge drives and converts torque with: the CAN task turns
    /// Drive::Torque into current and parses feedback with it.
    pub fn motor_model(&self) -> u16 {
        self.settings.motor_model
    }
    /// Full-scale speed of the 0..1000 command: the selected motor's rated speed.
    fn speed_scale(&self) -> f32 {
        self.motor()
            .rated_speed_rad_s()
            .min(self.cfg.max_speed_rad_s)
    }
    fn torque_ceiling(&self) -> f32 {
        self.motor().max_torque
    }
    fn position_byte(&self, p: f32) -> u8 {
        let span = self.cfg.closed_rad - self.cfg.open_rad;
        if !p.is_finite() || !span.is_finite() || span.abs() < f32::EPSILON {
            return 0;
        }
        // gPO uses the rPR step: 1/256 of the travel, saturating at 255.
        ((((p - self.cfg.open_rad) / span).clamp(0.0, 1.0) * control::POSITION_STEPS as f32 + 0.5)
            as u32)
            .min(255) as u8
    }
    fn position_units(&self, p: f32) -> u16 {
        let span = self.cfg.closed_rad - self.cfg.open_rad;
        if !p.is_finite() || !span.is_finite() || span.abs() < f32::EPSILON {
            return 0;
        }
        (((p - self.cfg.open_rad) / span).clamp(0.0, 1.0) * control::POSITION_SCALE as f32 + 0.5)
            as u16
    }
    /// How near a stopped shaft must be to its target for the move to have arrived.
    fn arrival_tolerance(&self) -> f32 {
        let tolerance = self.cfg.position_tolerance_rad;
        if self.calibrated {
            ((self.cfg.closed_rad - self.cfg.open_rad).abs() / ARRIVAL_STEPS).max(tolerance)
        } else {
            tolerance
        }
    }
    fn position_rad(&self, b: u16) -> f32 {
        self.cfg.open_rad
            + (self.cfg.closed_rad - self.cfg.open_rad) * b as f32 / control::POSITION_SCALE as f32
    }

    fn start_homing(&mut self, now: u64) -> Drive {
        self.homing = Some(Homing::Closing);
        self.homing_closed = 0.0;
        self.homing_sample = None;
        self.object = 0;
        self.calibrated = false;
        self.rest_torque_off = false;
        (self.cfg.open_rad, self.cfg.closed_rad) = self.endpoints;
        self.zero(now)
    }

    /// Temporary origin here (manual 4.1.6; the motor forgets it at power-off). The servo
    /// feedback only spans +-3200 deg, and the motor keeps its count across activations,
    /// so homing starts from zero, zeroes again before closing runs off the range, and
    /// once more on the closed end: a homed gripper is closed at 0 deg.
    fn zero(&mut self, now: u64) -> Drive {
        self.zeroing = Some(now);
        self.homing_since = None;
        Drive::Can(cubemars::set_origin(self.motor_id))
    }

    /// Advanced status word 14 bits 4-5: 0 not homed, 1 closing, 2 opening, 3 homed.
    fn homing_state(&self) -> u16 {
        match self.homing {
            Some(Homing::Closing) => 1,
            Some(Homing::Opening { .. }) => 2,
            None if self.calibrated => 3,
            None => 0,
        }
    }

    /// One homing tick with the MCU position loop at the homing speed and torque: close
    /// until the shaft stalls, then open by the gripper travel, slowing for its last part
    /// (or to just short of an earlier stall). Then open..closed become the 0..1024
    /// endpoints and the bridge is activated at the open end with the torque off.
    fn home(&mut self, now: u64, fb: Feedback, g: Gripper, phase: Homing, dt: f32) -> Drive {
        if now.saturating_sub(self.activation_started.unwrap_or(now)) > g.homing_timeout_ms {
            self.trip_with_detail(0x0d, FaultDetail::HomingNoEnd);
            return Drive::Disabled;
        }
        if let Some(sent) = self.zeroing {
            if fb.received_ms >= sent + ORIGIN_SETTLE_MS
                && fb.position_rad.abs() <= ORIGIN_TOLERANCE_RAD
            {
                self.zeroing = None;
                self.homing_origin = fb.position_rad;
                self.setpoint = fb.position_rad;
            } else if now.saturating_sub(sent) > ORIGIN_TIMEOUT_MS {
                self.trip_with_detail(0x0d, FaultDetail::OriginNotSet);
                return Drive::Disabled;
            } else {
                return Drive::Torque(0.0);
            }
        }
        let close = g.close_sign;
        let travel = g.travel_rad();
        let mut limit = g.homing_torque_nm.min(self.torque_ceiling());
        let mut speed = g.homing_speed_rad_s().min(self.speed_scale());
        let (goal, push) = match phase {
            Homing::Closing => {
                let since_zero = (fb.position_rad - self.homing_origin) * close;
                if self.homing_closed + since_zero > HOMING_MAX_CLOSE * travel {
                    self.trip_with_detail(0x0d, FaultDetail::HomingNoEnd);
                    return Drive::Disabled;
                }
                // Zero again a turn before the servo feedback range ends.
                if fb.position_rad * close >= config::FEEDBACK_POSITION_RAD - core::f32::consts::TAU
                {
                    self.homing_closed += since_zero;
                    return self.zero(now);
                }
                (self.setpoint + close * speed * dt, close)
            }
            Homing::Opening {
                closed,
                target,
                blocked,
            } => {
                // Meet an open end nearer than computed, or back off one, gently.
                if blocked || (fb.position_rad - closed).abs() >= g.approach_from * travel {
                    speed *= g.approach_speed;
                    limit *= g.approach_torque;
                }
                (target, -close)
            }
        };
        let step = (goal - self.setpoint).clamp(-speed * dt, speed * dt);
        self.setpoint = follow(self.setpoint + step, fb.position_rad, limit, self.cfg.kp);
        let desired_velocity = if dt > 0.0 { step / dt } else { 0.0 };
        let raw = self.cfg.kp * (self.setpoint - fb.position_rad)
            + self.cfg.kd * (desired_velocity - fb.velocity_rad_s);
        let drive = Drive::Torque(raw.clamp(-limit, limit));
        if self.homing_sample == Some(fb.received_ms) {
            return drive;
        }
        self.homing_sample = Some(fb.received_ms);
        let still = fb.velocity_rad_s.abs() <= HYBRID_STOPPED_SPEED_RAD_S;
        // Saturated toward the end and the motor really draws it: the end is reached.
        let stalled =
            still && raw * push >= 0.9 * limit && (fb.iq_a * self.motor().kt).abs() >= 0.8 * limit;
        let settled = still
            && matches!(phase, Homing::Opening { target, .. }
                if (target - self.setpoint).abs() <= self.cfg.position_tolerance_rad);
        let event = match phase {
            Homing::Closing => stalled,
            Homing::Opening { blocked, .. } => settled || stalled && !blocked,
        };
        if !event {
            self.homing_since = None;
            return drive;
        }
        let since = *self.homing_since.get_or_insert(fb.received_ms);
        if fb.received_ms.saturating_sub(since) < g.homing_dwell_ms {
            return drive;
        }
        self.homing_since = None;
        match phase {
            Homing::Closing => {
                // The closed end becomes 0 deg; the opening runs in that frame.
                let range = config::FEEDBACK_POSITION_RAD;
                self.homing = Some(Homing::Opening {
                    closed: 0.0,
                    target: (-close * travel).clamp(-range, range),
                    blocked: false,
                });
                return self.zero(now);
            }
            Homing::Opening {
                closed,
                blocked: false,
                ..
            } if stalled => {
                if (fb.position_rad - closed).abs() < 0.5 * travel {
                    self.trip_with_detail(0x0d, FaultDetail::HomingShortStroke);
                    return Drive::Disabled;
                }
                self.homing = Some(Homing::Opening {
                    closed,
                    target: fb.position_rad + close * g.rad(g.open_margin_mm),
                    blocked: true,
                });
                self.setpoint = fb.position_rad;
            }
            Homing::Opening { closed, target, .. } => {
                self.cfg.open_rad = target;
                self.cfg.closed_rad = closed - close * g.rad(g.closed_margin_mm);
                self.calibrated = true;
                self.homing = None;
                self.activated = true;
                self.setpoint = fb.position_rad;
                self.object = 0;
                // Activation ends fully open, which rests with the torque off. A go held
                // through it stays refused (action_locked) until the master clears it.
                self.rest_torque_off = true;
                return Drive::Torque(0.0);
            }
        }
        drive
    }
    fn report(&self, cmd: Command, fb: Option<Feedback>, drive: Drive) -> Output {
        let mut status = [0; 16];
        status[0] = if cmd.active() { 1 } else { 0 };
        if self.activated && self.release_started.is_none() {
            status[0] |= 3 << 4;
            if cmd.go() && self.release_started.is_none() {
                status[0] |= 8 | (self.object << 6);
            }
        } else if !self.activated && self.activation_started.is_some() && self.fault == 0 {
            status[0] |= 1 << 4;
        }
        status[2] = if self.release_started.is_some() && self.fault == 0 {
            0x0B
        } else {
            self.fault
        };
        status[3] = control::legacy_position(cmd.position);
        if let Some(fb) = fb {
            status[4] = self.position_byte(fb.position_rad);
            // Robotiq gCU unit: 10mA, saturated; the AK40-10 (8.5 A peak) and AK60-39 (17 A)
            // exceed the 2.55A field range.
            status[5] = (fb.iq_a.abs() * 100.0).clamp(0.0, 255.0) as u8;
        }
        let mut status1000 = [0; extended::STATUS_BYTES];
        let mut word = |index: usize, value: u16| {
            status1000[index * 2..index * 2 + 2].copy_from_slice(&value.to_be_bytes());
        };
        word(0, 0x414b);
        word(1, 8);
        word(2, status[0] as u16);
        word(3, status[2] as u16);
        word(4, cmd.position);
        word(
            5,
            fb.map_or(control::POSITION_SCALE / 2, |f| {
                self.position_units(f.position_rad)
            }),
        );
        let millidegrees = fb.map_or(0, |f| {
            // Once homed: the angle from fully open, positive toward closed.
            let angle = match self.gripper {
                Some(g) if self.calibrated => (f.position_rad - self.cfg.open_rad) * g.close_sign,
                _ => f.position_rad,
            };
            let degrees = angle * 180_000.0 / core::f32::consts::PI;
            if degrees >= 0.0 {
                (degrees + 0.5) as i32
            } else {
                (degrees - 0.5) as i32
            }
        });
        word(6, (millidegrees as u32 >> 16) as u16);
        word(7, millidegrees as u16);
        word(
            8,
            fb.map_or(0, |f| (f.iq_a.abs() * 1000.0).clamp(0.0, 65535.0) as u16),
        );
        word(9, cmd.speed);
        word(10, cmd.force);
        word(
            11,
            fb.map_or(u16::MAX, |f| {
                self.previous_tick
                    .unwrap_or(0)
                    .saturating_sub(f.received_ms)
                    .min(65534) as u16
            }),
        );
        word(12, self.fault_detail as u16);
        word(13, fb.map_or(0, |f| f.error as u16));
        word(14, self.motor_id as u16);
        word(15, control::SCALE);
        let mut advanced_status = [0; advanced::STATUS_BYTES];
        let mut extra =
            |i: usize, w: u16| advanced_status[i * 2..i * 2 + 2].copy_from_slice(&w.to_be_bytes());
        extra(0, 0x4144);
        extra(1, 3);
        extra(2, self.settings.mode as u16);
        extra(3, self.settings.kp_centi);
        extra(4, self.settings.kd_milli);
        extra(5, self.settings.mit_kp_centi);
        extra(6, self.settings.mit_kd_milli);
        extra(7, fb.map_or(0x8000, |f| i16::from(f.temperature_c) as u16));
        let iq = fb.map_or(0, |f| (f.iq_a * 1000.0) as i32);
        let kt = self.motor().kt;
        let torque = fb.map_or(0, |f| (f.iq_a * kt * 1000.0) as i32);
        let velocity = fb.map_or(0, |f| (f.velocity_rad_s * 1000.0) as i32);
        for (i, value) in [(8, iq), (10, torque), (12, velocity)] {
            extra(i, (value as u32 >> 16) as u16);
            extra(i + 1, value as u16);
        }
        // Word 14: bit 0 was the MIT torque limiter (MIT mode removed); bits 8+ the motor model.
        extra(
            14,
            (self.hybrid_phase << 1)
                | (u16::from(self.hybrid_since.is_some()) << 3)
                | (self.homing_state() << 4)
                | (u16::from(self.rest_torque_off) << 6)
                | (u16::from(self.action_locked) << 7)
                | (self.settings.motor_model << 8),
        );
        extra(15, 0x7f);
        Output {
            drive,
            status,
            status1000,
            advanced_status,
        }
    }

    pub fn step_with_settings(
        &mut self,
        now: u64,
        cmd: Command,
        last_host_ms: Option<u64>,
        fb: Option<Feedback>,
        settings: Settings,
    ) -> Output {
        if !settings.valid()
            || self.previous.go()
                && cmd.go()
                && settings.mode == Mode::Hybrid
                && !self.settings.same_hybrid_target(settings)
        {
            self.trip_with_detail(0x0d, FaultDetail::InvalidConfig);
        } else if self.previous.go() && cmd.go() && settings.mode != self.settings.mode
            || self.previous.active()
                && cmd.active()
                && settings.motor_model != self.settings.motor_model
        {
            // A running move never changes mode; a stopped (holding) motor may, and the new
            // mode starts with the next go edge. The motor model (torque constant and speed
            // scale) changes only while the motor is OFF.
            self.trip_with_detail(0x0d, FaultDetail::ModeChange);
        } else {
            if settings.mode != self.settings.mode {
                self.hybrid_phase = 0;
                self.hybrid_since = None;
                self.hybrid_last_sample = None;
                self.hybrid_brake = None;
                self.contact_since = None;
                self.object = 0;
            }
            self.settings = settings;
            self.cfg.kp = f32::from(settings.kp_centi) / 100.0;
            self.cfg.kd = f32::from(settings.kd_milli) / 1000.0;
        }
        self.step_scaled(now, cmd, last_host_ms, fb)
    }

    fn advanced_drive(&mut self, cmd: Command, fb: Feedback) -> Drive {
        use crate::cubemars;
        let limit =
            (self.cfg.max_torque_nm * f32::from(cmd.force) / 1000.0).min(self.torque_ceiling());
        if limit == 0.0 {
            self.setpoint = fb.position_rad;
            return Drive::Torque(0.0);
        }
        // Native closed loops have no per-frame current ceiling. Stop on measured
        // over-limit current; do not present this as a hardware torque guarantee.
        if (fb.iq_a * self.motor().kt).abs() > limit + 0.1 {
            self.trip_with_detail(0x0e, FaultDetail::TorqueLimit);
            return Drive::Disabled;
        }
        let speed = self.speed_scale() * f32::from(cmd.speed) / 1000.0;
        // A homed gripper may close toward negative angles (Gripper::close_sign).
        let low = self.cfg.open_rad.min(self.cfg.closed_rad);
        let high = self.cfg.open_rad.max(self.cfg.closed_rad);
        let inward = |value: f32| {
            if (fb.position_rad >= high && value > 0.0) || (fb.position_rad <= low && value < 0.0) {
                0.0
            } else {
                value
            }
        };
        let velocity =
            inward((f32::from(self.settings.velocity_mrad) / 1000.0).clamp(-speed, speed));
        let torque = inward((f32::from(self.settings.torque_mnm) / 1000.0).clamp(-limit, limit));
        let target = self.settings.position_mdeg as f32 * core::f32::consts::PI / 180000.0;
        let model = self.settings.motor_model;
        let packet = match self.settings.mode {
            Mode::Velocity => cubemars::velocity_command(model, self.motor_id, velocity),
            Mode::Cst => cubemars::current_command(model, self.motor_id, torque, false),
            Mode::Brake => cubemars::current_command(
                model,
                self.motor_id,
                (f32::from(self.settings.torque_mnm) / 1000.0)
                    .abs()
                    .min(limit),
                true,
            ),
            Mode::Profile => {
                if cubemars::electrical_rpm(model, speed).abs() < 10.0 {
                    return Drive::Torque(0.0);
                }
                cubemars::profile_command(
                    model,
                    self.motor_id,
                    target,
                    speed,
                    f32::from(self.settings.acceleration_mrad) / 1000.0,
                )
            }
            Mode::Position | Mode::Hybrid => unreachable!(),
        };
        self.object = if self.settings.mode == Mode::Profile
            && (target - fb.position_rad).abs() <= self.cfg.position_tolerance_rad
            && fb.velocity_rad_s.abs() <= self.cfg.stopped_speed_rad_s
        {
            3
        } else {
            0
        };
        Drive::Can(packet)
    }

    pub fn step(
        &mut self,
        now: u64,
        cmd: crate::robotiq::Command,
        last_host_ms: Option<u64>,
        fb: Option<Feedback>,
    ) -> Output {
        self.step_scaled(now, cmd.into(), last_host_ms, fb)
    }

    pub fn step_scaled(
        &mut self,
        now: u64,
        cmd: Command,
        last_host_ms: Option<u64>,
        fb: Option<Feedback>,
    ) -> Output {
        // Cap a delayed step: never jump a trajectory forward after a scheduling stall.
        let dt = self
            .previous_tick
            .map(|t| now.saturating_sub(t))
            .unwrap_or(1)
            .min(5) as f32
            * 0.001;
        self.previous_tick = Some(now);
        // Only rACT acts on a gripper until activation homing is done (gSTA 3): go or auto
        // release set before then stays refused (gGTO 0) until the master clears it, so it
        // never starts a move once homed either.
        const ACTIONS: u8 = 0x38;
        if !cmd.active() || cmd.action & ACTIONS == 0 {
            self.action_locked = false;
        } else if !self.activated && self.gripper.is_some() {
            self.action_locked = true;
        }
        let cmd = if self.action_locked {
            Command {
                action: cmd.action & !ACTIONS,
                ..cmd
            }
        } else {
            cmd
        };
        let previous = self.previous;
        self.previous = cmd;
        if !cmd.active() {
            self.hybrid_phase = 0;
            self.hybrid_since = None;
            self.hybrid_last_sample = None;
            self.activated = false;
            self.rest_torque_off = false;
            self.homing = None;
            self.activation_started = None;
            self.fault = if cmd.go() || cmd.release() { 0x07 } else { 0 };
            self.fault_detail = FaultDetail::None;
            self.contact_since = None;
            self.object = 0;
            self.release_started = None;
            return self.report(cmd, fb, Drive::Disabled);
        }
        if !self.cfg.valid() || !cmd.valid() {
            self.trip_with_detail(0x0D, FaultDetail::InvalidConfig);
        }
        if self.fault != 0 {
            return self.report(cmd, fb, Drive::Disabled);
        }
        if last_host_ms.is_none_or(|t| now.saturating_sub(t) > self.cfg.host_timeout_ms) {
            self.trip_with_detail(0x09, FaultDetail::HostTimeout);
            return self.report(cmd, fb, Drive::Disabled);
        }
        if self.activation_started.is_none() {
            self.activation_started = Some(now);
        }
        let fresh = fb.filter(|f| {
            now.saturating_sub(f.received_ms) <= self.cfg.feedback_timeout_ms
                && f.received_ms >= self.activation_started.unwrap_or(now)
        });
        // Old errors cannot prevent the first new command after an explicit reset.
        if let Some(fb) = fresh {
            if !fb.position_rad.is_finite()
                || !fb.velocity_rad_s.is_finite()
                || !fb.iq_a.is_finite()
            {
                self.trip_with_detail(0x0D, FaultDetail::InvalidFeedback);
            } else if fb.error != 0 {
                self.trip_with_detail(
                    match fb.error {
                        1 | 6 => 0x08,
                        2 => 0x0E,
                        3 | 5 => 0x0C,
                        4 => 0x0A,
                        _ => 0x0D,
                    },
                    FaultDetail::Motor,
                );
            }
        }
        if self.fault != 0 {
            return self.report(cmd, fb, Drive::Disabled);
        }
        let Some(feedback) = fresh else {
            if self.activated
                || now.saturating_sub(self.activation_started.unwrap_or(now))
                    > self.cfg.activation_timeout_ms
            {
                let detail = if self.activated {
                    FaultDetail::FeedbackTimeout
                } else {
                    FaultDetail::ActivationFeedbackTimeout
                };
                self.trip_with_detail(0x09, detail);
                return self.report(cmd, fb, Drive::Disabled);
            }
            // Enable with zero gains and zero requested torque, then wait for actual feedback.
            return self.report(cmd, fb, Drive::Torque(0.0));
        };
        // Slider endpoints limit targets. A valid measured position may be outside
        // them; start there so a command back into range has no trajectory jump.
        if !self.activated {
            let Some(gripper) = self.gripper else {
                self.activated = true;
                self.setpoint = feedback.position_rad;
                self.object = 0;
                // A go sent with rACT (a run straight from OFF) starts on the next tick as
                // a go edge, exactly like rGTO set after activation.
                self.previous = Command {
                    action: cmd.action & !0x08,
                    ..cmd
                };
                return self.report(cmd, fb, Drive::Torque(0.0));
            };
            // Robotiq-style activation (gSTA 1 until done): find the travel, end open.
            let Some(phase) = self.homing else {
                let drive = self.start_homing(now);
                return self.report(cmd, fb, drive);
            };
            let drive = self.home(now, feedback, gripper, phase, dt);
            return self.report(cmd, fb, drive);
        }
        if !matches!(self.settings.mode, Mode::Position | Mode::Hybrid) && cmd.release() {
            self.trip_with_detail(0x0d, FaultDetail::InvalidConfig);
            return self.report(cmd, fb, Drive::Disabled);
        }
        let hybrid = self.settings.mode == Mode::Hybrid;
        if hybrid {
            self.track_travel(feedback);
            // A new target is a new CSP move: CST only ever joins one it has gripped.
            if !cmd.go() || !previous.go() || cmd.position != previous.position {
                self.hybrid_phase = if cmd.go() { 1 } else { 0 };
                self.hybrid_since = None;
                self.hybrid_last_sample = None;
                self.hybrid_cruised = false;
                self.hybrid_brake = None;
                if cmd.go() {
                    self.setpoint = feedback.position_rad;
                }
                self.object = 0;
            }
            let limit =
                (self.cfg.max_torque_nm * f32::from(cmd.force) / 1000.0).min(self.torque_ceiling());
            let threshold = f32::from(self.settings.threshold_mnm) / 1000.0;
            if cmd.go() && limit > 0.0 && threshold > limit {
                self.trip_with_detail(0x0d, FaultDetail::InvalidConfig);
                return self.report(cmd, fb, Drive::Disabled);
            }
            if self.hybrid_phase == 1 {
                if cmd.speed == 0 || cmd.force == 0 {
                    self.hybrid_since = None;
                    self.hybrid_last_sample = None;
                } else if self.hybrid_last_sample != Some(feedback.received_ms) {
                    self.hybrid_last_sample = Some(feedback.received_ms);
                    let torque = feedback.iq_a * self.motor().kt;
                    let speed = self.speed_scale() * f32::from(cmd.speed) / 1000.0;
                    let held_speed =
                        (HYBRID_HELD_SPEED_SHARE * speed).max(HYBRID_STOPPED_SPEED_RAD_S);
                    let velocity = feedback.velocity_rad_s.abs();
                    if velocity >= HYBRID_CRUISE_SPEED_SHARE * speed {
                        self.hybrid_cruised = true;
                    }
                    // The grip leads the shaft toward the target by the lag that saturates
                    // the loop at the CST torque. Only a load that holds the fingers farther
                    // short of the target than that is a grip; nearer, CSP alone holds,
                    // whatever the current reads.
                    let error = self.position_rad(cmd.position) - feedback.position_rad;
                    let cst = (f32::from(self.settings.torque_mnm) / 1000.0).abs();
                    let lead = if self.cfg.kp > 0.0 {
                        FOLLOWING_LAG_MARGIN * cst / self.cfg.kp
                    } else {
                        0.0
                    };
                    if torque.abs() >= threshold
                        && (self.hybrid_cruised || velocity <= held_speed)
                        && error.abs() > lead.max(self.cfg.position_tolerance_rad)
                    {
                        // CSP pushes toward its target, so a grip is held that way (the
                        // measured current's sign convention is the motor's, not the
                        // command's). One contact pushes one way, so a reversal restarts
                        // the dwell.
                        let direction = if error > 0.0 { 1.0 } else { -1.0 };
                        if direction != self.hybrid_direction {
                            self.hybrid_direction = direction;
                            self.hybrid_since = None;
                        }
                        let since = *self.hybrid_since.get_or_insert(feedback.received_ms);
                        if feedback.received_ms.saturating_sub(since)
                            >= u64::from(self.settings.dwell_ms)
                        {
                            // CST joins CSP, pushing with the CST torque at once.
                            self.hybrid_phase = 2;
                            self.hybrid_since = None;
                            self.setpoint = feedback.position_rad + direction * lead;
                        }
                    } else {
                        self.hybrid_since = None;
                    }
                }
            }
        }
        if cmd.go() && !matches!(self.settings.mode, Mode::Position | Mode::Hybrid) {
            let drive = self.advanced_drive(cmd, feedback);
            return self.report(cmd, fb, drive);
        }
        // Arrived, the torque stays off, through a stop too, until a new move (a go edge or
        // a new target) or an auto release starts from wherever the fingers are.
        if self.rest_torque_off
            && (cmd.release() || cmd.go() && (!previous.go() || cmd.position != previous.position))
        {
            self.rest_torque_off = false;
        }
        if self.rest_torque_off {
            self.setpoint = feedback.position_rad;
            self.contact_since = None;
            return self.report(cmd, fb, Drive::Torque(0.0));
        }
        if cmd.release() && self.release_started.is_none() {
            self.release_started = Some(now);
            self.release_open = cmd.release_open();
            self.contact_since = None;
            self.object = 0;
        }
        let releasing = self.release_started.is_some();
        // CSP+CST grip: the position loop runs on toward the target with the CST torque
        // as its limit, so CST only pushes while something holds the fingers short of it.
        let gripping = hybrid && self.hybrid_phase == 2 && !releasing;
        // A homed gripper releases across its whole travel at the homing speed.
        let (release_speed, release_timeout_ms) = match self.gripper {
            Some(g) => (g.homing_speed_rad_s(), g.homing_timeout_ms),
            None => (self.cfg.release_speed_rad_s, self.cfg.release_timeout_ms),
        };
        if releasing && now.saturating_sub(self.release_started.unwrap_or(now)) > release_timeout_ms
        {
            self.trip_with_detail(0x0D, FaultDetail::ReleaseTimeout);
            return self.report(cmd, fb, Drive::Disabled);
        }
        if !releasing
            && (cmd.position != previous.position
                || cmd.go() != previous.go()
                || (previous.speed == 0 && cmd.speed > 0)
                || (previous.force == 0 && cmd.force > 0))
        {
            self.contact_since = None;
            self.object = 0;
            self.setpoint = feedback.position_rad;
        }
        let requested_torque = (self.cfg.min_torque_nm
            + (self.cfg.max_torque_nm - self.cfg.min_torque_nm) * cmd.force as f32
                / control::SCALE as f32)
            .min(self.torque_ceiling());
        let torque_limit = if releasing {
            self.cfg.release_torque_nm.min(requested_torque)
        } else if gripping {
            // The CST setting gives only the size; CSP gives the way, toward the target.
            (f32::from(self.settings.torque_mnm) / 1000.0)
                .abs()
                .min(requested_torque)
        } else {
            requested_torque
        };
        let moving = cmd.go() || releasing;
        let target = if releasing {
            self.position_rad(if self.release_open {
                0
            } else {
                control::POSITION_SCALE
            })
        } else {
            self.position_rad(cmd.position)
        };
        let requested_speed = self.cfg.min_speed_rad_s
            + (self.speed_scale() - self.cfg.min_speed_rad_s) * cmd.speed as f32
                / control::SCALE as f32;
        let speed = if releasing {
            release_speed.min(requested_speed)
        } else {
            requested_speed
        };
        if torque_limit == 0.0 || speed == 0.0 {
            self.contact_since = None;
            if !gripping {
                self.object = 0;
            }
            if releasing {
                // A deliberately zero command pauses automatic release too.
                self.release_started = Some(now);
            }
            if torque_limit == 0.0 {
                // No hidden trajectory is accumulated while torque is inhibited.
                self.setpoint = feedback.position_rad;
                return self.report(cmd, fb, Drive::Torque(0.0));
            }
            // A grip keeps its reference lead, and so its CST push, without a ramp.
            if !gripping
                && (previous.speed != 0 || previous.force == 0 || cmd.position != previous.position)
            {
                self.setpoint = feedback.position_rad;
            }
            // Zero speed holds the measured stop position without advancing a ramp.
            let hold = self.cfg.kp * (self.setpoint - feedback.position_rad)
                - self.cfg.kd * feedback.velocity_rad_s;
            return self.report(
                cmd,
                fb,
                Drive::Torque(hold.clamp(-torque_limit, torque_limit)),
            );
        }
        let mut desired_velocity = 0.0;
        if moving && (gripping || self.object == 0 || self.object == 3) {
            let delta = target - self.setpoint;
            let step = delta.clamp(-speed * dt, speed * dt);
            self.setpoint += step;
            desired_velocity = if dt > 0.0 { step / dt } else { 0.0 };
        }
        if moving {
            self.setpoint = follow(
                self.setpoint,
                feedback.position_rad,
                torque_limit,
                self.cfg.kp,
            );
        }
        let raw_torque = self.cfg.kp * (self.setpoint - feedback.position_rad)
            + self.cfg.kd * (desired_velocity - feedback.velocity_rad_s);
        let mut torque = raw_torque.clamp(-torque_limit, torque_limit);
        if moving {
            let error = target - feedback.position_rad;
            // Once the ramp is done, a homed gripper stopped within half a step has arrived.
            let ramped = (target - self.setpoint).abs() <= self.cfg.position_tolerance_rad;
            let settled = feedback.velocity_rad_s.abs() <= self.cfg.stopped_speed_rad_s
                && (error.abs() <= self.cfg.position_tolerance_rad
                    || ramped && error.abs() <= self.arrival_tolerance());
            if settled {
                self.contact_since = None;
                // If external motion reaches the target ahead of the ramp, hold
                // the target rather than pulling back toward the old ramp point.
                self.setpoint = target;
                torque = (self.cfg.kp * error - self.cfg.kd * feedback.velocity_rad_s)
                    .clamp(-torque_limit, torque_limit);
                if releasing {
                    self.trip(0x0F);
                    return self.report(cmd, fb, Drive::Disabled);
                }
                if gripping {
                    self.hybrid_phase = 1;
                }
                self.object = 3;
                // At its target nothing holds the fingers: a homed gripper turns the
                // torque off there (open or closed alike) and stays activated, ready
                // for the next move. A grip never gets here; it keeps the CST torque.
                if self.calibrated {
                    self.rest_torque_off = true;
                    return self.report(cmd, fb, Drive::Torque(0.0));
                }
            } else if gripping {
                // A yielding or slipping object is followed at no more than the CSP speed
                // and never past the target. The reference reaches the target only once
                // the fingers are within the CST lag of it: stopped there, nothing holds
                // them short any more (the object gave way or fell out), so CST lets go
                // and CSP holds the target, even where friction keeps them just short.
                if (target - self.setpoint).abs() <= self.cfg.position_tolerance_rad
                    && feedback.velocity_rad_s.abs() <= self.cfg.stopped_speed_rad_s
                {
                    self.hybrid_phase = 1;
                    self.object = 0;
                } else {
                    // Report the held object like Robotiq.
                    self.object = if self.hybrid_direction
                        * (self.cfg.closed_rad - self.cfg.open_rad)
                        > 0.0
                    {
                        2
                    } else {
                        1
                    };
                }
            } else if self.object == 1 || self.object == 2 {
                // Hold the requested force after detecting contact, until new position/stop/reset.
                torque = self.contact_direction * torque_limit;
            } else if !releasing
                && !hybrid
                && raw_torque.abs() >= 0.9 * torque_limit
                && (feedback.iq_a * self.motor().kt).abs() >= 0.9 * torque_limit
                && feedback.velocity_rad_s.abs() <= self.cfg.stopped_speed_rad_s
                && error.abs() > self.cfg.position_tolerance_rad
            {
                let since = *self.contact_since.get_or_insert(now);
                if now.saturating_sub(since) >= self.cfg.contact_dwell_ms {
                    self.object = if error * (self.cfg.closed_rad - self.cfg.open_rad) > 0.0 {
                        2
                    } else {
                        1
                    };
                    self.contact_direction = if error >= 0.0 { 1.0 } else { -1.0 };
                    torque = self.contact_direction * torque_limit;
                }
            } else {
                self.contact_since = None;
                self.object = 0;
            }
        } else {
            self.object = 0;
            self.contact_since = None;
        }
        if hybrid && moving {
            // A move to a travel end overshoots by its braking distance: brake early
            // enough to stop inside, then resume the ramp from where the shaft stops.
            if self.hybrid_brake.is_none() && self.travel_end_near(feedback) {
                self.start_brake(feedback);
            }
            if self.hybrid_brake.is_some() {
                self.setpoint = feedback.position_rad;
                if self.still_braking(feedback) {
                    // A grip may have joined a move the full limit had sped up.
                    let brake = if gripping {
                        requested_torque
                    } else {
                        torque_limit
                    };
                    return self.report(cmd, fb, Drive::Can(self.brake_packet(brake)));
                }
                self.hybrid_brake = None;
                torque = 0.0;
            }
        }
        // The CAN sender rounds torque toward zero and uses exact 0 A when needed.
        self.report(cmd, fb, Drive::Torque(torque))
    }
}
