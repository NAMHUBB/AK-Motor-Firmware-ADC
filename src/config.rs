//! Commissioning settings. Angles and torques refer to the motor OUTPUT shaft.

/// Supplied V3 software manual: extended ID 0x08xx, KP/KD/P/V/T order.
/// Do not substitute the older standard-ID MIT protocol or its scaling ranges.
/// Confirmed from live 0x2968 feedback and the 0x0F68 -> 0x2968 disable ACK.
/// CAN ID of the one motor a single-motor build drives, chosen at build time like the
/// slave address: `MOTOR_CAN_ID=2 cargo build --release` drives the motor set to ID 2
/// (the Right/ID2 motor of the dual-motor board). Default 104 (0x68).
pub const MOTOR_ID: u8 = parse_motor_id(option_env!("MOTOR_CAN_ID"));
pub const CAN_BITRATE: u32 = 1_000_000;
/// Modbus slave address, chosen per board at build time so several boards can share
/// one RS-485 bus: `MODBUS_SLAVE_ID=2 cargo build --release`. Default 9 (Robotiq).
pub const MODBUS_SLAVE_ID: u8 = parse_slave_id(option_env!("MODBUS_SLAVE_ID"));

/// Parses the build-time slave address; a value outside 1..=247 fails the build.
pub const fn parse_slave_id(value: Option<&str>) -> u8 {
    let Some(text) = value else {
        return 9;
    };
    let digits = text.as_bytes();
    assert!(
        !digits.is_empty() && digits.len() <= 3,
        "MODBUS_SLAVE_ID must be 1..=247"
    );
    let mut id = 0u32;
    let mut i = 0;
    while i < digits.len() {
        assert!(
            digits[i].is_ascii_digit(),
            "MODBUS_SLAVE_ID must be 1..=247"
        );
        id = id * 10 + (digits[i] - b'0') as u32;
        i += 1;
    }
    assert!(id >= 1 && id <= 247, "MODBUS_SLAVE_ID must be 1..=247");
    id as u8
}

/// Parses the build-time motor CAN ID; a value outside 1..=127 fails the build.
pub const fn parse_motor_id(value: Option<&str>) -> u8 {
    let Some(text) = value else {
        return 0x68;
    };
    let digits = text.as_bytes();
    assert!(
        !digits.is_empty() && digits.len() <= 3,
        "MOTOR_CAN_ID must be 1..=127"
    );
    let mut id = 0u32;
    let mut i = 0;
    while i < digits.len() {
        assert!(digits[i].is_ascii_digit(), "MOTOR_CAN_ID must be 1..=127");
        id = id * 10 + (digits[i] - b'0') as u32;
        i += 1;
    }
    assert!(id >= 1 && id <= 127, "MOTOR_CAN_ID must be 1..=127");
    id as u8
}

/// One AK motor on the CAN bus and the Modbus slave address that drives it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Channel {
    pub slave: u8,
    pub motor_id: u8,
}

/// One board, one motor: MODBUS_SLAVE_ID drives CAN ID MOTOR_ID.
pub const SINGLE_MOTOR: [Channel; 1] = [Channel {
    slave: MODBUS_SLAVE_ID,
    motor_id: MOTOR_ID,
}];
/// One board, two motors on one CAN bus behind one RS-485 port: slave 1 drives
/// CAN ID 1 (Left) and slave 2 drives CAN ID 2 (Right). Set the motor IDs with R-LINK.
pub const DUAL_MOTOR: [Channel; 2] = [
    Channel {
        slave: 1,
        motor_id: 1,
    },
    Channel {
        slave: 2,
        motor_id: 2,
    },
];
/// Motors this build drives: the default `dual-motor` feature selects DUAL_MOTOR,
/// `--no-default-features` the one motor of SINGLE_MOTOR.
pub const CHANNELS: &[Channel] = if cfg!(feature = "dual-motor") {
    &DUAL_MOTOR
} else {
    &SINGLE_MOTOR
};
pub const CHANNEL_COUNT: usize = CHANNELS.len();

/// Every slave address and every CAN ID is distinct and in range.
pub const fn channels_valid(channels: &[Channel]) -> bool {
    let mut i = 0;
    while i < channels.len() {
        let a = channels[i];
        if a.slave == 0 || a.slave > 247 || a.motor_id == 0 || a.motor_id > 0x7f {
            return false;
        }
        let mut j = i + 1;
        while j < channels.len() {
            if a.slave == channels[j].slave || a.motor_id == channels[j].motor_id {
                return false;
            }
            j += 1;
        }
        i += 1;
    }
    !channels.is_empty()
}
const _: () = assert!(channels_valid(CHANNELS), "invalid motor channel table");
const _: () = assert!(
    !cfg!(feature = "dual-motor") || option_env!("MODBUS_SLAVE_ID").is_none(),
    "dual-motor (the default) fixes Modbus slaves 1 and 2: unset MODBUS_SLAVE_ID, or build a one-motor board with --no-default-features"
);
const _: () = assert!(
    !cfg!(feature = "dual-motor") || option_env!("MOTOR_CAN_ID").is_none(),
    "dual-motor (the default) fixes CAN IDs 1 and 2: unset MOTOR_CAN_ID, or build a one-motor board with --no-default-features"
);
pub const MODBUS_BAUD: u32 = 115_200;
/// Ceiling of the 0..1000 speed scale: the fastest supported motor (AK40-10, 370 RPM).
/// Each motor's scale is its own rated speed below this (AK60-39: 70 RPM).
pub const MAX_SPEED_RPM: f32 = 370.0;
pub const MAX_SPEED_RAD_S: f32 = MAX_SPEED_RPM * core::f32::consts::TAU / 60.0;
/// Target range: POSITION_TURNS output turns each way (-2880..+2880 deg), inside the
/// +-3200 deg servo feedback range.
pub const POSITION_TURNS: f32 = 8.0;
pub const MAX_POSITION_MDEG: u32 = 2_880_000;
/// Servo feedback reports position as int16 * 0.1 deg (CubeMars range +-3200 deg).
pub const FEEDBACK_POSITION_RAD: f32 = 3200.0 * core::f32::consts::PI / 180.0;
/// Every motor boots as the AK40-10 (advanced settings word 12 = 2); the host selects the
/// AK60-39 (3) with a settings write while the motor is OFF.
pub const DEFAULT_MOTOR_MODEL: u16 = crate::cubemars::AK40_10;
/// Boot MCU position gains for the AK40-10: Kp 1 / Kd 0.05 (0.01 and 0.001 units). The
/// host overwrites them with every settings write.
pub const DEFAULT_KP_CENTI: u16 = 100;
pub const DEFAULT_KD_MILLI: u16 = 50;

#[derive(Clone, Copy, Debug)]
pub struct MotionConfig {
    pub commissioned: bool,
    pub open_rad: f32,
    pub closed_rad: f32,
    pub min_speed_rad_s: f32,
    pub max_speed_rad_s: f32,
    pub min_torque_nm: f32,
    pub max_torque_nm: f32,
    pub kp: f32,
    pub kd: f32,
    pub position_tolerance_rad: f32,
    pub stopped_speed_rad_s: f32,
    pub contact_dwell_ms: u64,
    pub feedback_timeout_ms: u64,
    pub host_timeout_ms: u64,
    pub activation_timeout_ms: u64,
    pub release_timeout_ms: u64,
    pub release_speed_rad_s: f32,
    pub release_torque_nm: f32,
}

/// Standalone motor test enabled at the user's request; no linkage is attached.
/// User-requested command limits: 0..rated speed and 0..9 N*m. The 9 N*m force scale stays
/// as the host knows it; every request is capped at the motor's peak (AK40-10 4.1 N*m; the
/// AK60-39 72 N*m peak is above the scale, so it reaches 9 N*m, 37.5 % of its rating).
/// The +-8 turn endpoints (-2880..+2880 deg) are standalone test limits.
/// These are not measured gripper geometry.
pub const MOTION: MotionConfig = MotionConfig {
    commissioned: true,
    open_rad: -POSITION_TURNS * core::f32::consts::TAU,
    closed_rad: POSITION_TURNS * core::f32::consts::TAU,
    min_speed_rad_s: 0.0,
    max_speed_rad_s: MAX_SPEED_RAD_S,
    min_torque_nm: 0.0,
    max_torque_nm: 9.0,
    kp: 8.0,
    kd: 0.4,
    // About 0.286 degrees: above 0.1-degree feedback resolution and below a
    // 5.625-degree target increment on the 0..1024 position scale.
    position_tolerance_rad: 0.005,
    stopped_speed_rad_s: 0.03,
    contact_dwell_ms: 150,
    feedback_timeout_ms: 100,
    host_timeout_ms: 1000,
    activation_timeout_ms: 1000,
    release_timeout_ms: 5000,
    release_speed_rad_s: 0.05,
    release_torque_nm: 0.3,
};

/// Ball-screw adaptive gripper on the motor output (Li & Kim, "Backdrivable Adaptive
/// Gripper: Design and Force Measurement", v16) and the Robotiq-style activation that
/// finds its travel. The motor encoder restarts at zero on every motor power-up, so each
/// activation closes onto the closed end, makes it the motor's temporary origin (0 deg),
/// then opens by the computed nut travel; 0..255 then spans fully open..fully closed. The
/// AK40-10 output turns the screw directly. Speeds and margins are nut travel, so the 2 mm
/// and 3 mm grippers move their fingers alike.
#[derive(Clone, Copy, Debug)]
pub struct Gripper {
    /// Ball-screw lead: nut travel per output turn.
    pub lead_mm: f32,
    /// Output turns per turn the motor reports: 1 when its feedback is the true output
    /// angle. Measured on the gripper, since nut travel is what the fingers follow.
    pub rotation_scale: f32,
    /// Nut travel from fully closed to fully open.
    pub nut_travel_mm: f32,
    /// +1 when positive output rotation closes the fingers.
    pub close_sign: f32,
    /// Nut speed of both homing moves.
    pub homing_speed_mm_s: f32,
    /// Torque ceiling of both homing moves; a stall at it marks an end.
    pub homing_torque_nm: f32,
    /// Past this share of the travel the opening runs at `approach` times the homing speed
    /// and torque, so an open end nearer than computed is met gently.
    pub approach_from: f32,
    pub approach_speed: f32,
    pub approach_torque: f32,
    /// A stall must last this long (fresh feedback samples).
    pub homing_dwell_ms: u64,
    pub homing_timeout_ms: u64,
    /// Fully closed (255) stops this much nut travel short of the closed stall.
    pub closed_margin_mm: f32,
    /// An opening that stalls makes fully open (0) this much short of the stall.
    pub open_margin_mm: f32,
}

impl Gripper {
    /// Reported output rotation that moves the nut `mm`.
    pub fn rad(&self, mm: f32) -> f32 {
        mm / (self.lead_mm * self.rotation_scale) * core::f32::consts::TAU
    }
    /// Output rotation from fully closed to fully open.
    pub fn travel_rad(&self) -> f32 {
        self.rad(self.nut_travel_mm)
    }
    pub fn homing_speed_rad_s(&self) -> f32 {
        self.rad(self.homing_speed_mm_s)
    }
}

/// Finger stroke, both fingers together (paper Table 1).
pub const GRIPPER_STROKE_MM: f32 = 100.0;
/// Nut travel per mm of one finger. By virtual work over eqs. (12)-(15),
/// f_e = (pi * eta / L) * dy/dx. The measured grip force at the 40 mm stroke, 240.74 N
/// (L = 2 mm) and 160.10 N (L = 3 mm) at 0.57 N*m, gives eta * dy/dx = 0.2689 and 0.2682;
/// with ball-screw efficiency eta = 0.9 that is dy/dx = 0.298 for both leads.
pub const NUT_PER_FINGER_MM: f32 = 0.2685 / 0.9;
/// 50 mm per finger * 0.298 = 14.9 mm of nut travel: 7.46 turns (2686 deg) at 2 mm lead,
/// 4.97 turns (1790 deg) at 3 mm. dy/dx changes along the stroke, so an opening that
/// stalls earlier takes that stall as the open end. On Left (2 mm) this opening is the
/// one wanted (2026-10-29).
pub const GRIPPER_LEFT: Gripper = Gripper {
    lead_mm: 2.0,
    rotation_scale: 1.0,
    nut_travel_mm: GRIPPER_STROKE_MM / 2.0 * NUT_PER_FINGER_MM,
    // On the installed grippers positive output rotation opens the fingers (Left, first
    // activation 2026-10-29). `--features close-positive` restores positive = closing.
    close_sign: if cfg!(feature = "close-positive") {
        1.0
    } else {
        -1.0
    },
    // Left 10.05 rad/s (96 RPM), Right 4.47 rad/s (43 RPM): the same finger speed.
    homing_speed_mm_s: 3.2,
    // About 127 N per finger with 2 mm lead (422 N/(N*m), GRIP_FORCE_AK60-6_ko.md).
    homing_torque_nm: 0.3,
    // The last 15 % of the opening at half speed. Full torque: at half torque Left
    // stalled on its own friction and stopped short (2026-10-29).
    approach_from: 0.85,
    approach_speed: 0.5,
    approach_torque: 1.0,
    homing_dwell_ms: 100,
    homing_timeout_ms: 30_000,
    // 18 deg with 2 mm lead, 12 deg with 3 mm.
    closed_margin_mm: 0.1,
    open_margin_mm: 0.5,
};
/// Right has the 3 mm screw. Its AK60-6 reported 1/1.5 of its rotation (2026-10-29); the
/// AK40-10 (reduction 10) is expected to report its true output angle, so the scale is 1.0,
/// the 11.25 mm opening is 1350 deg and Right homes at 6.70 rad/s (2026-10-31).
pub const GRIPPER_RIGHT: Gripper = Gripper {
    lead_mm: 3.0,
    rotation_scale: 1.0,
    // Fully open at 11.25 mm of nut travel from the closed end, set on the gripper
    // (2026-10-29: 900 deg reported by the AK60-6 at 4.5 mm per reported turn).
    nut_travel_mm: 900.0 / 360.0 * 3.0 * 1.5,
    ..GRIPPER_LEFT
};
/// Left (slave 1) and Right (slave 2): per motor on the dual-motor board, by slave address
/// on a one-motor board.
pub const GRIPPERS: &[Gripper] = if cfg!(feature = "dual-motor") {
    &[GRIPPER_LEFT, GRIPPER_RIGHT]
} else if MODBUS_SLAVE_ID == 2 {
    &[GRIPPER_RIGHT]
} else {
    &[GRIPPER_LEFT]
};
const _: () = assert!(GRIPPERS.len() == CHANNEL_COUNT);

impl MotionConfig {
    pub fn valid(&self) -> bool {
        let finite = [
            self.open_rad,
            self.closed_rad,
            self.min_speed_rad_s,
            self.max_speed_rad_s,
            self.min_torque_nm,
            self.max_torque_nm,
            self.kp,
            self.kd,
            self.position_tolerance_rad,
            self.stopped_speed_rad_s,
            self.release_speed_rad_s,
            self.release_torque_nm,
        ]
        .iter()
        .all(|v| v.is_finite());
        finite
            && self.commissioned
            && self.open_rad.abs() <= FEEDBACK_POSITION_RAD
            && self.closed_rad.abs() <= FEEDBACK_POSITION_RAD
            && (self.closed_rad - self.open_rad).abs() > 2.0 * self.position_tolerance_rad
            && self.min_speed_rad_s >= 0.0
            && self.max_speed_rad_s > 0.0
            && self.max_speed_rad_s >= self.min_speed_rad_s
            && self.max_speed_rad_s <= 65.0
            && self.min_torque_nm >= 0.0
            && self.max_torque_nm > 0.0
            && self.max_torque_nm >= self.min_torque_nm
            && self.max_torque_nm <= 25.0
            && self.kp > 0.0
            && self.kd >= 0.0
            && self.position_tolerance_rad > 0.0
            && self.stopped_speed_rad_s > 0.0
            && self.feedback_timeout_ms > 0
            && self.host_timeout_ms > 0
            && self.activation_timeout_ms > 0
            && self.contact_dwell_ms > 0
            && self.release_timeout_ms > 0
            && self.release_speed_rad_s > 0.0
            && self.release_speed_rad_s <= self.max_speed_rad_s
            && self.release_torque_nm > 0.0
            && self.release_torque_nm <= self.max_torque_nm
    }
}
