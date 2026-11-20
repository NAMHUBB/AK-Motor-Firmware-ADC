//! Supplied AK series V3 driver software manual, PDF pp. 25, 29-35: the servo-mode
//! extended-ID commands. This firmware drives the AK40-10 V3.0 KV170 and the AK60-39 V3.0
//! KV80, chosen per motor by the advanced settings motor model word (12). The AK80-9 and
//! AK60-6 profiles and their MIT commands were removed (2026-10-31).

/// Motor model codes of the advanced settings word 12. 0 (AK80-9) and 1 (AK60-6) are no
/// longer supported; a settings write with them is ignored.
pub const AK40_10: u16 = 2;
pub const AK60_39: u16 = 3;
pub const fn supported(model: u16) -> bool {
    matches!(model, AK40_10 | AK60_39)
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorProfile {
    pub kt: f32,
    /// Output-shaft torque ceiling for every torque and current request.
    pub max_torque: f32,
    /// Output-shaft rated speed from the CubeMars spec sheet; the 0..1000 speed scale.
    pub rated_speed_rpm: f32,
    pub electrical_ratio: f32,
}
impl MotorProfile {
    pub fn rated_speed_rad_s(&self) -> f32 {
        self.rated_speed_rpm * core::f32::consts::TAU / 60.0
    }
    /// Direct-velocity wire limit in signed i16 mrad/s, rounded up from the rated speed.
    pub fn velocity_limit_mrad(&self) -> u16 {
        let mrad = self.rated_speed_rad_s() * 1000.0;
        let whole = mrad as u32;
        (if (whole as f32) < mrad {
            whole + 1
        } else {
            whole
        })
        .min(i16::MAX as u32) as u16
    }
}
/// AK40-10 V3.0 KV170 (24 V driver). The v3.2.0 manual lists neither its MIT ranges nor Kt,
/// so every torque request is a servo current command, Kt is the spec-sheet rated torque /
/// rated current (1.3 N*m / 2.7 A) and the ceiling is its 4.1 N*m peak torque.
pub const AK40_10_PROFILE: MotorProfile = MotorProfile {
    kt: 1.3 / 2.7,
    max_torque: 4.1,
    rated_speed_rpm: 370.0,
    electrical_ratio: 14.0 * 10.0,
};
/// AK60-39 V3.0 KV80 (48 V, AK54-4810-2D-A2 driver; CubeMars spec sheet, 2026-11-13):
/// 24 N*m rated / 72 N*m peak, 70 RPM rated (98 RPM no load) at 48 V, 4.5 A rated / 17 A
/// peak, 14 pole pairs, 39:1. Like the AK40-10, Kt is rated torque / rated current
/// (24 / 4.5 = 5.33 N*m/A at the output); the listed motor-side 0.12 N*m/A x 39 = 4.68 would
/// ask about 14 % more current for the same torque.
pub const AK60_39_PROFILE: MotorProfile = MotorProfile {
    kt: 24.0 / 4.5,
    max_torque: 72.0,
    rated_speed_rpm: 70.0,
    electrical_ratio: 14.0 * 39.0,
};
/// The profile of a settings motor model; anything unsupported reads as the AK40-10, but
/// advanced::Settings never holds an unsupported model.
pub const fn profile(model: u16) -> MotorProfile {
    match model {
        AK60_39 => AK60_39_PROFILE,
        _ => AK40_10_PROFILE,
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Packet {
    pub id: u32,
    pub data: [u8; 8],
    pub len: usize,
}

#[derive(Clone, Copy, Debug)]
pub struct Feedback {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub iq_a: f32,
    pub temperature_c: i8,
    pub error: u8,
    pub received_ms: u64,
}

/// Request exactly zero Iq using the documented signed int32 current command,
/// keeping the motor communication active.
pub fn zero_current(id: u8) -> Packet {
    Packet {
        id: 0x100 | id as u32,
        data: [0; 8],
        len: 4,
    }
}

/// Servo current / braking / velocity packets from the supplied V3 CAN table.
pub fn scalar_command(id: u8, mode: u8, value: i32) -> Packet {
    let mut data = [0; 8];
    data[..4].copy_from_slice(&value.to_be_bytes());
    Packet {
        id: (u32::from(mode) << 8) | u32::from(id),
        data,
        len: 4,
    }
}
pub fn current_command(model: u16, id: u8, torque: f32, brake: bool) -> Packet {
    let motor = profile(model);
    let torque = torque.clamp(-motor.max_torque, motor.max_torque);
    let current_ma = (torque / motor.kt * 1000.0) as i32;
    scalar_command(id, if brake { 2 } else { 1 }, current_ma)
}
pub fn electrical_rpm(model: u16, output_rad_s: f32) -> f32 {
    output_rad_s * 60.0 * profile(model).electrical_ratio / core::f32::consts::TAU
}
pub fn velocity_command(model: u16, id: u8, velocity: f32) -> Packet {
    scalar_command(id, 3, electrical_rpm(model, velocity) as i32)
}
pub fn profile_command(model: u16, id: u8, position: f32, speed: f32, acceleration: f32) -> Packet {
    let mut data = [0; 8];
    let position = (position * 180.0 / core::f32::consts::PI * 10000.0) as i32;
    data[..4].copy_from_slice(&position.to_be_bytes());
    data[4..6].copy_from_slice(&((electrical_rpm(model, speed).abs() / 10.0) as i16).to_be_bytes());
    data[6..8].copy_from_slice(
        &((electrical_rpm(model, acceleration).abs() / 10.0) as i16)
            .max(1)
            .to_be_bytes(),
    );
    Packet {
        id: 0x600 | u32::from(id),
        data,
        len: 8,
    }
}

/// A torque request as the servo current command, capped at the motor's peak; exactly
/// zero current for zero. Non-finite requests are rejected.
pub fn torque_command(model: u16, id: u8, torque: f32) -> Option<Packet> {
    if !torque.is_finite() {
        return None;
    }
    if torque == 0.0 {
        return Some(zero_current(id));
    }
    Some(current_command(model, id, torque, false))
}

pub fn disable(id: u8) -> Packet {
    Packet {
        id: 0xF00 | id as u32,
        data: [0; 8],
        len: 0,
    }
}

/// Servo "set origin" (V3.2 manual 4.1.6), mode 0 = temporary: the current position
/// becomes 0 until the motor powers off. Mode 1 would write the motor's flash.
pub fn set_origin(id: u8) -> Packet {
    Packet {
        id: (5 << 8) | u32::from(id),
        data: [0; 8],
        len: 1,
    }
}

/// Servo feedback of motor `id`; only the velocity depends on the model (its electrical ratio).
pub fn parse_feedback(
    model: u16,
    id: u8,
    extended_id: u32,
    data: &[u8],
    now_ms: u64,
) -> Option<Feedback> {
    if extended_id != (0x2900 | id as u32) || data.len() != 8 {
        return None;
    }
    // 0x77 is a disable acknowledgement, not a fault or a fresh running sample.
    if data[7] == 0x77 {
        return None;
    }
    let i16_at = |i| i16::from_be_bytes([data[i], data[i + 1]]);
    Some(Feedback {
        position_rad: i16_at(0) as f32 * 0.1 * core::f32::consts::PI / 180.0,
        velocity_rad_s: i16_at(2) as f32 * 10.0 * (2.0 * core::f32::consts::PI)
            / (60.0 * profile(model).electrical_ratio),
        iq_a: i16_at(4) as f32 * 0.01,
        temperature_c: data[6] as i8,
        error: data[7],
        received_ms: now_ms,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        AK40_10, AK60_39, parse_feedback, profile, supported, torque_command, velocity_command,
        zero_current,
    };

    fn milliamps(model: u16, id: u8, torque: f32) -> i32 {
        let packet = torque_command(model, id, torque).unwrap();
        assert_eq!(packet.id, 0x100 | u32::from(id), "servo current command");
        assert_eq!(packet.len, 4);
        i32::from_be_bytes(packet.data[..4].try_into().unwrap())
    }

    #[test]
    fn zero_current_uses_documented_current_mode_and_four_byte_zero() {
        let packet = zero_current(0x68);
        assert_eq!(packet.id, 0x168);
        assert_eq!(packet.len, 4);
        assert_eq!(&packet.data[..packet.len], &[0, 0, 0, 0]);
        assert_eq!(i32::from_be_bytes(packet.data[..4].try_into().unwrap()), 0);
    }

    #[test]
    fn only_the_ak40_10_and_ak60_39_are_supported() {
        assert!(supported(AK40_10) && supported(AK60_39));
        for model in [0, 1, 4, 255, u16::MAX] {
            assert!(!supported(model), "{model}");
        }
    }

    #[test]
    fn torque_is_a_current_command_capped_at_the_ak40_10_peak_on_each_id() {
        for id in [1, 2] {
            assert_eq!(torque_command(AK40_10, id, 0.0), Some(zero_current(id)));
            assert_eq!(torque_command(AK40_10, id, -0.0), Some(zero_current(id)));
            // 1.3 N*m / 2.7 A: 1 N*m is 2077 mA, the 4.1 N*m peak 8515 mA.
            assert_eq!(milliamps(AK40_10, id, 1.0), 2076);
            assert_eq!(milliamps(AK40_10, id, -1.0), -2076);
            let ak40 = profile(AK40_10);
            let peak = (ak40.max_torque / ak40.kt * 1000.0) as i32;
            assert_eq!(peak, 8515);
            for request in [4.1, 9.0, 25.0, f32::MAX] {
                assert_eq!(milliamps(AK40_10, id, request), peak);
                assert_eq!(milliamps(AK40_10, id, -request), -peak);
            }
            for request in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                assert_eq!(torque_command(AK40_10, id, request), None);
            }
        }
    }

    #[test]
    fn ak60_39_torque_uses_its_rated_kt_and_72_nm_peak() {
        for id in [1, 2] {
            assert_eq!(torque_command(AK60_39, id, 0.0), Some(zero_current(id)));
            // 24 N*m / 4.5 A = 5.33 N*m/A: 1 N*m is 187 mA, the 24 N*m rating 4.5 A.
            assert_eq!(milliamps(AK60_39, id, 1.0), 187);
            assert_eq!(milliamps(AK60_39, id, -1.0), -187);
            assert!((milliamps(AK60_39, id, 24.0) - 4500).abs() <= 1);
            // The 9 N*m force scale asks 1.69 A, far below the 17 A peak current.
            assert_eq!(milliamps(AK60_39, id, 9.0), 1687);
            // 72 N*m peak: 13.5 A.
            for request in [72.0, 100.0, f32::MAX] {
                assert!((milliamps(AK60_39, id, request) - 13500).abs() <= 1);
                assert!((milliamps(AK60_39, id, -request) + 13500).abs() <= 1);
            }
            assert_eq!(torque_command(AK60_39, id, f32::NAN), None);
        }
    }

    #[test]
    fn ak60_39_speed_scale_is_70_rpm_through_14_pole_pairs_and_39_to_1() {
        let ak60 = profile(AK60_39);
        // 70 RPM = 7.330 rad/s, rounded up to 7331 mrad/s on the wire.
        assert_eq!(ak60.velocity_limit_mrad(), 7331);
        // 10 output RPM = 10 x 39 x 14 = 5460 ERPM.
        let v = velocity_command(AK60_39, 3, 10.0 * core::f32::consts::TAU / 60.0);
        assert_eq!(v.id, 0x303);
        let erpm = i32::from_be_bytes(v.data[..4].try_into().unwrap());
        assert!((erpm - 5460).abs() <= 1, "{erpm}");
    }

    #[test]
    fn feedback_is_taken_only_from_the_motors_own_id_at_ak40_10_scale() {
        // 90.0 deg, 1400 electrical RPM (10 output RPM), 1.23 A, 30 C.
        let data = [0x03, 0x84, 0x00, 0x8c, 0x00, 0x7b, 30, 0];
        let fb = parse_feedback(AK40_10, 1, 0x2901, &data, 7).unwrap();
        assert!((fb.position_rad - core::f32::consts::FRAC_PI_2).abs() < 1e-5);
        assert!((fb.velocity_rad_s - 10.0 * core::f32::consts::TAU / 60.0).abs() < 1e-4);
        assert!((fb.iq_a - 1.23).abs() < 1e-6);
        assert_eq!((fb.temperature_c, fb.error, fb.received_ms), (30, 0, 7));
        assert!(parse_feedback(AK40_10, 2, 0x2901, &data, 7).is_none());
        assert!(parse_feedback(AK40_10, 1, 0x2902, &data, 7).is_none());
        let mut ack = data;
        ack[7] = 0x77;
        assert!(parse_feedback(AK40_10, 1, 0x2901, &ack, 7).is_none());
    }

    #[test]
    fn ak60_39_feedback_velocity_uses_its_electrical_ratio() {
        // 90.0 deg, 5460 electrical RPM (546 x 10 ERPM units) = 10 output RPM, 1.23 A.
        let data = [0x03, 0x84, 0x02, 0x22, 0x00, 0x7b, 30, 0];
        let fb = parse_feedback(AK60_39, 2, 0x2902, &data, 7).unwrap();
        assert!((fb.position_rad - core::f32::consts::FRAC_PI_2).abs() < 1e-5);
        assert!((fb.velocity_rad_s - 10.0 * core::f32::consts::TAU / 60.0).abs() < 1e-4);
        assert!((fb.iq_a - 1.23).abs() < 1e-6);
    }
}
