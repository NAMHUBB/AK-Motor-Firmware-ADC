//! Runtime controller settings and telemetry, additive to the scale-1000 bank.
use crate::{control::Command, robotiq};

pub const COMMAND_BASE: u16 = 0x1200;
pub const STATUS_BASE: u16 = 0x1300;
pub const REGISTERS: usize = 16;
pub const STATUS_BYTES: usize = 32;

/// Word 0. 1 was MIT mode, which only the removed AK80-9 / AK60-6 profiles had; it is
/// rejected like any unknown mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u16)]
pub enum Mode {
    #[default]
    Position = 0,
    Velocity = 2,
    Cst = 3,
    Brake = 4,
    Profile = 5,
    Hybrid = 6,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    pub mode: Mode,
    pub kp_centi: u16,
    pub kd_milli: u16,
    /// Words 3-4 held the gains of the removed MIT mode. They are still stored and read
    /// back, so the register layout is unchanged, but nothing uses them.
    pub mit_kp_centi: u16,
    pub mit_kd_milli: u16,
    pub position_mdeg: i32,
    pub velocity_mrad: i16,
    pub torque_mnm: i16,
    pub acceleration_mrad: u16,
    pub threshold_mnm: u16,
    pub dwell_ms: u16,
    /// Word 12: cubemars::AK40_10 (2) or cubemars::AK60_39 (3). It changes only while the
    /// motor is OFF (rACT 0), since it sets the torque constant and speed scale.
    pub motor_model: u16,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            mode: Mode::Position,
            kp_centi: crate::config::DEFAULT_KP_CENTI,
            kd_milli: crate::config::DEFAULT_KD_MILLI,
            mit_kp_centi: 800,
            mit_kd_milli: 400,
            position_mdeg: 0,
            velocity_mrad: 0,
            torque_mnm: 0,
            acceleration_mrad: 1000,
            threshold_mnm: 600,
            dwell_ms: 60,
            motor_model: crate::config::DEFAULT_MOTOR_MODEL,
        }
    }
}
impl Settings {
    pub fn same_hybrid_target(self, other: Self) -> bool {
        self.threshold_mnm == other.threshold_mnm
            && self.dwell_ms == other.dwell_ms
            && self.torque_mnm == other.torque_mnm
            && self.position_mdeg == other.position_mdeg
    }
    pub fn valid(self) -> bool {
        use crate::{config, cubemars};
        cubemars::supported(self.motor_model)
            && self.kp_centi > 0
            && self.kp_centi <= 50000
            && self.kd_milli <= 5000
            && self.mit_kp_centi <= 50000
            && self.mit_kd_milli <= 5000
            && self.position_mdeg.unsigned_abs() <= config::MAX_POSITION_MDEG
            && self.velocity_mrad.unsigned_abs()
                <= cubemars::profile(self.motor_model).velocity_limit_mrad()
            && self.torque_mnm.unsigned_abs() <= 9000
            && (1..=20000).contains(&self.acceleration_mrad)
            && (1..=9000).contains(&self.threshold_mnm)
            && (10..=1000).contains(&self.dwell_ms)
    }
    pub fn words(self) -> [u16; REGISTERS] {
        [
            self.mode as u16,
            self.kp_centi,
            self.kd_milli,
            self.mit_kp_centi,
            self.mit_kd_milli,
            (self.position_mdeg as u32 >> 16) as u16,
            self.position_mdeg as u16,
            self.velocity_mrad as u16,
            self.torque_mnm as u16,
            self.acceleration_mrad,
            self.threshold_mnm,
            self.dwell_ms,
            self.motor_model,
            0,
            0,
            0,
        ]
    }
    pub fn decode(words: [u16; REGISTERS]) -> Option<Self> {
        let mode = match words[0] {
            0 => Mode::Position,
            2 => Mode::Velocity,
            3 => Mode::Cst,
            4 => Mode::Brake,
            5 => Mode::Profile,
            6 => Mode::Hybrid,
            _ => return None,
        };
        if words[13..].iter().any(|&w| w != 0) {
            return None;
        }
        let value = Self {
            mode,
            kp_centi: words[1],
            kd_milli: words[2],
            mit_kp_centi: words[3],
            mit_kd_milli: words[4],
            position_mdeg: ((u32::from(words[5]) << 16) | u32::from(words[6])) as i32,
            velocity_mrad: words[7] as i16,
            torque_mnm: words[8] as i16,
            acceleration_mrad: words[9],
            threshold_mnm: words[10],
            dwell_ms: words[11],
            motor_model: words[12],
        };
        value.valid().then_some(value)
    }
}

/// Full, atomic FC16 settings writes. Mode changes are refused only during a run (rGTO=1),
/// motor model changes whenever the motor is on (rACT=1); word 12 must be 2 or 3.
pub fn process(
    slave: u8,
    rx: &[u8],
    tx: &mut [u8; robotiq::MAX_ADU],
    settings: &mut Settings,
    command: Command,
    status: &[u8; STATUS_BYTES],
) -> Option<robotiq::Reply> {
    if rx.len() < 8
        || (rx[0] != slave && rx[0] != 0)
        || robotiq::crc16(&rx[..rx.len() - 2])
            != u16::from_le_bytes([rx[rx.len() - 2], rx[rx.len() - 1]])
    {
        return None;
    }
    let word = |at| u16::from_be_bytes([rx[at], rx[at + 1]]);
    let length;
    let written;
    match rx[1] {
        0x10 => {
            if rx.len() != 9 + REGISTERS * 2
                || word(2) != COMMAND_BASE
                || word(4) != REGISTERS as u16
                || rx[6] != (REGISTERS * 2) as u8
            {
                return None;
            }
            let next = Settings::decode(core::array::from_fn(|i| word(7 + i * 2)))?;
            if command.active() && settings.motor_model != next.motor_model {
                return None;
            }
            if command.go() && settings.mode != next.mode {
                return None;
            }
            if command.go() && settings.mode == Mode::Hybrid && !settings.same_hybrid_target(next) {
                return None;
            }
            *settings = next;
            tx[..6].copy_from_slice(&rx[..6]);
            length = 6;
            written = true;
        }
        3 | 4 => {
            if rx[0] == 0 || rx.len() != 8 {
                return None;
            }
            let count = usize::from(word(4));
            let source: [u8; STATUS_BYTES];
            let (base, registers) = if word(2) >= STATUS_BASE {
                source = *status;
                (STATUS_BASE, 16)
            } else if rx[1] == 3 {
                let mut bytes = [0; STATUS_BYTES];
                for (i, w) in settings.words().iter().enumerate() {
                    bytes[i * 2..i * 2 + 2].copy_from_slice(&w.to_be_bytes());
                }
                source = bytes;
                (COMMAND_BASE, REGISTERS)
            } else {
                return None;
            };
            let offset = usize::from(word(2).checked_sub(base)?);
            if count == 0 || offset.checked_add(count)? > registers {
                return None;
            }
            tx[..3].copy_from_slice(&[slave, rx[1], (count * 2) as u8]);
            tx[3..3 + count * 2].copy_from_slice(&source[offset * 2..(offset + count) * 2]);
            length = 3 + count * 2;
            written = false;
        }
        _ => return None,
    }
    let crc = robotiq::crc16(&tx[..length]).to_le_bytes();
    tx[length..length + 2].copy_from_slice(&crc);
    Some(robotiq::Reply {
        len: if rx[0] == 0 { 0 } else { length + 2 },
        written,
    })
}
