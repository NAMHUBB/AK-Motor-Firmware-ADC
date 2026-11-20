//! Shared command values, independent of the Modbus register layout:
//! position 0..1024, speed and force 0..1000.

use crate::robotiq;

pub const SCALE: u16 = 1000;
/// Position units over the open..closed travel. One Robotiq rPR step is exactly
/// 4 units (11.25 deg for +-4 turns): 512 is 0 deg, 64 units is 180 deg.
pub const POSITION_SCALE: u16 = 1024;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Command {
    pub action: u8,
    pub position: u16,
    pub speed: u16,
    pub force: u16,
}

impl Default for Command {
    fn default() -> Self {
        Self {
            action: 0,
            position: POSITION_SCALE / 2,
            speed: 0,
            force: 0,
        }
    }
}

impl Command {
    pub fn active(self) -> bool {
        self.action & 1 != 0
    }

    pub fn go(self) -> bool {
        self.action & 8 != 0
    }

    pub fn release(self) -> bool {
        self.action & 0x10 != 0
    }

    pub fn release_open(self) -> bool {
        self.action & 0x20 != 0
    }

    pub fn valid(self) -> bool {
        self.action & !0x39 == 0
            && self.position <= POSITION_SCALE
            && self.speed <= SCALE
            && self.force <= SCALE
    }
}

impl From<robotiq::Command> for Command {
    fn from(command: robotiq::Command) -> Self {
        let scaled = |value: u8| ((u32::from(value) * u32::from(SCALE) + 127) / 255) as u16;
        Self {
            action: command.action,
            position: position_from_legacy(command.position),
            speed: scaled(command.speed),
            force: scaled(command.force),
        }
    }
}

/// rPR / gPR / gPO steps are 1/256 of the travel, so 128 is exactly the midpoint
/// (0 deg) and every 32 steps is a whole turn. 255 stops one step short of the end.
pub const POSITION_STEPS: u32 = 256;
const UNITS_PER_STEP: u16 = POSITION_SCALE / POSITION_STEPS as u16;

/// Legacy rPR to position units; exact, no rounding.
pub fn position_from_legacy(value: u8) -> u16 {
    u16::from(value) * UNITS_PER_STEP
}

/// Nearest legacy position step, saturating at 255; inverts `position_from_legacy`.
pub fn legacy_position(value: u16) -> u8 {
    ((value.min(POSITION_SCALE) + UNITS_PER_STEP / 2) / UNITS_PER_STEP).min(255) as u8
}

/// Round to the nearest legacy 0..255 speed / force value, saturating out-of-range input.
pub fn legacy_value(value: u16) -> u8 {
    ((u32::from(value.min(SCALE)) * 255 + u32::from(SCALE) / 2) / u32::from(SCALE)) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_inactive_at_midpoint_and_flags_match_robotiq() {
        let command = Command::default();
        assert_eq!(command.position, 512);
        assert_eq!(command.speed, 0);
        assert_eq!(command.force, 0);
        assert!(command.valid());
        assert!(!command.active());
        assert!(!command.go());
        assert!(!command.release());
        assert!(!command.release_open());
        let command = Command {
            action: 0x39,
            ..command
        };
        assert!(command.valid());
        assert!(command.active());
        assert!(command.go());
        assert!(command.release());
        assert!(command.release_open());
    }

    #[test]
    fn reserved_action_bits_and_every_out_of_range_field_are_invalid() {
        for action in [2, 4, 0x40, 0x80, 0xff] {
            assert!(
                !Command {
                    action,
                    ..Command::default()
                }
                .valid()
            );
        }
        for field in 0..3 {
            let mut command = Command::default();
            match field {
                0 => command.position = POSITION_SCALE + 1,
                1 => command.speed = 1001,
                _ => command.force = 1001,
            }
            assert!(!command.valid());
        }
    }

    #[test]
    fn legacy_conversion_rounds_without_overflow_and_preserves_every_legacy_step() {
        for value in 0..=u8::MAX {
            let command = Command::from(robotiq::Command {
                action: 0x39,
                position: value,
                speed: value,
                force: value,
            });
            assert!(command.valid());
            assert_eq!(command.action, 0x39);
            assert_eq!(legacy_position(command.position), value);
            assert_eq!(legacy_value(command.speed), value);
            assert_eq!(legacy_value(command.force), value);
        }
        for (value, expected) in [(0, 0), (127, 498), (128, 502), (255, 1000)] {
            assert_eq!(
                Command::from(robotiq::Command {
                    speed: value,
                    ..robotiq::Command::default()
                })
                .speed,
                expected
            );
        }
        // Position: 4 units per step, 128 is exactly the midpoint, 16 steps is 180 deg.
        for (value, expected) in [
            (0, 0),
            (32, 128),
            (112, 448),
            (127, 508),
            (128, 512),
            (144, 576),
            (224, 896),
            (255, 1020),
        ] {
            assert_eq!(
                Command::from(robotiq::Command {
                    position: value,
                    ..robotiq::Command::default()
                })
                .position,
                expected
            );
        }
        assert_eq!(legacy_position(512), 128);
        assert_eq!(legacy_position(1020), 255);
        assert_eq!(legacy_position(POSITION_SCALE), 255);
        assert_eq!(legacy_position(u16::MAX), 255);
        assert_eq!(legacy_value(0), 0);
        assert_eq!(legacy_value(500), 128);
        assert_eq!(legacy_value(1000), 255);
        assert_eq!(legacy_value(u16::MAX), 255);
    }
}
