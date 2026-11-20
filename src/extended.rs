//! Extended Modbus registers for native commands: position 0..1024, speed / force 0..1000.
//!
//! Command writes must replace all four registers in one validated transaction.
//! Status is supplied by the caller as a snapshot; FC23 returns that snapshot.

use crate::{control, robotiq};

pub const COMMAND_BASE: u16 = 0x1000;
pub const COMMAND_REGISTERS: usize = 4;
pub const STATUS_BASE: u16 = 0x1100;
pub const STATUS_REGISTERS: usize = 16;
pub const STATUS_BYTES: usize = STATUS_REGISTERS * 2;
const COMMAND_BYTES: usize = COMMAND_REGISTERS * 2;

fn word(bytes: &[u8], offset: usize) -> u16 {
    u16::from_be_bytes([bytes[offset], bytes[offset + 1]])
}

fn range(start: u16, count: u16, base: u16, registers: usize) -> Option<core::ops::Range<usize>> {
    let offset = usize::from(start.checked_sub(base)?);
    if count == 0 || offset.checked_add(usize::from(count))? > registers {
        return None;
    }
    Some(offset * 2..(offset + usize::from(count)) * 2)
}

fn decode_command(bytes: &[u8]) -> Option<control::Command> {
    let command = control::Command {
        action: u8::try_from(word(bytes, 0)).ok()?,
        position: word(bytes, 2),
        speed: word(bytes, 4),
        force: word(bytes, 6),
    };
    command.valid().then_some(command)
}

fn command_bytes(command: control::Command) -> [u8; COMMAND_BYTES] {
    let mut bytes = [0; COMMAND_BYTES];
    for (index, value) in [
        u16::from(command.action),
        command.position,
        command.speed,
        command.force,
    ]
    .into_iter()
    .enumerate()
    {
        bytes[index * 2..index * 2 + 2].copy_from_slice(&value.to_be_bytes());
    }
    bytes
}

fn finish(
    tx: &mut [u8; robotiq::MAX_ADU],
    length: usize,
    broadcast: bool,
    written: bool,
) -> robotiq::Reply {
    let crc = robotiq::crc16(&tx[..length]).to_le_bytes();
    tx[length..length + 2].copy_from_slice(&crc);
    robotiq::Reply {
        len: if broadcast { 0 } else { length + 2 },
        written,
    }
}

/// None means silent discard or a request outside this extension's registers.
/// The caller may then try the legacy Robotiq parser with the original request.
pub fn process(
    slave: u8,
    rx: &[u8],
    tx: &mut [u8; robotiq::MAX_ADU],
    command: &mut control::Command,
    status: &[u8; STATUS_BYTES],
) -> Option<robotiq::Reply> {
    if rx.len() < 4
        || rx.len() > robotiq::MAX_ADU
        || (rx[0] != slave && rx[0] != 0)
        || robotiq::crc16(&rx[..rx.len() - 2])
            != u16::from_le_bytes([rx[rx.len() - 2], rx[rx.len() - 1]])
    {
        return None;
    }
    let broadcast = rx[0] == 0;
    match rx[1] {
        3 | 4 => {
            if broadcast || rx.len() != 8 {
                return None;
            }
            let start = word(rx, 2);
            let count = word(rx, 4);
            let encoded = command_bytes(*command);
            let (bytes, selected): (&[u8], _) = if rx[1] == 3 && start < STATUS_BASE {
                (
                    &encoded,
                    range(start, count, COMMAND_BASE, COMMAND_REGISTERS)?,
                )
            } else {
                (status, range(start, count, STATUS_BASE, STATUS_REGISTERS)?)
            };
            let length = selected.len();
            tx[0] = slave;
            tx[1] = rx[1];
            tx[2] = length as u8;
            tx[3..3 + length].copy_from_slice(&bytes[selected]);
            Some(finish(tx, 3 + length, false, false))
        }
        0x10 => {
            if rx.len() != 9 + COMMAND_BYTES
                || word(rx, 2) != COMMAND_BASE
                || word(rx, 4) != COMMAND_REGISTERS as u16
                || usize::from(rx[6]) != COMMAND_BYTES
            {
                return None;
            }
            let next = decode_command(&rx[7..7 + COMMAND_BYTES])?;
            // Commit only after every field, including the last one, is valid.
            *command = next;
            tx[0] = slave;
            tx[1] = rx[1];
            tx[2..6].copy_from_slice(&rx[2..6]);
            Some(finish(tx, 6, broadcast, true))
        }
        0x17 => {
            if broadcast
                || rx.len() != 13 + COMMAND_BYTES
                || word(rx, 6) != COMMAND_BASE
                || word(rx, 8) != COMMAND_REGISTERS as u16
                || usize::from(rx[10]) != COMMAND_BYTES
            {
                return None;
            }
            let selected = range(word(rx, 2), word(rx, 4), STATUS_BASE, STATUS_REGISTERS)?;
            let next = decode_command(&rx[11..11 + COMMAND_BYTES])?;
            // Both read and write ranges must be valid before the command changes.
            *command = next;
            let length = selected.len();
            tx[0] = slave;
            tx[1] = rx[1];
            tx[2] = length as u8;
            tx[3..3 + length].copy_from_slice(&status[selected]);
            Some(finish(tx, 3 + length, false, true))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn adu(bytes: &[u8]) -> Vec<u8> {
        let mut frame = bytes.to_vec();
        frame.extend_from_slice(&robotiq::crc16(bytes).to_le_bytes());
        frame
    }

    fn write(slave: u8, values: [u16; 4]) -> Vec<u8> {
        let mut frame = Vec::from([slave, 0x10, 0x10, 0, 0, 4, 8]);
        for value in values {
            frame.extend_from_slice(&value.to_be_bytes());
        }
        adu(&frame)
    }

    fn read(slave: u8, function: u8, start: u16, count: u16) -> Vec<u8> {
        let mut frame = Vec::from([slave, function]);
        frame.extend_from_slice(&start.to_be_bytes());
        frame.extend_from_slice(&count.to_be_bytes());
        adu(&frame)
    }

    fn read_write(slave: u8, start: u16, count: u16, values: [u16; 4]) -> Vec<u8> {
        let mut frame = Vec::from([slave, 0x17]);
        frame.extend_from_slice(&start.to_be_bytes());
        frame.extend_from_slice(&count.to_be_bytes());
        frame.extend_from_slice(&[0x10, 0, 0, 4, 8]);
        for value in values {
            frame.extend_from_slice(&value.to_be_bytes());
        }
        adu(&frame)
    }

    fn status() -> [u8; STATUS_BYTES] {
        core::array::from_fn(|index| index as u8)
    }

    fn crc_is_valid(reply: robotiq::Reply, tx: &[u8; robotiq::MAX_ADU]) {
        assert_eq!(
            robotiq::crc16(&tx[..reply.len - 2]),
            u16::from_le_bytes([tx[reply.len - 2], tx[reply.len - 1]])
        );
    }

    #[test]
    fn full_write_and_command_read_preserve_native_midpoint_and_endpoints() {
        let mut command = control::Command::default();
        let mut tx = [0; robotiq::MAX_ADU];
        for value in [0, 500, 1000] {
            let reply = process(
                9,
                &write(9, [0x39, value, value, value]),
                &mut tx,
                &mut command,
                &status(),
            )
            .unwrap();
            assert!(reply.written);
            assert_eq!(reply.len, 8);
            assert_eq!(&tx[..6], &[9, 0x10, 0x10, 0, 0, 4]);
            crc_is_valid(reply, &tx);
            assert_eq!(command.position, value);
            assert_eq!(command.speed, value);
            assert_eq!(command.force, value);
            let reply = process(
                9,
                &read(9, 3, COMMAND_BASE, 4),
                &mut tx,
                &mut command,
                &status(),
            )
            .unwrap();
            assert!(!reply.written);
            assert_eq!(&tx[..5], &[9, 3, 8, 0, 0x39]);
            for offset in [5, 7, 9] {
                assert_eq!(&tx[offset..offset + 2], &value.to_be_bytes());
            }
            crc_is_valid(reply, &tx);
        }
    }

    #[test]
    fn invalid_last_field_action_header_or_crc_never_partially_commits() {
        let before = control::Command {
            action: 1,
            position: 500,
            speed: 321,
            force: 654,
        };
        let mut command = before;
        let mut tx = [0; robotiq::MAX_ADU];
        let mut requests = Vec::new();
        for values in [
            [0x100, 500, 500, 500],
            [2, 500, 500, 500],
            [0x40, 500, 500, 500],
            [1, 1025, 500, 500],
            [1, 500, 1001, 500],
            [1, 500, 500, 1001],
        ] {
            requests.push(write(9, values));
        }
        for (offset, value) in [(3, 1), (5, 3), (5, 5), (6, 7)] {
            let mut frame = write(9, [0x39, 1000, 1000, 1000]);
            frame[offset] = value;
            requests.push(adu(&frame[..frame.len() - 2]));
        }
        requests.push(write(10, [0x39, 1000, 1000, 1000]));
        let mut bad_crc = write(9, [0x39, 1000, 1000, 1000]);
        bad_crc[16] ^= 1;
        requests.push(bad_crc);
        requests.push(adu(&[9, 0x10, 0x10, 0, 0, 3, 6, 0, 1, 0, 2, 0, 3]));
        requests.push(adu(&[9, 0x10]));
        requests.push(adu(&[9, 6, 0x10, 0, 0, 1]));
        let mut oversized = [0; robotiq::MAX_ADU];
        oversized[0] = 9;
        oversized[1] = 0x10;
        requests.push(adu(&oversized));
        for request in requests {
            assert!(process(9, &request, &mut tx, &mut command, &status()).is_none());
            assert_eq!(command, before);
        }
    }

    #[test]
    fn broadcast_full_write_is_silent_but_broadcast_reads_are_rejected() {
        let mut command = control::Command::default();
        let mut tx = [0; robotiq::MAX_ADU];
        let reply = process(
            9,
            &write(0, [1, 500, 0, 1000]),
            &mut tx,
            &mut command,
            &status(),
        )
        .unwrap();
        assert!(reply.written);
        assert_eq!(reply.len, 0);
        assert_eq!(command.position, 500);
        assert_eq!(command.force, 1000);
        let before = command;
        for request in [
            read(0, 3, COMMAND_BASE, 4),
            read(0, 4, STATUS_BASE, 16),
            read_write(0, STATUS_BASE, 16, [1, 0, 0, 0]),
        ] {
            assert!(process(9, &request, &mut tx, &mut command, &status()).is_none());
            assert_eq!(command, before);
        }
    }

    #[test]
    fn status_reads_support_fc04_fc03_alias_and_exact_range_boundaries() {
        let mut command = control::Command {
            force: 1000,
            ..control::Command::default()
        };
        let mut tx = [0; robotiq::MAX_ADU];
        for function in [3, 4] {
            let reply = process(
                9,
                &read(9, function, STATUS_BASE, 16),
                &mut tx,
                &mut command,
                &status(),
            )
            .unwrap();
            assert!(!reply.written);
            assert_eq!(reply.len, 37);
            assert_eq!(&tx[..3], &[9, function, 32]);
            assert_eq!(&tx[3..35], &status());
            crc_is_valid(reply, &tx);
            let reply = process(
                9,
                &read(9, function, STATUS_BASE + 15, 1),
                &mut tx,
                &mut command,
                &status(),
            )
            .unwrap();
            assert_eq!(&tx[..5], &[9, function, 2, 30, 31]);
            crc_is_valid(reply, &tx);
        }
        let reply = process(
            9,
            &read(9, 3, COMMAND_BASE + 3, 1),
            &mut tx,
            &mut command,
            &status(),
        )
        .unwrap();
        assert_eq!(&tx[..5], &[9, 3, 2, 3, 0xe8]);
        crc_is_valid(reply, &tx);
        for request in [
            read(9, 4, COMMAND_BASE, 4),
            read(9, 4, STATUS_BASE, 0),
            read(9, 4, STATUS_BASE + 15, 2),
            read(9, 4, STATUS_BASE - 1, 1),
            read(9, 4, u16::MAX, u16::MAX),
            read(9, 3, COMMAND_BASE + 3, 2),
        ] {
            let before = command;
            assert!(process(9, &request, &mut tx, &mut command, &status()).is_none());
            assert_eq!(command, before);
        }
    }

    #[test]
    fn fc23_validates_read_range_and_entire_command_before_commit() {
        let before = control::Command::default();
        let mut command = before;
        let mut tx = [0; robotiq::MAX_ADU];
        let mut requests = Vec::from([
            read_write(9, STATUS_BASE + 15, 2, [0x39, 1000, 1000, 1000]),
            read_write(9, STATUS_BASE, 0, [0x39, 1000, 1000, 1000]),
            read_write(9, STATUS_BASE, 16, [0x39, 1000, 1000, 1001]),
            read_write(9, STATUS_BASE, 16, [0x100, 1000, 1000, 1000]),
        ]);
        for (offset, value) in [(7, 1), (9, 3), (10, 7)] {
            let mut request = read_write(9, STATUS_BASE, 16, [0x39, 1000, 1000, 1000]);
            request[offset] = value;
            requests.push(adu(&request[..request.len() - 2]));
        }
        for request in requests {
            assert!(process(9, &request, &mut tx, &mut command, &status()).is_none());
            assert_eq!(command, before);
        }
        let reply = process(
            9,
            &read_write(9, STATUS_BASE, 16, [0x39, 500, 1000, 0]),
            &mut tx,
            &mut command,
            &status(),
        )
        .unwrap();
        assert!(reply.written);
        assert_eq!(command.position, 500);
        assert_eq!(command.speed, 1000);
        assert_eq!(command.force, 0);
        assert_eq!(reply.len, 37);
        assert_eq!(&tx[..3], &[9, 0x17, 32]);
        assert_eq!(&tx[3..35], &status());
        crc_is_valid(reply, &tx);
    }

    #[test]
    fn truncated_frames_and_legacy_registers_leave_command_unchanged() {
        let before = control::Command::default();
        let mut command = before;
        let mut tx = [0; robotiq::MAX_ADU];
        for request in [
            write(9, [0x39, 500, 1000, 0]),
            read_write(9, STATUS_BASE, 16, [0x39, 500, 1000, 0]),
            read(9, 4, STATUS_BASE, 16),
        ] {
            for length in 0..request.len() {
                assert!(process(9, &request[..length], &mut tx, &mut command, &status()).is_none());
                assert_eq!(command, before);
            }
        }
        for request in [
            read(9, 3, robotiq::OUTPUT_BASE, 3),
            read(9, 4, robotiq::INPUT_BASE, 3),
            adu(&[9, 0x10, 3, 0xe8, 0, 3, 6, 1, 0, 0, 255, 255, 255]),
            adu(&[
                9, 0x17, 7, 0xd0, 0, 3, 3, 0xe8, 0, 3, 6, 1, 0, 0, 255, 255, 255,
            ]),
        ] {
            assert!(process(9, &request, &mut tx, &mut command, &status()).is_none());
            assert_eq!(command, before);
        }
    }
}
