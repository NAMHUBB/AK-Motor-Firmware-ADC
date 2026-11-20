use crate::{
    advanced::{Mode, Settings},
    bridge::{Bridge, Drive},
    config::{self, CHANNELS, Channel, DUAL_MOTOR, SINGLE_MOTOR},
    control::Command,
    cubemars::{self, Feedback},
    dispatch::{self, Handled, Registers},
    robotiq::{self, MAX_ADU, REGISTER_BYTES},
};
use std::vec::Vec;

fn adu(bytes: &[u8]) -> Vec<u8> {
    let mut b = bytes.to_vec();
    b.extend_from_slice(&robotiq::crc16(bytes).to_le_bytes());
    b
}
fn word(bytes: &[u8], at: usize) -> u16 {
    u16::from_be_bytes([bytes[at], bytes[at + 1]])
}
/// Channel registers whose status word 14 carries the motor's CAN ID.
fn registers(motor_id: u8) -> Registers {
    let mut status1000 = [0; 32];
    status1000[29] = motor_id;
    Registers {
        settings: Settings::default(),
        command: Command::default(),
        status: [motor_id; REGISTER_BYTES],
        status1000,
        advanced_status: [0; 32],
        monitor: [0; crate::monitor::BYTES],
        adc: [0; crate::adc::BYTES],
    }
}
/// Serves one request on the dual-motor table; returns the reply and who accepted it.
fn serve(rx: &[u8], outputs: &mut [[u8; REGISTER_BYTES]; 2]) -> (Vec<u8>, Vec<(usize, Handled)>) {
    let mut tx = [0; MAX_ADU];
    let mut accepted = Vec::new();
    let len = dispatch::serve(
        &DUAL_MOTOR,
        rx,
        &mut tx,
        outputs,
        |i| registers(DUAL_MOTOR[i].motor_id),
        |i, handled| accepted.push((i, handled)),
    );
    (tx[..len].to_vec(), accepted)
}
fn extended_write(slave: u8) -> Vec<u8> {
    // action 9, position 512, speed 1000, force 1000
    adu(&[slave, 0x10, 0x10, 0, 0, 4, 8, 0, 9, 2, 0, 3, 0xe8, 3, 0xe8])
}
const RUN: Command = Command {
    action: 9,
    position: 512,
    speed: 1000,
    force: 1000,
};

#[test]
fn dual_motor_table_maps_slave_n_to_can_id_n_and_rejects_clashes() {
    assert_eq!(DUAL_MOTOR.map(|c| (c.slave, c.motor_id)), [(1, 1), (2, 2)]);
    assert!(config::channels_valid(&DUAL_MOTOR));
    assert!(config::channels_valid(&SINGLE_MOTOR));
    let ch = |slave, motor_id| Channel { slave, motor_id };
    assert!(!config::channels_valid(&[]));
    assert!(!config::channels_valid(&[ch(1, 1), ch(1, 2)]));
    assert!(!config::channels_valid(&[ch(1, 2), ch(2, 2)]));
    assert!(!config::channels_valid(&[ch(0, 1)]));
    assert!(!config::channels_valid(&[ch(1, 0)]));
    assert!(!config::channels_valid(&[ch(248, 1)]));
}

#[test]
fn only_the_dual_motor_feature_selects_two_motors() {
    if cfg!(feature = "dual-motor") {
        assert_eq!(CHANNELS, &DUAL_MOTOR);
    } else {
        assert_eq!(CHANNELS, &SINGLE_MOTOR);
        assert_eq!(SINGLE_MOTOR[0].slave, config::MODBUS_SLAVE_ID);
        assert_eq!(SINGLE_MOTOR[0].motor_id, config::MOTOR_ID);
    }
    assert_eq!(config::CHANNEL_COUNT, CHANNELS.len());
}

#[test]
fn a_bridge_for_can_id_two_addresses_its_frames_and_status_to_it() {
    assert_eq!(Bridge::new(config::MOTION).motor_id(), config::MOTOR_ID);
    let settings = Settings {
        mode: Mode::Velocity,
        velocity_mrad: 1000,
        ..Settings::default()
    };
    let feedback = |now| Feedback {
        position_rad: 0.0,
        velocity_rad_s: 0.0,
        iq_a: 0.0,
        temperature_c: 25,
        error: 0,
        received_ms: now,
    };
    for motor_id in [1, 2] {
        let mut b = Bridge::for_motor(config::MOTION, motor_id);
        let active = Command { action: 1, ..RUN };
        b.step_with_settings(0, Command::default(), Some(0), None, settings);
        b.step_with_settings(1, active, Some(1), Some(feedback(1)), settings);
        let out = b.step_with_settings(2, RUN, Some(2), Some(feedback(2)), settings);
        let Drive::Can(packet) = out.drive else {
            panic!("{:?}", out.drive)
        };
        // Velocity mode 3 in the servo extended ID, motor CAN ID in the low byte.
        assert_eq!(packet.id, 0x300 | motor_id as u32);
        assert_eq!(word(&out.status1000, 28), motor_id as u16);
        assert_eq!(cubemars::disable(motor_id).id, 0xf00 | motor_id as u32);
    }
}

#[test]
fn feedback_reaches_only_the_motor_with_that_can_id() {
    let data = [0, 10, 0, 0, 0, 0, 25, 0];
    assert!(cubemars::parse_feedback(cubemars::AK40_10, 1, 0x2902, &data, 5).is_none());
    assert!(cubemars::parse_feedback(cubemars::AK40_10, 2, 0x2901, &data, 5).is_none());
    let fb = cubemars::parse_feedback(cubemars::AK40_10, 2, 0x2902, &data, 5).unwrap();
    assert_eq!(fb.received_ms, 5);
}

#[test]
fn a_write_to_slave_two_drives_only_the_second_motor() {
    let mut outputs = [[0; REGISTER_BYTES]; 2];
    let (reply, accepted) = serve(&extended_write(2), &mut outputs);
    assert_eq!(accepted.len(), 1);
    assert_eq!(accepted[0].0, 1);
    assert_eq!(accepted[0].1.command, Some(RUN));
    assert_eq!(reply.len(), 8);
    assert_eq!(reply[0], 2);
    assert_eq!(
        robotiq::crc16(&reply[..6]).to_le_bytes(),
        [reply[6], reply[7]]
    );

    let (reply, accepted) = serve(&extended_write(1), &mut outputs);
    assert_eq!(accepted.iter().map(|a| a.0).collect::<Vec<_>>(), [0]);
    assert_eq!(reply[0], 1);
}

#[test]
fn a_broadcast_write_reaches_both_motors_without_a_reply() {
    let mut outputs = [[0; REGISTER_BYTES]; 2];
    let (reply, accepted) = serve(&extended_write(0), &mut outputs);
    assert!(reply.is_empty());
    assert_eq!(accepted.iter().map(|a| a.0).collect::<Vec<_>>(), [0, 1]);
    assert!(
        accepted
            .iter()
            .all(|a| a.1.command == Some(RUN) && a.1.len == 0)
    );
}

#[test]
fn each_slave_reads_its_own_motors_status() {
    let mut outputs = [[0; REGISTER_BYTES]; 2];
    for (slave, motor_id) in [(1, 1), (2, 2)] {
        let (reply, accepted) = serve(&adu(&[slave, 4, 0x11, 0, 0, 16]), &mut outputs);
        assert_eq!(accepted.len(), 1);
        assert_eq!(accepted[0].1.command, None);
        assert_eq!(reply[..3], [slave, 4, 32]);
        // Status word 14 is the CAN ID of the motor behind that slave.
        assert_eq!(word(&reply, 3 + 28), motor_id);
    }
}

#[test]
fn other_addresses_and_broadcast_reads_get_no_reply() {
    let mut outputs = [[0; REGISTER_BYTES]; 2];
    for rx in [
        extended_write(3),
        extended_write(9),
        adu(&[0, 4, 0x11, 0, 0, 16]),
        adu(&[0, 4, 0x07, 0xd0, 0, 3]),
    ] {
        let (reply, accepted) = serve(&rx, &mut outputs);
        assert!(reply.is_empty());
        assert!(
            accepted
                .iter()
                .all(|a| a.1.command.is_none() && a.1.settings.is_none())
        );
    }
    let (reply, accepted) = serve(&extended_write(3), &mut outputs);
    assert!(reply.is_empty() && accepted.is_empty());
}

#[test]
fn legacy_robotiq_outputs_are_kept_per_motor() {
    let mut outputs = [[0; REGISTER_BYTES]; 2];
    let write = adu(&[1, 0x10, 0x03, 0xe8, 0, 3, 6, 9, 0, 0, 200, 100, 50]);
    let (reply, accepted) = serve(&write, &mut outputs);
    assert_eq!(reply[0], 1);
    assert_eq!(accepted.len(), 1);
    let (index, handled) = accepted[0];
    assert_eq!(index, 0);
    assert_eq!(handled.settings.map(|s| s.mode), Some(Mode::Position));
    assert!(handled.command.is_some_and(|c| c.go()));
    assert_eq!(outputs[0][..6], [9, 0, 0, 200, 100, 50]);
    assert_eq!(outputs[1], [0; REGISTER_BYTES]);
    // Slave 2 still reads back its own, untouched output bank.
    let (reply, _) = serve(&adu(&[2, 3, 0x03, 0xe8, 0, 3]), &mut outputs);
    assert_eq!(reply[..9], [2, 3, 6, 0, 0, 0, 0, 0, 0]);
}
