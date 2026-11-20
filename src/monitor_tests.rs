use crate::{
    config::DUAL_MOTOR,
    control::Command,
    dispatch::{self, Registers},
    monitor::{
        self, BusErrorCode, CLEAR, MIT_EXIT, Monitor, Probe, REGISTERS, START_SCAN, Scan, process,
    },
    robotiq::{self, MAX_ADU, REGISTER_BYTES},
};
use std::vec::Vec;

fn adu(bytes: &[u8]) -> Vec<u8> {
    let mut b = bytes.to_vec();
    b.extend_from_slice(&robotiq::crc16(bytes).to_le_bytes());
    b
}
fn word(bytes: &[u8], index: usize) -> u16 {
    u16::from_be_bytes([bytes[index * 2], bytes[index * 2 + 1]])
}
/// Entry `i` of encoded registers: (extended, id, count, age, dlc, data).
fn entry(regs: &[u8], i: usize) -> (bool, u32, u16, u16, u16, [u8; 8]) {
    let at = 10 + i * 9;
    let head = word(regs, at);
    let mut data = [0; 8];
    data.copy_from_slice(&regs[(at + 5) * 2..(at + 9) * 2]);
    (
        head & 0x8000 != 0,
        u32::from(head & 0x1fff) << 16 | u32::from(word(regs, at + 1)),
        word(regs, at + 2),
        word(regs, at + 3),
        word(regs, at + 4),
        data,
    )
}
/// Runs a scan to the end with every frame queued; returns what it sent.
fn full_scan(m: &mut Monitor) -> Vec<Probe> {
    let mut sent = Vec::new();
    for _ in 0..1000 {
        m.scan(true, |p| {
            sent.push(p);
            true
        });
    }
    sent
}

#[test]
fn frames_are_grouped_by_identifier_and_kind() {
    let mut m = Monitor::new();
    m.record(true, 0x2968, &[0, 1, 2, 3, 4, 5, 6, 0x77], 10);
    m.record(true, 0x2968, &[9; 8], 25);
    // The same number as a standard identifier is a different frame.
    m.record(false, 0x2968 & 0x7ff, &[1, 2], 30);
    m.record(false, 0x001, &[], 40);
    let regs = m.encode(100);
    assert_eq!(word(&regs, 0), monitor::MAGIC);
    assert_eq!(word(&regs, 1), monitor::VERSION);
    assert_eq!((word(&regs, 2), word(&regs, 3)), (0, 4));
    assert_eq!(entry(&regs, 0), (true, 0x2968, 2, 75, 8, [9; 8]));
    assert_eq!(
        entry(&regs, 1),
        (false, 0x2968 & 0x7ff, 1, 70, 2, [1, 2, 0, 0, 0, 0, 0, 0])
    );
    assert_eq!(entry(&regs, 2), (false, 1, 1, 60, 0, [0; 8]));
    // Unused entries: not marked used, age 0xFFFF.
    assert_eq!(word(&regs, 10 + 3 * 9) & 0x4000, 0);
    assert_eq!(word(&regs, 10 + 3 * 9 + 3), u16::MAX);
    assert_ne!(word(&regs, 10) & 0x4000, 0);
}

#[test]
fn a_full_table_replaces_the_identifier_seen_longest_ago() {
    let mut m = Monitor::new();
    for id in 0..monitor::ENTRIES as u32 {
        m.record(true, 0x2900 + id, &[], 100 + u64::from(id));
    }
    // Refresh the oldest one so the second oldest goes.
    m.record(true, 0x2900, &[], 200);
    m.record(true, 0x2a00, &[], 300);
    let regs = m.encode(300);
    let ids: Vec<u32> = (0..monitor::ENTRIES).map(|i| entry(&regs, i).1).collect();
    assert!(ids.contains(&0x2900) && ids.contains(&0x2a00));
    assert!(!ids.contains(&0x2901));
}

#[test]
fn error_counters_and_controller_state_are_reported() {
    let mut m = Monitor::new();
    m.record_error(BusErrorCode::Acknowledge);
    m.record_error(BusErrorCode::BusPassive);
    // ESR: REC 3, TEC 136, LEC 3 (ACK), passive + warning.
    m.set_esr(3 << 24 | 136 << 16 | 3 << 4 | 0b011);
    let regs = m.encode(0);
    assert_eq!(word(&regs, 4), 2);
    assert_eq!(word(&regs, 5), BusErrorCode::BusPassive as u16);
    assert_eq!(word(&regs, 6), 136);
    assert_eq!(word(&regs, 7), 3);
    assert_eq!(word(&regs, 8), 0x33);
    m.command(CLEAR);
    let regs = m.encode(0);
    assert_eq!((word(&regs, 2), word(&regs, 3), word(&regs, 4)), (0, 0, 0));
}

#[test]
fn the_scan_sends_only_disable_frames_to_every_id_once() {
    let mut m = Monitor::new();
    // Nothing is sent before the host asks.
    assert!(full_scan(&mut m).is_empty());
    assert!(m.command(START_SCAN));
    let sent = full_scan(&mut m);
    assert_eq!(m.scan_state(), Scan::Done);
    let servo: Vec<u32> = sent
        .iter()
        .filter(|p| p.extended)
        .map(|p| p.id & 0xff)
        .collect();
    let mit: Vec<u32> = sent.iter().filter(|p| !p.extended).map(|p| p.id).collect();
    assert_eq!(servo, (1..=255).collect::<Vec<_>>());
    assert_eq!(mit, (1..=127).collect::<Vec<_>>());
    for p in &sent {
        if p.extended {
            // Servo 0x0F: disable, no data.
            assert_eq!((p.id & !0xff, p.len), (0xf00, 0));
        } else {
            // MIT exit motor mode only: never enter (FC) or set zero (FE).
            assert_eq!((p.data, p.len), (MIT_EXIT, 8));
            assert!(p.id <= 0x7ff);
        }
    }
    let regs = m.encode(0);
    assert_eq!(word(&regs, 9) >> 12, 2);
}

#[test]
fn the_scan_waits_for_free_mailboxes_and_stops_when_a_motor_is_switched_on() {
    let mut m = Monitor::new();
    m.command(START_SCAN);
    // The MIT probe could not be queued: the same ID is tried again.
    let mut tries = Vec::new();
    m.scan(true, |p| {
        tries.push(p.extended);
        p.extended
    });
    assert_eq!(m.scan_state(), Scan::Running(1));
    m.scan(true, |_| true);
    assert_eq!(m.scan_state(), Scan::Running(2));
    assert_eq!(tries, [true, false]);
    let mut sent = 0;
    m.scan(false, |_| {
        sent += 1;
        true
    });
    assert_eq!((m.scan_state(), sent), (Scan::Aborted(2), 0));
    assert_eq!(word(&m.encode(0), 9), 3 << 12 | 2);
    // An aborted scan stays stopped until the host starts it again.
    assert!(full_scan(&mut m).is_empty());
    m.command(START_SCAN);
    assert_eq!(m.scan_state(), Scan::Running(1));
    assert!(!m.command(7));
}

#[test]
fn modbus_reads_and_commands_on_the_monitor_bank() {
    let mut m = Monitor::new();
    m.record(true, 0x2901, &[0, 0, 0, 0, 0, 0, 30, 0x77], 5);
    let regs = m.encode(5);
    let mut tx = [0; MAX_ADU];
    let r = process(
        2,
        &adu(&[2, 4, 0x14, 0, 0, REGISTERS as u8]),
        &mut tx,
        &regs,
    )
    .unwrap();
    assert_eq!(r.len, 3 + REGISTERS * 2 + 2);
    assert_eq!(r.command, None);
    assert_eq!(tx[..3], [2, 4, REGISTERS as u8 * 2]);
    assert_eq!(tx[3..3 + REGISTERS * 2], regs);
    let crc = robotiq::crc16(&tx[..r.len - 2]).to_le_bytes();
    assert_eq!(tx[r.len - 2..r.len], crc);
    // FC03 alias, a window inside the bank.
    let r = process(2, &adu(&[2, 3, 0x14, 10, 0, 2]), &mut tx, &regs).unwrap();
    assert_eq!(tx[..r.len - 2], [2, 3, 4, 0xc0, 0, 0x29, 0x01]);
    // Out of range, broadcast and foreign reads are not this bank's.
    for rx in [
        adu(&[2, 4, 0x14, 0, 0, REGISTERS as u8 + 1]),
        adu(&[2, 4, 0x13, 0xff, 0, 2]),
        adu(&[2, 4, 0x14, 0, 0, 0]),
        adu(&[0, 4, 0x14, 0, 0, 2]),
        adu(&[3, 4, 0x14, 0, 0, 2]),
        adu(&[2, 6, 0x14, 0, 0, 9]),
        adu(&[2, 6, 0x14, 1, 0, 1]),
    ] {
        assert_eq!(process(2, &rx, &mut tx, &regs), None, "{rx:02x?}");
    }
    let r = process(2, &adu(&[2, 6, 0x14, 0, 0, 1]), &mut tx, &regs).unwrap();
    assert_eq!((r.len, r.command), (8, Some(START_SCAN)));
    assert_eq!(tx[..6], [2, 6, 0x14, 0, 0, 1]);
    let r = process(2, &adu(&[0, 6, 0x14, 0, 0, 2]), &mut tx, &regs).unwrap();
    assert_eq!((r.len, r.command), (0, Some(CLEAR)));
}

#[test]
fn either_motor_address_serves_the_board_monitor() {
    let mut m = Monitor::new();
    m.record(false, 0x7f, &[1], 0);
    let monitor = m.encode(0);
    let registers = |_| Registers {
        settings: Default::default(),
        command: Command::default(),
        status: [0; REGISTER_BYTES],
        status1000: [0; 32],
        advanced_status: [0; 32],
        monitor,
        adc: [0; crate::adc::BYTES],
    };
    let mut outputs = [[0; REGISTER_BYTES]; 2];
    for slave in [1, 2] {
        let mut tx = [0; MAX_ADU];
        let mut accepted = Vec::new();
        let len = dispatch::serve(
            &DUAL_MOTOR,
            &adu(&[slave, 4, 0x14, 0, 0, 2]),
            &mut tx,
            &mut outputs,
            registers,
            |i, h| accepted.push((i, h)),
        );
        assert_eq!(len, 9);
        assert_eq!(tx[..7], [slave, 4, 4, 0x43, 0x4d, 0, 1]);
        assert_eq!(accepted.len(), 1);
        assert_eq!(accepted[0].0, usize::from(slave - 1));
        let mut accepted = Vec::new();
        dispatch::serve(
            &DUAL_MOTOR,
            &adu(&[slave, 6, 0x14, 0, 0, 1]),
            &mut tx,
            &mut outputs,
            registers,
            |i, h| accepted.push((i, h)),
        );
        let handled = accepted[0].1;
        assert_eq!(handled.monitor, Some(START_SCAN));
        assert_eq!((handled.command, handled.settings), (None, None));
    }
}
