//! CAN bus monitor: every frame the board receives, grouped by identifier, the
//! controller's error state, and an ID scan, read over Modbus at 0x1400.
//!
//! Registers (FC03/FC04, any motor's slave address, 82 words from 0x1400):
//! 0 magic 0x434D, 1 version, 2-3 frames received, 4 bus errors, 5 last bus error
//! (`BusErrorCode`), 6 TEC, 7 REC, 8 ESR flags (bit0 warning, bit1 passive, bit2
//! bus-off, bits 4-6 LEC), 9 scan state << 12 | next scan ID. Then 8 entries of 9
//! words: id (bit15 extended, bit14 used, bits 0-12 id >> 16), id low 16 bits, frame
//! count, ms since last seen, DLC, data bytes 0-7.
//!
//! FC06 to 0x1400: 1 starts an ID scan, 2 clears the table and the counters.
use crate::robotiq::{self, MAX_ADU};

pub const BASE: u16 = 0x1400;
pub const ENTRIES: usize = 8;
const HEADER_WORDS: usize = 10;
const ENTRY_WORDS: usize = 9;
pub const REGISTERS: usize = HEADER_WORDS + ENTRIES * ENTRY_WORDS;
pub const BYTES: usize = REGISTERS * 2;
pub const MAGIC: u16 = 0x434d;
pub const VERSION: u16 = 1;
pub const START_SCAN: u16 = 1;
pub const CLEAR: u16 = 2;
/// The scan addresses servo driver IDs 1..=255 and MIT IDs 1..=127.
pub const SCAN_LAST_SERVO_ID: u16 = 255;
pub const SCAN_LAST_MIT_ID: u16 = 127;
/// MIT "exit motor mode": a motor in MIT mode answers it with its state.
pub const MIT_EXIT: [u8; 8] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd];

/// Bus error kinds as reported in word 5.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u16)]
pub enum BusErrorCode {
    Stuff = 1,
    Form = 2,
    Acknowledge = 3,
    BitRecessive = 4,
    BitDominant = 5,
    Crc = 6,
    Software = 7,
    BusOff = 8,
    BusPassive = 9,
    BusWarning = 10,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Scan {
    Idle,
    /// Next ID to probe.
    Running(u16),
    Done,
    /// A motor was switched on mid-scan; the scan stopped there.
    Aborted(u16),
}

/// One frame the scan sends: a servo disable (extended 0x0F00 | id, no data) or the
/// MIT exit command (standard id). Both only switch a motor off.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Probe {
    pub extended: bool,
    pub id: u32,
    pub data: [u8; 8],
    pub len: usize,
}

#[derive(Clone, Copy, Debug, Default)]
struct Entry {
    used: bool,
    extended: bool,
    id: u32,
    count: u16,
    last_ms: u64,
    len: u8,
    data: [u8; 8],
}

pub struct Monitor {
    entries: [Entry; ENTRIES],
    frames: u32,
    errors: u16,
    last_error: u16,
    esr: u32,
    scan: Scan,
}

impl Default for Monitor {
    fn default() -> Self {
        Self::new()
    }
}

impl Monitor {
    pub const fn new() -> Self {
        Self {
            entries: [Entry {
                used: false,
                extended: false,
                id: 0,
                count: 0,
                last_ms: 0,
                len: 0,
                data: [0; 8],
            }; ENTRIES],
            frames: 0,
            errors: 0,
            last_error: 0,
            esr: 0,
            scan: Scan::Idle,
        }
    }

    pub fn scan_state(&self) -> Scan {
        self.scan
    }

    /// Counts a received frame under its identifier. A full table drops the
    /// identifier seen longest ago.
    pub fn record(&mut self, extended: bool, id: u32, data: &[u8], now: u64) {
        self.frames = self.frames.saturating_add(1);
        let slot = self
            .entries
            .iter()
            .position(|e| e.used && e.extended == extended && e.id == id)
            .or_else(|| self.entries.iter().position(|e| !e.used))
            .unwrap_or_else(|| {
                (0..ENTRIES)
                    .min_by_key(|&i| self.entries[i].last_ms)
                    .unwrap_or(0)
            });
        let entry = &mut self.entries[slot];
        if !(entry.used && entry.extended == extended && entry.id == id) {
            *entry = Entry {
                used: true,
                extended,
                id,
                ..Entry::default()
            };
        }
        let len = data.len().min(8);
        entry.count = entry.count.saturating_add(1);
        entry.last_ms = now;
        entry.len = len as u8;
        entry.data = [0; 8];
        entry.data[..len].copy_from_slice(&data[..len]);
    }

    pub fn record_error(&mut self, code: BusErrorCode) {
        self.errors = self.errors.saturating_add(1);
        self.last_error = code as u16;
    }

    /// Latest CAN error status register (ESR): flags, LEC, TEC and REC.
    pub fn set_esr(&mut self, esr: u32) {
        self.esr = esr;
    }

    /// Applies a host command; false for an unknown value.
    pub fn command(&mut self, value: u16) -> bool {
        match value {
            START_SCAN => self.scan = Scan::Running(1),
            CLEAR => *self = Self::new(),
            _ => return false,
        }
        true
    }

    /// One scan step per call while every motor is OFF (`idle`): queues the probes
    /// for the next ID through `send` and moves on only once all of them were
    /// queued. A motor switched on stops the scan.
    pub fn scan(&mut self, idle: bool, mut send: impl FnMut(Probe) -> bool) {
        let Scan::Running(id) = self.scan else {
            return;
        };
        if !idle {
            self.scan = Scan::Aborted(id);
            return;
        }
        let servo = Probe {
            extended: true,
            id: 0xf00 | u32::from(id),
            data: [0; 8],
            len: 0,
        };
        let mit = (id <= SCAN_LAST_MIT_ID).then_some(Probe {
            extended: false,
            id: u32::from(id),
            data: MIT_EXIT,
            len: 8,
        });
        if send(servo) && mit.is_none_or(&mut send) {
            self.scan = if id >= SCAN_LAST_SERVO_ID {
                Scan::Done
            } else {
                Scan::Running(id + 1)
            };
        }
    }

    pub fn encode(&self, now: u64) -> [u8; BYTES] {
        let mut out = [0; BYTES];
        let mut put = |index: usize, value: u16| {
            out[index * 2..index * 2 + 2].copy_from_slice(&value.to_be_bytes())
        };
        put(0, MAGIC);
        put(1, VERSION);
        put(2, (self.frames >> 16) as u16);
        put(3, self.frames as u16);
        put(4, self.errors);
        put(5, self.last_error);
        put(6, ((self.esr >> 16) & 0xff) as u16);
        put(7, (self.esr >> 24) as u16);
        put(8, (self.esr & 0x77) as u16);
        put(
            9,
            match self.scan {
                Scan::Idle => 0,
                Scan::Running(id) => 1 << 12 | id,
                Scan::Done => 2 << 12 | (SCAN_LAST_SERVO_ID + 1),
                Scan::Aborted(id) => 3 << 12 | id,
            },
        );
        for (i, e) in self.entries.iter().enumerate() {
            let at = HEADER_WORDS + i * ENTRY_WORDS;
            if !e.used {
                put(at + 3, u16::MAX);
                continue;
            }
            put(
                at,
                u16::from(e.extended) << 15 | 1 << 14 | (e.id >> 16) as u16 & 0x1fff,
            );
            put(at + 1, e.id as u16);
            put(at + 2, e.count);
            put(at + 3, now.saturating_sub(e.last_ms).min(0xfffe) as u16);
            put(at + 4, u16::from(e.len));
            for pair in 0..4 {
                put(
                    at + 5 + pair,
                    u16::from_be_bytes([e.data[pair * 2], e.data[pair * 2 + 1]]),
                );
            }
        }
        out
    }
}

/// A monitor request one slave accepted: the reply length (0 for a broadcast) and
/// the command it wrote, if any.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reply {
    pub len: usize,
    pub command: Option<u16>,
}

/// FC03/FC04 reads of the monitor bank and FC06 commands to its first register.
/// Anything else is left to the other banks.
pub fn process(
    slave: u8,
    rx: &[u8],
    tx: &mut [u8; MAX_ADU],
    registers: &[u8; BYTES],
) -> Option<Reply> {
    if rx.len() != 8
        || (rx[0] != slave && rx[0] != 0)
        || robotiq::crc16(&rx[..6]) != u16::from_le_bytes([rx[6], rx[7]])
    {
        return None;
    }
    let broadcast = rx[0] == 0;
    let word = |at: usize| u16::from_be_bytes([rx[at], rx[at + 1]]);
    let (start, value) = (word(2), word(4));
    let length = match rx[1] {
        3 | 4 => {
            let offset = usize::from(start.checked_sub(BASE)?);
            let count = usize::from(value);
            if broadcast || count == 0 || offset + count > REGISTERS {
                return None;
            }
            tx[2] = (count * 2) as u8;
            tx[3..3 + count * 2].copy_from_slice(&registers[offset * 2..(offset + count) * 2]);
            3 + count * 2
        }
        6 => {
            if start != BASE || !matches!(value, START_SCAN | CLEAR) {
                return None;
            }
            tx[2..6].copy_from_slice(&rx[2..6]);
            6
        }
        _ => return None,
    };
    tx[0] = slave;
    tx[1] = rx[1];
    let crc = robotiq::crc16(&tx[..length]).to_le_bytes();
    tx[length..length + 2].copy_from_slice(&crc);
    Some(Reply {
        len: if broadcast { 0 } else { length + 2 },
        command: (rx[1] == 6).then_some(value),
    })
}
