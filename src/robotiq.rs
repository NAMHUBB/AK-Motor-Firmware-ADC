//! Robotiq bytes are packed TWO per big-endian Modbus register.
pub const OUTPUT_BASE: u16 = 0x03E8;
pub const INPUT_BASE: u16 = 0x07D0;
pub const REGISTER_COUNT: usize = 8;
pub const REGISTER_BYTES: usize = REGISTER_COUNT * 2;
pub const MAX_ADU: usize = 256;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Command {
    pub action: u8,
    pub position: u8,
    pub speed: u8,
    pub force: u8,
}
impl Command {
    pub fn from_bytes(b: &[u8; REGISTER_BYTES]) -> Self {
        Self {
            action: b[0] & 0x39,
            position: b[3],
            speed: b[4],
            force: b[5],
        }
    }
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
}

pub fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xffff;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xa001
            } else {
                crc >> 1
            };
        }
    }
    crc
}

#[derive(Clone, Copy, Debug)]
pub struct Reply {
    pub len: usize,
    pub written: bool,
}

fn range(start: u16, count: u16, base: u16) -> Option<core::ops::Range<usize>> {
    let offset = start.checked_sub(base)? as usize;
    if count == 0 || offset.checked_add(count as usize)? > REGISTER_COUNT {
        return None;
    }
    Some(offset * 2..(offset + count as usize) * 2)
}
fn word(b: &[u8], at: usize) -> u16 {
    u16::from_be_bytes([b[at], b[at + 1]])
}
fn finish(tx: &mut [u8; MAX_ADU], n: usize, broadcast: bool, written: bool) -> Reply {
    let crc = crc16(&tx[..n]).to_le_bytes();
    tx[n..n + 2].copy_from_slice(&crc);
    Reply {
        len: if broadcast { 0 } else { n + 2 },
        written,
    }
}

/// None = silent discard (including malformed, unsupported, foreign and bad CRC).
/// Validate the WHOLE frame before changing output bytes, including FC23 read range.
/// The manual specifies no Modbus exception responses.
pub fn process(
    slave: u8,
    rx: &[u8],
    tx: &mut [u8; MAX_ADU],
    outputs: &mut [u8; REGISTER_BYTES],
    status: &[u8; REGISTER_BYTES],
) -> Option<Reply> {
    if rx.len() < 4
        || rx.len() > MAX_ADU
        || (rx[0] != slave && rx[0] != 0)
        || crc16(&rx[..rx.len() - 2]) != word_le_crc(rx)
    {
        return None;
    }
    let broadcast = rx[0] == 0;
    tx[0] = slave;
    tx[1] = rx[1];
    match rx[1] {
        3 | 4 => {
            if broadcast || rx.len() != 8 {
                return None;
            }
            let start = word(rx, 2);
            let count = word(rx, 4);
            // FC03 status alias supports the older Control.pdf command examples.
            let (bytes, r) = if rx[1] == 3 && start < INPUT_BASE {
                (&*outputs, range(start, count, OUTPUT_BASE)?)
            } else {
                (status, range(start, count, INPUT_BASE)?)
            };
            tx[2] = r.len() as u8;
            let n = r.len();
            tx[3..3 + n].copy_from_slice(&bytes[r]);
            Some(finish(tx, 3 + n, false, false))
        }
        0x10 => {
            if rx.len() < 9 {
                return None;
            }
            let r = range(word(rx, 2), word(rx, 4), OUTPUT_BASE)?;
            let n = rx[6] as usize;
            if n != r.len() || rx.len() != n + 9 {
                return None;
            }
            outputs[r].copy_from_slice(&rx[7..7 + n]);
            sanitize(outputs);
            tx[2..6].copy_from_slice(&rx[2..6]);
            Some(finish(tx, 6, broadcast, true))
        }
        0x17 => {
            // Read/write is a unicast transaction, per Modbus FC23 semantics.
            if broadcast || rx.len() < 13 {
                return None;
            }
            let rd = range(word(rx, 2), word(rx, 4), INPUT_BASE)?;
            let wr = range(word(rx, 6), word(rx, 8), OUTPUT_BASE)?;
            let n = rx[10] as usize;
            if n != wr.len() || rx.len() != n + 13 {
                return None;
            }
            outputs[wr].copy_from_slice(&rx[11..11 + n]);
            sanitize(outputs);
            let n = rd.len();
            tx[2] = n as u8;
            tx[3..3 + n].copy_from_slice(&status[rd]);
            Some(finish(tx, 3 + n, false, true))
        }
        _ => None,
    }
}
fn word_le_crc(rx: &[u8]) -> u16 {
    u16::from_le_bytes([rx[rx.len() - 2], rx[rx.len() - 1]])
}
fn sanitize(outputs: &mut [u8; REGISTER_BYTES]) {
    outputs[0] &= 0x39;
    outputs[1] = 0;
    outputs[2] = 0;
    outputs[6..].fill(0);
}
