//! Strain-gauge inputs ADC0..ADC5: an AD7175-8 24-bit sigma-delta ADC (MikroE ADC 4 Click)
//! on the Clicker 4 for STM32F4, served over Modbus at 0x1500. Each channel measures a
//! differential pair, as the CANopen firmware does (`../stm-clicker4-adc-can-485-261119/src/
//! adc4.rs`): ADCn = AIN(2n) - AIN(2n+1), bridge output + on AIN(2n) and - on AIN(2n+1), against
//! the internal 2.5 V reference with the input buffers on, and ADCn scaled by CH_GAIN[n]
//! (ADC1 x 0.87038). 2026-11-20; on 2026-11-19 each channel was one pin against GND (version 2),
//! which reads the bridge's common-mode voltage (about 2.5 V, over range) instead of its output.
//! The SPI driver is `src/ad7175.rs`; everything here is hardware-free and tested on the host.
//!
//! Wiring, the Clicker 4 pins on this STM32F407: SPI3 PC10 = SCK, PC11 = MISO (Click SDO),
//! PC12 = MOSI (Click SDI), PD15 = CS, plus 3.3 V and GND.
//!
//! Registers (FC03/FC04, any motor's slave address, 20 words from 0x1500):
//! 0 magic 0x5347 ("SG"), 1 version (1 = differential pairs, 2 = single-ended against GND), 2 enabled mask (bit n = ADCn), 3 fresh mask (converted
//! since the last mask change), 4 over-range mask (the channel's last sample saturated),
//! 5 state (`State`), 6 AD7175 ID register as read, 7 conversions (wrapping count),
//! 8-19 ADC0..ADC5 in microvolts as i32 (times CH_GAIN), high word first (0 while disabled or
//! not yet converted).
//!
//! FC06 to 0x1502: the enabled mask, 0..=0x3F. Boot default 0x03 (ADC0 and ADC1).
//!
//! The same ADC0 and ADC1 also read at D3000..D3005 (FC03/FC04, read only), the layout of the
//! CANopen firmware: D3000/D3001 ADC0/ADC1 in 1 mV (i16), D3002/D3003 ADC0 and D3004/D3005 ADC1
//! in uV (i32, high word first). D3000 is clear of every motor bank: Robotiq D1000..D1007 and
//! D2000..D2007, and 0x1000..0x14FF (2026-11-20: the CANopen firmware's ADC at D2010/D2020 sat on
//! the motor status bank D2000).
use crate::robotiq::{self, MAX_ADU};

pub const BASE: u16 = 0x1500;
pub const CHANNELS: usize = 6;
const HEADER_WORDS: usize = 8;
pub const REGISTERS: usize = HEADER_WORDS + CHANNELS * 2;
pub const BYTES: usize = REGISTERS * 2;
pub const MAGIC: u16 = 0x5347;
/// 1: ADCn = AIN(2n) - AIN(2n+1) (since 2026-11-20, and until 2026-11-19).
/// 2 (2026-11-19 only): ADCn = AIN(2n) against GND.
pub const VERSION: u16 = 1;
/// The writable enabled-mask register.
pub const MASK_REGISTER: u16 = BASE + 2;
/// ADC0 and ADC1 in the CANopen firmware's layout (decimal register numbers, as D3000).
pub const D3000_BASE: u16 = 3000;
pub const D3000_REGISTERS: usize = 6;
pub const D3000_BYTES: usize = D3000_REGISTERS * 2;
pub const ALL_CHANNELS: u16 = (1 << CHANNELS) - 1;
/// ADC0 and ADC1 at boot.
pub const DEFAULT_MASK: u16 = 0b11;
/// The enabled mask at boot: `ADC_MASK=0x3f cargo run --release` converts (and logs) all six
/// without a host writing 0x1502. Hex (`0x..`) or decimal, 0..=0x3F; unset = DEFAULT_MASK.
pub const BOOT_MASK: u16 = parse_mask(option_env!("ADC_MASK"));

/// `ADC_MASK` as a mask; an invalid value stops the build.
pub const fn parse_mask(value: Option<&str>) -> u16 {
    let Some(text) = value else {
        return DEFAULT_MASK;
    };
    let bytes = text.as_bytes();
    let (radix, mut i) =
        if bytes.len() > 2 && bytes[0] == b'0' && (bytes[1] == b'x' || bytes[1] == b'X') {
            (16, 2)
        } else {
            (10, 0)
        };
    assert!(i < bytes.len(), "ADC_MASK is empty");
    let mut mask: u32 = 0;
    while i < bytes.len() {
        let digit = match bytes[i] {
            b'0'..=b'9' => bytes[i] - b'0',
            b'a'..=b'f' if radix == 16 => bytes[i] - b'a' + 10,
            b'A'..=b'F' if radix == 16 => bytes[i] - b'A' + 10,
            _ => panic!("ADC_MASK must be 0x00..0x3F or 0..63"),
        };
        mask = mask * radix + digit as u32;
        assert!(
            mask <= ALL_CHANNELS as u32,
            "ADC_MASK must be 0x00..0x3F or 0..63"
        );
        i += 1;
    }
    mask as u16
}

/// Microvolts as millivolts for the log: sign, whole millivolts and the three decimals
/// (-694974 uV = "-694.974 mV"), so -5 uV keeps its sign as "-0.005".
pub const fn millivolt_parts(microvolts: i32) -> (bool, u32, u32) {
    let magnitude = microvolts.unsigned_abs();
    (microvolts < 0, magnitude / 1000, magnitude % 1000)
}

// Startup self-check (src/ad7175.rs): internal test inputs of the AD7175-8 (channel register
// AINPOS / AINNEG codes) and the three reference choices of SETUPCON0.
pub const INPUT_TEMP_POS: u16 = 0b10001; // temperature sensor+
pub const INPUT_TEMP_NEG: u16 = 0b10010; // temperature sensor-
/// The temperature sensor: about 477 uV/K, so about 140 mV at room temperature.
pub const TEMP_UV_PER_K: i64 = 477;
pub const INPUT_AVDD_POS: u16 = 0b10011; // ((AVDD1 - AVSS) / 5)+
pub const INPUT_AVDD_NEG: u16 = 0b10100; // ((AVDD1 - AVSS) / 5)-
pub const INPUT_REF_POS: u16 = 0b10101; // REF+
pub const INPUT_REF_NEG: u16 = 0b10110; // REF-
/// The negative input of every channel: REF-, tied to GND on the ADC 4 Click.
pub const INPUT_GND: u16 = INPUT_REF_NEG;
/// AIN0..AIN16: the input pins the startup scan measures against GND.
pub const INPUT_PINS: u16 = 17;
/// REF_SEL 00, the REF+/REF- pins: the Clicker 4 setup (SETUPCON0).
pub const SETUPCON_EXTERNAL: u16 = SETUPCON0;
/// REF_SEL 10, the internal 2.5 V reference (ADCMODE REF_EN is set), input buffers off.
pub const SETUPCON_INTERNAL: u16 = 0x1020;
/// The internal reference with the input buffers on: the AVDD1 / 5 supply check.
pub const SETUPCON_SUPPLY_CHECK: u16 = 0x1320;
/// The strain channels: bipolar, AIN buffers on, the internal 2.5 V reference, as the CANopen
/// firmware (0x1320). The conversion assumes 2.5 V, so it does not depend on the Click's REF
/// jumper (4.096 V by default, which made every value 0.61 times too small).
pub const SETUPCON_MEASURE: u16 = SETUPCON_SUPPLY_CHECK;
/// REF_SEL 11, AVDD1 - AVSS, input buffers off (REF- sits on AVSS, outside the buffer range):
/// measures the REF+/REF- pins.
pub const SETUPCON_AVDD_UNBUFFERED: u16 = 0x1030;

/// An enabled channel register converting `positive` against `negative` with setup 0.
pub const fn input_pair(positive: u16, negative: u16) -> u16 {
    0x8000 | (positive << 5) | negative
}
/// A 24-bit bipolar code in microvolts for a reference of `reference_uv`.
pub fn scaled_microvolts(raw: u32, reference_uv: i64) -> i64 {
    (i64::from(raw & 0x00ff_ffff) - MIDSCALE) * reference_uv / MIDSCALE
}
/// The REF+/REF- pins carry the 2.5 V every conversion assumes (within 4 %).
pub fn reference_usable(reference_uv: i64) -> bool {
    (2_400_000..=2_600_000).contains(&reference_uv)
}

/// Raw codes of one self-check measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stats {
    pub count: u32,
    sum: i64,
    pub min: u32,
    pub max: u32,
    pub over_range: bool,
}
impl Default for Stats {
    fn default() -> Self {
        Self::new()
    }
}
impl Stats {
    pub const fn new() -> Self {
        Self {
            count: 0,
            sum: 0,
            min: u32::MAX,
            max: 0,
            over_range: false,
        }
    }
    pub fn add(&mut self, sample: Sample) {
        self.count += 1;
        self.sum += i64::from(sample.raw);
        self.min = self.min.min(sample.raw);
        self.max = self.max.max(sample.raw);
        self.over_range |= sample.over_range;
    }
    pub fn mean_raw(&self) -> u32 {
        if self.count == 0 {
            return MIDSCALE as u32;
        }
        (self.sum / i64::from(self.count)) as u32
    }
    /// Mean and peak-to-peak swing in microvolts for this reference.
    pub fn microvolts(&self, reference_uv: i64) -> (i64, i64) {
        let swing = if self.count == 0 {
            0
        } else {
            scaled_microvolts(self.max, reference_uv) - scaled_microvolts(self.min, reference_uv)
        };
        (scaled_microvolts(self.mean_raw(), reference_uv), swing)
    }
}

/// What the self-check says about one strain channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    Ok,
    /// The input is outside +-2.5 V.
    OverRange,
    /// More than 1 mV of swing within 0.3 s: an open (floating) input.
    Unstable,
    /// Steady but below GND by more than the unbuffered input allows (AVSS - 50 mV): the
    /// amplifier output goes negative or its ground is not the Click GND.
    BelowGround,
}
/// The lowest pin voltage the unbuffered AD7175-8 input accepts (AVSS = GND - 50 mV).
pub const BELOW_GROUND_UV: i64 = -50_000;
pub fn verdict(mean_uv: i64, swing_uv: i64, over_range: bool) -> Verdict {
    if over_range {
        Verdict::OverRange
    } else if swing_uv > 1_000 {
        Verdict::Unstable
    } else if mean_uv < BELOW_GROUND_UV {
        Verdict::BelowGround
    } else {
        Verdict::Ok
    }
}

/// Bipolar full scale: +-2.5 V reference (the Clicker 4 conversion), in microvolts.
pub const REFERENCE_UV: i64 = 2_500_000;
/// 2^23: the bipolar code of 0 V.
pub const MIDSCALE: i64 = 1 << 23;

// AD7175-8 registers (datasheet register map) and the Clicker 4 setup values.
pub const REG_STATUS: u8 = 0x00;
pub const REG_ADCMODE: u8 = 0x01;
pub const REG_IFMODE: u8 = 0x02;
pub const REG_DATA: u8 = 0x04;
pub const REG_ID: u8 = 0x07;
pub const REG_CH0: u8 = 0x10;
pub const REG_SETUPCON0: u8 = 0x20;
pub const REG_FILTCON0: u8 = 0x28;
/// Command byte of a register read (bit 6 = R/W).
pub const READ: u8 = 0x40;
/// SETUPCON0: bipolar, AIN+ / AIN- buffers OFF, REF_SEL 00 (external REF+/REF-).
/// With the buffers on, every input must stay 0.1 V above AVSS (= GND); a pin measured
/// against GND sits at 0 V, and an amplifier output at its zero sits near it. Unbuffered,
/// the inputs may go down to AVSS - 50 mV. (Until 2026-11-19: 0x1300, buffers on.)
pub const SETUPCON0: u16 = 0x1000;
/// FILTCON0: sinc5 + sinc1, 100 SPS.
pub const FILTCON0: u16 = 0x060e;
/// ADCMODE: internal reference on, single-cycle settling, continuous conversion.
pub const ADCMODE_CONTINUOUS: u16 = 0xa000;
/// ADCMODE with MODE = standby: every channel disabled.
pub const ADCMODE_STANDBY: u16 = 0xa020;
/// IFMODE: DATA_STAT, the status byte (with the channel number) follows every data read.
pub const IFMODE: u16 = 0x0040;
/// ID register of an AD7175-8: 0x3CDx.
pub const ID_MASK: u16 = 0xfff0;
pub const AD7175_8_ID: u16 = 0x3cd0;

pub fn is_ad7175_8(id: u16) -> bool {
    id & ID_MASK == AD7175_8_ID
}

/// The positive input of ADCn: AIN(2n), so ADC0 = AIN0 and ADC1 = AIN2 as wired on the Clicker 4.
pub const fn input_pin(channel: usize) -> usize {
    2 * channel
}
/// The negative input of ADCn: AIN(2n+1).
pub const fn negative_pin(channel: usize) -> usize {
    2 * channel + 1
}
/// Channel register n: ADCn = AIN(2n) - AIN(2n+1), setup 0. Enabled:
/// 0x8001, 0x8043, 0x8085, 0x80C7, 0x8109, 0x814B.
pub const fn channel_register(channel: usize, enabled: bool) -> u16 {
    ((enabled as u16) << 15) | ((input_pin(channel) as u16) << 5) | negative_pin(channel) as u16
}

/// A 24-bit bipolar code in microvolts: 2.5 V x (code - 2^23) / 2^23.
pub fn microvolts(raw: u32) -> i32 {
    ((i64::from(raw & 0x00ff_ffff) - MIDSCALE) * REFERENCE_UV / MIDSCALE) as i32
}
/// Per-channel output gain CH_GAIN[n] / GAIN_DEN, as in the CANopen firmware (2026-11-20:
/// ADC1 x 0.87038). At most 4.0 (400_000) keeps the product inside i64.
pub const GAIN_DEN: i64 = 100_000;
pub const CH_GAIN: [i64; CHANNELS] = [GAIN_DEN, 87_038, GAIN_DEN, GAIN_DEN, GAIN_DEN, GAIN_DEN];
/// ADCn in microvolts with its gain, the same integer formula as the CANopen firmware:
/// 2.5 V x (code - 2^23) x gain / (2^23 x GAIN_DEN).
pub fn channel_microvolts(channel: usize, raw: u32) -> i32 {
    ((i64::from(raw & 0x00ff_ffff) - MIDSCALE) * REFERENCE_UV * CH_GAIN[channel]
        / (MIDSCALE * GAIN_DEN)) as i32
}
/// The self-check verdict of a differential pair: a negative bridge output is valid, so
/// there is no below-GND case (the pins' own voltages are not measured).
pub fn pair_verdict(mean_uv: i64, swing_uv: i64, over_range: bool) -> Verdict {
    match verdict(mean_uv, swing_uv, over_range) {
        Verdict::BelowGround => Verdict::Ok,
        v => v,
    }
}

/// One DATA read with DATA_STAT: three data bytes, then the status byte.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub channel: usize,
    pub raw: u32,
    /// STATUS ADC_ERROR: the input is outside the range (the code is clamped).
    pub over_range: bool,
}
pub fn decode_sample(bytes: [u8; 4]) -> Sample {
    Sample {
        channel: usize::from(bytes[3] & 0x0f),
        raw: u32::from_be_bytes([0, bytes[0], bytes[1], bytes[2]]),
        over_range: bytes[3] & 0x40 != 0,
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u16)]
pub enum State {
    /// Before the first ID read.
    #[default]
    Starting = 0,
    Running = 1,
    /// The ID read back is not an AD7175-8: the Click is missing or miswired.
    NotFound = 2,
}

/// What the ADC block shows the host.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Block {
    pub mask: u16,
    pub fresh: u16,
    pub over_range: u16,
    pub state: State,
    pub id: u16,
    pub conversions: u16,
    pub microvolts: [i32; CHANNELS],
}
impl Block {
    pub const fn new(mask: u16) -> Self {
        Self {
            mask: mask & ALL_CHANNELS,
            fresh: 0,
            over_range: 0,
            state: State::Starting,
            id: 0,
            conversions: 0,
            microvolts: [0; CHANNELS],
        }
    }
    /// A new enabled mask: every value restarts, disabled channels read 0.
    pub fn set_mask(&mut self, mask: u16) {
        self.mask = mask & ALL_CHANNELS;
        self.fresh = 0;
        self.over_range = 0;
        self.microvolts = [0; CHANNELS];
    }
    /// The chip answered (or not) with this ID; a lost chip clears every value.
    pub fn detected(&mut self, id: u16) {
        self.id = id;
        if is_ad7175_8(id) {
            self.state = State::Running;
        } else {
            self.state = State::NotFound;
            self.fresh = 0;
            self.over_range = 0;
            self.microvolts = [0; CHANNELS];
        }
    }
    /// Takes a conversion of an enabled channel; anything else is ignored.
    pub fn record(&mut self, sample: Sample) -> bool {
        if sample.channel >= CHANNELS || self.mask & (1 << sample.channel) == 0 {
            return false;
        }
        let bit = 1 << sample.channel;
        self.microvolts[sample.channel] = channel_microvolts(sample.channel, sample.raw);
        self.fresh |= bit;
        if sample.over_range {
            self.over_range |= bit;
        } else {
            self.over_range &= !bit;
        }
        self.conversions = self.conversions.wrapping_add(1);
        true
    }
    pub const fn encode(&self) -> [u8; BYTES] {
        let mut bytes = [0; BYTES];
        let header = [
            MAGIC,
            VERSION,
            self.mask,
            self.fresh,
            self.over_range,
            self.state as u16,
            self.id,
            self.conversions,
        ];
        let mut i = 0;
        while i < HEADER_WORDS {
            let w = header[i].to_be_bytes();
            bytes[i * 2] = w[0];
            bytes[i * 2 + 1] = w[1];
            i += 1;
        }
        let mut ch = 0;
        while ch < CHANNELS {
            let v = self.microvolts[ch].to_be_bytes();
            let at = (HEADER_WORDS + ch * 2) * 2;
            bytes[at] = v[0];
            bytes[at + 1] = v[1];
            bytes[at + 2] = v[2];
            bytes[at + 3] = v[3];
            ch += 1;
        }
        bytes
    }
}

/// D3000..D3005 from the 0x1500 block: ADC0/ADC1 in mV (truncated, i16), then in uV (i32).
pub fn d3000(registers: &[u8; BYTES]) -> [u8; D3000_BYTES] {
    let uv = |channel: usize| {
        let at = (HEADER_WORDS + channel * 2) * 2;
        i32::from_be_bytes([
            registers[at],
            registers[at + 1],
            registers[at + 2],
            registers[at + 3],
        ])
    };
    let (adc0, adc1) = (uv(0), uv(1));
    let words = [
        (adc0 / 1000) as i16 as u16,
        (adc1 / 1000) as i16 as u16,
        (adc0 as u32 >> 16) as u16,
        adc0 as u16,
        (adc1 as u32 >> 16) as u16,
        adc1 as u16,
    ];
    let mut bytes = [0; D3000_BYTES];
    for (i, w) in words.iter().enumerate() {
        bytes[i * 2..i * 2 + 2].copy_from_slice(&w.to_be_bytes());
    }
    bytes
}

pub struct Reply {
    pub len: usize,
    /// A new enabled mask the host wrote.
    pub mask: Option<u16>,
}

/// FC03/FC04 reads inside the block or D3000..D3005, and FC06 writes of the enabled mask.
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
        3 | 4 if (D3000_BASE..D3000_BASE + D3000_REGISTERS as u16).contains(&start) => {
            let offset = usize::from(start - D3000_BASE);
            let count = usize::from(value);
            if broadcast || count == 0 || offset + count > D3000_REGISTERS {
                return None;
            }
            tx[2] = (count * 2) as u8;
            tx[3..3 + count * 2]
                .copy_from_slice(&d3000(registers)[offset * 2..(offset + count) * 2]);
            3 + count * 2
        }
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
            if start != MASK_REGISTER || value > ALL_CHANNELS {
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
        mask: (rx[1] == 6).then_some(value),
    })
}
