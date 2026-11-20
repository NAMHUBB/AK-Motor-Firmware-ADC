//! ADC 4 Click (AD7175-8) strain-gauge block at 0x1500: conversion, register values and Modbus.
use crate::{
    adc::{
        self, ALL_CHANNELS, BASE, BYTES, Block, DEFAULT_MASK, MAGIC, REGISTERS, Sample, State,
        Stats, VERSION, Verdict, channel_microvolts, channel_register, decode_sample, input_pair,
        is_ad7175_8, microvolts, millivolt_parts, pair_verdict, parse_mask, process,
        reference_usable, scaled_microvolts, verdict,
    },
    config::DUAL_MOTOR,
    control::Command,
    dispatch::{self, Registers},
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
fn uv(bytes: &[u8], channel: usize) -> i32 {
    let at = (8 + channel * 2) * 2;
    i32::from_be_bytes(bytes[at..at + 4].try_into().unwrap())
}
/// The raw code of `microvolts` (rounded down to the code).
fn code(microvolts: i64) -> u32 {
    (adc::MIDSCALE + microvolts * adc::MIDSCALE / adc::REFERENCE_UV) as u32
}

#[test]
fn channel_registers_measure_the_differential_pairs_ain2n_ain2n_plus_1() {
    // AINPOS = AIN(2n), AINNEG = AIN(2n+1): the CANopen firmware's 0x8001 and 0x8043.
    assert_eq!(VERSION, 1);
    let clicker4 = [0x8001, 0x8043, 0x8085, 0x80c7, 0x8109, 0x814b];
    for (channel, value) in clicker4.into_iter().enumerate() {
        assert_eq!(channel_register(channel, true), value, "ADC{channel}");
        assert_eq!(
            channel_register(channel, false),
            value & 0x7fff,
            "ADC{channel}"
        );
    }
}

#[test]
fn codes_become_microvolts_over_the_bipolar_2_5_v_range() {
    assert_eq!(microvolts(1 << 23), 0);
    assert_eq!(microvolts(0), -2_500_000);
    assert_eq!(microvolts(0x00ff_ffff), 2_499_999);
    // One LSB is 0.298 uV; 1 mV is 3355.4 codes, so 3356 codes read as 1000 uV.
    assert_eq!(microvolts((1 << 23) + 3356), 1000);
    assert_eq!(microvolts((1 << 23) - 3356), -1000);
    // A 2 mV/V bridge on 5 V gives 10 mV full scale: 0.3 uV steps resolve it finely.
    assert_eq!(microvolts(code(10_000) + 1), 10_000);
    // The status byte never leaks into the code.
    assert_eq!(microvolts(0xff80_0000), 0);
}

#[test]
fn a_data_read_carries_its_channel_and_over_range_flag() {
    assert_eq!(
        decode_sample([0x80, 0x0d, 0x1c, 0x02]),
        Sample {
            channel: 2,
            raw: 0x800d1c,
            over_range: false
        }
    );
    let s = decode_sample([0xff, 0xff, 0xff, 0x45]);
    assert_eq!((s.channel, s.over_range), (5, true));
}

#[test]
fn only_an_ad7175_8_id_counts_as_found() {
    for id in [0x3cd0, 0x3cd4, 0x3cdf] {
        assert!(is_ad7175_8(id), "{id:#06x}");
    }
    // Missing chip (pull-up / floating low) and the two-channel AD7175-2.
    for id in [0xffff, 0x0000, 0x0cd0, 0x3ce0] {
        assert!(!is_ad7175_8(id), "{id:#06x}");
    }
}

#[test]
fn the_block_boots_with_adc0_and_adc1_and_records_only_enabled_channels() {
    let mut b = Block::new(DEFAULT_MASK);
    assert_eq!((b.mask, b.state), (0b11, State::Starting));
    b.detected(0x3cd2);
    assert_eq!(b.state, State::Running);
    assert!(b.record(Sample {
        channel: 1,
        raw: code(-1500),
        over_range: false
    }));
    assert!(!b.record(Sample {
        channel: 2,
        raw: code(5000),
        over_range: false
    }));
    assert!(!b.record(Sample {
        channel: 9,
        raw: 0,
        over_range: false
    }));
    assert_eq!((b.fresh, b.conversions), (0b10, 1));
    // ADC1 carries its gain (x 0.87038): -1500 uV in reads about -1306 uV out.
    assert_eq!(b.microvolts[1], channel_microvolts(1, code(-1500)));
    assert!((b.microvolts[1] + 1306).abs() <= 1);
    assert_eq!(b.microvolts[2], 0);
    // Over-range follows the channel's last sample.
    b.record(Sample {
        channel: 0,
        raw: 0xffffff,
        over_range: true,
    });
    assert_eq!(b.over_range, 0b01);
    b.record(Sample {
        channel: 0,
        raw: code(10),
        over_range: false,
    });
    assert_eq!(b.over_range, 0);
    // All six, then a new mask restarts every value.
    b.set_mask(ALL_CHANNELS);
    assert_eq!((b.mask, b.fresh, b.microvolts), (0x3f, 0, [0; 6]));
    assert!(b.record(Sample {
        channel: 5,
        raw: code(2_000_000),
        over_range: false
    }));
    b.set_mask(0xffff);
    assert_eq!(b.mask, 0x3f, "only six channels exist");
    // A lost chip clears the values.
    b.record(Sample {
        channel: 3,
        raw: code(42),
        over_range: false,
    });
    b.detected(0xffff);
    assert_eq!((b.state, b.id, b.fresh), (State::NotFound, 0xffff, 0));
    assert_eq!(b.microvolts, [0; 6]);
}

#[test]
fn the_block_encodes_header_and_signed_microvolts_high_word_first() {
    let mut b = Block::new(0b100011);
    b.detected(0x3cd0);
    for (channel, value) in [(0, -2_500_000), (1, 12_345), (5, 1)] {
        b.record(Sample {
            channel,
            raw: code(value),
            over_range: channel == 5,
        });
    }
    let r = b.encode();
    assert_eq!(r.len(), BYTES);
    assert_eq!(REGISTERS, 20);
    assert_eq!(
        (0..8).map(|i| word(&r, i)).collect::<Vec<_>>(),
        [MAGIC, VERSION, 0b100011, 0b100011, 0b100000, 1, 0x3cd0, 3]
    );
    assert_eq!(uv(&r, 0), -2_500_000);
    assert_eq!(uv(&r, 1), channel_microvolts(1, code(12_345)));
    assert_eq!(uv(&r, 2), 0);
    assert!(uv(&r, 5).abs() <= 1);
    // The boot registers the transport starts with.
    let boot = Block::new(DEFAULT_MASK).encode();
    assert_eq!(
        (word(&boot, 0), word(&boot, 2), word(&boot, 5)),
        (MAGIC, 3, 0)
    );
}

#[test]
fn modbus_reads_the_block_and_writes_only_a_six_bit_mask() {
    let mut b = Block::new(DEFAULT_MASK);
    b.detected(0x3cd0);
    b.record(Sample {
        channel: 0,
        raw: code(-7),
        over_range: false,
    });
    let regs = b.encode();
    let mut tx = [0; MAX_ADU];
    for fc in [3, 4] {
        let r = process(1, &adu(&[1, fc, 0x15, 0, 0, 20]), &mut tx, &regs).unwrap();
        assert_eq!((r.len, r.mask), (3 + 40 + 2, None));
        assert_eq!(tx[..3], [1, fc, 40]);
        assert_eq!(tx[3..43], regs);
        assert_eq!(
            robotiq::crc16(&tx[..43]).to_le_bytes(),
            [tx[43], tx[44]],
            "reply CRC"
        );
    }
    // A part: ADC0 alone (words 8-9).
    let r = process(2, &adu(&[2, 4, 0x15, 8, 0, 2]), &mut tx, &regs).unwrap();
    assert_eq!(r.len, 3 + 4 + 2);
    assert!((i32::from_be_bytes(tx[3..7].try_into().unwrap()) + 7).abs() <= 1);
    // Past the end, outside the block, broadcast reads, wrong slave, bad CRC.
    assert!(process(1, &adu(&[1, 4, 0x15, 0, 0, 21]), &mut tx, &regs).is_none());
    assert!(process(1, &adu(&[1, 4, 0x15, 19, 0, 2]), &mut tx, &regs).is_none());
    assert!(process(1, &adu(&[1, 4, 0x14, 0, 0, 2]), &mut tx, &regs).is_none());
    assert!(process(1, &adu(&[1, 4, 0x15, 0, 0, 0]), &mut tx, &regs).is_none());
    assert!(process(1, &adu(&[0, 4, 0x15, 0, 0, 2]), &mut tx, &regs).is_none());
    assert!(process(1, &adu(&[2, 4, 0x15, 0, 0, 2]), &mut tx, &regs).is_none());
    let mut bad = adu(&[1, 4, 0x15, 0, 0, 2]);
    bad[7] ^= 1;
    assert!(process(1, &bad, &mut tx, &regs).is_none());
    // FC06 0x1502: 0..=0x3F, echoed; a broadcast applies without a reply.
    for mask in [0, 0b1, 0b111, 0x3f] {
        let req = adu(&[1, 6, 0x15, 2, 0, mask]);
        let r = process(1, &req, &mut tx, &regs).unwrap();
        assert_eq!((r.len, r.mask), (8, Some(u16::from(mask))));
        assert_eq!(tx[..8], req[..]);
    }
    let r = process(1, &adu(&[0, 6, 0x15, 2, 0, 3]), &mut tx, &regs).unwrap();
    assert_eq!((r.len, r.mask), (0, Some(3)));
    assert!(process(1, &adu(&[1, 6, 0x15, 2, 0, 0x40]), &mut tx, &regs).is_none());
    assert!(process(1, &adu(&[1, 6, 0x15, 0, 0, 3]), &mut tx, &regs).is_none());
    assert!(process(1, &adu(&[1, 16, 0x15, 2, 0, 1]), &mut tx, &regs).is_none());
    assert_eq!(BASE, 0x1500);
}

#[test]
fn either_motor_address_serves_the_board_adc_and_takes_its_mask() {
    let mut b = Block::new(DEFAULT_MASK);
    b.detected(0x3cd0);
    let adc = b.encode();
    let registers = |_| Registers {
        settings: Default::default(),
        command: Command::default(),
        status: [0; REGISTER_BYTES],
        status1000: [0; 32],
        advanced_status: [0; 32],
        monitor: [0; crate::monitor::BYTES],
        adc,
    };
    let mut outputs = [[0; REGISTER_BYTES]; 2];
    for slave in [1, 2] {
        let mut tx = [0; MAX_ADU];
        let mut accepted = Vec::new();
        let len = dispatch::serve(
            &DUAL_MOTOR,
            &adu(&[slave, 4, 0x15, 0, 0, 3]),
            &mut tx,
            &mut outputs,
            registers,
            |i, h| accepted.push((i, h)),
        );
        assert_eq!(len, 11);
        // Version 1: differential pairs.
        assert_eq!(tx[..9], [slave, 4, 6, 0x53, 0x47, 0, 1, 0, 3]);
        assert_eq!(accepted[0].0, usize::from(slave - 1));
        assert_eq!(accepted[0].1.adc_mask, None);
        let mut accepted = Vec::new();
        dispatch::serve(
            &DUAL_MOTOR,
            &adu(&[slave, 6, 0x15, 2, 0, 0x0f]),
            &mut tx,
            &mut outputs,
            registers,
            |i, h| accepted.push((i, h)),
        );
        let handled = accepted[0].1;
        assert_eq!(handled.adc_mask, Some(0x0f));
        assert_eq!(
            (handled.command, handled.settings, handled.monitor),
            (None, None, None)
        );
    }
    // The motor banks still answer next to it: the extended status at 0x1100.
    let mut tx = [0; MAX_ADU];
    let len = dispatch::serve(
        &DUAL_MOTOR,
        &adu(&[1, 4, 0x11, 0, 0, 16]),
        &mut tx,
        &mut outputs,
        registers,
        |_, _| {},
    );
    assert_eq!(len, 3 + 32 + 2);
}

#[test]
fn the_log_prints_microvolts_as_millivolts_with_three_decimals_and_its_sign() {
    assert_eq!(millivolt_parts(-694_974), (true, 694, 974));
    assert_eq!(millivolt_parts(27_676), (false, 27, 676));
    assert_eq!(millivolt_parts(-5), (true, 0, 5));
    assert_eq!(millivolt_parts(0), (false, 0, 0));
    assert_eq!(millivolt_parts(1_000), (false, 1, 0));
    assert_eq!(millivolt_parts(-2_500_000), (true, 2500, 0));
    assert_eq!(millivolt_parts(i32::MIN), (true, 2_147_483, 648));
}

#[test]
fn adc_mask_at_build_time_picks_the_boot_channels() {
    assert_eq!(parse_mask(None), DEFAULT_MASK);
    assert_eq!(parse_mask(Some("0x3f")), 0x3f);
    assert_eq!(parse_mask(Some("0X3F")), 0x3f);
    assert_eq!(parse_mask(Some("63")), 0x3f);
    assert_eq!(parse_mask(Some("5")), 0b101);
    assert_eq!(parse_mask(Some("0")), 0);
    assert_eq!(adc::BOOT_MASK, DEFAULT_MASK, "tests build without ADC_MASK");
    for bad in ["0x40", "64", "0x", "", "3f", "-1", "0x3g"] {
        assert!(
            std::panic::catch_unwind(|| parse_mask(Some(bad))).is_err(),
            "{bad:?}"
        );
    }
}

#[test]
fn self_check_inputs_and_references_are_the_ad7175_8_codes() {
    // CH register: EN | AINPOS << 5 | AINNEG. AVDD1/5 = 10011/10100, REF+/REF- = 10101/10110.
    assert_eq!(input_pair(adc::INPUT_AVDD_POS, adc::INPUT_AVDD_NEG), 0x8274);
    assert_eq!(input_pair(adc::INPUT_REF_POS, adc::INPUT_REF_NEG), 0x82b6);
    assert_eq!(input_pair(0, 0), 0x8000);
    // SETUPCON0: bipolar (bit 12), AIN buffers (9, 8), REF_SEL (5:4). The strain pairs use
    // the internal 2.5 V reference with the buffers on (the CANopen firmware's 0x1320); the
    // pin scan against GND needs the buffers off.
    assert_eq!(adc::SETUPCON_MEASURE, 0x1320);
    assert_eq!(adc::SETUPCON_EXTERNAL, 0x1000);
    assert_eq!(adc::SETUPCON_INTERNAL, 0x1020);
    assert_eq!(adc::SETUPCON_SUPPLY_CHECK, 0x1320);
    assert_eq!(adc::SETUPCON_AVDD_UNBUFFERED, 0x1030);
}

#[test]
fn self_check_scales_avdd_and_the_reference_pins() {
    // AVDD1 = 5 V reads 1 V on (AVDD1 - AVSS)/5 against the internal 2.5 V.
    let one_volt = code(1_000_000);
    let avdd = 5 * scaled_microvolts(one_volt, adc::REFERENCE_UV);
    // The code is rounded down and x5: a few LSB (0.3 uV each) of error.
    assert!((avdd - 5_000_000).abs() < 10, "{avdd}");
    // REF+/REF- = 2.5 V against AVDD1 = 5 V is half of full scale.
    let half = (adc::MIDSCALE + adc::MIDSCALE / 2) as u32;
    assert_eq!(scaled_microvolts(half, 5_000_000), 2_500_000);
    assert!(reference_usable(2_500_000));
    assert!(reference_usable(2_450_000));
    // Floating or missing reference pins, or REF+ tied to a 5 V or 3.3 V supply.
    for bad in [0, 120_000, 1_250_000, 3_300_000, 5_000_000] {
        assert!(!reference_usable(bad), "{bad}");
    }
}

#[test]
fn self_check_tells_an_open_input_and_a_non_bridge_input_from_a_bridge() {
    let mut s = Stats::new();
    assert_eq!(s.microvolts(adc::REFERENCE_UV), (0, 0));
    for uv in [1_200, 1_204, 1_198, 1_201] {
        s.add(Sample {
            channel: 0,
            raw: code(uv),
            over_range: false,
        });
    }
    let (mean, swing) = s.microvolts(adc::REFERENCE_UV);
    assert!((mean - 1_200).abs() <= 2, "{mean}");
    assert!((swing - 6).abs() <= 1, "{swing}");
    assert_eq!(verdict(mean, swing, s.over_range), Verdict::Ok);
    // The readings seen on the board: mV jumps within 0.1 s, and -695 mV standing still.
    assert_eq!(verdict(1_688, 7_000, false), Verdict::Unstable);
    // An amplifier output a little below GND is still inside the unbuffered range; well
    // below it is not.
    assert_eq!(verdict(-28_760, 20, false), Verdict::Ok);
    assert_eq!(verdict(-694_974, 900, false), Verdict::BelowGround);
    assert_eq!(verdict(2_400_000, 50, false), Verdict::Ok);
    assert_eq!(verdict(0, 0, true), Verdict::OverRange);
    // A pair may read negative: no below-GND case there.
    assert_eq!(pair_verdict(-694_974, 900, false), Verdict::Ok);
    assert_eq!(pair_verdict(1_688, 7_000, false), Verdict::Unstable);
    assert_eq!(pair_verdict(0, 0, true), Verdict::OverRange);
    s.add(Sample {
        channel: 0,
        raw: 0x00ff_ffff,
        over_range: true,
    });
    assert!(s.over_range);
}

#[test]
fn adc1_carries_the_canopen_firmware_gain_with_the_same_integer_formula() {
    // gain = 1.0 is the plain conversion; ADC1 is x 0.87038.
    for raw in [0, code(-1500), 1 << 23, code(15_747), 0x00ff_ffff] {
        assert_eq!(channel_microvolts(0, raw), microvolts(raw), "{raw:#x}");
        let expected = ((i64::from(raw) - adc::MIDSCALE) * 2_500_000 * 87_038
            / (adc::MIDSCALE * 100_000)) as i32;
        assert_eq!(channel_microvolts(1, raw), expected, "{raw:#x}");
    }
    assert!((channel_microvolts(1, code(10_000)) - 8_704).abs() <= 1);
    assert_eq!(adc::CH_GAIN[2..], [adc::GAIN_DEN; 4]);
    // The block stores ADC1 with its gain.
    let mut b = Block::new(DEFAULT_MASK);
    b.detected(0x3cde);
    b.record(Sample {
        channel: 1,
        raw: code(10_000),
        over_range: false,
    });
    assert!((b.microvolts[1] - 8_704).abs() <= 1);
}

#[test]
fn adc0_and_adc1_also_read_at_d3000_clear_of_every_motor_bank() {
    let mut b = Block::new(DEFAULT_MASK);
    b.detected(0x3cde);
    for (channel, value) in [(0, 15_747), (1, -8_570)] {
        b.record(Sample {
            channel,
            raw: code(value),
            over_range: false,
        });
    }
    let regs = b.encode();
    let (adc0, adc1) = (b.microvolts[0], b.microvolts[1]);
    assert_eq!(adc::D3000_BASE, 3000);
    // D3000/D3001 mV, then the uV pairs, for FC03 and FC04.
    let mut tx = [0; MAX_ADU];
    for fc in [3, 4] {
        let r = process(1, &adu(&[1, fc, 0x0b, 0xb8, 0, 6]), &mut tx, &regs).unwrap();
        assert_eq!((r.len, r.mask), (3 + 12 + 2, None));
        assert_eq!(tx[..3], [1, fc, 12]);
        assert_eq!(word(&tx[3..], 0) as i16, (adc0 / 1000) as i16);
        assert_eq!(word(&tx[3..], 1) as i16, (adc1 / 1000) as i16);
        assert_eq!(i32::from_be_bytes(tx[7..11].try_into().unwrap()), adc0);
        assert_eq!(i32::from_be_bytes(tx[11..15].try_into().unwrap()), adc1);
        assert_eq!(robotiq::crc16(&tx[..15]).to_le_bytes(), [tx[15], tx[16]]);
    }
    assert_eq!(((adc0 / 1000), (adc1 / 1000)), (15, -7));
    // A part: ADC1 uV alone (D3004..D3005).
    let r = process(1, &adu(&[1, 4, 0x0b, 0xbc, 0, 2]), &mut tx, &regs).unwrap();
    assert_eq!(r.len, 3 + 4 + 2);
    assert_eq!(i32::from_be_bytes(tx[3..7].try_into().unwrap()), adc1);
    // Past the end, a write, a broadcast read: not served.
    assert!(process(1, &adu(&[1, 4, 0x0b, 0xb8, 0, 7]), &mut tx, &regs).is_none());
    assert!(process(1, &adu(&[1, 4, 0x0b, 0xbd, 0, 2]), &mut tx, &regs).is_none());
    assert!(process(1, &adu(&[1, 6, 0x0b, 0xb8, 0, 1]), &mut tx, &regs).is_none());
    assert!(process(1, &adu(&[0, 4, 0x0b, 0xb8, 0, 6]), &mut tx, &regs).is_none());
    // Through the dispatcher: D3000 goes to the ADC on both motor addresses, while the motor
    // banks D1000 (outputs) and D2000 (status) still answer as before.
    let registers = |_| Registers {
        settings: Default::default(),
        command: Command::default(),
        status: [0x11; REGISTER_BYTES],
        status1000: [0; 32],
        advanced_status: [0; 32],
        monitor: [0; crate::monitor::BYTES],
        adc: regs,
    };
    let mut outputs = [[0; REGISTER_BYTES]; 2];
    for slave in [1, 2] {
        let len = dispatch::serve(
            &DUAL_MOTOR,
            &adu(&[slave, 4, 0x0b, 0xb8, 0, 6]),
            &mut tx,
            &mut outputs,
            registers,
            |_, _| {},
        );
        assert_eq!(len, 3 + 12 + 2);
        assert_eq!(i32::from_be_bytes(tx[7..11].try_into().unwrap()), adc0);
        let len = dispatch::serve(
            &DUAL_MOTOR,
            &adu(&[slave, 4, 0x07, 0xd0, 0, 8]),
            &mut tx,
            &mut outputs,
            registers,
            |_, _| {},
        );
        assert_eq!(
            (len, tx[3]),
            (3 + 16 + 2, 0x11),
            "D2000 is still the motor status"
        );
        let len = dispatch::serve(
            &DUAL_MOTOR,
            &adu(&[slave, 3, 0x03, 0xe8, 0, 8]),
            &mut tx,
            &mut outputs,
            registers,
            |_, _| {},
        );
        assert_eq!(len, 3 + 16 + 2, "D1000 is still the motor command");
    }
}
