//! Routes one Modbus RTU request to the motor channels it addresses.
use crate::{
    adc,
    advanced::{self, Mode, Settings},
    config::Channel,
    control::Command,
    extended, monitor,
    robotiq::{self, MAX_ADU, REGISTER_BYTES},
};

/// What one channel shows the host: its live settings and command, the status
/// banks its bridge last published, and the board's CAN monitor and ADC block.
#[derive(Clone, Copy)]
pub struct Registers {
    pub settings: Settings,
    pub command: Command,
    pub status: [u8; REGISTER_BYTES],
    pub status1000: [u8; extended::STATUS_BYTES],
    pub advanced_status: [u8; advanced::STATUS_BYTES],
    pub monitor: [u8; monitor::BYTES],
    pub adc: [u8; adc::BYTES],
}

/// A request one channel accepted: the reply length in `tx` (0 for a broadcast) and
/// whatever it wrote.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Handled {
    pub len: usize,
    pub command: Option<Command>,
    pub settings: Option<Settings>,
    /// CAN monitor command (monitor::START_SCAN or CLEAR).
    pub monitor: Option<u16>,
    /// New ADC enabled mask (adc::MASK_REGISTER).
    pub adc_mask: Option<u16>,
}

/// One request against one channel as slave `slave`: the CAN monitor, ADC, advanced,
/// extended and Robotiq banks in that order. `outputs` is that channel's Robotiq output bank.
pub fn handle(
    slave: u8,
    rx: &[u8],
    tx: &mut [u8; MAX_ADU],
    outputs: &mut [u8; REGISTER_BYTES],
    registers: &Registers,
) -> Option<Handled> {
    if let Some(reply) = monitor::process(slave, rx, tx, &registers.monitor) {
        return Some(Handled {
            len: reply.len,
            command: None,
            settings: None,
            monitor: reply.command,
            adc_mask: None,
        });
    }
    if let Some(reply) = adc::process(slave, rx, tx, &registers.adc) {
        return Some(Handled {
            len: reply.len,
            command: None,
            settings: None,
            monitor: None,
            adc_mask: reply.mask,
        });
    }
    let mut settings = registers.settings;
    if let Some(reply) = advanced::process(
        slave,
        rx,
        tx,
        &mut settings,
        registers.command,
        &registers.advanced_status,
    ) {
        return Some(Handled {
            len: reply.len,
            command: None,
            settings: reply.written.then_some(settings),
            monitor: None,
            adc_mask: None,
        });
    }
    let mut command = registers.command;
    if let Some(reply) = extended::process(slave, rx, tx, &mut command, &registers.status1000) {
        return Some(Handled {
            len: reply.len,
            command: reply.written.then_some(command),
            settings: None,
            monitor: None,
            adc_mask: None,
        });
    }
    robotiq::process(slave, rx, tx, outputs, &registers.status).map(|reply| Handled {
        len: reply.len,
        command: reply
            .written
            .then(|| robotiq::Command::from_bytes(outputs).into()),
        // Legacy Robotiq masters always select the original position controller.
        settings: reply.written.then_some(Settings {
            mode: Mode::Position,
            ..registers.settings
        }),
        monitor: None,
        adc_mask: None,
    })
}

/// Serves a request on every channel it addresses: the one channel with that slave
/// address, or every channel for a broadcast (address 0). `registers(i)` reads
/// channel i just before it is served; `accepted(i, ..)` follows each channel that
/// took the request. Returns the reply length in `tx`, 0 when nothing is sent.
pub fn serve(
    channels: &[Channel],
    rx: &[u8],
    tx: &mut [u8; MAX_ADU],
    outputs: &mut [[u8; REGISTER_BYTES]],
    mut registers: impl FnMut(usize) -> Registers,
    mut accepted: impl FnMut(usize, Handled),
) -> usize {
    let Some(&address) = rx.first() else {
        return 0;
    };
    let mut reply = 0;
    for (index, channel) in channels.iter().enumerate() {
        if address != 0 && address != channel.slave {
            continue;
        }
        if let Some(handled) = handle(
            channel.slave,
            rx,
            tx,
            &mut outputs[index],
            &registers(index),
        ) {
            accepted(index, handled);
            reply = handled.len;
        }
        if address != 0 {
            break;
        }
    }
    reply
}
