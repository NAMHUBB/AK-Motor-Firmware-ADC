use core::cell::RefCell;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use stm_gripper_controller::{
    adc,
    advanced::{self, Mode, Settings},
    config,
    control::Command,
    dispatch::Registers,
    extended, monitor,
    robotiq::REGISTER_BYTES,
};
/// State the Modbus server and the CAN task share for one motor channel.
#[derive(Clone, Copy)]
pub struct Snapshot {
    pub settings: Settings,
    pub advanced_status: [u8; advanced::STATUS_BYTES],
    pub command: Command,
    pub last_host_ms: Option<u64>,
    pub reset_generation: u32,
    pub status: [u8; REGISTER_BYTES],
    pub status1000: [u8; extended::STATUS_BYTES],
}
const fn initial_status1000(motor_id: u8) -> [u8; extended::STATUS_BYTES] {
    let mut status = [0; extended::STATUS_BYTES];
    status[0] = 0x41;
    status[1] = 0x4b;
    status[3] = 7;
    // Command echo and measured position: 512 = 0 deg.
    status[8] = 2;
    status[10] = 2;
    status[22] = 0xff;
    status[23] = 0xff;
    status[29] = motor_id;
    status[30] = 3;
    status[31] = 0xe8;
    status
}
const fn initial(motor_id: u8) -> Snapshot {
    Snapshot {
        settings: Settings {
            mode: Mode::Position,
            kp_centi: config::DEFAULT_KP_CENTI,
            kd_milli: config::DEFAULT_KD_MILLI,
            mit_kp_centi: 800,
            mit_kd_milli: 400,
            position_mdeg: 0,
            velocity_mrad: 0,
            torque_mnm: 0,
            acceleration_mrad: 1000,
            threshold_mnm: 600,
            dwell_ms: 60,
            motor_model: config::DEFAULT_MOTOR_MODEL,
        },
        advanced_status: [0; advanced::STATUS_BYTES],
        command: Command {
            action: 0,
            position: 512,
            speed: 0,
            force: 0,
        },
        last_host_ms: None,
        reset_generation: 0,
        status: [0; REGISTER_BYTES],
        status1000: initial_status1000(motor_id),
    }
}
const fn initial_all() -> [Snapshot; config::CHANNEL_COUNT] {
    let mut all = [initial(0); config::CHANNEL_COUNT];
    let mut i = 0;
    while i < config::CHANNEL_COUNT {
        all[i] = initial(config::CHANNELS[i].motor_id);
        i += 1;
    }
    all
}
static SHARED: Mutex<CriticalSectionRawMutex, RefCell<[Snapshot; config::CHANNEL_COUNT]>> =
    Mutex::new(RefCell::new(initial_all()));
/// Board-wide CAN monitor: registers the CAN task last published, and a host
/// command waiting for it.
struct MonitorShared {
    registers: [u8; monitor::BYTES],
    request: Option<u16>,
}
static MONITOR: Mutex<CriticalSectionRawMutex, RefCell<MonitorShared>> =
    Mutex::new(RefCell::new(MonitorShared {
        registers: [0; monitor::BYTES],
        request: None,
    }));
/// Board-wide ADC block (ADC 4 Click): registers the ADC task last published, and an
/// enabled mask the host wrote and the task has not applied yet.
struct AdcShared {
    registers: [u8; adc::BYTES],
    request: Option<u16>,
}
static ADC: Mutex<CriticalSectionRawMutex, RefCell<AdcShared>> =
    Mutex::new(RefCell::new(AdcShared {
        registers: adc::Block::new(adc::BOOT_MASK).encode(),
        request: None,
    }));
pub fn snapshot(channel: usize) -> Snapshot {
    SHARED.lock(|s| s.borrow()[channel])
}
pub fn registers(channel: usize) -> Registers {
    let s = snapshot(channel);
    Registers {
        settings: s.settings,
        command: s.command,
        status: s.status,
        status1000: s.status1000,
        advanced_status: s.advanced_status,
        monitor: MONITOR.lock(|m| m.borrow().registers),
        adc: ADC.lock(|a| a.borrow().registers),
    }
}
pub fn request_received(
    channel: usize,
    now: u64,
    command: Option<Command>,
    settings: Option<Settings>,
) {
    SHARED.lock(|s| {
        let s = &mut s.borrow_mut()[channel];
        s.last_host_ms = Some(now);
        if let Some(value) = settings {
            s.settings = value;
        }
        if let Some(c) = command {
            if !c.active() {
                s.reset_generation = s.reset_generation.wrapping_add(1);
            }
            s.command = c;
        }
    });
}
pub fn publish_status(
    channel: usize,
    status: [u8; REGISTER_BYTES],
    status1000: [u8; extended::STATUS_BYTES],
    advanced_status: [u8; advanced::STATUS_BYTES],
) {
    SHARED.lock(|s| {
        let shared = &mut s.borrow_mut()[channel];
        shared.status = status;
        shared.status1000 = status1000;
        shared.advanced_status = advanced_status;
    });
}
pub fn publish_monitor(registers: [u8; monitor::BYTES]) {
    MONITOR.lock(|m| m.borrow_mut().registers = registers);
}
pub fn monitor_request(command: u16) {
    MONITOR.lock(|m| m.borrow_mut().request = Some(command));
}
pub fn take_monitor_request() -> Option<u16> {
    MONITOR.lock(|m| m.borrow_mut().request.take())
}
pub fn publish_adc(registers: [u8; adc::BYTES]) {
    ADC.lock(|a| a.borrow_mut().registers = registers);
}
pub fn adc_request(mask: u16) {
    ADC.lock(|a| a.borrow_mut().request = Some(mask));
}
pub fn take_adc_request() -> Option<u16> {
    ADC.lock(|a| a.borrow_mut().request.take())
}
