use crate::transport;
use embassy_stm32::{
    Peri, bind_interrupts,
    can::{
        self, Can, ExtendedId, Fifo, Frame, Id, Mailbox, StandardId,
        enums::{BusError, TryReadError},
        filter::Mask32,
    },
    pac, peripherals,
};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use stm_gripper_controller::{
    bridge::{Bridge, Drive, FaultDetail, Output},
    config::{self, Channel, Gripper},
    control::Command,
    cubemars::{self, Feedback, Packet},
    monitor::{BusErrorCode, Monitor, Probe},
};
bind_interrupts!(struct Irqs {
    CAN1_TX => can::TxInterruptHandler<peripherals::CAN1>;
    CAN1_RX0 => can::Rx0InterruptHandler<peripherals::CAN1>;
    CAN1_RX1 => can::Rx1InterruptHandler<peripherals::CAN1>;
    CAN1_SCE => can::SceInterruptHandler<peripherals::CAN1>;
});
pub struct CanPeripherals {
    pub can1: Peri<'static, peripherals::CAN1>,
    pub pb8: Peri<'static, peripherals::PB8>,
    pub pb9: Peri<'static, peripherals::PB9>,
}
pub async fn init_and_start_can_service(spawner: &embassy_executor::Spawner, p: CanPeripherals) {
    let mut can = Can::new(p.can1, p.pb8, p.pb9, Irqs);
    // Never retransmit an old torque request after a cable reconnects.
    can.modify_config()
        .set_bitrate(config::CAN_BITRATE)
        .set_automatic_retransmit(false);
    // Accept every frame so the monitor sees the whole bus; each motor's feedback is
    // picked out by its identifier in software (cubemars::parse_feedback).
    can.modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());
    // Joining the bus waits for 11 recessive bits. A bus held dominant (miswired CANH/CANL,
    // one line on GND) would block forever, so it happens in the CAN task: Modbus, the
    // boot log and the ADC task start regardless of the CAN wiring.
    spawner.spawn(can_task(can)).unwrap();
}
/// Leaves bxCAN init mode once the bus is idle. While it is not, a note after 2 s and then
/// every 30 s: ADC and Modbus run regardless, and a board used without motors (or with the
/// CAN cable miswired) should not fill the `cargo run` log.
async fn join_bus(can: &mut Can<'static>) {
    let started = Instant::now();
    let mut next_note = Duration::from_secs(2);
    while with_timeout(Duration::from_secs(2), can.enable())
        .await
        .is_err()
    {
        if started.elapsed() >= next_note {
            next_note = started.elapsed() + Duration::from_secs(30);
            defmt::warn!(
                "CAN: bus not idle for {=u64} s (RX held dominant: check CANH/CANL, GND, 120 ohm). Motors are off; ADC and Modbus keep running. Next note in 30 s",
                started.elapsed().as_secs()
            );
        }
    }
    if started.elapsed() >= Duration::from_secs(2) {
        defmt::info!(
            "CAN: bus idle, joined after {=u64} s",
            started.elapsed().as_secs()
        );
    }
}
fn abort_all(can: &mut Can<'static>) {
    for m in [Mailbox::Mailbox0, Mailbox::Mailbox1, Mailbox::Mailbox2] {
        can.abort(m);
    }
}
fn frame(p: Packet) -> Frame {
    Frame::new_data(ExtendedId::new(p.id).unwrap(), &p.data[..p.len]).unwrap()
}
fn probe_frame(p: Probe) -> Frame {
    let data = &p.data[..p.len];
    if p.extended {
        Frame::new_data(ExtendedId::new(p.id).unwrap(), data).unwrap()
    } else {
        Frame::new_data(StandardId::new(p.id as u16).unwrap(), data).unwrap()
    }
}
fn error_code(e: BusError) -> BusErrorCode {
    match e {
        BusError::Stuff => BusErrorCode::Stuff,
        BusError::Form => BusErrorCode::Form,
        BusError::Acknowledge => BusErrorCode::Acknowledge,
        BusError::BitRecessive => BusErrorCode::BitRecessive,
        BusError::BitDominant => BusErrorCode::BitDominant,
        BusError::Crc => BusErrorCode::Crc,
        BusError::Software => BusErrorCode::Software,
        BusError::BusOff => BusErrorCode::BusOff,
        BusError::BusPassive => BusErrorCode::BusPassive,
        BusError::BusWarning => BusErrorCode::BusWarning,
    }
}
/// Queues a frame without displacing another pending one: priority scheduling would
/// otherwise drop the lowest-priority pending frame, possibly the other motor's.
fn send(can: &mut Can<'static>, packet: Packet) -> bool {
    can.try_write(&frame(packet))
        .is_ok_and(|status| status.dequeued_frame().is_none())
}
/// Ticks of motor feedback kept per motor and printed over RTT when it faults.
const TRACE: usize = 20;
#[derive(Clone, Copy)]
struct Sample {
    at: u64,
    feedback: Option<Feedback>,
    /// CAN ID of the frame sent that tick (0 for none).
    sent: u32,
}
/// One motor on the bus and the state the CAN task keeps for it.
struct Motor {
    channel: Channel,
    bridge: Bridge,
    feedback: Option<Feedback>,
    /// Motor model the feedback was parsed with; a change drops the stale sample.
    model: u16,
    generation: u32,
    was_driving: bool,
    last_disable: Option<u64>,
    trace: [Option<Sample>; TRACE],
    next: usize,
    fault: u16,
}
impl Motor {
    fn new(channel: Channel, gripper: Gripper) -> Self {
        Self {
            channel,
            bridge: if cfg!(feature = "gripper-homing") {
                Bridge::for_motor(config::MOTION, channel.motor_id).with_gripper(gripper)
            } else {
                Bridge::for_motor(config::MOTION, channel.motor_id)
            },
            feedback: None,
            model: config::DEFAULT_MOTOR_MODEL,
            generation: 0,
            was_driving: false,
            last_disable: None,
            trace: [None; TRACE],
            next: 0,
            fault: 0,
        }
    }
    fn record(&mut self, at: u64, sent: Option<Packet>) {
        self.trace[self.next] = Some(Sample {
            at,
            feedback: self.feedback,
            sent: sent.map_or(0, |p| p.id),
        });
        self.next = (self.next + 1) % TRACE;
    }
    /// The fault, what the host asked for, and the last ticks of feedback before it.
    fn report_fault(&self, now: u64, output: &Output, command: Command, mode: u16) {
        let word =
            |i: usize| u16::from_be_bytes([output.status1000[i * 2], output.status1000[i * 2 + 1]]);
        defmt::warn!(
            "Slave {} CAN {}: fault {=u16:#04x} detail {} | mode {} action {=u8:#04x} position {} speed {} force {}",
            self.channel.slave,
            self.channel.motor_id,
            word(3),
            word(12),
            mode,
            command.action,
            command.position,
            command.speed,
            command.force
        );
        for k in 0..TRACE {
            let Some(s) = self.trace[(self.next + k) % TRACE] else {
                continue;
            };
            match s.feedback {
                Some(f) => defmt::warn!(
                    "  t-{}ms sent {=u32:#05x} | fb {}ms old: pos {} rad vel {} rad/s iq {} A err {}",
                    now.saturating_sub(s.at),
                    s.sent,
                    s.at.saturating_sub(f.received_ms),
                    f.position_rad,
                    f.velocity_rad_s,
                    f.iq_a,
                    f.error
                ),
                None => defmt::warn!(
                    "  t-{}ms sent {=u32:#05x} | no feedback",
                    now.saturating_sub(s.at),
                    s.sent
                ),
            }
        }
    }
}
#[embassy_executor::task]
async fn can_task(mut can: Can<'static>) {
    join_bus(&mut can).await;
    let mut motors: [Motor; config::CHANNEL_COUNT] =
        core::array::from_fn(|i| Motor::new(config::CHANNELS[i], config::GRIPPERS[i]));
    let mut monitor = Monitor::new();
    let mut last_tick = Instant::now();
    loop {
        let now = Instant::now();
        let now_ms = now.as_millis();
        if let Some(command) = transport::take_monitor_request() {
            monitor.command(command);
        }
        let shared: [transport::Snapshot; config::CHANNEL_COUNT] =
            core::array::from_fn(transport::snapshot);
        // Bound RX work so bus traffic cannot starve watchdogs or Modbus.
        for _ in 0..32 {
            match can.try_read() {
                Ok(e) => {
                    let (extended, raw) = match e.frame.header().id() {
                        Id::Standard(id) => (false, u32::from(id.as_raw())),
                        Id::Extended(id) => (true, id.as_raw()),
                    };
                    monitor.record(extended, raw, e.frame.data(), now_ms);
                    if let (false, Id::Extended(id)) =
                        (e.frame.header().rtr(), e.frame.header().id())
                    {
                        for motor in &mut motors {
                            if let Some(fb) = cubemars::parse_feedback(
                                motor.model,
                                motor.channel.motor_id,
                                id.as_raw(),
                                e.frame.data(),
                                e.ts.as_millis(),
                            ) {
                                motor.feedback = Some(fb);
                            }
                        }
                    }
                }
                Err(TryReadError::Empty) => break,
                Err(TryReadError::BusError(e)) => {
                    monitor.record_error(error_code(e));
                    // The bus is shared: a bus error stops every motor on it.
                    for motor in &mut motors {
                        motor.bridge.trip_with_detail(0x09, FaultDetail::CanBus);
                    }
                    break;
                }
            }
        }
        let late = (now - last_tick).as_millis() > 10;
        last_tick = now;
        // Step every motor before touching the mailboxes, so aborting a stopped
        // motor's stale frames never drops a frame queued this tick.
        let mut commands = [Command::default(); config::CHANNEL_COUNT];
        let mut packets: [Option<Packet>; config::CHANNEL_COUNT] = [None; config::CHANNEL_COUNT];
        let mut stopped = false;
        let mut outputs: [Output; config::CHANNEL_COUNT] = core::array::from_fn(|i| {
            let (motor, shared) = (&mut motors[i], &shared[i]);
            // Preserve reset edges even when multiple host writes occur between ticks.
            commands[i] = if motor.generation != shared.reset_generation {
                motor.generation = shared.reset_generation;
                Command::default()
            } else {
                shared.command
            };
            if motor.was_driving && late {
                motor.bridge.trip_with_detail(0x09, FaultDetail::TickLate);
            }
            let output = motor.bridge.step_with_settings(
                now_ms,
                commands[i],
                shared.last_host_ms,
                motor.feedback,
                shared.settings,
            );
            // A new model (only while OFF) rescales the feedback velocity: wait for a fresh
            // sample parsed with it.
            if motor.bridge.motor_model() != motor.model {
                motor.model = motor.bridge.motor_model();
                motor.feedback = None;
            }
            packets[i] = match output.drive {
                Drive::Can(packet) => {
                    motor.was_driving = true;
                    Some(packet)
                }
                Drive::Torque(torque) => {
                    motor.was_driving = true;
                    Some(
                        cubemars::torque_command(motor.model, motor.channel.motor_id, torque)
                            .unwrap(),
                    )
                }
                Drive::Disabled => {
                    stopped |= motor.was_driving;
                    let due = motor.was_driving
                        || motor
                            .last_disable
                            .is_none_or(|t| now_ms.saturating_sub(t) >= 20);
                    motor.was_driving = false;
                    if due {
                        motor.last_disable = Some(now_ms);
                        Some(cubemars::disable(motor.channel.motor_id))
                    } else {
                        None
                    }
                }
            };
            motor.record(now_ms, packets[i]);
            output
        });
        if stopped {
            abort_all(&mut can);
        }
        // Keep the state machines runnable even if all CAN mailboxes are occupied.
        // The mailboxes serve the whole bus, so a full queue stops every motor.
        if !packets
            .iter()
            .flatten()
            .all(|&packet| send(&mut can, packet))
        {
            abort_all(&mut can);
            for motor in &mut motors {
                motor.bridge.trip_with_detail(0x09, FaultDetail::TxQueue);
                let _ = can.try_write(&frame(cubemars::disable(motor.channel.motor_id)));
            }
            for (i, (motor, shared)) in motors.iter_mut().zip(&shared).enumerate() {
                outputs[i] = motor.bridge.step_with_settings(
                    now_ms,
                    commands[i],
                    shared.last_host_ms,
                    motor.feedback,
                    shared.settings,
                );
                motor.was_driving = false;
            }
        } else {
            // The ID scan sends only disable frames, and only while every motor is OFF.
            let idle = commands.iter().all(|c| !c.active());
            monitor.scan(idle, |probe| {
                can.try_write(&probe_frame(probe))
                    .is_ok_and(|status| status.dequeued_frame().is_none())
            });
        }
        monitor.set_esr(pac::CAN1.esr().read().0);
        transport::publish_monitor(monitor.encode(now_ms));
        for (i, output) in outputs.iter().enumerate() {
            let fault = u16::from_be_bytes([output.status1000[6], output.status1000[7]]);
            if fault != 0 && motors[i].fault == 0 {
                motors[i].report_fault(now_ms, output, commands[i], shared[i].settings.mode as u16);
            }
            motors[i].fault = fault;
            transport::publish_status(i, output.status, output.status1000, output.advanced_status);
        }
        Timer::after_micros(1000).await;
    }
}
