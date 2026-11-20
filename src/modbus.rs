use crate::transport;
use embassy_executor::Spawner;
use embassy_stm32::{
    Peri, bind_interrupts,
    gpio::{Level, Output, Speed},
    mode, peripherals, usart,
    usart::{Config, Uart},
};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use stm_gripper_controller::{
    config, dispatch,
    robotiq::{MAX_ADU, REGISTER_BYTES},
};
bind_interrupts!(struct Irqs { USART2 => usart::InterruptHandler<peripherals::USART2>; });
pub struct ModbusPeripherals {
    pub usart2: Peri<'static, peripherals::USART2>,
    pub pa2: Peri<'static, peripherals::PA2>,           // TX
    pub pa3: Peri<'static, peripherals::PA3>,           // RX
    pub pc15: Peri<'static, peripherals::PC15>,         // DE (Driver Enable)
    pub dma1_ch5: Peri<'static, peripherals::DMA1_CH5>, // RX DMA (DMA1 stream 5)
    pub dma1_ch6: Peri<'static, peripherals::DMA1_CH6>, // TX DMA (DMA1 stream 6)
}
#[embassy_executor::task]
async fn server(mut uart: Uart<'static, mode::Async>, mut de: Output<'static>) {
    // Robotiq output bank per motor channel.
    let mut outputs = [[0; REGISTER_BYTES]; config::CHANNEL_COUNT];
    let mut rx = [0; MAX_ADU];
    let mut tx = [0; MAX_ADU];
    let char_us = 10_000_000u64.div_ceil(config::MODBUS_BAUD as u64);
    let gap_us = if config::MODBUS_BAUD > 19200 {
        1750
    } else {
        35_000_000u64.div_ceil(config::MODBUS_BAUD as u64)
    };
    let inter_us = if config::MODBUS_BAUD > 19200 {
        750
    } else {
        15_000_000u64.div_ceil(config::MODBUS_BAUD as u64)
    };
    loop {
        // USART IDLE is one character. Accumulate chunks until a complete RTU quiet gap.
        let mut n = match uart.read_until_idle(&mut rx).await {
            Ok(n) if n > 0 => n,
            _ => continue,
        };
        let mut valid = true;
        // IDLE was observed one character after the final stop bit. A full buffer
        // may instead return immediately; the extra character wait is conservative.
        let mut last_end = Instant::now();
        let mut first_gap = true;
        loop {
            // Read single bytes after the initial DMA chunk: a timeout must never
            // cancel a long chunk that has already started receiving valid data.
            let mut byte = [0; 1];
            match with_timeout(
                Duration::from_micros(if first_gap {
                    gap_us.saturating_sub(char_us)
                } else {
                    gap_us
                }),
                uart.read(&mut byte),
            )
            .await
            {
                Ok(Ok(())) => {
                    let elapsed = (Instant::now() - last_end).as_micros();
                    if elapsed > inter_us + if first_gap { 0 } else { char_us } {
                        valid = false;
                    }
                    first_gap = false;
                    last_end = Instant::now();
                    if n == MAX_ADU {
                        valid = false;
                    } else if valid {
                        rx[n] = byte[0];
                        n += 1;
                    }
                }
                Err(_) => break,
                _ => {
                    valid = false;
                    break;
                }
            }
        }
        if !valid {
            continue;
        }
        // Each motor answers its own slave address; a broadcast reaches every motor.
        let now = Instant::now().as_millis();
        let len = dispatch::serve(
            config::CHANNELS,
            &rx[..n],
            &mut tx,
            &mut outputs,
            transport::registers,
            |channel, handled| {
                transport::request_received(channel, now, handled.command, handled.settings);
                if let Some(command) = handled.monitor {
                    transport::monitor_request(command);
                }
                if let Some(mask) = handled.adc_mask {
                    transport::adc_request(mask);
                }
            },
        );
        if len > 0 {
            // RS-485 direction is manual (Clicker 4, as in its CST firmware): PC15 drives the
            // transceiver DE, HIGH transmits and LOW receives; PA4 holds /RE low (main.rs).
            // DMA completion alone is not line completion; flush waits for USART TC
            // so DE is released only after the final stop bit.
            de.set_high();
            Timer::after_micros(10).await;
            let result = with_timeout(Duration::from_millis(100), async {
                uart.write(&tx[..len]).await?;
                uart.flush().await
            })
            .await;
            de.set_low();
            if !matches!(result, Ok(Ok(()))) {
                defmt::warn!("RS485 TX failure");
            }
        }
    }
}
pub async fn init_modbus(spawner: &Spawner, p: ModbusPeripherals) {
    // Start in receive mode (DE LOW) before the USART drives TXD.
    let de = Output::new(p.pc15, Level::Low, Speed::Medium);
    let mut cfg = Config::default();
    cfg.baudrate = config::MODBUS_BAUD;
    cfg.data_bits = usart::DataBits::DataBits8;
    cfg.parity = usart::Parity::ParityNone;
    cfg.stop_bits = usart::StopBits::STOP1;
    let uart = Uart::new(p.usart2, p.pa3, p.pa2, Irqs, p.dma1_ch6, p.dma1_ch5, cfg).unwrap();
    Timer::after_micros(100).await;
    spawner.spawn(server(uart, de)).unwrap();
}
