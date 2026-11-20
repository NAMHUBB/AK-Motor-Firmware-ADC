//! AD7175-8 (MikroE ADC 4 Click) on SPI3: the Clicker 4 ADC task
//! (`../stm-gripper-261119/src/adc4.rs`) with the same STM32F407 pins.
//! Register values, the microvolt conversion and the Modbus block are in `adc` (host-tested).
//!
//! PC10 = SCK, PC11 = MISO (Click SDO), PC12 = MOSI (Click SDI), PD15 = CS (active low).
//! SPI3 TX/RX DMA: DMA1 streams 7 and 2, channel 0, as on the Clicker 4 (USART2 uses DMA1 streams 6 and 5).
use crate::transport;
use embassy_executor::Spawner;
use embassy_stm32::{
    Peri,
    gpio::{Level, Output, Pull, Speed},
    mode, peripherals,
    spi::{self, Spi},
    time::Hertz,
};
use embassy_time::{Duration, Instant, Timer};
use stm_gripper_controller::adc::{self, Block, State};

pub struct AdcPeripherals {
    pub spi3: Peri<'static, peripherals::SPI3>,
    pub pc10: Peri<'static, peripherals::PC10>, // SCK
    pub pc12: Peri<'static, peripherals::PC12>, // MOSI (Click SDI)
    pub pc11: Peri<'static, peripherals::PC11>, // MISO (Click SDO)
    pub pd15: Peri<'static, peripherals::PD15>, // CS (active low)
    pub dma1_ch7: Peri<'static, peripherals::DMA1_CH7>, // SPI3 TX (DMA1 stream 7)
    pub dma1_ch2: Peri<'static, peripherals::DMA1_CH2>, // SPI3 RX (DMA1 stream 2)
}

/// STATUS polling interval: a conversion takes 10 ms at 100 SPS.
const POLL: Duration = Duration::from_micros(500);
/// No conversion for this long means the chip stopped answering: detect it again.
const READY_TIMEOUT: Duration = Duration::from_millis(500);
/// The ID is read again this often while running, so a pulled cable shows as NotFound
/// instead of stale or invented values.
const ID_CHECK: Duration = Duration::from_millis(1000);
/// One `ADC mV` log line (all six channels) this often, for strain readings in `cargo run`.
const LOG_PERIOD: Duration = Duration::from_millis(100);

/// One channel in the `ADC mV` line: millivolts with three decimals (= microvolts), `off`
/// while disabled, `--` before its first conversion, `OVR` appended when it saturated.
struct Millivolts {
    microvolts: i32,
    enabled: bool,
    fresh: bool,
    over_range: bool,
}
impl defmt::Format for Millivolts {
    fn format(&self, f: defmt::Formatter) {
        if !(self.enabled && self.fresh) {
            defmt::write!(f, "{=str}", if self.enabled { "--" } else { "off" });
        } else {
            let (negative, whole, decimals) = adc::millivolt_parts(self.microvolts);
            defmt::write!(
                f,
                "{=str}{=u32}.{=u32:03}{=str}",
                if negative { "-" } else { "" },
                whole,
                decimals,
                if self.over_range { " OVR" } else { "" }
            );
        }
    }
}
/// Microvolts printed as millivolts with three decimals.
struct Mv(i64);
impl defmt::Format for Mv {
    fn format(&self, f: defmt::Formatter) {
        let (negative, whole, decimals) =
            adc::millivolt_parts(self.0.clamp(i32::MIN.into(), i32::MAX.into()) as i32);
        defmt::write!(
            f,
            "{=str}{=u32}.{=u32:03}",
            if negative { "-" } else { "" },
            whole,
            decimals
        );
    }
}
/// Microvolts printed as volts with three decimals.
struct Volts(i64);
impl defmt::Format for Volts {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{}", Mv(self.0 / 1000));
    }
}
fn millivolts(block: &Block, channel: usize) -> Millivolts {
    let bit = 1 << channel;
    Millivolts {
        microvolts: block.microvolts[channel],
        enabled: block.mask & bit != 0,
        fresh: block.fresh & bit != 0,
        over_range: block.over_range & bit != 0,
    }
}

struct Ad7175 {
    spi: Spi<'static, mode::Async>,
    cs: Output<'static>,
}
impl Ad7175 {
    /// Interface reset: at least 64 SCLK cycles with DIN high.
    async fn reset(&mut self) {
        self.cs.set_low();
        let _ = self.spi.write(&[0xff_u8; 8]).await;
        self.cs.set_high();
        Timer::after_millis(2).await;
    }
    async fn write(&mut self, register: u8, value: u16) {
        let [high, low] = value.to_be_bytes();
        self.cs.set_low();
        let _ = self.spi.write(&[register, high, low]).await;
        self.cs.set_high();
    }
    async fn read<const N: usize>(&mut self, register: u8) -> [u8; N] {
        // MISO has a pull-up: a missing chip reads all ones (RDY high, ID 0xFFFF).
        let mut data = [0xff; N];
        self.cs.set_low();
        let _ = self.spi.write(&[adc::READ | register]).await;
        let _ = self.spi.read(&mut data).await;
        self.cs.set_high();
        data
    }
    async fn id(&mut self) -> u16 {
        u16::from_be_bytes(self.read::<2>(adc::REG_ID).await)
    }
    /// The Clicker 4 setup with only the masked channels enabled and the chosen reference
    /// (`adc::SETUPCON_*`); standby for none.
    async fn configure(&mut self, mask: u16, setup: u16) {
        if mask == 0 {
            self.write(adc::REG_ADCMODE, adc::ADCMODE_STANDBY).await;
            return;
        }
        for channel in 0..adc::CHANNELS {
            let enabled = mask & (1 << channel) != 0;
            self.write(
                adc::REG_CH0 + channel as u8,
                adc::channel_register(channel, enabled),
            )
            .await;
        }
        self.write(adc::REG_SETUPCON0, setup).await;
        self.write(adc::REG_FILTCON0, adc::FILTCON0).await;
        self.write(adc::REG_ADCMODE, adc::ADCMODE_CONTINUOUS).await;
        self.write(adc::REG_IFMODE, adc::IFMODE).await;
    }
    /// Converts one input pair alone (channel register `pair`, SETUPCON0 `setup`): the first
    /// three conversions settle the filter and the reference, the next `samples` count.
    async fn measure(&mut self, pair: u16, setup: u16, samples: u32) -> Option<adc::Stats> {
        self.write(adc::REG_CH0, pair).await;
        for channel in 1..adc::CHANNELS {
            self.write(
                adc::REG_CH0 + channel as u8,
                adc::channel_register(channel, false),
            )
            .await;
        }
        self.write(adc::REG_SETUPCON0, setup).await;
        self.write(adc::REG_FILTCON0, adc::FILTCON0).await;
        self.write(adc::REG_ADCMODE, adc::ADCMODE_CONTINUOUS).await;
        self.write(adc::REG_IFMODE, adc::IFMODE).await;
        let mut stats = adc::Stats::new();
        for n in 0..samples + 3 {
            if !self.ready().await {
                return None;
            }
            let sample = adc::decode_sample(self.read::<4>(adc::REG_DATA).await);
            if n >= 3 {
                stats.add(sample);
            }
        }
        Some(stats)
    }
    /// Startup self-check, logged for `cargo run`: AVDD1 against the internal 2.5 V
    /// reference, the REF+/REF- pins against AVDD1, the zero offset, every pin against GND,
    /// then every enabled channel (mean and swing). Returns the setup to measure with: always
    /// the internal 2.5 V reference with the buffers on, as the CANopen firmware.
    async fn self_check(&mut self, mask: u16) -> u16 {
        let avdd = self
            .measure(
                adc::input_pair(adc::INPUT_AVDD_POS, adc::INPUT_AVDD_NEG),
                adc::SETUPCON_SUPPLY_CHECK,
                8,
            )
            .await
            .map(|s| 5 * s.microvolts(adc::REFERENCE_UV).0);
        let setup = adc::SETUPCON_MEASURE;
        let Some(avdd) = avdd else {
            defmt::warn!("ADC check: no conversion; measuring with the internal 2.5 V reference");
            return setup;
        };
        let reference = self
            .measure(
                adc::input_pair(adc::INPUT_REF_POS, adc::INPUT_REF_NEG),
                adc::SETUPCON_AVDD_UNBUFFERED,
                8,
            )
            .await
            .map(|s| s.microvolts(avdd).0)
            .unwrap_or(0);
        defmt::info!(
            "ADC check: AVDD1 {} V, REF+/REF- {} V{=str}; measuring with the internal 2.5 V reference",
            Volts(avdd),
            Volts(reference),
            if adc::reference_usable(reference) {
                ""
            } else {
                " (not 2.5 V: the external reference is not used)"
            }
        );
        // Two readings that need a working analog supply but not AVDD1 as their reference: the
        // die temperature (about 140 mV at room temperature) and the REF pins against the
        // internal 2.5 V. Near 0 V on both: the Click's analog side has no power.
        if let Some(t) = self
            .measure(
                adc::input_pair(adc::INPUT_TEMP_POS, adc::INPUT_TEMP_NEG),
                adc::SETUPCON_SUPPLY_CHECK,
                8,
            )
            .await
        {
            let (uv, _) = t.microvolts(adc::REFERENCE_UV);
            defmt::info!(
                "ADC check: temperature sensor {} mV = {=i64} C (about 140 mV at room temperature; near 0: no analog supply)",
                Mv(uv),
                uv / adc::TEMP_UV_PER_K - 273
            );
        }
        if let Some(r) = self
            .measure(
                adc::input_pair(adc::INPUT_REF_POS, adc::INPUT_REF_NEG),
                adc::SETUPCON_INTERNAL,
                8,
            )
            .await
        {
            let (uv, _) = r.microvolts(adc::REFERENCE_UV);
            defmt::info!(
                "ADC check: REF+/REF- against the internal 2.5 V: {} mV{=str}",
                Mv(uv),
                if r.over_range {
                    " (over range: above 2.5 V, e.g. the 4.096 V reference)"
                } else {
                    ""
                }
            );
        }
        if let Some(offset) = self.measure(adc::input_pair(0, 0), setup, 16).await {
            let (mean, swing) = offset.microvolts(adc::REFERENCE_UV);
            defmt::info!(
                "ADC check: zero (AIN0 against itself) {} mV, swing {} mV (the ADC's own noise)",
                Mv(mean),
                Mv(swing)
            );
        }
        // Every input pin against GND, so the log shows which pins are driven (steady) and which
        // are open (they wander): AIN0..AIN16, about 0.1 s each. Unbuffered, since a pin may sit
        // near GND (the buffers need 0.1 V above AVSS).
        for pin in 0..adc::INPUT_PINS {
            let Some(stats) = self
                .measure(
                    adc::input_pair(pin, adc::INPUT_GND),
                    adc::SETUPCON_INTERNAL,
                    8,
                )
                .await
            else {
                break;
            };
            let (mean, swing) = stats.microvolts(adc::REFERENCE_UV);
            let note = match adc::verdict(mean, swing, stats.over_range) {
                adc::Verdict::Ok => "",
                adc::Verdict::OverRange => " <- over range (outside +-2.5 V)",
                adc::Verdict::Unstable => " <- wanders: open (nothing connected)",
                adc::Verdict::BelowGround => " <- below GND: outside the input range",
            };
            defmt::info!(
                "ADC pin scan: AIN{=u16} {} mV, swing {} mV{=str}",
                pin,
                Mv(mean),
                Mv(swing),
                note
            );
        }
        for channel in 0..adc::CHANNELS {
            if mask & (1 << channel) == 0 {
                continue;
            }
            let Some(stats) = self
                .measure(adc::channel_register(channel, true), setup, 30)
                .await
            else {
                continue;
            };
            // The self-check reports the pair before CH_GAIN (the raw input).
            let (mean, swing) = stats.microvolts(adc::REFERENCE_UV);
            let (pin, neg) = (adc::input_pin(channel), adc::negative_pin(channel));
            match adc::pair_verdict(mean, swing, stats.over_range) {
                adc::Verdict::Ok | adc::Verdict::BelowGround => defmt::info!(
                    "ADC check: ADC{=usize} (AIN{=usize} - AIN{=usize}) {} mV, swing {} mV -> OK",
                    channel,
                    pin,
                    neg,
                    Mv(mean),
                    Mv(swing)
                ),
                adc::Verdict::OverRange => defmt::warn!(
                    "ADC check: ADC{=usize} (AIN{=usize} - AIN{=usize}) over range (outside +-2.5 V), or a pin outside the buffered input range (0.1 V above GND to 0.1 V below AVDD1)",
                    channel,
                    pin,
                    neg
                ),
                adc::Verdict::Unstable => defmt::warn!(
                    "ADC check: ADC{=usize} (AIN{=usize} - AIN{=usize}) {} mV, swing {} mV -> UNSTABLE: a pin is open (floating). Bridge output + to AIN{=usize}, - to AIN{=usize}, and its ground to the Click GND",
                    channel,
                    pin,
                    neg,
                    Mv(mean),
                    Mv(swing),
                    pin,
                    neg
                ),
            }
        }
        setup
    }
    /// Waits for STATUS RDY low (a new conversion); false after `READY_TIMEOUT`.
    async fn ready(&mut self) -> bool {
        let start = Instant::now();
        loop {
            if self.read::<1>(adc::REG_STATUS).await[0] & 0x80 == 0 {
                return true;
            }
            if start.elapsed() > READY_TIMEOUT {
                return false;
            }
            Timer::after(POLL).await;
        }
    }
}

pub fn start(spawner: &Spawner, p: AdcPeripherals) {
    let mut config = spi::Config::default();
    // AD7175-8: CPOL 1 / CPHA 1. SPI3 is on APB1 (42 MHz): 42 MHz / 32 = 1.3 MHz, gentle on
    // wires; a 100 SPS conversion needs only 5 bytes.
    config.mode = spi::MODE_3;
    config.frequency = Hertz::khz(2500);
    config.miso_pull = Pull::Up;
    config.gpio_speed = Speed::Medium;
    let spi = Spi::new(
        p.spi3, p.pc10, p.pc12, p.pc11, p.dma1_ch7, p.dma1_ch2, config,
    );
    let cs = Output::new(p.pd15, Level::High, Speed::Medium);
    spawner.spawn(adc_task(Ad7175 { spi, cs })).unwrap();
}

#[embassy_executor::task]
async fn adc_task(mut chip: Ad7175) {
    let mut block = Block::new(adc::BOOT_MASK);
    let mut missing_logged = false;
    loop {
        if let Some(mask) = transport::take_adc_request() {
            block.set_mask(mask);
        }
        chip.reset().await;
        let id = chip.id().await;
        block.detected(id);
        transport::publish_adc(block.encode());
        if block.state != State::Running {
            if !missing_logged {
                defmt::warn!(
                    "ADC 4 Click: no AD7175-8 on SPI3 (PC10-PC12, CS PD15; ID {=u16:#06x}); retrying",
                    id
                );
                missing_logged = true;
            }
            Timer::after_millis(500).await;
            continue;
        }
        missing_logged = false;
        defmt::info!(
            "ADC 4 Click: AD7175-8 ID {=u16:#06x}, enabled mask {=u16:#04x}",
            id,
            block.mask
        );
        let setup = chip.self_check(block.mask).await;
        chip.configure(block.mask, setup).await;
        let mut id_checked = Instant::now();
        let mut logged = Instant::now();
        loop {
            if let Some(mask) = transport::take_adc_request() {
                block.set_mask(mask);
                chip.configure(mask, setup).await;
                transport::publish_adc(block.encode());
                defmt::info!("ADC 4 Click: enabled mask {=u16:#04x}", mask);
            }
            if id_checked.elapsed() >= ID_CHECK {
                id_checked = Instant::now();
                if !adc::is_ad7175_8(chip.id().await) {
                    defmt::warn!("ADC 4 Click: AD7175-8 stopped answering");
                    break;
                }
            }
            if block.mask == 0 {
                Timer::after_millis(10).await;
                continue;
            }
            if !chip.ready().await {
                defmt::warn!("ADC 4 Click: no conversion in 500 ms");
                break;
            }
            let sample = adc::decode_sample(chip.read::<4>(adc::REG_DATA).await);
            if block.record(sample) {
                transport::publish_adc(block.encode());
            }
            if logged.elapsed() >= LOG_PERIOD {
                logged = Instant::now();
                defmt::info!(
                    "ADC mV | ADC0 {} | ADC1 {} | ADC2 {} | ADC3 {} | ADC4 {} | ADC5 {}",
                    millivolts(&block, 0),
                    millivolts(&block, 1),
                    millivolts(&block, 2),
                    millivolts(&block, 3),
                    millivolts(&block, 4),
                    millivolts(&block, 5)
                );
            }
        }
    }
}
