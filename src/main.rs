#![no_std]
#![no_main]
mod ad7175;
mod can;
mod modbus;
mod transport;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllSource, Sysclk,
    },
    time::Hertz,
};
use embassy_time::{Duration, Timer};
use panic_probe as _;
use stm_gripper_controller::config;
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();

    config.rcc.hse = Some(Hse {
        freq: Hertz(25_000_000),   // HSE 주파수 = 25MHz
        mode: HseMode::Oscillator, // HSE 모드 = 외부 크리스탈 (Oscillator)
    });
    config.rcc.sys = Sysclk::PLL1_P; // SYSCLK = PLL1_P = 168MHz
    config.rcc.pll_src = PllSource::HSE; // PLL 소스 = HSE (25MHz)
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV25,  // 25MHz / 25 = 1MHz (VCO 입력)
        mul: PllMul::MUL336,       // 1 MHz * 336 = 336 MHz (VCO 출력)
        divp: Some(PllPDiv::DIV2), // 336 / 2 = 168 MHz SYSCLK
        divq: Some(PllQDiv::DIV7), // 336 / 7 ≈ 48 MHz (USB/SDIO)
        divr: None,                // 사용 안 함
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1; // 168MHz (AHB)
    config.rcc.apb1_pre = APBPrescaler::DIV4; // 42MHz (APB1, 최대 45MHz)
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 84MHz (APB2, 최대 90MHz)

    let p = embassy_stm32::init(config);

    // 보드: MikroE Clicker 4 for STM32 (STM32F407VG). 핀은 클리커4 CST 펌웨어와 같다.
    // RS485 수신(/RE) 활성화를 위한 PA4 Low 설정.
    // NOTE: Output을 즉시 drop하면 핀이 해제되어(입력/Hi-Z 등) RS485가 간헐적으로 꺼질 수 있음.
    // 스코프에 유지해서 실행 내내 Enable 상태가 유지되게 한다.
    let _rs485_en = Output::new(p.PA4, Level::Low, Speed::Medium);

    // Modbus-RTU (Robotiq) 초기화: USART2 PA2(TX)/PA3(RX), PC15 DE, DMA1_CH6(TX)/DMA1_CH5(RX)
    // Start host communication before waiting for the motor-side CAN service.
    modbus::init_modbus(
        &spawner,
        modbus::ModbusPeripherals {
            usart2: p.USART2,
            pa2: p.PA2,
            pa3: p.PA3,
            pc15: p.PC15,
            dma1_ch5: p.DMA1_CH5,
            dma1_ch6: p.DMA1_CH6,
        },
    )
    .await;

    // CAN 트랜시버 STB(PE7) Low = 정상 동작(High면 대기 모드라 버스에 아무것도 나가지 않음).
    let _can1_stb = Output::new(p.PE7, Level::Low, Speed::Medium);
    // CAN 하드웨어 서비스 시작: CAN1 PB8(RX)/PB9(TX), AK40-10 / AK60-39 서보 모드(확장 ID)
    can::init_and_start_can_service(
        &spawner,
        can::CanPeripherals {
            can1: p.CAN1,
            pb8: p.PB8,
            pb9: p.PB9,
        },
    )
    .await;

    defmt::info!(
        "Clicker 4 STM32F407VG / AK40-10 + AK60-39 + ADC 4 Click Robotiq RTU bridge · {} motor(s) · boot motor model {}",
        config::CHANNEL_COUNT,
        config::DEFAULT_MOTOR_MODEL
    );
    for channel in config::CHANNELS {
        defmt::info!(
            "Modbus slave {} -> CAN ID {}",
            channel.slave,
            channel.motor_id
        );
    }

    // ADC 4 Click (AD7175-8) 스트레인 게이지 ADC0~5: 클리커4와 같은 SPI3
    // PC10 SCK / PC11 MISO / PC12 MOSI / PD15 CS, DMA1 스트림 7·2. Modbus 0x1500 (src/adc.rs).
    ad7175::start(
        &spawner,
        ad7175::AdcPeripherals {
            spi3: p.SPI3,
            pc10: p.PC10,
            pc12: p.PC12,
            pc11: p.PC11,
            pd15: p.PD15,
            dma1_ch7: p.DMA1_CH7,
            dma1_ch2: p.DMA1_CH2,
        },
    );

    // 동작 확인용 LED (클리커4 LED1 = PC8)
    let mut led = Output::new(p.PC8, Level::Low, Speed::Low);

    loop {
        Timer::after(Duration::from_millis(500)).await;
        led.toggle();
    }
}
