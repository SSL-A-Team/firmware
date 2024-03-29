#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]

use apa102_spi::Apa102;
use ateam_common_packets::bindings_radio::KickRequest;
use ateam_control_board::{
    drivers::{radio::TeamColor, radio::WifiNetwork, rotary::Rotary, shell_indicator::ShellIndicator},
    stm32_interface::get_bootloader_uart_config,
};
use control::Control;
use defmt::info;
use embassy_stm32::{
    dma::NoDma,
    executor::InterruptExecutor,
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    interrupt::{self, InterruptExt},
    spi,
    time::{hz, mhz},
    usart::{self, Uart},
};
use embassy_time::{Duration, Ticker, Timer};
use futures_util::StreamExt;
use panic_probe as _;
use pins::{
    PowerStateExti, PowerStatePin, RadioReset, RadioRxDMA, RadioTxDMA, RadioUART,
    ShutdownCompletePin,
};
use radio::{
    RadioTest, BUFFERS_RX, BUFFERS_TX, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH,
    TX_BUF_DEPTH,
};
use smart_leds::{SmartLedsWrite, RGB8};
use static_cell::StaticCell;

mod control;
mod pins;
mod radio;

static RADIO_TEST: RadioTest<
    MAX_TX_PACKET_SIZE,
    MAX_RX_PACKET_SIZE,
    TX_BUF_DEPTH,
    RX_BUF_DEPTH,
    RadioUART,
    RadioRxDMA,
    RadioTxDMA,
    RadioReset,
> = RadioTest::new(unsafe { &mut BUFFERS_TX }, unsafe { &mut BUFFERS_RX });

static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    let p = embassy_stm32::init(stm32_config);
    let config = usart::Config::default();

    // Delay so dotstar and STSPIN can turn on
    Timer::after(Duration::from_millis(50)).await;

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();

    let imu_spi = spi::Spi::new_txonly(
        p.SPI3,
        p.PB3,
        p.PB5,
        NoDma,
        NoDma,
        hz(1_000_000),
        spi::Config::default(),
    );

    let mut dotstar = Apa102::new(imu_spi);
    let _ = dotstar.write([RGB8 { r: 10, g: 0, b: 0 }].iter().cloned());

    info!("booted");

    let radio_int = interrupt::take!(USART10);
    let radio_usart = Uart::new(
        p.USART10, p.PE2, p.PE3, radio_int, p.DMA2_CH0, p.DMA2_CH1, config,
    );

    let _rotary = Rotary::new(p.PG9, p.PG10, p.PG11, p.PG12);
    let mut shell_indicator = ShellIndicator::new(p.PD0, p.PD1, p.PD3, p.PD4);
    let _kicker_det = Input::new(p.PG8, Pull::Down);
    let dip1 = Input::new(p.PG7, Pull::Down);
    let dip2 = Input::new(p.PG6, Pull::Down);
    let dip3 = Input::new(p.PG5, Pull::Down);
    let dip4 = Input::new(p.PG4, Pull::Down);
    let dip5 = Input::new(p.PG3, Pull::Down);
    let dip6 = Input::new(p.PG2, Pull::Down);
    let dip7 = Input::new(p.PD15, Pull::Down);


    // let mut kicker = Output::new(p.PF9, Level::Low, Speed::High);

    // let robot_id = rotary.read();
    let robot_id = (dip1.is_high() as u8) << 3
    | (dip2.is_high() as u8) << 2
    | (dip3.is_high() as u8) << 1
    | (dip4.is_high() as u8);
    info!("id: {}", robot_id);
    shell_indicator.set(robot_id);

    let wifi_network = if dip5.is_high() & dip6.is_high() {
        WifiNetwork::Team
    } else if dip5.is_low() & dip6.is_high() {
        WifiNetwork::CompMain
    } else if dip5.is_high() & dip6.is_low() {
        WifiNetwork::CompPractice
    } else {
        WifiNetwork::Team
    };

    let team = if dip7.is_high() {
        TeamColor::Blue
    } else {
        TeamColor::Yellow
    };

    // let mut test = Output::new(p.PA8, Level::High, Speed::Medium);
    // let mut test2 = Output::new(p.PA9, Level::High, Speed::Medium);
    // // let mut kicker_boot0_pin = Output::new(p.PA8, Level::High, Speed::Medium);
    // // let mut kicker_reset_pin = Output::new(p.PA9, Level::High, Speed::Medium);

    // loop {}

    // let kicker_int = interrupt::take!(USART6);
    // let kicker_usart = Uart::new(
    //     p.USART6,
    //     p.PC7,
    //     p.PC6,
    //     kicker_int,
    //     p.DMA2_CH4,
    //     p.DMA2_CH5,
    //     get_bootloader_uart_config(),
    // );
    // let (kicker_tx, kicker_rx) = kicker_usart.split();
    // let mut kicker_boot0_pin = Output::new(p.PA8, Level::Low, Speed::Medium);
    // let mut kicker_reset_pin = Output::new(p.PA9, Level::Low, Speed::Medium);
    // let kicker_reset_pin_bad = OutputOpenDrain::new(p.PA2, Level::Low, Speed::Medium, Pull::None);

    // spawner
    //     .spawn(KICKER_QUEUE_RX.spawn_task(kicker_rx))
    //     .unwrap();
    // spawner
    //     .spawn(KICKER_QUEUE_TX.spawn_task(kicker_tx))
    //     .unwrap();

    // kicker_boot0_pin.set_high();
    // let mut kicker_stm32_interface = Stm32Interface::new(
    //     &KICKER_QUEUE_RX,
    //     &KICKER_QUEUE_TX,
    //     Some(kicker_boot0_pin),
    //     Some(kicker_reset_pin_bad),
    // );
    // info!("start program");

    // kicker_reset_pin.set_high();
    // Timer::after(Duration::from_micros(100)).await;
    // kicker_reset_pin.set_low();

    // kicker_stm32_interface.load_firmware_image(KICKER_FW_IMG).await;
    // kicker_reset_pin.set_high();
    // Timer::after(Duration::from_micros(100)).await;
    // kicker_reset_pin.set_low();

    // info!("programmed");

    // loop {}

    // let mut imu_spi = spi::Spi::new(
    //     p.SPI6,
    //     p.PA5,
    //     p.PA7,
    //     p.PA6,
    //     p.BDMA_CH0,
    //     p.BDMA_CH1,
    //     hz(1_000_000),
    //     spi::Config::default(),
    // );

    // // acceleromter
    // let mut imu_cs1 = Output::new(p.PC4, Level::High, Speed::VeryHigh);
    // // gyro
    // let mut imu_cs2 = Output::new(p.PC5, Level::High, Speed::VeryHigh);

    // Timer::after(Duration::from_millis(1)).await;

    // // let mut buf = [0u8, 2];

    // unsafe {
    //     SPI6_BUF[0] = 0x80;
    //     // info!("xfer {=[u8]:x}", SPI6_BUF[0..1]);
    //     imu_cs1.set_low();
    //     imu_spi.transfer_in_place(&mut SPI6_BUF[0..2]).await;
    //     imu_cs1.set_high();
    //     let accel_id = SPI6_BUF[1];
    //     info!("accelerometer id: 0x{:x}", accel_id);

    //     SPI6_BUF[0] = 0x80;
    //     imu_cs2.set_low();
    //     imu_spi.transfer_in_place(&mut SPI6_BUF[0..2]).await;
    //     imu_cs2.set_high();
    //     let gyro_id = SPI6_BUF[1];
    //     info!("gyro id: 0x{:x}", gyro_id);

    //     loop {
    //         SPI6_BUF[0] = 0x86;
    //         // SPI6_BUF[0] = 0x86;
    //         imu_cs2.set_low();
    //         imu_spi.transfer_in_place(&mut SPI6_BUF[0..3]).await;
    //         imu_cs2.set_high();
    //         let rate_z = (SPI6_BUF[2] as u16 * 256 + SPI6_BUF[1] as u16) as i16;
    //         info!("z rate: {}", rate_z);
    //     }
    // }

    // loop {}

    // // loop {
    //     let mut buf = [0x00u8; 4];
    //     buf[0] = 0x86;
    //     buf[2] = 0x87;
    //     imu_cs2.set_low();
    //     imu_spi.blocking_transfer_in_place(&mut buf);
    //     imu_cs2.set_high();
    //     // info!("xfer {=[u8]:x}", buf);
    //     info!("xfer {}", (buf[3] << 8 + buf[1]) as i16);
    // // }

    // loop {}

    let front_right_int = interrupt::take!(USART1);
    let front_left_int = interrupt::take!(UART4);
    let back_left_int = interrupt::take!(UART7);
    let back_right_int = interrupt::take!(UART8);
    let drib_int = interrupt::take!(UART5);

    let front_right_usart = Uart::new(
        p.USART1,
        p.PB15,
        p.PB14,
        front_right_int,
        p.DMA1_CH0,
        p.DMA1_CH1,
        get_bootloader_uart_config(),
    );
    let front_left_usart = Uart::new(
        p.UART4,
        p.PA1,
        p.PA0,
        front_left_int,
        p.DMA1_CH2,
        p.DMA1_CH3,
        get_bootloader_uart_config(),
    );
    let back_left_usart = Uart::new(
        p.UART7,
        p.PF6,
        p.PF7,
        back_left_int,
        p.DMA1_CH4,
        p.DMA1_CH5,
        get_bootloader_uart_config(),
    );
    let back_right_usart = Uart::new(
        p.UART8,
        p.PE0,
        p.PE1,
        back_right_int,
        p.DMA1_CH6,
        p.DMA1_CH7,
        get_bootloader_uart_config(),
    );
    let drib_usart = Uart::new(
        p.UART5,
        p.PB12,
        p.PB13,
        drib_int,
        p.DMA2_CH2,
        p.DMA2_CH3,
        get_bootloader_uart_config(),
    );

    let ball_detected_thresh = 1.0;
    let mut control = Control::new(
        &spawner,
        front_right_usart,
        front_left_usart,
        back_left_usart,
        back_right_usart,
        drib_usart,
        p.PD8,
        p.PC1,
        p.PF8,
        p.PB9,
        p.PD13,
        p.PD9,
        p.PC0,
        p.PF9,
        p.PB8,
        p.PD12,
        ball_detected_thresh,
    );

    let _ = dotstar.write([RGB8 { r: 0, g: 0, b: 10 }].iter().cloned());

    control.load_firmware().await;

    let _ = dotstar.write([RGB8 { r: 0, g: 10, b: 0 }, RGB8 { r: 0, g: 0, b: 10 }].iter().cloned());

    let token = unsafe {
        (&mut *(&RADIO_TEST as *const _
            as *mut RadioTest<
                MAX_TX_PACKET_SIZE,
                MAX_RX_PACKET_SIZE,
                TX_BUF_DEPTH,
                RX_BUF_DEPTH,
                RadioUART,
                RadioRxDMA,
                RadioTxDMA,
                RadioReset,
            >))
            .setup(&spawner, radio_usart, p.PC13, robot_id, team, wifi_network)
            .await
    };
    spawner.spawn(token).unwrap();

    let _ = dotstar.write([RGB8 { r: 0, g: 10, b: 0 }, RGB8 { r: 0, g: 10, b: 0 }].iter().cloned());

    let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));

    let mut last_kicked = 1000;
    loop {
        let latest = RADIO_TEST.get_latest_control();
        // let latest = Some(BasicControl{
        //     vel_x_linear: 0.,
        //     vel_y_linear: 0.,
        //     vel_z_angular: 0.05,
        //     kick_vel: 0.,
        //     dribbler_speed: 0.2,
        //     kick_request: 0,
        // });
        let telemetry = control.tick(latest);
        if let Some(telemetry) = telemetry {
            // info!("{:?}", defmt::Debug2Format(&telemetry));
            RADIO_TEST.send_telemetry(telemetry).await;
        }
        if let Some(latest) = &latest {
            if last_kicked > 100 && latest.kick_request == KickRequest::KR_KICK_NOW {
                // kicker.set_high();
                // Timer::after(Duration::from_micros(3500)).await;
                // kicker.set_low();
                last_kicked = 0;
            }
        }
        last_kicked = core::cmp::min(last_kicked + 1, 1000);

        main_loop_rate_ticker.next().await;
    }
}

#[embassy_executor::task]
async fn power_off_task(
    power_state_pin: PowerStatePin,
    exti: PowerStateExti,
    shutdown_pin: ShutdownCompletePin,
) {
    let power_state = Input::new(power_state_pin, Pull::None);
    let mut shutdown = Output::new(shutdown_pin, Level::Low, Speed::Low);
    let mut power_state = ExtiInput::new(power_state, exti);
    power_state.wait_for_falling_edge().await;
    shutdown.set_high();
    loop {}
}
