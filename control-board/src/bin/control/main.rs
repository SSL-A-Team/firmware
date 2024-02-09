#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]
#![feature(async_closure)]

use apa102_spi::Apa102;
use ateam_control_board::{
    adc_raw_to_v, adc_v_to_battery_voltage,
    drivers::{
        radio::TeamColor, radio::WifiNetwork, rotary::Rotary, shell_indicator::ShellIndicator, kicker::Kicker,
    },
    queue::Buffer,
    stm32_interface::{get_bootloader_uart_config, Stm32Interface},
    uart_queue::{UartReadQueue, UartWriteQueue},
    BATTERY_BUFFER_SIZE, BATTERY_MAX_VOLTAGE, BATTERY_MIN_VOLTAGE, include_kicker_bin, parameter_interface::ParameterInterface, tasks,
};
use control::Control;
use defmt::info;
use embassy_stm32::{
    adc::{Adc, SampleTime},
    dma::NoDma,
    executor::InterruptExecutor,
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    interrupt::{self, InterruptExt},
    peripherals::{DMA2_CH4, DMA2_CH5, USART6},
    spi,
    time::{hz, mhz},
    usart::{self, Uart},
    wdg::IndependentWatchdog,
};
use embassy_time::{Delay, Duration, Ticker, Timer};
use futures_util::StreamExt;
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

use embassy_stm32::rcc::AdcClockSource;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::pubsub::{PubSubChannel};

mod control;
mod pins;
mod radio;

#[cfg(not(feature = "no-private-credentials"))]
use credentials::private_credentials::wifi::wifi_credentials;
#[cfg(feature = "no-private-credentials")]
use credentials::public_credentials::wifi::wifi_credentials;

// Uncomment for testing:
// use panic_probe as _;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    // Delay to give it a change to print
    cortex_m::asm::delay(10_000_000);
    cortex_m::peripheral::SCB::sys_reset();
}

include_kicker_bin! {KICKER_FW_IMG, "kicker.bin"}

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

// pub sub channel for the battery raw adc vals
// CAP queue size, n_subs, n_pubs
static BATTERY_CHANNEL: PubSubChannel<ThreadModeRawMutex, f32, 2, 2, 2> = PubSubChannel::new();

#[link_section = ".sram4"]
static mut SPI6_BUF: [u8; 4] = [0x0; 4];

#[link_section = ".axisram.buffers"]
static mut KICKER_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static KICKER_QUEUE_TX: UartWriteQueue<USART6, DMA2_CH4, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut KICKER_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut KICKER_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static KICKER_QUEUE_RX: UartReadQueue<USART6, DMA2_CH5, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut KICKER_BUFFERS_RX });

// static RADIO: Radio<RadioUART, RadioRxDMA, RadioTxDMA> = Radio::new();
static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    stm32_config.rcc.per_ck = Some(mhz(64));
    stm32_config.rcc.adc_clock_source = AdcClockSource::PerCk;
    let p = embassy_stm32::init(stm32_config);
    let config = usart::Config::default();

    // Delay so dotstar and STSPIN can turn on
    Timer::after(Duration::from_millis(50)).await;

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();

    let dotstar_spi = spi::Spi::new_txonly(
        p.SPI3,
        p.PB3,
        p.PB5,
        NoDma,
        NoDma,
        hz(1_000_000),
        spi::Config::default(),
    );

    let mut dotstar = Apa102::new(dotstar_spi);
    let _ = dotstar.write([RGB8 { r: 10, g: 0, b: 0 }].iter().cloned());

    info!("booted");

    let radio_int = interrupt::take!(USART10);
    let radio_usart = Uart::new(
        p.USART10, p.PE2, p.PE3, radio_int, p.DMA2_CH0, p.DMA2_CH1, config,
    );

    let _rotary = Rotary::new(p.PG9, p.PG10, p.PG11, p.PG12);
    let mut shell_indicator = ShellIndicator::new(p.PD0, p.PD1, p.PD3, p.PD4);
    let kicker_det = Input::new(p.PG8, Pull::Down);
    let dip1 = Input::new(p.PG7, Pull::Down);
    let dip2 = Input::new(p.PG6, Pull::Down);
    let dip3 = Input::new(p.PG5, Pull::Down);
    let dip4 = Input::new(p.PG4, Pull::Down);
    let dip5 = Input::new(p.PG3, Pull::Down);
    let _dip6 = Input::new(p.PG2, Pull::Down);
    let dip7 = Input::new(p.PD15, Pull::Down);

    // let robot_id = rotary.read();

    ////////////////////////
    // Dip Switch Inputs  //
    ////////////////////////
    let robot_id = (dip1.is_high() as u8) << 3
        | (dip2.is_high() as u8) << 2
        | (dip3.is_high() as u8) << 1
        | (dip4.is_high() as u8);
    info!("id: {}", robot_id);
    shell_indicator.set(robot_id);

    let wifi_network = WifiNetwork::Team;
    // let wifi_network = if dip5.is_high() & dip6.is_high() {
    //     WifiNetwork::Team
    // } else if dip5.is_low() & dip6.is_high() {
    //     WifiNetwork::CompMain
    // } else if dip5.is_high() & dip6.is_low() {
    //     WifiNetwork::CompPractice
    // } else {
    //     WifiNetwork::Team
    // };

    let control_debug_telemetry_enabled = dip5.is_high();
    if control_debug_telemetry_enabled {
        info!("Debug control telemetry transmission enabled");
    }

    let team = if dip7.is_high() {
        TeamColor::Blue
    } else {
        TeamColor::Yellow
    };

    //////////////////////
    // Battery Voltage  //
    //////////////////////

    let mut adc3 = Adc::new(p.ADC3, &mut Delay);
    adc3.set_sample_time(SampleTime::Cycles1_5);
    let mut battery_pin = p.PF5;
    let mut battery_voltage_buffer: [f32; BATTERY_BUFFER_SIZE] =
        [BATTERY_MAX_VOLTAGE; BATTERY_BUFFER_SIZE];
    let battery_pub = BATTERY_CHANNEL.publisher().unwrap();

    tasks::imu::start_imu_task(_spawner, p.SPI6, p.PA5, p.PA7, p.PA6, p.BDMA_CH0, p.BDMA_CH1, p.PC4, p.PC5, p.PB1, p.PB2, p.EXTI1, p.EXTI2).expect("unable to start IMU task");
    let accel_sub = tasks::imu::get_accel_sub().expect("accel data channel had no subscribers left");
    let gyro_sub = tasks::imu::get_gyro_sub().expect("gyro data channel had no subscribers left");

    // TODO remove?
    Timer::after(Duration::from_millis(1)).await;

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

    let battery_sub = BATTERY_CHANNEL.subscriber().unwrap();

    if kicker_det.is_low() {
        defmt::warn!("kicker appears unplugged!");
    }

    let kicker_int = interrupt::take!(USART6);
    let kicker_usart = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        kicker_int,
        p.DMA2_CH4,
        p.DMA2_CH5,
        get_bootloader_uart_config(),
    );

    let (kicker_tx, kicker_rx) = kicker_usart.split();
    spawner
        .spawn(KICKER_QUEUE_RX.spawn_task(kicker_rx))
        .unwrap();
    spawner
        .spawn(KICKER_QUEUE_TX.spawn_task(kicker_tx))
        .unwrap();

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
        gyro_sub,
        accel_sub,
        battery_sub,
    );

    let _ = dotstar.write([RGB8 { r: 0, g: 0, b: 10 }].iter().cloned());

    control.load_firmware().await;

    info!("flashing kicker...");

    let kicker_boot0_pin = Output::new(p.PA8, Level::Low, Speed::Medium);
    let kicker_reset_pin = Output::new(p.PA9, Level::Low, Speed::Medium);
    let kicker_stm32_interface = Stm32Interface::new_noninverted_reset(
        &KICKER_QUEUE_RX,
        &KICKER_QUEUE_TX,
        Some(kicker_boot0_pin),
        Some(kicker_reset_pin),
    );
    let kicker_firmware_image = KICKER_FW_IMG;
    let mut kicker = Kicker::new(kicker_stm32_interface, kicker_firmware_image);
    let res = kicker.load_default_firmware_image().await;

    if res.is_err() {
        defmt::warn!("kicker flashing failed.");
        loop {}
    } else {
        info!("kicker flash complete");
    }

    let _ = dotstar.write(
        [RGB8 { r: 0, g: 10, b: 0 }, RGB8 { r: 0, g: 0, b: 10 }]
            .iter()
            .cloned(),
    );

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

    let _ = dotstar.write(
        [RGB8 { r: 0, g: 10, b: 0 }, RGB8 { r: 0, g: 10, b: 0 }]
            .iter()
            .cloned(),
    );

    let mut wdg = IndependentWatchdog::new(p.IWDG1, 1_000_000);
    unsafe { wdg.unleash() }

    let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));

    kicker.set_telemetry_enabled(true);
    kicker.send_command();

    defmt::info!("using SSID: {}", wifi_credentials[0].get_ssid());

    loop {
        unsafe { wdg.pet() };

        ///////////////////////
        //  Battery reading  //
        ///////////////////////

        let current_battery_v =
            adc_v_to_battery_voltage(adc_raw_to_v(adc3.read(&mut battery_pin) as f32));
        // Shift buffer through
        for i in (0..(BATTERY_BUFFER_SIZE - 1)).rev() {
            battery_voltage_buffer[i + 1] = battery_voltage_buffer[i];
        }
        // Add new battery read
        battery_voltage_buffer[0] = current_battery_v;
        let battery_voltage_sum: f32 = battery_voltage_buffer.iter().sum();
        let filter_battery_v = battery_voltage_sum / (BATTERY_BUFFER_SIZE as f32);
        battery_pub.publish_immediate(filter_battery_v);

        if filter_battery_v < BATTERY_MIN_VOLTAGE {
            let _ = dotstar.write(
                [RGB8 { r: 10, g: 0, b: 0 }, RGB8 { r: 10, g: 0, b: 0 }]
                    .iter()
                    .cloned(),
            );
            defmt::error!("Error filtered battery voltage too low");
            critical_section::with(|_| loop {
                cortex_m::asm::delay(1_000_000);
                unsafe { wdg.pet() };
            });
        }

        /////////////////////////
        //  Parameter Updates  //
        /////////////////////////
        
        let latest_param_cmd = RADIO_TEST.get_latest_params_cmd();

        if let Some(latest_param_cmd) = latest_param_cmd {
            let param_cmd_resp = control.apply_command(&latest_param_cmd);
            
            // if param_cmd_resp is Err, then the requested parameter update had no submodule acceping the
            // field, or the type was invalid, or the update code is unimplemented
            // if param_cmd_resp is Ok, then the read/write was successful
            if let Ok(resp) = param_cmd_resp {
                defmt::info!("sending successful parameter update command response");
                RADIO_TEST.send_parameter_response(resp).await;
            } else if let Err(resp) = param_cmd_resp {
                defmt::warn!("sending failed parameter updated command response");
                RADIO_TEST.send_parameter_response(resp).await;
            }
        }

        ////////////////
        //  Telemtry  //
        ////////////////

        let latest_control_cmd = RADIO_TEST.get_latest_control();

        let telemetry = control.tick(latest_control_cmd).await;
        if let (Some(mut telemetry), control_debug_telem) = telemetry {
            // info!("{:?}", defmt::Debug2Format(&telemetry));

            telemetry.kicker_charge_level = kicker.hv_rail_voltage();

            RADIO_TEST.send_telemetry(telemetry).await;

            if control_debug_telemetry_enabled {
                RADIO_TEST.send_control_debug_telemetry(control_debug_telem).await;
            }
        }

        kicker.process_telemetry();

        if let Some(control) = latest_control_cmd {
            kicker.set_kick_strength(control.kick_vel);
            kicker.request_kick(control.kick_request);
            kicker.send_command();
        }

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
