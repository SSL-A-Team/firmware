#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]
#![feature(async_closure)]
#![feature(sync_unsafe_cell)]

use apa102_spi::Apa102;
use ateam_control_board::{
    adc_raw_to_v, adc_v_to_battery_voltage, drivers::{
        kicker::Kicker, radio::{TeamColor, WifiNetwork}, rotary::Rotary, shell_indicator::ShellIndicator
    }, get_system_config, include_kicker_bin, parameter_interface::ParameterInterface, stm32_interface::{get_bootloader_uart_config, Stm32Interface}, BATTERY_BUFFER_SIZE, BATTERY_MAX_VOLTAGE, BATTERY_MIN_VOLTAGE
};
use control::Control;
use defmt::info;
use embassy_stm32::{
    adc::{Adc, SampleTime}, bind_interrupts, exti::ExtiInput, gpio::{Input, Level, Output, Pull, Speed}, interrupt, pac::Interrupt, peripherals::{self, BDMA_CH0}, spi, time::{hz, mhz}, usart::{self, Uart}, wdg::IndependentWatchdog
};
use embassy_executor::InterruptExecutor;
use embassy_time::{Duration, Ticker, Timer};
use ateam_control_board::pins::{
    PowerStateExti, PowerStatePin, RadioReset, RadioRxDMA, RadioTxDMA, RadioUART,
    ShutdownCompletePin,
};

use smart_leds::{SmartLedsWrite, RGB8};

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::pubsub::PubSubChannel;

use ateam_lib_stm32::make_uart_queues;

use ateam_control_board::motion::tasks::control;
use ateam_control_board::pins::*;
use ateam_control_board::radio::RadioTest;

#[cfg(not(feature = "no-private-credentials"))]
use credentials::private_credentials::wifi::wifi_credentials;
#[cfg(feature = "no-private-credentials")]
use credentials::public_credentials::wifi::wifi_credentials;
use static_cell::ConstStaticCell;

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

pub const RADIO_MAX_TX_PACKET_SIZE: usize = 256;
pub const RADIO_TX_BUF_DEPTH: usize = 4;
pub const RADIO_MAX_RX_PACKET_SIZE: usize = 256;
pub const RADIO_RX_BUF_DEPTH: usize = 4;

pub const KICKER_MAX_TX_PACKET_SIZE: usize = 256;
pub const KICKER_TX_BUF_DEPTH: usize = 4;
pub const KICKER_MAX_RX_PACKET_SIZE: usize = 256;
pub const KICKER_RX_BUF_DEPTH: usize = 4;

make_uart_queues!(RADIO,
    RadioUART, RadioRxDMA, RadioTxDMA,
    RADIO_MAX_RX_PACKET_SIZE, RADIO_RX_BUF_DEPTH,
    RADIO_MAX_TX_PACKET_SIZE, RADIO_TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

make_uart_queues!(KICKER,
    KickerUart, KickerRxDma, KickerTxDma,
    KICKER_MAX_RX_PACKET_SIZE, KICKER_RX_BUF_DEPTH,
    KICKER_MAX_TX_PACKET_SIZE, KICKER_TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

#[link_section = ".sram4"]
static mut SPI6_BUF: [u8; 4] = [0x0; 4];

static RADIO_TEST: ConstStaticCell<RadioTest<
        RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH,
        RadioUART, RadioRxDMA, RadioTxDMA, RadioReset>> = 
    ConstStaticCell::new(RadioTest::new(&RADIO_TX_UART_QUEUE, &RADIO_RX_UART_QUEUE));
// static RADIO_TEST: RadioTest<
//     RADIO_MAX_TX_PACKET_SIZE,
//     RADIO_MAX_RX_PACKET_SIZE,
//     RADIO_TX_BUF_DEPTH,
//     RADIO_RX_BUF_DEPTH,
//     RadioUART,
//     RadioRxDMA,
//     RadioTxDMA,
//     RadioReset,
// > = RadioTest::new(&RADIO_TX_UART_QUEUE, &RADIO_RX_UART_QUEUE);

// pub sub channel for the gyro vals
// CAP queue size, n_subs, n_pubs
static GYRO_CHANNEL: PubSubChannel<ThreadModeRawMutex, f32, 2, 2, 2> = PubSubChannel::new();

// pub sub channel for the battery raw adc vals
// CAP queue size, n_subs, n_pubs
static BATTERY_CHANNEL: PubSubChannel<ThreadModeRawMutex, f32, 2, 2, 2> = PubSubChannel::new();

// static RADIO: Radio<RadioUART, RadioRxDMA, RadioTxDMA> = Radio::new();
static EXECUTOR_UART_QUEUE: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    EXECUTOR_UART_QUEUE.on_interrupt();
}

bind_interrupts!(struct Irqs {
    USART10 => usart::InterruptHandler<peripherals::USART10>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    UART7 => usart::InterruptHandler<peripherals::UART7>;
    UART8 => usart::InterruptHandler<peripherals::UART8>;
    UART5 => usart::InterruptHandler<peripherals::UART5>;
});

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    let config = usart::Config::default();

    // Delay so dotstar and STSPIN can turn on
    Timer::after(Duration::from_millis(50)).await;

    interrupt::InterruptExt::set_priority(interrupt::CEC, interrupt::Priority::P6);
    let spawner = EXECUTOR_UART_QUEUE.start(Interrupt::CEC);

    let mut dotstar_spi_config = spi::Config::default();
    dotstar_spi_config.frequency = hz(1_000_000);
    let dotstar_spi = spi::Spi::new_txonly(
        p.SPI3,
        p.PB3,
        p.PB5,
        p.DMA2_CH6,
        dotstar_spi_config,
    );

    let mut dotstar = Apa102::new(dotstar_spi);
    let _ = dotstar.write([RGB8 { r: 10, g: 0, b: 0 }].iter().cloned());

    info!("booted");

    let radio_usart = Uart::new(
        p.USART10, p.PE2, p.PE3, Irqs, p.DMA2_CH0, p.DMA2_CH1, config,
    ).unwrap();

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

    let mut adc3 = Adc::new(p.ADC3);
    adc3.set_sample_time(SampleTime::CYCLES1_5);
    let mut battery_pin = p.PF5;
    let mut battery_voltage_buffer: [f32; BATTERY_BUFFER_SIZE] =
        [BATTERY_MAX_VOLTAGE; BATTERY_BUFFER_SIZE];
    let battery_pub = BATTERY_CHANNEL.publisher().unwrap();

    let mut imu_spi_config = spi::Config::default();
    imu_spi_config.frequency = mhz(1);
    let mut imu_spi = spi::Spi::new(
        p.SPI6,
        p.PA5,
        p.PA7,
        p.PA6,
        p.BDMA_CH0,
        p.BDMA_CH1,
        imu_spi_config,
    );

    // acceleromter
    let mut imu_cs1 = Output::new(p.PC4, Level::High, Speed::VeryHigh);
    // gyro
    let mut imu_cs2 = Output::new(p.PC5, Level::High, Speed::VeryHigh);

    Timer::after(Duration::from_millis(1)).await;

    let gyro_pub = GYRO_CHANNEL.publisher().unwrap();
    unsafe {
        SPI6_BUF[0] = 0x80;
        // info!("xfer {=[u8]:x}", SPI6_BUF[0..1]);
        imu_cs1.set_low();
        let _ = imu_spi.transfer_in_place(&mut SPI6_BUF[0..2]).await;
        imu_cs1.set_high();
        let accel_id = SPI6_BUF[1];
        info!("accelerometer id: 0x{:x}", accel_id);

        SPI6_BUF[0] = 0x80;
        imu_cs2.set_low();
        let _ = imu_spi.transfer_in_place(&mut SPI6_BUF[0..2]).await;
        imu_cs2.set_high();
        let gyro_id = SPI6_BUF[1];
        info!("gyro id: 0x{:x}", gyro_id);
    }

    let front_right_usart = Uart::new(
        p.USART1,
        p.PB15,
        p.PB14,
        Irqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        get_bootloader_uart_config(),
    ).unwrap();
    let front_left_usart = Uart::new(
        p.UART4,
        p.PA1,
        p.PA0,
        Irqs,
        p.DMA1_CH2,
        p.DMA1_CH3,
        get_bootloader_uart_config(),
    ).unwrap();
    let back_left_usart = Uart::new(
        p.UART7,
        p.PF6,
        p.PF7,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        get_bootloader_uart_config(),
    ).unwrap();
    let back_right_usart = Uart::new(
        p.UART8,
        p.PE0,
        p.PE1,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        get_bootloader_uart_config(),
    ).unwrap();
    let drib_usart = Uart::new(
        p.UART5,
        p.PB12,
        p.PB13,
        Irqs,
        p.DMA2_CH2,
        p.DMA2_CH3,
        get_bootloader_uart_config(),
    ).unwrap();

    let gyro_sub = GYRO_CHANNEL.subscriber().unwrap();
    let battery_sub = BATTERY_CHANNEL.subscriber().unwrap();

    if kicker_det.is_low() {
        defmt::warn!("kicker appears unplugged!");
    }

    let kicker_usart = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        Irqs,
        p.DMA2_CH4,
        p.DMA2_CH5,
        get_bootloader_uart_config(),
    ).unwrap();

    let (kicker_tx, kicker_rx) = Uart::split(kicker_usart);
    spawner
        .spawn(KICKER_RX_UART_QUEUE.spawn_task(kicker_rx))
        .unwrap();
    spawner
        .spawn(KICKER_TX_UART_QUEUE.spawn_task(kicker_tx))
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
        battery_sub,
    );

    let _ = dotstar.write([RGB8 { r: 0, g: 0, b: 10 }].iter().cloned());

    control.load_firmware().await;

    info!("flashing kicker...");

    let kicker_boot0_pin = Output::new(p.PA8, Level::Low, Speed::Medium);
    let kicker_reset_pin = Output::new(p.PA9, Level::Low, Speed::Medium);
    let kicker_stm32_interface = Stm32Interface::new_noninverted_reset(
        &KICKER_RX_UART_QUEUE,
        &KICKER_TX_UART_QUEUE,
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

    // let token = unsafe {
    //     (&mut *(&RADIO_TEST as *const _
    //         as *mut RadioTest<
    //             RADIO_MAX_TX_PACKET_SIZE,
    //             RADIO_MAX_RX_PACKET_SIZE,
    //             RADIO_TX_BUF_DEPTH,
    //             RADIO_RX_BUF_DEPTH,
    //             RadioUART,
    //             RadioRxDMA,
    //             RadioTxDMA,
    //             RadioReset,
    //         >))
    //         .setup(&spawner, radio_usart, p.PC13, robot_id, team, wifi_network)
    //         .await
    // };
    let radio_test = RADIO_TEST.take();
    let token = unsafe {
        (&mut *(&RADIO_TEST as *const _
            as *mut RadioTest<
                RADIO_MAX_TX_PACKET_SIZE,
                RADIO_MAX_RX_PACKET_SIZE,
                RADIO_TX_BUF_DEPTH,
                RADIO_RX_BUF_DEPTH,
                RadioUART,
                RadioRxDMA,
                RadioTxDMA,
                RadioReset,
            >))
            .setup(&spawner, radio_usart, p.PC13, robot_id, team, wifi_network)
            .await
    };
    // let token = radio_test.setup(&spawner, radio_usart, p.PC13, robot_id, team, wifi_network).await;
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

        unsafe {
            SPI6_BUF[0] = 0x86;
            imu_cs2.set_low();
            let _ = imu_spi.transfer_in_place(&mut SPI6_BUF[0..3]).await;
            imu_cs2.set_high();
            let rate_z = (SPI6_BUF[2] as u16 * 256 + SPI6_BUF[1] as u16) as i16;
            // info!("z rate: {}", rate_z);
            let gyro_conversion = 2000.0 / 32767.0;
            gyro_pub.publish_immediate((rate_z as f32) * gyro_conversion);
        }

        // could just feed gyro in here but the comment in control said to use a channel

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
        
        let latest_param_cmd = radio_test.get_latest_params_cmd();

        if let Some(latest_param_cmd) = latest_param_cmd {
            let param_cmd_resp = control.apply_command(&latest_param_cmd);
            
            // if param_cmd_resp is Err, then the requested parameter update had no submodule acceping the
            // field, or the type was invalid, or the update code is unimplemented
            // if param_cmd_resp is Ok, then the read/write was successful
            if let Ok(resp) = param_cmd_resp {
                defmt::info!("sending successful parameter update command response");
                radio_test.send_parameter_response(resp).await;
            } else if let Err(resp) = param_cmd_resp {
                defmt::warn!("sending failed parameter updated command response");
                radio_test.send_parameter_response(resp).await;
            }
        }

        ////////////////
        //  Telemtry  //
        ////////////////

        let latest_control_cmd = radio_test.get_latest_control();

        let telemetry = control.tick(latest_control_cmd).await;
        if let (Some(mut telemetry), control_debug_telem) = telemetry {
            // info!("{:?}", defmt::Debug2Format(&telemetry));

            telemetry.kicker_charge_level = kicker.hv_rail_voltage();
            telemetry.set_breakbeam_ball_detected(kicker.ball_detected() as u32);

            radio_test.send_telemetry(telemetry).await;

            if control_debug_telemetry_enabled {
                radio_test.send_control_debug_telemetry(control_debug_telem).await;
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
    let mut shutdown = Output::new(shutdown_pin, Level::Low, Speed::Low);
    let mut power_state = ExtiInput::new(power_state_pin, exti, Pull::None);
    power_state.wait_for_falling_edge().await;
    shutdown.set_high();
    loop {}
}
