#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]
#![feature(sync_unsafe_cell)]

use static_cell::StaticCell;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_stm32::{
    adc::{Adc, Resolution, SampleTime},
    gpio::{Level, Output, Speed},
    interrupt,
    interrupt::InterruptExt,
    pac::Interrupt,
    rcc::{AHBPrescaler, APBPrescaler, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllSource, Sysclk},
    usart::{Config, Parity, StopBits, Uart}
};
use embassy_stm32::{bind_interrupts, peripherals, usart};

use embassy_time::{Duration, Instant, Ticker, Timer};

use ateam_kicker_board::{
    adc_200v_to_rail_voltage, adc_raw_to_v,
    kick_manager::{
        KickManager, 
        KickType}, 
    pins::*
};

use ateam_lib_stm32::{make_uart_queue_pair, queue_pair_register_and_spawn};
use ateam_lib_stm32::uart::queue::{UartReadQueue, UartWriteQueue};

use ateam_common_packets::bindings::{KickerControl, KickerTelemetry, KickRequest};

const MAX_TX_PACKET_SIZE: usize = 16;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 16;
const RX_BUF_DEPTH: usize = 3;

make_uart_queue_pair!(COMS,
    ComsUartModule, ComsUartRxDma, ComsUartTxDma,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".bss"]);

#[embassy_executor::task]
async fn high_pri_kick_task(
        coms_reader: &'static UartReadQueue<ComsUartModule, ComsUartRxDma, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH>,
        coms_writer: &'static UartWriteQueue<ComsUartModule, ComsUartTxDma, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>,
        mut adc: Adc<'static, embassy_stm32::peripherals::ADC1>,
        charge_pin: ChargePin,
        kick_pin: KickPin,
        chip_pin: ChipPin,
        mut rail_pin: PowerRail200vReadPin,
        err_led_pin: RedStatusLedPin,
        ball_detected_led1_pin: BlueStatusLed1Pin,
        ball_detected_led2_pin: BlueStatusLed2Pin) -> ! {

    // pins/safety management
    let charge_pin = Output::new(charge_pin, Level::Low, Speed::Medium);
    let kick_pin = Output::new(kick_pin, Level::Low, Speed::Medium);
    let chip_pin = Output::new(chip_pin, Level::Low, Speed::Medium);
    let mut kick_manager = KickManager::new(charge_pin, kick_pin, chip_pin);

    // debug LEDs
    let mut err_led = Output::new(err_led_pin, Level::Low, Speed::Low);
    let mut ball_detected_led1 = Output::new(ball_detected_led1_pin, Level::Low, Speed::Low);
    let mut ball_detected_led2 = Output::new(ball_detected_led2_pin, Level::Low, Speed::Low);

    // TODO dotstars

    // coms buffers
    let mut telemetry_enabled: bool;
    let mut kicker_control_packet: KickerControl = Default::default();
    let mut kicker_telemetry_packet: KickerTelemetry = Default::default();

    // loop rate control
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut last_packet_sent_time = Instant::now();

    loop {
        let mut vrefint = adc.enable_vrefint();
        let vrefint_sample = adc.read(&mut vrefint) as f32;

        let rail_voltage = adc_200v_to_rail_voltage(adc_raw_to_v(adc.read(&mut rail_pin) as f32, vrefint_sample));
        // optionally pre-flag errors? 

        /////////////////////////////////////
        //  process any available packets  //
        /////////////////////////////////////
        
        while let Ok(res) = coms_reader.try_dequeue() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<KickerControl>() {
                defmt::warn!("got invalid packet of len {:?} data: {:?}", buf.len(), buf);
                continue;
            }

            // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            unsafe {
                // copy receieved uart bytes into packet
                let state = &mut kicker_control_packet as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<KickerControl>() {
                    *state.offset(i as isize) = buf[i];
                }                
            }
        }

        // TODO: read breakbeam
        let ball_detected = false;

        ///////////////////////////////////////////////
        //  manage repetitive kick commands + state  //
        ///////////////////////////////////////////////

        // update telemetry requests
        telemetry_enabled = kicker_control_packet.telemetry_enabled() != 0;

        // no charge/kick in coms test
        let res = kick_manager.command(22.5, rail_voltage, false, KickType::None, 0.0).await;

        // send telemetry packet
        if telemetry_enabled {
            let cur_time = Instant::now();
            if Instant::checked_duration_since(&cur_time, last_packet_sent_time).unwrap().as_millis() > 20 {
                kicker_telemetry_packet._bitfield_1 = KickerTelemetry::new_bitfield_1(res.is_err() as u16, 0, 0, 0, ball_detected as u16, 0, Default::default());
                kicker_telemetry_packet.rail_voltage = rail_voltage;
                kicker_telemetry_packet.battery_voltage = 22.5;

                // raw interpretaion of a struct for wire transmission is unsafe
                unsafe {
                    // get a slice to packet for transmission
                    let struct_bytes = core::slice::from_raw_parts(
                        (&kicker_telemetry_packet as *const KickerTelemetry) as *const u8,
                        core::mem::size_of::<KickerTelemetry>(),
                    );
        
                    // send the packet
                    let _res = coms_writer.enqueue_copy(struct_bytes);
                }

                if ball_detected_led1.is_set_high() {
                    ball_detected_led1.set_low();
                } else {
                    ball_detected_led1.set_high();
                }

                if ball_detected_led2.is_set_high() {
                    ball_detected_led2.set_low();
                } else {
                    ball_detected_led2.set_high();
                }

                last_packet_sent_time = cur_time;
            }
        }

        // LEDs
        if res.is_err() {
            err_led.set_high();
        } else {
            err_led.set_low();
        }

        if ball_detected {
            ball_detected_led1.set_high();
            ball_detected_led2.set_high();
        } else {
            ball_detected_led1.set_low();
            ball_detected_led2.set_low();

        }
        // TODO Dotstar

        // loop rate control @1KHz
        ticker.next().await;
    }

}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn TIM2() {
    EXECUTOR_HIGH.on_interrupt();
}

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hsi = true;
    stm32_config.rcc.pll_src = PllSource::HSI; // internal 16Mhz source
    stm32_config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV8, // base 2MHz 
        mul: PllMul::MUL168, // mul up to 336 MHz
        divp: Some(PllPDiv::DIV2), // div down to 168 MHz, AHB (max)
        divq: Some(PllQDiv::DIV7), // div down to 48 Mhz for dedicated 48MHz clk (exact)
        divr: None, // turn off I2S clk
    });
    stm32_config.rcc.ahb_pre = AHBPrescaler::DIV1; // AHB 168 MHz (max)
    stm32_config.rcc.apb1_pre = APBPrescaler::DIV4; // APB1 42 MHz (max)
    stm32_config.rcc.apb2_pre = APBPrescaler::DIV2; // APB2 84 MHz (max)
    stm32_config.rcc.sys = Sysclk::PLL1_P; // enable the system

    let p = embassy_stm32::init(stm32_config);

    info!("kicker startup!");

    let _status_led = Output::new(p.PA11, Level::High, Speed::Low);

    let mut adc = Adc::new(p.ADC1);
    adc.set_resolution(Resolution::BITS12);
    adc.set_sample_time(SampleTime::CYCLES480);

    // high priority executor handles kicking system
    // High-priority executor: I2C1, priority level 6
    // TODO CHECK THIS IS THE HIGHEST PRIORITY
    embassy_stm32::interrupt::TIM2.set_priority(embassy_stm32::interrupt::Priority::P6);
    let hp_spawner = EXECUTOR_HIGH.start(Interrupt::TIM2);

    //////////////////////////////////
    //  COMMUNICATIONS TASKS SETUP  //
    //////////////////////////////////

    let mut coms_uart_config = Config::default();
    coms_uart_config.baudrate = 2_000_000; // 2 Mbaud
    coms_uart_config.parity = Parity::ParityEven;
    coms_uart_config.stop_bits = StopBits::STOP1;

    let coms_usart = Uart::new(
        p.USART1,
        p.PA10,
        p.PA9,
        Irqs,
        p.DMA2_CH7,
        p.DMA2_CH2,
        coms_uart_config,
    ).unwrap();

    let (coms_uart_tx, coms_uart_rx) = Uart::split(coms_usart);
    queue_pair_register_and_spawn!(spawner, COMS, coms_uart_rx, coms_uart_tx);

    hp_spawner.spawn(high_pri_kick_task(&COMS_RX_UART_QUEUE, &COMS_TX_UART_QUEUE, adc, p.PE4, p.PE5, p.PE6, p.PC0, p.PE1, p.PE2, p.PE3)).unwrap();


    loop {
        Timer::after_millis(1000).await;
    }
}