#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]

use core::mem;
use static_cell::StaticCell;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;

use embassy_executor::{Executor, InterruptExecutor};
use embassy_stm32::{
    adc::{Adc, SampleTime},
    pac::Interrupt,
    gpio::{Level, Output, Speed},
    interrupt,
    time::mhz,
    usart::{Uart, Parity, StopBits, Config}
};
use embassy_stm32::{bind_interrupts, peripherals, usart};

use embassy_time::{Delay, Duration, Instant, Ticker};

use ateam_kicker_board::{
    adc_raw_to_v,
    adc_v_to_battery_voltage,
    adc_v_to_rail_voltage,
    kick_manager::{
        KickManager, 
        KickType},
    pins::{
        HighVoltageReadPin, BatteryVoltageReadPin,
        ChargePin, KickPin, ChipPin,
        ComsUartModule, ComsUartRxDma, ComsUartTxDma, RedStatusLedPin, BlueStatusLedPin},
    queue::Buffer,
    uart_queue::{
        UartReadQueue,
        UartWriteQueue,
    }
};

use ateam_common_packets::bindings_kicker::{KickerControl, KickerTelemetry, KickRequest};

const MAX_TX_PACKET_SIZE: usize = 16;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 16;
const RX_BUF_DEPTH: usize = 3;

// control communications tx buffer
#[link_section = ".bss"]
static mut COMS_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static COMS_QUEUE_TX: UartWriteQueue<ComsUartModule, ComsUartTxDma, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut COMS_BUFFERS_TX });

// control communications rx buffer
#[link_section = ".bss"]
static mut COMS_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static COMS_QUEUE_RX: UartReadQueue<ComsUartModule, ComsUartRxDma, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut COMS_BUFFERS_RX });

fn get_empty_control_packet() -> KickerControl {
    KickerControl {
        _bitfield_align_1: [],
        _bitfield_1: KickerControl::new_bitfield_1(0, 0, 0),
        kick_request: KickRequest::KR_DISABLE,
        kick_speed: 0.0,
    }
}

fn get_empty_telem_packet() -> KickerTelemetry {
    KickerTelemetry {
        _bitfield_align_1: [],
        _bitfield_1: KickerTelemetry::new_bitfield_1(0, 0, 0, 0),
        rail_voltage: 0.0,
        battery_voltage: 0.0,
    }
}

#[embassy_executor::task]
async fn high_pri_kick_task(
        coms_reader: &'static UartReadQueue<'static, ComsUartModule, ComsUartRxDma, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH>,
        coms_writer: &'static UartWriteQueue<'static, ComsUartModule, ComsUartTxDma, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>,
        mut adc: Adc<'static, embassy_stm32::peripherals::ADC>,
        charge_pin: ChargePin,
        kick_pin: KickPin,
        chip_pin: ChipPin,
        mut rail_pin: HighVoltageReadPin,
        mut battery_voltage_pin: BatteryVoltageReadPin,
        err_led_pin: RedStatusLedPin,
        ball_detected_led_pin: BlueStatusLedPin) -> ! {

    // pins/safety management
    let charge_pin = Output::new(charge_pin, Level::Low, Speed::Medium);
    let kick_pin = Output::new(kick_pin, Level::Low, Speed::Medium);
    let chip_pin = Output::new(chip_pin, Level::Low, Speed::Medium);
    let mut kick_manager = KickManager::new(charge_pin, kick_pin, chip_pin);

    // debug LEDs
    let mut err_led = Output::new(err_led_pin, Level::Low, Speed::Low);
    let mut ball_detected_led = Output::new(ball_detected_led_pin, Level::Low, Speed::Low);
    // TODO dotstars

    // coms buffers
    let mut telemetry_enabled: bool = false;
    let mut kicker_control_packet: KickerControl = get_empty_control_packet();
    let mut kicker_telemetry_packet: KickerTelemetry = get_empty_telem_packet();

    // loop rate control
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut last_packet_sent_time = Instant::now();

    loop {
        let rail_voltage = adc_v_to_rail_voltage(adc_raw_to_v(adc.read(&mut rail_pin) as f32));
        let battery_voltage = adc_v_to_battery_voltage(adc_raw_to_v(adc.read(&mut battery_voltage_pin) as f32));
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
        let res = kick_manager.command(battery_voltage, rail_voltage, false, KickType::None, 0.0).await;

        // send telemetry packet
        if telemetry_enabled {
            let cur_time = Instant::now();
            if Instant::checked_duration_since(&cur_time, last_packet_sent_time).unwrap().as_millis() > 20 {
                kicker_telemetry_packet._bitfield_1 = KickerTelemetry::new_bitfield_1(0, 0, ball_detected as u32, res.is_err() as u32);
                kicker_telemetry_packet.rail_voltage = rail_voltage;
                kicker_telemetry_packet.battery_voltage = battery_voltage;

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

                if ball_detected_led.is_set_high() {
                    ball_detected_led.set_low();
                } else {
                    ball_detected_led.set_high();
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
            ball_detected_led.set_high();
        } else {
            ball_detected_led.set_low();
        }
        // TODO Dotstar

        // loop rate control @1KHz
        ticker.next().await;
    }

}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn I2C1() {
    EXECUTOR_HIGH.on_interrupt();
}

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[entry]
fn main() -> ! {
    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.sys_ck = Some(mhz(48));
    stm32_config.rcc.hclk = Some(mhz(48));
    stm32_config.rcc.pclk = Some(mhz(48));

    let p = embassy_stm32::init(stm32_config);

    info!("kicker startup!");
    
    let mut nvic: NVIC = unsafe { mem::transmute(()) };

    let _status_led = Output::new(p.PA11, Level::High, Speed::Low);

    let mut adc = Adc::new(p.ADC, &mut Delay);
    adc.set_sample_time(SampleTime::Cycles71_5);

    // high priority executor handles kicking system
    // High-priority executor: I2C1, priority level 6
    // TODO CHECK THIS IS THE HIGHEST PRIORITY
    unsafe { nvic.set_priority(Interrupt::I2C1, 6 << 4) };
    let spawner = EXECUTOR_HIGH.start(Interrupt::I2C1);
    unwrap!(spawner.spawn(high_pri_kick_task(&COMS_QUEUE_RX, &COMS_QUEUE_TX, adc, p.PB3, p.PB0, p.PB1, p.PA0, p.PA1, p.PA12, p.PA8)));
    // unwrap!(spawner.spawn(high_pri_tx_test(&COMS_QUEUE_TX, p.PB3, p.PB0, p.PB1)));


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
        p.DMA1_CH2,
        p.DMA1_CH3,
        coms_uart_config,
    );

    let (coms_uart_tx, coms_uart_rx) = coms_usart.split();

    // low priority executor handles coms and user IO
    // Low priority executor: runs in thread mode, using WFE/SEV
    let lp_executor = EXECUTOR_LOW.init(Executor::new());
    lp_executor.run(|spawner| {
        unwrap!(spawner.spawn(COMS_QUEUE_TX.spawn_task(coms_uart_tx)));
        unwrap!(spawner.spawn(COMS_QUEUE_RX.spawn_task(coms_uart_rx)));
    });
}