use core::mem::MaybeUninit;
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::usart::{self, DataBits, Parity, StopBits, Uart};

use ateam_lib_stm32::{audio::{songs::SongId, AudioCommand}, idle_buffered_uart_spawn_tasks, static_idle_buffered_uart_nl, uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue}};
use ateam_common_packets::bindings::{BatteryInfo, PowerCommand, PowerTelemetry};
use embassy_time::{Duration, Instant, Ticker};
use crate::{pins::*, power_state::SharedPowerState, SystemIrqs};

const MAX_RX_PACKET_SIZE: usize = core::mem::size_of::<PowerCommand>();
const MAX_TX_PACKET_SIZE: usize = core::mem::size_of::<PowerTelemetry>();
const RX_BUF_DEPTH: usize = 3;
const TX_BUF_DEPTH: usize = 2;
static_idle_buffered_uart_nl!(COMS, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH);

const POWER_TELEM_PERIOD_MS: u64 = 100;

const COMS_RX_TIMEOUT: Duration = Duration::from_millis(1000);

#[macro_export]
macro_rules! create_coms_task {
    ($main_spawner:ident, $uart_queue_spawner:ident, $shared_power_state:ident, $coms_telemetry_subscriber:ident, $coms_audio_publisher: ident, $p:ident) => {
        ateam_power_board::tasks::coms_task::start_coms_task(
            $main_spawner, $uart_queue_spawner, $shared_power_state, $coms_telemetry_subscriber, $coms_audio_publisher,
            $p.USART1, $p.PA10, $p.PA9, $p.DMA1_CH5, $p.DMA1_CH4,
        ).await;
    };
}

#[embassy_executor::task]
async fn coms_task_entry(
    _uart: &'static IdleBufferedUart<MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>,
    read_queue: &'static UartReadQueue<MAX_RX_PACKET_SIZE, RX_BUF_DEPTH>,
    write_queue: &'static UartWriteQueue<MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>,
    shared_power_state: &'static SharedPowerState,
    mut telemetry_subscriber: TelemetrySubscriber,
    audio_publisher: AudioPublisher
) {
    let mut loop_rate_ticker = Ticker::every(Duration::from_millis(10));
    let mut last_battery_telem_packet: BatteryInfo = Default::default();
    let mut last_sent_packet_time = Instant::now();
    let mut last_received_packet_time = Instant::MAX - Duration::from_secs(60);

    loop {
        let cur_power_state = shared_power_state.get_state().await;

        // read incoming packets
        while let Ok(res) = read_queue.try_dequeue() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<PowerCommand>() {
                defmt::warn!("COMS TASK - Got invalid packet of len {:?} (expected {:?}) data: {:?}", buf.len(), core::mem::size_of::<PowerCommand>(), buf);
                continue;
            }

            defmt::trace!("COMS TASK - received valid command packet from control board");

            last_received_packet_time = Instant::now();
            if !cur_power_state.coms_established {
                shared_power_state.set_coms_established(true).await;
            }

            // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            let mut command_packet: PowerCommand;
            unsafe {
                command_packet = MaybeUninit::zeroed().assume_init();

                // copy receieved uart bytes into packet
                let state = &mut command_packet as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<PowerCommand>() {
                    *state.offset(i as isize) = buf[i];
                }
            }

            // Update power state from command packet
            // Latch state for force shutdown, ready shutdown, request shutdown
            // Cancel shutdown will clear ready shutdown and request shutdown state
            if command_packet.force_shutdown() != 0 {
                defmt::info!("COMS TASK - force shutdown received from control board");
                shared_power_state.set_shutdown_force(true).await;
            }
            
            if command_packet.ready_shutdown() != 0 {
                defmt::info!("COMS TASK - ready shutdown received from control board");
                shared_power_state.set_shutdown_ready(true).await;
            }

            if command_packet.request_shutdown() != 0 {
                defmt::info!("COMS TASK - request shutdown received from control board");
                audio_publisher.publish(AudioCommand::PlaySong(SongId::ShutdownRequested)).await;
                shared_power_state.set_shutdown_requested(true).await;
            }

            if command_packet.cancel_shutdown() != 0 {
                defmt::info!("COMS TASK - cancel shutdown received from control board");
                shared_power_state.set_shutdown_ready(false).await;
                shared_power_state.set_shutdown_requested(false).await;
            }
        }

        // Read from telemetry channel, only keep latest telemetry
        while let Some(telemetry_packet) = telemetry_subscriber.try_next_message_pure() {
            defmt::trace!("COMS TASK - New battery telemetry packet received from power task");
            last_battery_telem_packet = telemetry_packet;
        }
        

        // if we've timed out coms, set it in the state
        if cur_power_state.coms_established && Instant::now() > last_received_packet_time + COMS_RX_TIMEOUT {
            shared_power_state.set_coms_established(false).await;
        }

        // Send telemetry to control board
        if Instant::now() - last_sent_packet_time >= Duration::from_millis(POWER_TELEM_PERIOD_MS) {

            // build telemetry packet from latest battery telemetry and power status
            let mut telem_packet: PowerTelemetry = Default::default();

            telem_packet.set_power_ok(cur_power_state.power_ok as u32);
            telem_packet.set_power_rail_3v3_ok(cur_power_state.power_rail_3v3_ok as u32);
            telem_packet.set_power_rail_5v0_ok(cur_power_state.power_rail_5v0_ok as u32);
            telem_packet.set_power_rail_12v0_ok(cur_power_state.power_rail_12v0_ok as u32);
            telem_packet.set_high_current_operations_allowed(cur_power_state.high_current_operations_allowed as u32);
            telem_packet.set_shutdown_requested(cur_power_state.shutdown_requested as u32);
            telem_packet.battery_info = last_battery_telem_packet;

            let struct_bytes;
            unsafe {
                struct_bytes = core::slice::from_raw_parts(
                    (&telem_packet as *const PowerTelemetry) as *const u8,
                    core::mem::size_of::<PowerTelemetry>(),
                );
            }
            let _ = write_queue.enqueue_copy(struct_bytes);
            last_sent_packet_time = Instant::now();
            defmt::trace!("COMS TAKS - Sent telemetry packet")
        }

        loop_rate_ticker.next().await;
    }
}

pub fn power_uart_config() -> usart::Config {
    let mut power_uart_config = usart::Config::default();
    power_uart_config.baudrate = 115_200;
    power_uart_config.data_bits = DataBits::DataBits8;
    power_uart_config.stop_bits = StopBits::STOP1;
    power_uart_config.parity = Parity::ParityNone;

    power_uart_config
}

pub async fn start_coms_task(
    spawner: Spawner, 
    uart_queue_spawner: SendSpawner,
    shared_power_state: &'static SharedPowerState,
    telemetry_subscriber: TelemetrySubscriber,
    audio_publisher: AudioPublisher,
    uart: ComsUart,
    uart_rx_pin: ComsUartRxPin,
    uart_tx_pin: ComsUartTxPin,
    uart_rx_dma: ComsDmaRx,
    uart_tx_dma: ComsDmaTx
) {
    let uart_config = power_uart_config();
    let coms_uart = Uart::new(uart, uart_rx_pin, uart_tx_pin, SystemIrqs, uart_tx_dma, uart_rx_dma, uart_config).unwrap();
    COMS_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, COMS, coms_uart);
    spawner.spawn(coms_task_entry(
        &COMS_IDLE_BUFFERED_UART, &COMS_IDLE_BUFFERED_UART.get_uart_read_queue(), &COMS_IDLE_BUFFERED_UART.get_uart_write_queue(), shared_power_state, telemetry_subscriber, audio_publisher
    )).expect("failed to spawn coms task");
}