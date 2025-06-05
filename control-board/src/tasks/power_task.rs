use core::mem::MaybeUninit;

use ateam_common_packets::bindings::{PowerCommandPacket, PowerStatusPacket};
use ateam_lib_stm32::{idle_buffered_uart_spawn_tasks, power, static_idle_buffered_uart, uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue}};
use embassy_executor::{SendSpawner, Spawner};
use embassy_futures::select::{select, Either};
use embassy_stm32::usart::{self, DataBits, Parity, StopBits, Uart};
use embassy_time::{Duration, Instant, Ticker, Timer};

use crate::{pins::*, robot_state::SharedRobotState, tasks::dotstar_task::{ControlBoardLedCommand, ControlGeneralLedCommand}, SystemIrqs};

#[macro_export]
macro_rules! create_power_task {
    ($main_spawner:ident, $uart_queue_spawner:ident, $robot_state:ident, $led_cmd_pub:ident, $p:ident) => {
        ateam_control_board::tasks::power_task::start_power_task(
            $main_spawner, $uart_queue_spawner,
            $robot_state, $led_cmd_pub,
            $p.UART9, $p.PG0, $p.PG1, $p.DMA2_CH4, $p.DMA2_CH5);
    };
}

pub const POWER_LOOP_RATE_MS: u64 = 10;

pub const POWER_MAX_TX_PACKET_SIZE: usize = core::mem::size_of::<PowerCommandPacket>();
pub const POWER_MAX_RX_PACKET_SIZE: usize = core::mem::size_of::<PowerStatusPacket>();
pub const POWER_TX_BUF_DEPTH: usize = 4;
pub const POWER_RX_BUF_DEPTH: usize = 4;

static_idle_buffered_uart!(POWER, POWER_MAX_RX_PACKET_SIZE, POWER_RX_BUF_DEPTH, POWER_MAX_TX_PACKET_SIZE, POWER_TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);


pub struct PowerTask<
        const POWER_MAX_TX_PACKET_SIZE: usize,
        const POWER_MAX_RX_PACKET_SIZE: usize,
        const POWER_TX_BUF_DEPTH: usize,
        const POWER_RX_BUF_DEPTH: usize> {
    shared_robot_state: &'static SharedRobotState,
    led_cmd_publisher: LedCommandPublisher,
    _power_uart: &'static IdleBufferedUart<POWER_MAX_RX_PACKET_SIZE, POWER_RX_BUF_DEPTH, POWER_MAX_TX_PACKET_SIZE, POWER_TX_BUF_DEPTH>,
    power_rx_uart_queue: &'static UartReadQueue<POWER_MAX_RX_PACKET_SIZE, POWER_RX_BUF_DEPTH>,
    power_tx_uart_queue: &'static UartWriteQueue<POWER_MAX_TX_PACKET_SIZE, POWER_TX_BUF_DEPTH>,
    last_power_status_time: Option<Instant>,
    last_power_status: PowerStatusPacket,
}

impl<
        const POWER_MAX_TX_PACKET_SIZE: usize,
        const POWER_MAX_RX_PACKET_SIZE: usize,
        const POWER_TX_BUF_DEPTH: usize,
        const POWER_RX_BUF_DEPTH: usize> 
    PowerTask<POWER_MAX_TX_PACKET_SIZE, POWER_MAX_RX_PACKET_SIZE, POWER_TX_BUF_DEPTH, POWER_RX_BUF_DEPTH> {

    async fn power_task_entry(&mut self) {
        defmt::info!("power task startup");

        let mut power_loop_rate_ticker = Ticker::every(Duration::from_millis(POWER_LOOP_RATE_MS));

        // allow default fallback state transition of none
        #[allow(unused_assignments)]
        // let mut next_connection_state = self.connection_state;
        loop {
            self.process_packets();
            if self.last_power_status_time.is_some() && self.last_power_status.shutdown_requested() == 1 {
                self.try_shutdown().await;
            }

            let cmd: PowerCommandPacket;
            unsafe {
                cmd = MaybeUninit::zeroed().assume_init();
            }
            // load any items into command
            self.send_command(cmd).await;

            power_loop_rate_ticker.next().await;
        }
    }

    async fn try_shutdown(&mut self) {
        defmt::warn!("shutdown initiated via power board! syncing...");

        self.shared_robot_state.flag_shutdown_requested();

        self.led_cmd_publisher.publish(ControlBoardLedCommand::General(ControlGeneralLedCommand::ShutdownRequested)).await;

    
        // wait for tasks to flag shutdown complete, power board will
        // hard temrinate after hard after 30s shutdown time
        loop {
            let cmd: PowerCommandPacket;
            unsafe {
                cmd = MaybeUninit::zeroed().assume_init();
            }
            // load any items into command
            self.send_command(cmd).await;

            defmt::info!("waiting for kicker board to complete discharge");
            if self.shared_robot_state.kicker_shutdown_complete() || self.shared_robot_state.get_kicker_inop() {
                break;
            }

            Timer::after_millis(100).await;
        }
    
        Timer::after_millis(100).await;

        loop {
            let mut cmd: PowerCommandPacket;
            unsafe {
                cmd = MaybeUninit::zeroed().assume_init();
            }
            cmd.set_ready_shutdown(1);
            defmt::info!("Sending shutdown ready acknowldgement to power board");
            Timer::after_millis(100).await;
            self.send_command(cmd).await;
        }
    }

    fn process_packets(&mut self) {
        // read any packets
        while let Ok(res) = self.power_rx_uart_queue.try_dequeue() {
            defmt::trace!("Received Power Telemetry Packet");
            let buf = res.data();

            if buf.len() != core::mem::size_of::<PowerStatusPacket>() {
                defmt::warn!("Power - Got invalid packet of len {:?} (expected {:?}) data: {:?}", buf.len(), core::mem::size_of::<PowerStatusPacket>(), buf);
                continue;
            }

            unsafe {
                // zero initialize a local response packet
                let mut status_packet: PowerStatusPacket = { MaybeUninit::zeroed().assume_init() };

                // copy receieved uart bytes into status_packet
                let state = &mut status_packet as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<PowerStatusPacket>() {
                    *state.offset(i as isize) = buf[i];
                }

                self.last_power_status = status_packet;
                self.last_power_status_time = Some(Instant::now());
            }
        }
    }

    async fn send_command(&mut self, cmd: PowerCommandPacket) {
        let struct_bytes;
        unsafe {
            struct_bytes = core::slice::from_raw_parts(
                (&cmd as *const PowerCommandPacket) as *const u8,
                core::mem::size_of::<PowerCommandPacket>(),
            );
        }
        self.power_tx_uart_queue.enqueue_copy(struct_bytes);
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

#[embassy_executor::task]
async fn power_task_entry(mut power_task: PowerTask<POWER_MAX_TX_PACKET_SIZE, POWER_MAX_RX_PACKET_SIZE, POWER_TX_BUF_DEPTH, POWER_RX_BUF_DEPTH>) {
    loop {
        power_task.power_task_entry().await;
        defmt::error!("power task returned");
    }
}

pub fn start_power_task(power_task_spawner: Spawner,
        uart_queue_spawner: SendSpawner,
        robot_state: &'static SharedRobotState,
        led_command_publisher: LedCommandPublisher,
        power_uart: PowerUart,
        power_uart_rx_pin: PowerUartRxPin,
        power_uart_tx_pin: PowerUartTxPin,
        power_uart_rx_dma: PowerRxDma,
        power_uart_tx_dma: PowerTxDma,
        ) {


    let uart_config = power_uart_config();
    let power_uart = Uart::new(power_uart, power_uart_rx_pin, power_uart_tx_pin, SystemIrqs, power_uart_tx_dma, power_uart_rx_dma, uart_config).unwrap();

    defmt::trace!("Power UART initialized");

    POWER_IDLE_BUFFERED_UART.init();

    defmt::trace!("Power UART queue init");

    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, POWER, power_uart);

    let power_task = PowerTask {
        shared_robot_state: robot_state,
        led_cmd_publisher: led_command_publisher,
        _power_uart: &POWER_IDLE_BUFFERED_UART,
        power_rx_uart_queue: POWER_IDLE_BUFFERED_UART.get_uart_read_queue(),
        power_tx_uart_queue: POWER_IDLE_BUFFERED_UART.get_uart_write_queue(),
        last_power_status_time: None,
        last_power_status: unsafe { MaybeUninit::zeroed().assume_init() },
    };

    power_task_spawner.spawn(power_task_entry(power_task)).unwrap();
    defmt::info!("power task online");
}