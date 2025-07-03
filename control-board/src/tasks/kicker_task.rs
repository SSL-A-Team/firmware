use ateam_common_packets::radio::DataPacket;
use ateam_lib_stm32::{drivers::boot::stm32_interface, idle_buffered_uart_spawn_tasks, static_idle_buffered_uart, uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue}};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::{gpio::Pin, usart::Uart};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Ticker, Instant};

use crate::{drivers::kicker::Kicker, include_kicker_bin, pins::*, robot_state::SharedRobotState, SystemIrqs};

include_kicker_bin! {KICKER_FW_IMG, "kicker.bin"}

const MAX_TX_PACKET_SIZE: usize = 20;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 20;
const RX_BUF_DEPTH: usize = 20;
const TELEMETRY_TIMEOUT_MS: u64 = 2000;

static_idle_buffered_uart!(KICKER, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);

#[macro_export]
macro_rules! create_kicker_task {
    ($main_spawner:ident, $uart_queue_spawner:ident, $robot_state:ident,
        $command_subscriber:ident, $p:ident) => {
        ateam_control_board::tasks::kicker_task::start_kicker_task(
            $main_spawner, $uart_queue_spawner,
            $robot_state,
            $command_subscriber,
            $p.UART8, $p.PE0, $p.PE1,
            $p.DMA2_CH2, $p.DMA2_CH3,
            $p.PG2, $p.PG3).await;
    };
}

#[derive(PartialEq, PartialOrd, Debug)]
enum KickerTaskState {
    PoweredOff,
    PowerOn,
    ConnectUart,
    Connected,
    InitiateShutdown,
    WaitShutdown,
}

pub struct KickerTask<'a,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_RX: usize,
    const DEPTH_TX: usize> {
    kicker_driver: Kicker<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>,
    kicker_task_state: KickerTaskState,
    robot_state: &'static SharedRobotState,
    commands_subscriber: CommandsSubscriber,
}

impl<'a,
const LEN_RX: usize,
const LEN_TX: usize,
const DEPTH_RX: usize,
const DEPTH_TX: usize> KickerTask<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX> {
    pub fn new(kicker_driver: Kicker<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>,
            robot_state: &'static SharedRobotState,
            command_subscriber: CommandsSubscriber) -> Self {
        KickerTask {
            kicker_driver: kicker_driver,
            kicker_task_state: KickerTaskState::PoweredOff,
            robot_state: robot_state,
            commands_subscriber: command_subscriber,
        }
    }

    pub fn new_from_pins(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX>,
        boot0_pin: impl Pin,
        reset_pin: impl Pin,
        firmware_image: &'a [u8],
        robot_state: &'static SharedRobotState,
        command_subscriber: CommandsSubscriber) -> Self {

        let kicker_driver = Kicker::new_from_pins(uart, read_queue, write_queue, boot0_pin, reset_pin, firmware_image);

        Self::new(kicker_driver, robot_state, command_subscriber)
    }

    pub async fn kicker_task_entry(&mut self) {
        let mut main_loop_ticker = Ticker::every(Duration::from_hz(100));
        // Connection timeout start will be reset when connection is established and when a telemetry packet is received
        let mut connection_timeout_start = Instant::now();
        loop {
            let cur_robot_state = self.robot_state.get_state();

            if self.kicker_driver.process_telemetry() {
                // DEBUG REMOVE //
                defmt::trace!("Kicker Interface - Resetting connection timeout");
                //////////////////
                connection_timeout_start = Instant::now();
            }

            let cur_time = Instant::now();
            if self.kicker_task_state == KickerTaskState::Connected && Instant::checked_duration_since(&cur_time, connection_timeout_start).unwrap().as_millis() > TELEMETRY_TIMEOUT_MS {
                // Check if telemetry has been received in this timeout period
                defmt::error!("Kicker Interface - Kicker telemetry timed out, resetting kicker!");
                self.kicker_driver.reset().await;
            }

            // TODO global state overrides of kicker state
            // e.g. external shutdown requsts, battery votlage, etc
            if self.kicker_task_state == KickerTaskState::Connected && cur_robot_state.shutdown_requested {
                self.kicker_task_state = KickerTaskState::InitiateShutdown;
            }

            match self.kicker_task_state {
                KickerTaskState::PoweredOff => {
                    defmt::trace!("Kicker Interface - Kicker is in power off state");
                    if cur_robot_state.hw_init_state_valid && !cur_robot_state.shutdown_requested {
                        self.kicker_task_state = KickerTaskState::PowerOn;
                    }
                },
                KickerTaskState::PowerOn => {
                    // lets power settle on kicker
                    defmt::trace!("Kicker Interface - Reset kicker");

                    self.kicker_driver.enter_reset().await;
                    self.kicker_driver.leave_reset().await;
                    // power should be coming on, attempt connection
                    self.kicker_task_state = KickerTaskState::ConnectUart;
                },
                KickerTaskState::ConnectUart => {
                    let force_flash = cur_robot_state.hw_debug_mode;
                    if self.kicker_driver.init_default_firmware_image(force_flash).await.is_err() {
                        // attempt to power on the board again
                        // if the kicker is missing or bugged we'll stay in a power on -> attempt
                        // uart loop forever
                        self.kicker_task_state = KickerTaskState::PowerOn;

                        defmt::error!("Kicker Interface - Kicker firmware load failed, try power cycle");
                    } else {
                        self.kicker_task_state = KickerTaskState::Connected;
                        connection_timeout_start = Instant::now();
                        main_loop_ticker.reset();

                        defmt::info!("Kicker Interface - Kicker connection loop start");
                    }
                },
                KickerTaskState::Connected => {
                    self.connected_poll_loop().await;
                    // external events will move us out of this state
                },
                KickerTaskState::InitiateShutdown => {
                    defmt::trace!("Kicker Interface - Requesting shutdown.");
                    self.kicker_driver.request_shutdown();

                    // wait for kicker to ack shutdown
                    if self.kicker_driver.shutdown_acknowledge() {
                        defmt::info!("Kicker Interface - Kicker acknowledged shutdown request");
                        self.kicker_task_state = KickerTaskState::WaitShutdown;
                    }
                },
                KickerTaskState::WaitShutdown => {
                    if self.kicker_driver.shutdown_completed() {
                        defmt::info!("Kicker Interface - Kicker finished shutdown");
                        self.kicker_task_state = KickerTaskState::PoweredOff;
                        self.robot_state.set_kicker_shutdown_complete(true);
                    }
                },
            }

            // if we are in any substate of connected, then send
            // commands to the kicker
            if self.kicker_task_state >= KickerTaskState::Connected {
                self.kicker_driver.send_command();

                let ball_detected = self.kicker_driver.ball_detected();
                if ball_detected {
                    defmt::info!("Kicker Interface - Ball detected!");
                }

                self.robot_state.set_ball_detected(ball_detected);
            }

            self.robot_state.set_kicker_inop(self.kicker_task_state < KickerTaskState::Connected);

            main_loop_ticker.next().await;
        }
    }

    async fn connected_poll_loop(&mut self) {
        self.kicker_driver.set_telemetry_enabled(true);

        if let Some(pkt) = self.commands_subscriber.try_next_message() {
            match pkt {
                WaitResult::Lagged(amnt) => {
                    if amnt > 3 {
                        defmt::warn!("Kicker Interface - Kicker task lagged processing commands by {} msgs", amnt);
                    }
                },
                WaitResult::Message(cmd) => {
                    match cmd {
                        DataPacket::BasicControl(bc_pkt) => {
                            self.kicker_driver.set_kick_strength(bc_pkt.kick_vel);
                            self.kicker_driver.request_kick(bc_pkt.kick_request);
                            self.kicker_driver.set_drib_vel(bc_pkt.dribbler_speed);
                        },
                        DataPacket::ParameterCommand(_) => {
                            // we currently don't have any kicker parameters
                        },
                    }
                },
            }
        }
    }
}

#[embassy_executor::task]
async fn kicker_task_entry(mut kicker_task: KickerTask<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>) {
    loop {
        kicker_task.kicker_task_entry().await;
        defmt::error!("Kicker Interface - Kicker task returned");
    }
}

pub async fn start_kicker_task(kicker_task_spawner: Spawner,
    queue_spawner: SendSpawner,
    robot_state: &'static SharedRobotState,
    command_subscriber: CommandsSubscriber,
    kicker_uart: KickerUart,
    kicker_uart_rx_pin: KickerUartRxPin,
    kicker_uart_tx_pin: KickerUartTxPin,
    kicker_uart_rx_dma: KickerRxDma,
    kicker_uart_tx_dma: KickerTxDma,
    kicker_boot0_pin: KickerBootPin,
    kicker_reset_pin: KickerResetPin) {

    let initial_kicker_uart_conifg = stm32_interface::get_bootloader_uart_config();

    let kicker_uart = Uart::new(kicker_uart, kicker_uart_rx_pin, kicker_uart_tx_pin, SystemIrqs, kicker_uart_tx_dma, kicker_uart_rx_dma, initial_kicker_uart_conifg).unwrap();

    KICKER_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(queue_spawner, KICKER, kicker_uart);

    let kicker_task = KickerTask::new_from_pins(&KICKER_IDLE_BUFFERED_UART, KICKER_IDLE_BUFFERED_UART.get_uart_read_queue(), KICKER_IDLE_BUFFERED_UART.get_uart_write_queue(), kicker_boot0_pin, kicker_reset_pin, KICKER_FW_IMG, robot_state, command_subscriber);
    kicker_task_spawner.spawn(kicker_task_entry(kicker_task)).unwrap();
}