use ateam_common_packets::{bindings::KickRequest, radio::DataPacket};
use ateam_lib_stm32::{
    drivers::boot::stm32_interface,
    idle_buffered_uart_spawn_tasks, static_idle_buffered_uart,
    uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue},
};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::{gpio::Pin, usart::Uart};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Ticker};

use crate::{
    drivers::kicker::Kicker, include_kicker_bin, pins::*, robot_state::SharedRobotState,
    SystemIrqs, DEBUG_KICKER_UART_QUEUES,
};

include_kicker_bin! {KICKER_FW_IMG, "kicker.bin"}

const MAX_TX_PACKET_SIZE: usize = 32;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 20;
const TELEMETRY_TIMEOUT_MS: u64 = 2000;

static_idle_buffered_uart!(KICKER, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, DEBUG_KICKER_UART_QUEUES, #[link_section = ".axisram.buffers"]);

#[macro_export]
macro_rules! create_kicker_task {
    ($main_spawner:ident, $uart_queue_spawner:ident, $robot_state:ident,
        $command_subscriber:ident, $kicker_telemetry_publisher:ident, $p:ident) => {
        ateam_control_board::tasks::kicker_task::start_kicker_task(
            $main_spawner,
            $uart_queue_spawner,
            $robot_state,
            $command_subscriber,
            $kicker_telemetry_publisher,
            $p.UART8,
            $p.PE0,
            $p.PE1,
            $p.DMA2_CH2,
            $p.DMA2_CH3,
            $p.PG2,
            $p.PG3,
        )
        .await;
    };
}

#[derive(PartialEq, PartialOrd, Debug, defmt::Format)]
enum KickerTaskState {
    PoweredOff,
    PowerOn,
    InitFirmware,
    Reset,
    ConnectUart,
    Connected,
    InitiateShutdown,
    WaitShutdown,
}

pub struct KickerTask<
    'a,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_RX: usize,
    const DEPTH_TX: usize,
> {
    kicker_driver: Kicker<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>,
    kicker_task_state: KickerTaskState,
    robot_state: &'static SharedRobotState,
    commands_subscriber: CommandsSubscriber,
    last_command_received_time: Option<Instant>,
    kicker_telemetry_publisher: KickerTelemetryPublisher,
}

impl<
        'a,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize,
        const DEPTH_TX: usize,
    > KickerTask<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
{
    pub fn new(
        kicker_driver: Kicker<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>,
        robot_state: &'static SharedRobotState,
        command_subscriber: CommandsSubscriber,
        kicker_telemetry_publisher: KickerTelemetryPublisher,
    ) -> Self {
        KickerTask {
            kicker_driver: kicker_driver,
            kicker_task_state: KickerTaskState::PoweredOff,
            robot_state: robot_state,
            commands_subscriber: command_subscriber,
            last_command_received_time: None,
            kicker_telemetry_publisher,
        }
    }

    pub fn new_from_pins(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_KICKER_UART_QUEUES>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_KICKER_UART_QUEUES>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_KICKER_UART_QUEUES>,
        boot0_pin: impl Pin,
        reset_pin: impl Pin,
        firmware_image: &'a [u8],
        robot_state: &'static SharedRobotState,
        command_subscriber: CommandsSubscriber,
        kicker_telemetry_publisher: KickerTelemetryPublisher,
    ) -> Self {
        let kicker_driver = Kicker::new_from_pins(
            uart,
            read_queue,
            write_queue,
            boot0_pin,
            reset_pin,
            firmware_image,
        );

        Self::new(
            kicker_driver,
            robot_state,
            command_subscriber,
            kicker_telemetry_publisher,
        )
    }

    pub async fn kicker_task_entry(&mut self) {
        let mut main_loop_ticker = Ticker::every(Duration::from_hz(100));
        // Connection timeout start will be reset when entering ConnectUart state and when a telemetry packet is received
        let mut connection_timeout_start = Instant::now(); // This initial value shouldn't ever be used to calculate a timeout
        loop {
            let cur_robot_state = self.robot_state.get_state();

            // telemetry_received is also used to enter the Connected state
            let telemetry_received = self.kicker_driver.process_telemetry();
            if telemetry_received {
                // Publish telemetry
                self.kicker_telemetry_publisher
                    .publish_immediate(self.kicker_driver.get_lastest_state());
                // Reset the connection timeout period
                connection_timeout_start = Instant::now();
            } else if self.kicker_task_state >= KickerTaskState::ConnectUart {
                // Check if connection timeout occurred
                let time_elapsed =
                    Instant::checked_duration_since(&Instant::now(), connection_timeout_start)
                        .unwrap()
                        .as_millis();
                if time_elapsed > TELEMETRY_TIMEOUT_MS {
                    defmt::error!("Kicker Interface - Kicker telemetry timed out, current state is '{}', rolling state back to 'Reset'", self.kicker_task_state);
                    self.kicker_task_state = KickerTaskState::Reset;
                }
            }

            // TODO global state overrides of kicker state
            // e.g. external shutdown requsts, battery votlage, etc
            if cur_robot_state.shutdown_requested {
                // Attempt graceful shutdown with kicker accepting commands
                if self.kicker_task_state == KickerTaskState::Connected {
                    self.kicker_task_state = KickerTaskState::InitiateShutdown;
                }
                // Kicker connection isn't established, manually put kicker in powered off state
                if self.kicker_task_state <= KickerTaskState::Connected {
                    self.kicker_task_state = KickerTaskState::PoweredOff;
                }
            }

            match self.kicker_task_state {
                KickerTaskState::PoweredOff => {
                    if cur_robot_state.hw_init_state_valid && !cur_robot_state.shutdown_requested {
                        self.kicker_task_state = KickerTaskState::PowerOn;
                    }
                    // Should we hold the kicker in reset here?
                    // else
                    // {
                    //     // Hold kicker in reset until control board is ready
                    //     self.kicker_driver.enter_reset().await;
                    // }
                }
                KickerTaskState::PowerOn => {
                    // lets power settle on kicker
                    defmt::trace!(
                        "Kicker Interface - State '{}' - Resetting kicker",
                        self.kicker_task_state
                    );
                    self.kicker_driver.reset().await;
                    self.kicker_task_state = KickerTaskState::InitFirmware;
                }
                KickerTaskState::InitFirmware => {
                    // Ensure firmware image is up to date
                    let force_flash = cur_robot_state.hw_debug_mode;
                    // This await can be over 100 ms to check if kicker responds with current image hash
                    let res = self
                        .kicker_driver
                        .init_default_firmware_image(force_flash)
                        .await;
                    if res.is_ok() {
                        // Firmware is up to date, move on to Reset state
                        self.kicker_task_state = KickerTaskState::Reset;
                    } else {
                        defmt::error!("Kicker Interface - State '{}' - Firmware load failed, rolling state back to PowerOn", self.kicker_task_state);
                        self.kicker_task_state = KickerTaskState::PowerOn;
                        // if the kicker is missing or bugged we'll stay in a power on -> attempt
                        // flash forever

                        // IS RATE LIMIT NEEDED HERE? Maybe not because lots of timer awaits in
                        // init_default_firmware_image anyways.
                        //
                        // Rate limit the retry loop
                        // Timer::after_millis(10).await;
                    }
                }
                KickerTaskState::Reset => {
                    defmt::trace!(
                        "Kicker Interface - State '{}' - Resetting kicker",
                        self.kicker_task_state
                    );
                    self.kicker_driver.reset().await;
                    // Reset the connection timeout period
                    connection_timeout_start = Instant::now();
                    // Move on to ConnectUart state
                    self.kicker_task_state = KickerTaskState::ConnectUart;
                }
                KickerTaskState::ConnectUart => {
                    if telemetry_received {
                        defmt::info!(
                            "Kicker Interface - State '{}' - Kicker UART connection established",
                            self.kicker_task_state
                        );
                        // Move on to Connected state
                        self.kicker_task_state = KickerTaskState::Connected;
                    }
                }
                KickerTaskState::Connected => {
                    self.connected_poll_loop().await;
                    // external events will move us out of this state
                }
                KickerTaskState::InitiateShutdown => {
                    defmt::trace!(
                        "Kicker Interface - State '{}' - Requesting shutdown",
                        self.kicker_task_state
                    );
                    self.kicker_driver.request_shutdown();

                    // wait for kicker to ack shutdown
                    if self.kicker_driver.shutdown_acknowledge() {
                        defmt::info!(
                            "Kicker Interface - State '{}' - Kicker acknowledged shutdown request",
                            self.kicker_task_state
                        );
                        self.kicker_task_state = KickerTaskState::WaitShutdown;
                    }
                }
                KickerTaskState::WaitShutdown => {
                    if self.kicker_driver.shutdown_completed() {
                        defmt::info!(
                            "Kicker Interface - State '{}' - Kicker finished shutdown",
                            self.kicker_task_state
                        );
                        self.kicker_task_state = KickerTaskState::PoweredOff;
                        self.robot_state.set_kicker_shutdown_complete(true);
                    }
                }
            }

            // kicker and radio loop rates are both 100Hz, so 10ms packet interval
            // if we miss 10 in a row, something has gone quite wrong
            // override commands to safe ones
            if Instant::now() - self.last_command_received_time.unwrap_or(Instant::now())
                > Duration::from_millis(500)
            {
                // Avoid spamming logs while the system starts up
                defmt::error!("Kicker Interface - Kicker task has stopped receiving commands from the radio task and will de-arm the kicker board");
                self.kicker_driver.set_kick_strength(0.0);
                self.kicker_driver.request_kick(KickRequest::KR_DISABLE);
                self.kicker_driver.set_drib_vel(0.0);
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

            self.robot_state
                .set_kicker_inop(self.kicker_task_state < KickerTaskState::Connected);

            main_loop_ticker.next().await;
        }
    }

    async fn connected_poll_loop(&mut self) {
        self.kicker_driver.set_telemetry_enabled(true);

        if let Some(pkt) = self.commands_subscriber.try_next_message() {
            match pkt {
                WaitResult::Lagged(amnt) => {
                    if amnt > 3 {
                        defmt::warn!(
                            "Kicker Interface - Kicker task lagged processing commands by {} msgs",
                            amnt
                        );
                    }
                }
                WaitResult::Message(cmd) => {
                    match cmd {
                        DataPacket::BasicControl(bc_pkt) => {
                            self.last_command_received_time = Some(Instant::now());

                            self.kicker_driver.set_kick_strength(bc_pkt.kick_vel);
                            self.kicker_driver.request_kick(bc_pkt.kick_request);
                            self.kicker_driver.set_drib_vel(bc_pkt.dribbler_speed);
                            self.kicker_driver
                                .set_drib_multiplier(bc_pkt.dribbler_multiplier());
                        }
                        DataPacket::ParameterCommand(_) => {
                            // we currently don't have any kicker parameters
                        }
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn kicker_task_entry(
    mut kicker_task: KickerTask<
        'static,
        MAX_RX_PACKET_SIZE,
        MAX_TX_PACKET_SIZE,
        RX_BUF_DEPTH,
        TX_BUF_DEPTH,
    >,
) {
    loop {
        kicker_task.kicker_task_entry().await;
        defmt::error!("Kicker Interface - Kicker task returned");
    }
}

pub async fn start_kicker_task(
    kicker_task_spawner: Spawner,
    queue_spawner: SendSpawner,
    robot_state: &'static SharedRobotState,
    command_subscriber: CommandsSubscriber,
    kicker_telemetry_publisher: KickerTelemetryPublisher,
    kicker_uart: KickerUart,
    kicker_uart_rx_pin: KickerUartRxPin,
    kicker_uart_tx_pin: KickerUartTxPin,
    kicker_uart_rx_dma: KickerRxDma,
    kicker_uart_tx_dma: KickerTxDma,
    kicker_boot0_pin: KickerBootPin,
    kicker_reset_pin: KickerResetPin,
) {
    let initial_kicker_uart_conifg = stm32_interface::get_bootloader_uart_config();

    let kicker_uart = Uart::new(
        kicker_uart,
        kicker_uart_rx_pin,
        kicker_uart_tx_pin,
        SystemIrqs,
        kicker_uart_tx_dma,
        kicker_uart_rx_dma,
        initial_kicker_uart_conifg,
    )
    .unwrap();

    KICKER_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(queue_spawner, KICKER, kicker_uart);

    let kicker_task = KickerTask::new_from_pins(
        &KICKER_IDLE_BUFFERED_UART,
        KICKER_IDLE_BUFFERED_UART.get_uart_read_queue(),
        KICKER_IDLE_BUFFERED_UART.get_uart_write_queue(),
        kicker_boot0_pin,
        kicker_reset_pin,
        KICKER_FW_IMG,
        robot_state,
        command_subscriber,
        kicker_telemetry_publisher,
    );
    kicker_task_spawner
        .spawn(kicker_task_entry(kicker_task))
        .unwrap();
}
