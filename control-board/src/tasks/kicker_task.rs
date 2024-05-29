use ateam_common_packets::radio::DataPacket;
use ateam_lib_stm32::{make_uart_queue_pair, queue_pair_register_and_spawn, uart::queue::{UartReadQueue, UartWriteQueue}};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::{gpio::{Level, Output, Pin, Speed}, usart::{self, Uart}};
use embassy_sync::pubsub::WaitResult;
use embassy_time::Timer;

use crate::{drivers::kicker::Kicker, include_kicker_bin, pins::*, robot_state::SharedRobotState, stm32_interface, SystemIrqs};

include_kicker_bin! {KICKER_FW_IMG, "hwtest-blinky.bin"}

const MAX_TX_PACKET_SIZE: usize = 16;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 16;
const RX_BUF_DEPTH: usize = 20;

make_uart_queue_pair!(KICKER,
    KickerUart, KickerRxDma, KickerTxDma,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

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
    UART: usart::BasicInstance,
    DmaRx: usart::RxDma<UART>,
    DmaTx: usart::TxDma<UART>,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_RX: usize,
    const DEPTH_TX: usize> {
    kicker_driver: Kicker<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>,
    remote_power_btn: Output<'a>,
    kicker_task_state: KickerTaskState,
    robot_state: &'static SharedRobotState,
    commands_subscriber: CommandsSubscriber,
}

impl<'a,
UART: usart::BasicInstance,
DmaRx: usart::RxDma<UART>,
DmaTx: usart::TxDma<UART>,
const LEN_RX: usize,
const LEN_TX: usize,
const DEPTH_RX: usize,
const DEPTH_TX: usize> KickerTask<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX> {
    pub fn new(kicker_driver: Kicker<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>,
            power_output: Output<'a>,
            robot_state: &'static SharedRobotState,
            command_subscriber: CommandsSubscriber) -> Self {
        KickerTask {
            kicker_driver: kicker_driver,
            remote_power_btn: power_output,
            kicker_task_state: KickerTaskState::PoweredOff,
            robot_state: robot_state,
            commands_subscriber: command_subscriber,
        }
    }

    pub fn new_from_pins(read_queue: &'a UartReadQueue<UART, DmaRx, LEN_RX, DEPTH_RX>,
        write_queue: &'a UartWriteQueue<UART, DmaTx, LEN_TX, DEPTH_TX>,
        boot0_pin: impl Pin,
        reset_pin: impl Pin,
        power_pin: impl Pin,
        firmware_image: &'a [u8],
        robot_state: &'static SharedRobotState,
        command_subscriber: CommandsSubscriber) -> Self {

        let kicker_driver = Kicker::new_from_pins(read_queue, write_queue, boot0_pin, reset_pin, firmware_image);
        let power_output = Output::new(power_pin, Level::Low, Speed::Medium);

        Self::new(kicker_driver, power_output, robot_state, command_subscriber)
    }

    pub async fn kicker_task_entry(&mut self) {
        loop {
            let cur_robot_state = self.robot_state.get_state();

            self.kicker_driver.process_telemetry();

            // TODO global state overrides of kicker state
            // e.g. external shutdown requsts, battery votlage, etc
            if self.kicker_task_state == KickerTaskState::Connected && cur_robot_state.shutdown_requested {
                self.kicker_task_state = KickerTaskState::InitiateShutdown;
            }

            match self.kicker_task_state {
                KickerTaskState::PoweredOff => {
                    if cur_robot_state.hw_init_state_valid && cur_robot_state.shutdown_requested {
                        self.kicker_task_state = KickerTaskState::PowerOn;
                    }
                },
                KickerTaskState::PowerOn => {
                    self.remote_power_btn_press().await;
                    // power should be coming on, attempt connection
                    self.kicker_task_state = KickerTaskState::ConnectUart;
                },
                KickerTaskState::ConnectUart => {
                    if self.kicker_driver.load_default_firmware_image().await.is_err() {
                        // attempt to power on the board again
                        // if the kicker is missing or bugged we'll stay in a power on -> attempt
                        // uart loop forever
                        self.kicker_task_state = KickerTaskState::PowerOn;
                    } else {
                        self.kicker_task_state = KickerTaskState::Connected;
                    }
                },
                KickerTaskState::Connected => {
                    self.connected_poll_loop().await;
                    // external events will move us out of this state
                },
                KickerTaskState::InitiateShutdown => {
                    self.kicker_driver.request_shutdown();
                    self.kicker_task_state = KickerTaskState::WaitShutdown;
                },
                KickerTaskState::WaitShutdown => {
                    if self.kicker_driver.shutdown_completed() {
                        self.kicker_task_state = KickerTaskState::PoweredOff;
                    }
                },
            }

            // if we are in any substate of connected, then send
            // commands to the kicker
            if self.kicker_task_state >= KickerTaskState::Connected {
                self.kicker_driver.send_command();
            }
        }
    }

    pub async fn remote_power_btn_press(&mut self) {
        self.remote_power_btn.set_high();
        Timer::after_millis(200).await;
        self.remote_power_btn.set_low();
        Timer::after_millis(10).await;
    }

    pub async fn remote_power_btn_hold(&mut self) {
        self.remote_power_btn.set_high();
        Timer::after_millis(1500).await;
        self.remote_power_btn.set_low();
        Timer::after_millis(10).await;
    }

    pub async fn connected_poll_loop(&mut self) {
        if let Some(pkt) = self.commands_subscriber.try_next_message() {
            match pkt {
                WaitResult::Lagged(amnt) => {
                    defmt::warn!("kicker task lagged processing commands by {} msgs", amnt);
                },
                WaitResult::Message(cmd) => {
                    match cmd {
                        DataPacket::BasicControl(bc_pkt) => {
                            self.kicker_driver.set_kick_strength(bc_pkt.kick_vel);
                            self.kicker_driver.request_kick(bc_pkt.kick_request);
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
async fn kicker_task_entry(mut kicker_task: KickerTask<'static, KickerUart, KickerRxDma, KickerTxDma, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>) {
    loop {
        kicker_task.kicker_task_entry().await;
        defmt::error!("radio task returned");
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
    kicker_reset_pin: KickerResetPin,
    kicker_power_pin: KickerPowerOnPin) {

    let initial_kicker_uart_conifg = stm32_interface::get_bootloader_uart_config();

    let kicker_uart = Uart::new(kicker_uart, kicker_uart_rx_pin, kicker_uart_tx_pin, SystemIrqs, kicker_uart_tx_dma, kicker_uart_rx_dma, initial_kicker_uart_conifg).unwrap();

    let (kicker_tx, kicker_rx) = Uart::split(kicker_uart);
    queue_pair_register_and_spawn!(queue_spawner, KICKER, kicker_rx, kicker_tx);

    let kicker_task = KickerTask::new_from_pins(&KICKER_RX_UART_QUEUE, &KICKER_TX_UART_QUEUE, kicker_boot0_pin, kicker_reset_pin, kicker_power_pin, KICKER_FW_IMG, robot_state, command_subscriber);
    kicker_task_spawner.spawn(kicker_task_entry(kicker_task)).unwrap();
}