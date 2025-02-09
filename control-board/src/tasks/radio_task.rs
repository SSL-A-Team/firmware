
use ateam_common_packets::{bindings::BasicTelemetry, radio::TelemetryPacket};
use ateam_lib_stm32::{idle_buffered_uart_read_task, idle_buffered_uart_write_task, static_idle_buffered_uart, uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue}};
use credentials::WifiCredential;
use embassy_executor::{SendSpawner, Spawner};
use embassy_futures::select::{select, Either};
use embassy_stm32::{
    gpio::{Input, Pin, Pull},
    usart::{self, DataBits, Parity, StopBits, Uart}
};
use embassy_time::{Duration, Instant, Ticker, Timer};

use crate::{drivers::radio_robot::{RobotRadio, TeamColor}, pins::*, robot_state::SharedRobotState, SystemIrqs};

#[macro_export]
macro_rules! create_radio_task {
    ($main_spawner:ident, $rx_uart_queue_spawner:ident, $tx_uart_queue_spawner:ident, $robot_state:ident,
        $radio_command_publisher:ident, $radio_telemetry_subscriber:ident,
        $wifi_credentials:ident, $p:ident) => {
        ateam_control_board::tasks::radio_task::start_radio_task(
            $main_spawner, $rx_uart_queue_spawner, $tx_uart_queue_spawner, 
            $robot_state,
            $radio_command_publisher, $radio_telemetry_subscriber,
            &$wifi_credentials,
            $p.USART10, $p.PE2, $p.PE3, $p.PG13, $p.PG14,
            $p.DMA2_CH1, $p.DMA2_CH0,
            $p.PC13, $p.PE4).await; 
    };
}

pub const RADIO_LOOP_RATE_MS: u64 = 10;

pub const RADIO_MAX_TX_PACKET_SIZE: usize = 320;
pub const RADIO_TX_BUF_DEPTH: usize = 4;
pub const RADIO_MAX_RX_PACKET_SIZE: usize = 256;
pub const RADIO_RX_BUF_DEPTH: usize = 4;

static_idle_buffered_uart!(RADIO, RADIO_MAX_RX_PACKET_SIZE, RADIO_RX_BUF_DEPTH, RADIO_MAX_TX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);

#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
enum RadioConnectionState {
    Unconnected,
    ConnectedPhys,
    ConnectedUart,
    ConnectedNetwork,
    Connected
}

pub struct RadioTask<
        const RADIO_MAX_TX_PACKET_SIZE: usize,
        const RADIO_MAX_RX_PACKET_SIZE: usize,
        const RADIO_TX_BUF_DEPTH: usize,
        const RADIO_RX_BUF_DEPTH: usize> {
    shared_robot_state: &'static SharedRobotState,
    command_publisher: CommandsPublisher,
    telemetry_subscriber: TelemetrySubcriber,
    radio: RobotRadio<'static, RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH>,
    radio_ndet_input: Input<'static>,

    connection_state: RadioConnectionState,
    wifi_credentials: &'static [WifiCredential],

    last_software_packet: Instant,
    last_basic_telemetry: BasicTelemetry,
}

impl<
        const RADIO_MAX_TX_PACKET_SIZE: usize,
        const RADIO_MAX_RX_PACKET_SIZE: usize,
        const RADIO_TX_BUF_DEPTH: usize,
        const RADIO_RX_BUF_DEPTH: usize> 
    RadioTask<RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH> {
    // pub type TaskRobotRadio = RobotRadio<'static, RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH>;

    const RETRY_DELAY_MS: u64 = 1000;
    const RESPONSE_FROM_PC_TIMEOUT_MS: u64 = 1000;
    const UART_CONNECT_TIMEOUT_MS: u64 = 5000;

    pub fn new(robot_state: &'static SharedRobotState,
            command_publisher: CommandsPublisher,
            telemetry_subscriber: TelemetrySubcriber,
            radio: RobotRadio<'static, RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH>,
            radio_ndet_input: Input<'static>,
            wifi_credentials: &'static [WifiCredential]) -> Self {
        RadioTask {
            shared_robot_state: robot_state,
            command_publisher: command_publisher,
            telemetry_subscriber: telemetry_subscriber,
            radio: radio,
            radio_ndet_input: radio_ndet_input,
            connection_state: RadioConnectionState::Unconnected,
            wifi_credentials: wifi_credentials,
            last_software_packet: Instant::now(),
            last_basic_telemetry: BasicTelemetry {
                sequence_number: 0,
                robot_revision_major: 0,
                robot_revision_minor: 0,
                battery_level: 0.,
                battery_temperature: 0.,
                _bitfield_align_1: [],
                _bitfield_1: BasicTelemetry::new_bitfield_1(
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ),
                motor_0_temperature: 0.,
                motor_1_temperature: 0.,
                motor_2_temperature: 0.,
                motor_3_temperature: 0.,
                motor_4_temperature: 0.,
                kicker_charge_level: 0.,
            },
        }
    }

    pub fn new_from_pins(robot_state: &'static SharedRobotState,
            command_publisher: CommandsPublisher,
            telemetry_subscriber: TelemetrySubcriber,
            radio_uart: &'static IdleBufferedUart<RADIO_MAX_RX_PACKET_SIZE, RADIO_RX_BUF_DEPTH, RADIO_MAX_TX_PACKET_SIZE, RADIO_TX_BUF_DEPTH>,
            radio_rx_uart_queue: &'static UartReadQueue<RADIO_MAX_RX_PACKET_SIZE, RADIO_RX_BUF_DEPTH>,
            radio_tx_uart_queue: &'static UartWriteQueue<RADIO_MAX_TX_PACKET_SIZE, RADIO_TX_BUF_DEPTH>,
            radio_reset_pin: impl Pin,
            radio_ndet_pin: impl Pin,
            wifi_credentials: &'static [WifiCredential]) -> Self {

        let radio = RobotRadio::new(radio_uart, radio_rx_uart_queue, radio_tx_uart_queue, radio_reset_pin);

        let radio_ndet = Input::new(radio_ndet_pin, Pull::None);

        Self::new(robot_state, command_publisher, telemetry_subscriber, radio, radio_ndet, wifi_credentials)
    }

    async fn radio_task_entry(&mut self) {
        defmt::info!("radio task startup");

        let mut radio_loop_rate_ticker = Ticker::every(Duration::from_millis(RADIO_LOOP_RATE_MS));

        // initialize a copy of the robot state so we can track updates
        let mut last_robot_state = self.shared_robot_state.get_state();

        // allow default fallback state transition of none
        #[allow(unused_assignments)]
        // let mut next_connection_state = self.connection_state;
        loop {
            // Keep a local flag of radio issues.
            let mut radio_inop_flag_local = false;
            let mut radio_network_fail_local = false;
            // check for hardware config changes that affect radio connection

            let cur_robot_state = self.shared_robot_state.get_state();
            if cur_robot_state != last_robot_state {
                // we are connected to software and robot id or team was changed on hardware
                if self.connection_state == RadioConnectionState::Connected &&
                        (cur_robot_state.hw_robot_id != last_robot_state.hw_robot_id || 
                        cur_robot_state.hw_robot_team_is_blue != last_robot_state.hw_robot_team_is_blue) {
                    // Enter connected network state (disconnecting)
                    self.connection_state = RadioConnectionState::ConnectedNetwork;
                    defmt::info!("shell state change triggering software reconnect");
                }

                // We are at least connected on UART and the wifi network id changed
                if self.connection_state >= RadioConnectionState::ConnectedUart && 
                    cur_robot_state.hw_wifi_network_index != last_robot_state.hw_wifi_network_index {
                    defmt::info!("dip state change triggering wifi network change");

                    if self.radio.disconnect_network().await.is_err() {
                        // this is really only an error if we think we're connected
                        // this separation is really poorly handled right now
                        // TODO move all statefulness down into driver
                        defmt::error!("failed to cleanly disconnect - consider radio reboot");
                    }

                    // Go back to reset the full network flow.
                    self.connection_state = RadioConnectionState::ConnectedUart;
                }
            }

            // Check for missing radio. Highest priority clear of state.
            if self.radio_ndet_input.is_high() {
                defmt::error!("radio appears unplugged.");
                radio_inop_flag_local = true;
                self.connection_state = RadioConnectionState::Unconnected;
            }

            // execute on the connection state

            match self.connection_state {
                RadioConnectionState::Unconnected => {
                    // Radio detect pin says unplugged from above.
                    if radio_inop_flag_local {
                        self.connection_state = RadioConnectionState::Unconnected;
                    } else {
                        // Pin is detected, so connected physically.
                        self.connection_state = RadioConnectionState::ConnectedPhys;
                    }
                },
                RadioConnectionState::ConnectedPhys => {
                    if self.connect_uart().await.is_err() {
                        radio_inop_flag_local = true;
                        // If the pin is unconnected, will be overridden out of the state. 
                        // So just check UART again.
                        self.connection_state = RadioConnectionState::ConnectedPhys;
                    } else {
                        // UART is not in error, so good to go.
                        self.connection_state = RadioConnectionState::ConnectedUart;
                    }
                },
                RadioConnectionState::ConnectedUart => {
                    let wifi_network_ind = cur_robot_state.hw_wifi_network_index as usize;
                    let wifi_credential = if wifi_network_ind >= self.wifi_credentials.len() {
                        self.wifi_credentials[self.wifi_credentials.len() - 1]
                    } else {
                        self.wifi_credentials[wifi_network_ind]
                    };

                    defmt::debug!("connecting to network ({}): ssid: {}, password: {}", wifi_network_ind, wifi_credential.get_ssid(), wifi_credential.get_password());
                    if self.connect_network(wifi_credential, cur_robot_state.hw_robot_id).await.is_err() {
                        // If network connection failed, go back up to verify UART.
                        radio_network_fail_local = true;
                        self.connection_state = RadioConnectionState::ConnectedPhys;
                    } else {
                        self.connection_state = RadioConnectionState::ConnectedNetwork;
                    }
                },
                RadioConnectionState::ConnectedNetwork => {
                    if let Ok(connected) = self.connect_software(cur_robot_state.hw_robot_id, cur_robot_state.hw_robot_team_is_blue).await {
                        if connected {
                            // Refresh last software packet on first connect.
                            self.last_software_packet = Instant::now();
                            self.connection_state = RadioConnectionState::Connected;
                        } else {
                            // Software didn't respond to our hello, it may not be started yet.
                            radio_network_fail_local = true;
                            self.connection_state = RadioConnectionState::ConnectedNetwork;
                        }
                    } else {
                        radio_network_fail_local = true;
                        // If network connection failed, go back up to verify UART.
                        self.connection_state = RadioConnectionState::ConnectedPhys;
                    }
                },
                RadioConnectionState::Connected => {
                    let _ = self.process_packets().await;
                    // if we're stably connected, process packets at 100Hz

                    // If timeout have elapsed since we last got a packet, 
                    // reboot the robot (unless we had a shutdown request).
                    let cur_time = Instant::now();
                    if !cur_robot_state.shutdown_requested && 
                        Instant::checked_duration_since(&cur_time, self.last_software_packet).unwrap().as_millis() > Self::RESPONSE_FROM_PC_TIMEOUT_MS {                        
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                },
            }

            // set global radio connected flag           
            self.shared_robot_state.set_radio_network_ok(self.connection_state >= RadioConnectionState::ConnectedNetwork);
            self.shared_robot_state.set_radio_bridge_ok(self.connection_state == RadioConnectionState::Connected);
            self.shared_robot_state.set_radio_inop(radio_inop_flag_local);

            if radio_inop_flag_local {
                // If hardware problems is present, adds a delay.
                Timer::after_millis(Self::RETRY_DELAY_MS).await;
            }
            else if radio_network_fail_local {
                // If network problems is present, adds a delay.
                Timer::after_millis(Self::RESPONSE_FROM_PC_TIMEOUT_MS).await;
            }

            last_robot_state = cur_robot_state;

            radio_loop_rate_ticker.next().await;
        }

    }

    async fn connect_uart(&mut self) -> Result<(), ()> {
        defmt::info!("connecting radio uart");
        match select(self.radio.connect_uart(), Timer::after_millis(Self::UART_CONNECT_TIMEOUT_MS)).await {
            Either::First(res) => {
                if res.is_err() {
                    defmt::error!("failed to establish radio UART connection.");
                    return Err(());
                } else {
                    defmt::debug!("established radio uart coms");
                    return Ok(());
                }
            }
            Either::Second(_) => {
                defmt::error!("initial radio uart connection timed out");
                return Err(());
            }
        }
    }

    async fn connect_network(&mut self, wifi_network: WifiCredential, robot_id: u8) -> Result<(), ()> {
        if self.radio.connect_to_network(wifi_network, robot_id).await.is_err() {
            defmt::error!("failed to connect to wifi network.");
            return Err(());
        }
        defmt::info!("radio connected");
    
        let res = self.radio.open_multicast().await;
        if res.is_err() {
            defmt::error!("failed to establish multicast socket to network.");
            return Err(());
        }
        defmt::info!("multicast open");

        return Ok(());
    }

    async fn connect_software(&mut self, robot_id: u8, is_blue: bool) -> Result<bool, ()> {
        defmt::info!("sending hello");

        let team_color = if is_blue {
            TeamColor::Blue
        } else {
            TeamColor::Yellow
        };

        if self.radio.send_hello(robot_id, team_color).await.is_err() {
            defmt::error!("send hello failed.");
            return Err(());
        }
        
        let hello = self.radio.wait_hello(Duration::from_millis(Self::RESPONSE_FROM_PC_TIMEOUT_MS)).await;
        match hello {
            Ok(hello) => {
                defmt::info!(
                    "recieved hello resp to: {}.{}.{}.{}:{}",
                    hello.ipv4[0], hello.ipv4[1], hello.ipv4[2], hello.ipv4[3], hello.port
                );
                self.radio.close_peer().await.unwrap();
                defmt::info!("multicast peer closed");

                self.radio.open_unicast(hello.ipv4, hello.port).await.unwrap();
                defmt::info!("unicast open");

                return Ok(true);
            }
            Err(_) => {
                return Ok(false);
            }
        }
    }

    async fn process_packets(&mut self) -> Result<(), ()> {
        // read any packets
        loop {
            if let Ok(pkt) = self.radio.read_packet_nonblocking() {
                if let Some(c2_pkt) = pkt {
                    // update the last packet timestamp
                    self.last_software_packet = Instant::now();
                    self.command_publisher.publish_immediate(c2_pkt);
                } else {
                    break;
                }
            } else {
                defmt::warn!("RadioTask - error reading data packet");
            }
        }

        loop {
            if let Some(telemetry) = self.telemetry_subscriber.try_next_message_pure() {
                match telemetry {
                    TelemetryPacket::Basic(basic) => {
                        self.last_basic_telemetry = basic;
                    },
                    TelemetryPacket::Control(control) => {
                        if self.radio.send_control_debug_telemetry(control).await.is_err() {
                            defmt::warn!("RadioTask - failed to send control debug telemetry packet");
                        }
                    },
                    TelemetryPacket::ParameterCommandResponse(pc_resp) => {
                        if self.radio.send_parameter_response(pc_resp).await.is_err() {
                            defmt::warn!("RadioTask - failed to send control parameter response packet");
                        }
                    },
                }
            } else {
                break;
            }
        }

        // always send the latest telemetry
        if self.radio.send_telemetry(self.last_basic_telemetry).await.is_err() {
            defmt::warn!("RadioTask - failed to send basic telem packet");
        }

        return Ok(())
    }
}

pub fn startup_uart_config() -> usart::Config {
    let mut radio_uart_config = usart::Config::default();
    radio_uart_config.baudrate = 115_200;
    radio_uart_config.data_bits = DataBits::DataBits8;
    radio_uart_config.stop_bits = StopBits::STOP1;
    radio_uart_config.parity = Parity::ParityNone;

    radio_uart_config
}

#[embassy_executor::task]
async fn radio_task_entry(mut radio_task: RadioTask<RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH>) {
    loop {
        radio_task.radio_task_entry().await;
        defmt::error!("radio task returned");
    }
}

pub async fn start_radio_task(radio_task_spawner: Spawner,
        rx_queue_spawner: SendSpawner,
        tx_queue_spawner: SendSpawner,
        robot_state: &'static SharedRobotState,
        command_publisher: CommandsPublisher,
        telemetry_subscriber: TelemetrySubcriber,
        wifi_credentials: &'static [WifiCredential],
        radio_uart: RadioUART,
        radio_uart_rx_pin: RadioUartRxPin,
        radio_uart_tx_pin: RadioUartTxPin,
        _radio_uart_cts_pin: RadioUartCtsPin,
        _radio_uart_rts_pin: RadioUartRtsPin,
        radio_uart_rx_dma: RadioRxDMA,
        radio_uart_tx_dma: RadioTxDMA,
        radio_reset_pin: RadioResetPin,
        radio_ndet_pin: RadioNDetectPin) {


    let uart_conifg = startup_uart_config();
    let radio_uart = Uart::new(radio_uart, radio_uart_rx_pin, radio_uart_tx_pin, SystemIrqs, radio_uart_tx_dma, radio_uart_rx_dma, uart_conifg).unwrap();
    // let radio_uart = Uart::new_with_rtscts(radio_uart, radio_uart_rx_pin, radio_uart_tx_pin, SystemIrqs, _radio_uart_rts_pin, _radio_uart_cts_pin, radio_uart_tx_dma, radio_uart_rx_dma, radio_uart_config).unwrap();
    let (radio_uart_tx, radio_uart_rx) = Uart::split(radio_uart);

    RADIO_IDLE_BUFFERED_UART.init();
    rx_queue_spawner.spawn(idle_buffered_uart_read_task!(RADIO, radio_uart_rx)).unwrap();
    tx_queue_spawner.spawn(idle_buffered_uart_write_task!(RADIO, radio_uart_tx)).unwrap();

    let radio_task = RadioTask::new_from_pins(robot_state, command_publisher, telemetry_subscriber, &RADIO_IDLE_BUFFERED_UART, RADIO_IDLE_BUFFERED_UART.get_uart_read_queue(), RADIO_IDLE_BUFFERED_UART.get_uart_write_queue(), radio_reset_pin, radio_ndet_pin, wifi_credentials);

    radio_task_spawner.spawn(radio_task_entry(radio_task)).unwrap();
}