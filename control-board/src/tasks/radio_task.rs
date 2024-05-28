
use ateam_common_packets::radio::TelemetryPacket;
use ateam_lib_stm32::{drivers::radio::odin_w26x::Radio, make_uart_queue_pair, queue_pair_register_and_spawn, uart::{self, queue::{UartReadQueue, UartWriteQueue}}};
use credentials::WifiCredential;
use embassy_executor::{raw::TaskStorage, SendSpawner, SpawnToken, Spawner};
use embassy_futures::select::{select, Either};
use embassy_stm32::{
    gpio::{Input, Pin, Pull},
    usart::{self, DataBits, Parity, StopBits, Uart}
};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Timer};
use futures_util::Future;

use crate::{drivers::radio_robot::{RobotRadio, TeamColor}, pins::*, robot_state::{RobotState, SharedRobotState}, SystemIrqs};

pub const RADIO_MAX_TX_PACKET_SIZE: usize = 256;
pub const RADIO_TX_BUF_DEPTH: usize = 4;
pub const RADIO_MAX_RX_PACKET_SIZE: usize = 256;
pub const RADIO_RX_BUF_DEPTH: usize = 4;

make_uart_queue_pair!(RADIO,
    RadioUART, RadioRxDMA, RadioTxDMA,
    RADIO_MAX_RX_PACKET_SIZE, RADIO_RX_BUF_DEPTH,
    RADIO_MAX_TX_PACKET_SIZE, RADIO_TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

// pub type RadioTaskFuture<UartPeripherial: usart::BasicInstance,
// UartRxDma: usart::RxDma<UartPeripherial>,
// UartTxDma: usart::TxDma<UartPeripherial>,
// const RADIO_MAX_TX_PACKET_SIZE: usize,
// const RADIO_MAX_RX_PACKET_SIZE: usize,
// const RADIO_TX_BUF_DEPTH: usize,
// const RADIO_RX_BUF_DEPTH: usize>
//  = impl Future;

#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
enum RadioConnectionState {
    Unconnected,
    ConnectPhys,
    ConnectUart,
    ConnectNetwork,
    ConnectSoftware,
    Connected,
}

pub struct RadioTask<
        UartPeripherial: usart::BasicInstance,
        UartRxDma: usart::RxDma<UartPeripherial>,
        UartTxDma: usart::TxDma<UartPeripherial>,
        const RADIO_MAX_TX_PACKET_SIZE: usize,
        const RADIO_MAX_RX_PACKET_SIZE: usize,
        const RADIO_TX_BUF_DEPTH: usize,
        const RADIO_RX_BUF_DEPTH: usize> {
    shared_robot_state: &'static SharedRobotState,
    command_publisher: CommandsPublisher,
    telemetry_subscriber: TelemetrySubcriber,
    radio: RobotRadio<'static, UartPeripherial, UartRxDma, UartTxDma, RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH>,
    radio_ndet_input: Input<'static>,

    connection_state: RadioConnectionState,
    wifi_credentials: &'static [WifiCredential],
}

impl<
        UartPeripherial: usart::BasicInstance,
        UartRxDma: usart::RxDma<UartPeripherial>,
        UartTxDma: usart::TxDma<UartPeripherial>,
        const RADIO_MAX_TX_PACKET_SIZE: usize,
        const RADIO_MAX_RX_PACKET_SIZE: usize,
        const RADIO_TX_BUF_DEPTH: usize,
        const RADIO_RX_BUF_DEPTH: usize> 
    RadioTask<UartPeripherial, UartRxDma, UartTxDma, RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH> {
    pub type TaskRobotRadio = RobotRadio<'static, UartPeripherial, UartRxDma, UartTxDma, RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH>;

    const RETRY_DELAY_MS: u64 = 1000;
    const UART_CONNECT_TIMEOUT_MS: u64 = 5000;

    pub fn new(robot_state: &'static SharedRobotState,
            command_publisher: CommandsPublisher,
            telemetry_subscriber: TelemetrySubcriber,
            radio: Self::TaskRobotRadio,
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
            // task: TaskStorage::new(),
        }
    }

    pub fn new_from_pins(robot_state: &'static SharedRobotState,
            command_publisher: CommandsPublisher,
            telemetry_subscriber: TelemetrySubcriber,
            radio_rx_uart_queue: &'static UartReadQueue<UartPeripherial, UartRxDma, RADIO_MAX_RX_PACKET_SIZE, RADIO_RX_BUF_DEPTH>,
            radio_tx_uart_queue: &'static UartWriteQueue<UartPeripherial, UartTxDma, RADIO_MAX_TX_PACKET_SIZE, RADIO_TX_BUF_DEPTH>,
            radio_reset_pin: impl Pin,
            radio_ndet_pin: impl Pin,
            wifi_credentials: &'static [WifiCredential]) -> Self {

        let radio: Self::TaskRobotRadio = RobotRadio::new(radio_rx_uart_queue, radio_tx_uart_queue, radio_reset_pin);

        let radio_ndet = Input::new(radio_ndet_pin, Pull::None);

        Self::new(robot_state, command_publisher, telemetry_subscriber, radio, radio_ndet, wifi_credentials)
    }

    async fn radio_task_entry(&mut self) {
        defmt::info!("radio task startup");

        // initialize a copy of the robot state so we can track updates
        let mut last_robot_state = self.shared_robot_state.get_state();

        // allow default fallback state transition of none
        #[allow(unused_assignments)]
        // let mut next_connection_state = self.connection_state;
        loop {

            // check for hardware config changes that affect radio connection

            let cur_robot_state = self.shared_robot_state.get_state();
            if cur_robot_state != last_robot_state {
                // we are connected to software and robot id or team was changed on hardware
                if self.connection_state == RadioConnectionState::Connected &&
                        (cur_robot_state.hw_robot_id != last_robot_state.hw_robot_id 
                        || cur_robot_state.hw_robot_team_is_blue != last_robot_state.hw_robot_team_is_blue) {
                    // enter software connect state (disconnecting)
                    self.connection_state = RadioConnectionState::ConnectSoftware;
                    defmt::info!("shell state change triggering software reconnect");
                }

                // we ar at least connected to Wifi and the wifi network id changed
                if self.connection_state > RadioConnectionState::ConnectNetwork 
                && cur_robot_state.hw_wifi_network_index != last_robot_state.hw_wifi_network_index {
                    if self.radio.disconnect_network().await.is_err() {
                        defmt::error!("failed to cleanly disconnect - consider radio reboot");
                    }

                    self.connection_state = RadioConnectionState::ConnectNetwork;
                    defmt::info!("dip state change triggering wifi network change");
                }
            }
            last_robot_state = cur_robot_state;

            // check for missing radio

            if self.radio_ndet_input.is_high() {
                defmt::error!("radio appears unplugged.");
                self.connection_state = RadioConnectionState::ConnectPhys;
            }

            // execute on the connection state

            match self.connection_state {
                RadioConnectionState::Unconnected => {
                    self.connection_state = RadioConnectionState::ConnectPhys;
                },
                RadioConnectionState::ConnectPhys => {
                    if self.radio_ndet_input.is_high() {
                        Timer::after_millis(Self::RETRY_DELAY_MS).await;
                        self.connection_state = RadioConnectionState::ConnectPhys;
                    } else {
                        self.connection_state = RadioConnectionState::ConnectUart;
                    }
                },
                RadioConnectionState::ConnectUart => {
                    if self.connect_uart().await.is_err() {
                        Timer::after_millis(Self::RETRY_DELAY_MS).await;
                        // failed to connect, go back to physical connection check
                        self.connection_state = RadioConnectionState::ConnectPhys;
                    } else {
                        self.connection_state = RadioConnectionState::ConnectNetwork;
                    }
                },
                RadioConnectionState::ConnectNetwork => {
                    let wifi_network_ind = cur_robot_state.hw_wifi_network_index as usize;
                    let wifi_credential = if wifi_network_ind >= self.wifi_credentials.len() {
                        self.wifi_credentials[self.wifi_credentials.len() - 1]
                    } else {
                        self.wifi_credentials[wifi_network_ind]
                    };

                    defmt::debug!("connecting to network ({}): ssid: {}, password: {}", wifi_network_ind, wifi_credential.get_ssid(), wifi_credential.get_password());
                    if self.connect_network(wifi_credential, cur_robot_state.hw_robot_id).await.is_err() {
                        Timer::after_millis(Self::RETRY_DELAY_MS).await;
                        // TODO make error handling smarter, e.g. if we get a timeout or low level errors
                        // we should fall back to ConnectPhys or ConnectUart, not keep retrying forever
                    } else {
                        self.connection_state = RadioConnectionState::ConnectSoftware;
                    }
                },
                RadioConnectionState::ConnectSoftware => {
                    if let Ok(connected) = self.connect_software(cur_robot_state.hw_robot_id, cur_robot_state.hw_robot_team_is_blue).await {
                        if connected {
                            self.connection_state = RadioConnectionState::Connected;
                        } else {
                            // software didn't respond to our hello, it may not be started yet
                            Timer::after_millis(1000).await;
                        }
                    } else {
                        // a hard error occurred
                        Timer::after_millis(Self::RETRY_DELAY_MS).await;

                        // TODO where should we retry?
                    }
                },
                RadioConnectionState::Connected => {
                    let _ = self.process_packet().await;
                    // no delay to imediately process the next one
                },
            }
        }

    }

    async fn connect_uart(&mut self) -> Result<(), ()> {
        defmt::info!("connecting radio uart");
        match select(self.radio.connect_uart(), Timer::after_millis(Self::UART_CONNECT_TIMEOUT_MS)).await {
            Either::First(res) => {
                if res.is_err() {
                    defmt::error!("failed to establish radio UART connection.");
                    return Err(())
                } else {
                    defmt::debug!("established radio uart coms");
                    return Ok(())
                }
            }
            Either::Second(_) => {
                defmt::error!("initial radio uart connection timed out");
                return Err(())
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
            return Err(())
        }
        defmt::info!("multicast open");

        return Ok(())
    }

    async fn connect_software(&mut self, robot_id: u8, is_blue: bool) -> Result<bool, ()> {
        defmt::info!("sending hello");

        let team_color = if is_blue {
            TeamColor::Blue
        } else {
            TeamColor::Yellow
        };

        if self.radio.send_hello(robot_id, team_color).await.is_err() {
            return Err(())
        }
        
        let hello = self.radio.wait_hello(Duration::from_millis(1000)).await;
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

                Ok(true)
            }
            Err(_) => {
                Ok(false)
            }
        }
    }

    async fn process_packet(&mut self) -> Result<(), ()> {
        match select(self.radio.read_packet(), self.telemetry_subscriber.next_message()).await {
            Either::First(res) => {
                if let Ok(c2_pkt) = res {
                    self.command_publisher.publish_immediate(c2_pkt);
                    Ok(())
                } else {
                    defmt::warn!("radio read packet returned an error");
                    Err(())
                }
            }
            Either::Second(telem_msg) => {
                match telem_msg {
                    WaitResult::Lagged(num_missed_pkts) => {
                        defmt::warn!("radio task missed sending {} outbound packets. Should channel have higher capacity?", num_missed_pkts);
                        Ok(())
                    },
                    WaitResult::Message(msg) => {
                        match msg {
                            TelemetryPacket::Basic(basic_telem_pkt) => {
                                let res = self.radio.send_telemetry(basic_telem_pkt).await;
                                if res.is_err() {
                                    defmt::warn!("radio task failed to send basic telemetry packet. Is the backing queue too small?");
                                    Err(())
                                } else {
                                    Ok(())
                                }
                            }
                            TelemetryPacket::Control(control_telem_pkt) => {
                                let res = self.radio.send_control_debug_telemetry(control_telem_pkt).await;
                                if res.is_err() {
                                    defmt::warn!("radio task failed to send control debug telemetry packet. Is the backing queue too small?");
                                    Err(())
                                } else {
                                    Ok(())
                                }
                            }
                            TelemetryPacket::ParameterCommandResponse(param_resp) => {
                                let res = self.radio.send_parameter_response(param_resp).await;
                                if res.is_err() {
                                    defmt::warn!("radio task failed to send param resp packet. Is the backing queue too small?");
                                    Err(())
                                } else {
                                    Ok(())
                                }
                            }
                        }
                    }
                }
            }
        }
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
async fn radio_task_entry(mut radio_task: RadioTask<RadioUART, RadioRxDMA, RadioTxDMA, RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH>) {
    loop {
        radio_task.radio_task_entry().await;
        defmt::error!("radio task returned");
    }
}

#[embassy_executor::task]
async fn radio_task_entry2(
    robot_state: &'static SharedRobotState,
    command_publisher: CommandsPublisher,
    mut telemetry_subscriber: TelemetrySubcriber,
    wifi_network: WifiCredential,
    radio_reset_pin: RadioResetPin,
    radio_ndet_pin: RadioNDetectPin) {

    defmt::info!("radio task startup");

    let radio_ndet = Input::new(radio_ndet_pin, Pull::None);

    let mut radio: RobotRadio<'static, RadioUART, RadioRxDMA, RadioTxDMA, RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH> = 
        RobotRadio::new(&RADIO_RX_UART_QUEUE, &RADIO_TX_UART_QUEUE, radio_reset_pin);

    let mut startup_radio_uart_config = usart::Config::default();
    startup_radio_uart_config.baudrate = 115_200;
    startup_radio_uart_config.data_bits = DataBits::DataBits8;
    startup_radio_uart_config.stop_bits = StopBits::STOP1;
    startup_radio_uart_config.parity = Parity::ParityNone;

    #[allow(unused_labels)]
    'connect_loop: 
    loop {
        if radio_ndet.is_high() {
            defmt::error!("radio appears unplugged.");
            Timer::after_millis(1000).await;
            continue 'connect_loop;
        }

        if RADIO_TX_UART_QUEUE.update_uart_config(startup_radio_uart_config).await.is_err() {
            defmt::error!("failed to reset radio config.");
            Timer::after_millis(1000).await;
            continue 'connect_loop;
        }

        defmt::info!("connecting radio uart");
        match select(radio.connect_uart(), Timer::after_millis(10000)).await {
            Either::First(res) => {
                if res.is_err() {
                    defmt::error!("failed to establish radio UART connection.");
                    Timer::after_millis(1000).await;
                    continue 'connect_loop;
                } else {
                    defmt::debug!("established radio uart coms");
                }
            }
            Either::Second(_) => {
                defmt::error!("initial radio uart connection timed out");
                continue 'connect_loop;
            }
        }

        while radio.connect_to_network(wifi_network, robot_state.get_hw_robot_id()).await.is_err() {
            defmt::error!("failed to connect to wifi network.");
            Timer::after_millis(1000).await;
        }
        defmt::info!("radio connected");
    
        let res = radio.open_multicast().await;
        if res.is_err() {
            defmt::error!("failed to establish multicast socket to network.");
            continue 'connect_loop;
        }
        defmt::info!("multicast open");

        'connect_hello:
        loop {
            defmt::info!("sending hello");

            let robot_id = robot_state.get_hw_robot_id();
            let team_color = if robot_state.hw_robot_team_is_blue() {
                TeamColor::Blue
            } else {
                TeamColor::Yellow
            };
            radio.send_hello(robot_id, team_color).await.unwrap();
            let hello = radio.wait_hello(Duration::from_millis(1000)).await;

            match hello {
                Ok(hello) => {
                    defmt::info!(
                        "recieved hello resp to: {}.{}.{}.{}:{}",
                        hello.ipv4[0], hello.ipv4[1], hello.ipv4[2], hello.ipv4[3], hello.port
                    );
                    radio.close_peer().await.unwrap();
                    defmt::info!("multicast peer closed");

                    radio.open_unicast(hello.ipv4, hello.port).await.unwrap();
                    defmt::info!("unicast open");

                    break 'connect_hello;
                }
                Err(_) => {}
            }
        }

        // TODO add inbound timeout somewhere, maybe not here.
        'process_packets: 
        loop {
            match select(radio.read_packet(), telemetry_subscriber.next_message()).await {
                Either::First(res) => {
                    if let Ok(c2_pkt) = res {
                        command_publisher.publish_immediate(c2_pkt);
                    } else {
                        defmt::warn!("radio read packet returned an error");
                    }
                }
                Either::Second(telem_msg) => {
                    match telem_msg {
                        WaitResult::Lagged(num_missed_pkts) => {
                            defmt::warn!("radio task missed sending {} outbound packets. Should channel have higher capacity?", num_missed_pkts);
                        },
                        WaitResult::Message(msg) => {
                            match msg {
                                TelemetryPacket::Basic(basic_telem_pkt) => {
                                    let res = radio.send_telemetry(basic_telem_pkt).await;
                                    if res.is_err() {
                                        defmt::warn!("radio task failed to send basic telemetry packet. Is the backing queue too small?");
                                    }
                                }
                                TelemetryPacket::Control(control_telem_pkt) => {
                                    let res = radio.send_control_debug_telemetry(control_telem_pkt).await;
                                    if res.is_err() {
                                        defmt::warn!("radio task failed to send control debug telemetry packet. Is the backing queue too small?");
                                    }
                                }
                                TelemetryPacket::ParameterCommandResponse(param_resp) => {
                                    let res = radio.send_parameter_response(param_resp).await;
                                    if res.is_err() {
                                        defmt::warn!("radio task failed to send param resp packet. Is the backing queue too small?")
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if radio_ndet.is_high() {
                defmt::error!("radio was unplugged.");
                break 'process_packets;
            }
        }
    }
}

pub async fn start_radio_task(radio_task_spawner: Spawner,
        queue_spawner: SendSpawner,
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

    queue_pair_register_and_spawn!(queue_spawner, RADIO, radio_uart_rx, radio_uart_tx);

    let radio_task = RadioTask::new_from_pins(robot_state, command_publisher, telemetry_subscriber, &RADIO_RX_UART_QUEUE, &RADIO_TX_UART_QUEUE, radio_reset_pin, radio_ndet_pin, wifi_credentials);

    radio_task_spawner.spawn(radio_task_entry(radio_task)).unwrap();

    // radio_task_spawner.spawn(radio_task_entry(robot_state, command_publisher, telemetry_subscriber, wifi_network, radio_reset_pin, radio_ndet_pin)).unwrap();
}