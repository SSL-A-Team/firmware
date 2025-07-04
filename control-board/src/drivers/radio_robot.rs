use ateam_lib_stm32::drivers::radio::odin_w26x::{OdinRadioError, OdinW262, PeerConnection, WifiAuth};
use ateam_lib_stm32::uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue};
use ateam_common_packets::bindings::{
    self, BasicControl, BasicTelemetry, CommandCode, ExtendedTelemetry, HelloRequest, HelloResponse, ParameterCommand, RadioPacket, RadioPacket_Data
};
use ateam_common_packets::radio::DataPacket;
use const_format::formatcp;
use credentials::WifiCredential;
use embassy_stm32::uid;
use core::fmt::Write;
use core::mem::size_of;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::{Level, Pin, Speed, Output};
use embassy_stm32::usart::{self, DataBits, Parity, StopBits};
use embassy_time::{Duration, Timer};
use heapless::String;

use defmt::Format;

const MULTICAST_IP: &str = "224.4.20.69";
const MULTICAST_PORT: u16 = 42069;
const LOCAL_PORT: u16 = 42069;

#[derive(Copy, Clone)]
pub enum WifiNetwork {
    Team,
    CompMain,
    CompPractice
}

#[derive(Copy, Clone)]
pub enum TeamColor {
    Yellow,
    Blue,
}

#[derive(Clone, Copy, PartialEq, Debug, Format)]
pub enum RobotRadioError {
    DriverError(OdinRadioError),

    RequestTimedOut,

    ConnectUartBadStartup,
    ConnectUartBadEcho,
    ConnectUartBadRadioConfigUpdate,
    ConnectUartBadHostConfigUpdate,
    ConnectUartCannotEnterEdm,
    ConnectUartNoEdmStartup,

    ConnectWifiBadHostName,
    ConnectWifiBadConfig,
    ConnectWifiConnectionFailed,

    OpenMulticastError,

    DisconnectFailed,

    PeerMissing,

    SoftwareConnectAckHeaderInvalid,
    SoftwareHelloHeaderInvalid,

    ControlPacketDecodeInvalid,
    ParameterPacketDecodeInvalid,
    PacketTypeUnknown,
}

impl From<OdinRadioError> for RobotRadioError {
    fn from(err: OdinRadioError) -> Self {
        RobotRadioError::DriverError(err)
    }
}

unsafe impl<
        'a,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
    > Send for RobotRadio<'a, LEN_RX, LEN_TX, DEPTH_TX, DEPTH_RX> {}


pub struct RobotRadio<
    'a,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
> {
    odin_driver: OdinW262<
        'a,
        LEN_TX,
        LEN_RX,
        DEPTH_TX,
        DEPTH_RX,
    >,
    reset_pin: Output<'a>,
    peer: Option<PeerConnection>,
}

impl<
        'a,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
    > RobotRadio<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX>
{
    pub fn new(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX>,
        reset_pin: impl Pin,
    ) -> RobotRadio<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX> {
        let reset_pin = Output::new(reset_pin, Level::High, Speed::Medium);
        let radio = OdinW262::new(read_queue, write_queue, uart);

        Self {
            odin_driver: radio,
            reset_pin,
            peer: None,
        }
    }

    pub fn get_startup_uart_config(&self) -> usart::Config {
        let mut startup_radio_uart_config = usart::Config::default();
        startup_radio_uart_config.baudrate = 115_200;
        startup_radio_uart_config.data_bits = DataBits::DataBits8;
        startup_radio_uart_config.stop_bits = StopBits::STOP1;
        startup_radio_uart_config.parity = Parity::ParityNone;

        startup_radio_uart_config
    }

    pub fn get_highspeed_uart_config(&self) -> usart::Config {
        let mut highspeed_radio_uart_config = usart::Config::default();
        highspeed_radio_uart_config.baudrate = 5_250_000;
        highspeed_radio_uart_config.stop_bits = StopBits::STOP1;
        highspeed_radio_uart_config.data_bits = DataBits::DataBits8;
        highspeed_radio_uart_config.parity = usart::Parity::ParityEven;

        highspeed_radio_uart_config
    }

    pub async fn connect_uart(&mut self) -> Result<(), RobotRadioError> {
        // were about to reset the radio, so we also need to reset the uart queue config to match the startup config
        if self.odin_driver.update_host_uart_config(self.get_startup_uart_config()).await.is_err() {
            defmt::debug!("failed to reset host uart to startup config.");
        }

        // reset the radio so we can listen for the startup event
        self.reset_pin.set_high();
        Timer::after(Duration::from_millis(1)).await;
        self.reset_pin.set_low();

        // wait until startup event is received
        if self.odin_driver.wait_startup().await.is_err() {
            defmt::debug!("error processing radio wait startup command");
            return Err(RobotRadioError::ConnectUartBadStartup);
        }
        defmt::trace!("increasing link speed");

        let baudrate = 5_250_000;
        if self.odin_driver.set_echo(false).await.is_err() {
            defmt::debug!("error disabling echo on radio");
            return Err(RobotRadioError::ConnectUartBadEcho);
        }
        if self.odin_driver.config_uart(baudrate, true, 8, true).await.is_err() {
            defmt::debug!("error increasing radio baud rate.");
            return Err(RobotRadioError::ConnectUartBadRadioConfigUpdate);
        }
        defmt::trace!("configured radio link speed");


        if self.odin_driver.update_host_uart_config(self.get_highspeed_uart_config()).await.is_err() {
            defmt::debug!("error increasing host baud rate.");
            return Err(RobotRadioError::ConnectUartBadHostConfigUpdate);
        }
        defmt::trace!("configured host link speed");


        // Datasheet says wait at least 40ms after UART config change
        Timer::after(Duration::from_millis(50)).await;

        // Datasheet says wait at least 50ms after entering data mode
        if let Ok(got_edm_startup) = self.odin_driver.enter_edm().await {
            defmt::trace!("entered edm at high link speed");

            if ! got_edm_startup {
                if self.odin_driver.wait_edm_startup().await.is_err() {
                    defmt::debug!("error waiting for EDM startup after uart baudrate increase");
                    return Err(RobotRadioError::ConnectUartNoEdmStartup);
                }
            } else {
                defmt::trace!("got EDM startup command");
            }
        } else {
            defmt::debug!("error entering EDM mode after uart baudrate increase");
            return Err(RobotRadioError::ConnectUartCannotEnterEdm)
        }

        Timer::after(Duration::from_millis(50)).await;

        Ok(())
    }

    pub async fn disconnect_network(&mut self) -> Result<(), RobotRadioError> {
        let mut had_error = false;
        if let Some(peer) = self.peer.take() {
            defmt::debug!("closing peer..");
            if self.odin_driver.close_peer(peer.peer_id).await.is_err() {
                defmt::warn!("failed to close peer on network dc");
                had_error = true;
            } else {
                defmt::debug!("closed peer.")
            }
        }

        defmt::debug!("closing wifi.");
        if self.odin_driver.disconnect_wifi(1).await.is_err() {
            defmt::warn!("failed to disconnect network.");
            had_error = true;
        } else {
            defmt::debug!("disconnected wifi.")
        }

        if had_error {
            Err(RobotRadioError::DisconnectFailed)
        } else {
            Ok(())
        }
    }

    pub async fn connect_to_network(&mut self, wifi_credential: WifiCredential, robot_number: u8) -> Result<(), RobotRadioError> {
        // set radio hardware name enumeration
        let uid = uid::uid();
        let uid_u16 = (uid[1] as u16) << 8 | uid[0] as u16;

        let mut s = String::<25>::new();
        core::write!(&mut s, "A-Team Robot #{:02X} ({:04X})", robot_number, uid_u16).unwrap();
        // let mut s = String::<16>::new();
        // core::write!(&mut s, "A-Team Robot {:02X}", robot_number).unwrap();
        if self.odin_driver.set_host_name(s.as_str()).await.is_err() {
            defmt::trace!("could not set radio host name");
            return Err(RobotRadioError::ConnectWifiBadHostName);
        }

        // load the wifi network configuration into config slot 1
        let wifi_ssid = wifi_credential.get_ssid();
        let wifi_pass = WifiAuth::WPA {
            passphrase: wifi_credential.get_password(),
        };
        if self.odin_driver.config_wifi(1, wifi_ssid, wifi_pass).await.is_err() {
            defmt::trace!("could not configure wifi profile");
            return Err(RobotRadioError::ConnectWifiBadConfig);
        }

        // connect to config slot 1
        if self.odin_driver.connect_wifi(1).await.is_err() {
            defmt::trace!("could not connect to wifi");

            // can never configure a profile that "active" even when unconnected
            // we're not really in a known state with out a lot more effort
            // so ignore the result
            let _ = self.disconnect_network().await;

            return Err(RobotRadioError::ConnectWifiConnectionFailed);
        }

        // if we made it this far, we're connected
        Ok(())
    }

    pub async fn open_multicast(&mut self) -> Result<(), RobotRadioError> {
        let peer = self.odin_driver.connect_peer(formatcp!(
                "udp://{MULTICAST_IP}:{MULTICAST_PORT}/?flags=1&local_port={LOCAL_PORT}"
            ))
            .await;

        if peer.is_err() {
            defmt::debug!("failed to connect peer");
            return Err(RobotRadioError::OpenMulticastError);
        }

        self.peer = Some(peer.unwrap());
        Ok(())
    }

    pub async fn open_unicast(&mut self, ipv4: [u8; 4], port: u16) -> Result<(), RobotRadioError> {
        let mut s = String::<50>::new();
        core::write!(
            &mut s,
            "udp://{}.{}.{}.{}:{}/?local_port={LOCAL_PORT}",
            ipv4[0],
            ipv4[1],
            ipv4[2],
            ipv4[3],
            port
        )
        .unwrap();

        let peer = self.odin_driver.connect_peer(s.as_str()).await?;
        self.peer = Some(peer);
        Ok(())
    }

    pub async fn close_peer(&mut self) -> Result<(), RobotRadioError> {
        if let Some(peer) = &self.peer {
            self.odin_driver.close_peer(peer.peer_id).await?;
            self.peer = None;
            Ok(())
        } else {
            Err(RobotRadioError::PeerMissing)
        }
    }

    pub fn send_data(&self, data: &[u8]) -> Result<(), RobotRadioError> {
        if let Some(peer) = &self.peer {
            self.odin_driver.send_data(peer.channel_id, data)?;
            Ok(())
        } else {
            Err(RobotRadioError::PeerMissing)
        }
    }

    pub async fn read_data<RET, FN>(&'a self, fn_read: FN) -> Result<RET, RobotRadioError>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        if self.peer.is_some() {
            let ret = self.odin_driver.read_data(fn_read).await?;
            Ok(ret)
        } else {
            Err(RobotRadioError::PeerMissing)
        }
    }

    fn read_data_nonblocking<RET, FN>(&'a self, fn_read: FN) -> Result<Option<RET>, RobotRadioError>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        if self.peer.is_some() {
            if self.odin_driver.can_read_data() {
                match self.odin_driver.try_read_data(fn_read) {
                    Ok(ret) => {
                        Ok(Some(ret))
                    },
                    Err(e) => {
                        defmt::trace!("try read data failed after can read data reported data ready");
                        Err(RobotRadioError::DriverError(e))
                    },
                }
            } else {
                Ok(None)
            }
        } else {
            defmt::trace!("peer was none");
            Err(RobotRadioError::PeerMissing)
        }
    }

    pub async fn send_ack(&self, nack: bool) -> Result<(), RobotRadioError> {
        let packet = RadioPacket {
            crc32: 0,
            major_version: bindings::kProtocolVersionMajor,
            minor_version: bindings::kProtocolVersionMinor,
            command_code: if nack {
                CommandCode::CC_NACK
            } else {
                CommandCode::CC_ACK
            },
            data_length: 0,
            data: unsafe { core::mem::zeroed() },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioPacket_Data>(),
            )
        };
        self.send_data(packet_bytes)?;

        Ok(())
    }

    pub async fn wait_ack(&self, timeout: Duration) -> Result<bool, RobotRadioError> {
        let read_fut = self.read_data(|data| {
            if data.len() != size_of::<RadioPacket>() - size_of::<RadioPacket_Data>() {
                return Err(RobotRadioError::SoftwareConnectAckHeaderInvalid);
            }
            let packet = unsafe { &*(data as *const _ as *const RadioPacket) };

            match packet.command_code {
                CommandCode::CC_ACK => Ok(true),
                CommandCode::CC_NACK => Ok(false),
                _ => Err(RobotRadioError::SoftwareConnectAckHeaderInvalid),
            }
        });
        match select(read_fut, Timer::after(timeout)).await {
            Either::First(ret) => ret?,
            Either::Second(_) => Err(RobotRadioError::RequestTimedOut),
        }
    }

    pub async fn send_hello(&self, id: u8, team: TeamColor) -> Result<(), RobotRadioError> {
        let packet = RadioPacket {
            crc32: 0,
            major_version: bindings::kProtocolVersionMajor,
            minor_version: bindings::kProtocolVersionMinor,
            command_code: CommandCode::CC_HELLO_REQ,
            data_length: size_of::<HelloRequest>() as u16,
            data: RadioPacket_Data {
                hello_request: HelloRequest {
                    robot_id: id,
                    color: match team {
                        TeamColor::Yellow => bindings::TeamColor::TC_YELLOW,
                        TeamColor::Blue => bindings::TeamColor::TC_BLUE,
                    },
                },
            },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioPacket_Data>()
                    + size_of::<HelloRequest>(),
            )
        };
        self.send_data(packet_bytes)?;

        Ok(())
    }

    pub fn send_telemetry(&self, telemetry: BasicTelemetry) -> Result<(), RobotRadioError> {
        let packet = RadioPacket {
            crc32: 0,
            major_version: bindings::kProtocolVersionMajor,
            minor_version: bindings::kProtocolVersionMinor,
            command_code: CommandCode::CC_TELEMETRY,
            data_length: size_of::<BasicTelemetry>() as u16,
            data: RadioPacket_Data {
                telemetry: telemetry
            },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioPacket_Data>()
                    + size_of::<BasicTelemetry>(),
            )
        };
        self.send_data(packet_bytes)?;

        Ok(())
    }

    pub async fn send_control_debug_telemetry(&self, telemetry: ExtendedTelemetry) -> Result<(), RobotRadioError> {
        let packet = RadioPacket {
            crc32: 0,
            major_version: bindings::kProtocolVersionMajor,
            minor_version: bindings::kProtocolVersionMinor,
            command_code: CommandCode::CC_CONTROL_DEBUG_TELEMETRY,
            data_length: size_of::<ExtendedTelemetry>() as u16,
            data: RadioPacket_Data {
                control_debug_telemetry: telemetry
            },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioPacket_Data>()
                    + size_of::<ExtendedTelemetry>(),
            )
        };
        self.send_data(packet_bytes)?;

        Ok(())
    }

    pub async fn send_parameter_response(&self, parameter_cmd: ParameterCommand) -> Result<(), RobotRadioError> {
        let packet = RadioPacket {
            crc32: 0,
            major_version: bindings::kProtocolVersionMajor,
            minor_version: bindings::kProtocolVersionMinor,
            command_code: CommandCode::CC_ROBOT_PARAMETER_COMMAND,
            data_length: size_of::<ParameterCommand>() as u16,
            data: RadioPacket_Data {
                robot_parameter_command: parameter_cmd
            },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioPacket_Data>()
                    + size_of::<ParameterCommand>(),
            )
        };
        self.send_data(packet_bytes)?;

        Ok(())
    }

    pub async fn wait_hello(&self, timeout: Duration) -> Result<HelloResponse, RobotRadioError> {
        let read_fut = self.read_data(|data| {
            const PACKET_SIZE: usize = size_of::<RadioPacket>() - size_of::<RadioPacket_Data>()
                + size_of::<HelloResponse>();
            if data.len() != PACKET_SIZE {
                return Err(RobotRadioError::SoftwareHelloHeaderInvalid);
            }

            let mut data_copy = [0u8; size_of::<RadioPacket>()];
            data_copy[0..PACKET_SIZE].clone_from_slice(&data[0..PACKET_SIZE]);

            let packet = unsafe { &*(&data_copy as *const _ as *const RadioPacket) };

            if packet.command_code != CommandCode::CC_HELLO_RESP {
                return Err(RobotRadioError::SoftwareHelloHeaderInvalid);
            }
            // TODO: handle nack

            Ok(unsafe { packet.data.hello_response })
        });

        match select(read_fut, Timer::after(timeout)).await {
            Either::First(ret) => ret?,
            Either::Second(_) => Err(RobotRadioError::SoftwareHelloHeaderInvalid),
        }
    }

    pub fn parse_data_packet(&self, data: &[u8]) -> Result<DataPacket, RobotRadioError> {
        const CONTROL_PACKET_SIZE: usize = size_of::<RadioPacket>() - size_of::<RadioPacket_Data>()
            + size_of::<BasicControl>();
        const PARAMERTER_PACKET_SIZE: usize = size_of::<RadioPacket>() - size_of::<RadioPacket_Data>()
            + size_of::<ParameterCommand>();

        if data.len() == CONTROL_PACKET_SIZE {
            let mut data_copy = [0u8; size_of::<RadioPacket>()];
            data_copy[0..CONTROL_PACKET_SIZE].clone_from_slice(&data[0..CONTROL_PACKET_SIZE]);

            let packet = unsafe { &*(&data_copy as *const _ as *const RadioPacket) };

            if packet.command_code != CommandCode::CC_CONTROL {
                return Err(RobotRadioError::ControlPacketDecodeInvalid);
            }

            Ok(unsafe { DataPacket::BasicControl(packet.data.control) })
        } else if data.len() == PARAMERTER_PACKET_SIZE {
            let mut data_copy = [0u8; size_of::<RadioPacket>()];
            data_copy[0..PARAMERTER_PACKET_SIZE].clone_from_slice(&data[0..PARAMERTER_PACKET_SIZE]);

            let packet = unsafe { &*(&data_copy as *const _ as *const RadioPacket) };

            if packet.command_code != CommandCode::CC_ROBOT_PARAMETER_COMMAND {
                return Err(RobotRadioError::ParameterPacketDecodeInvalid);
            }

            Ok(unsafe { DataPacket::ParameterCommand(packet.data.robot_parameter_command) })
        } else {
            return Err(RobotRadioError::PacketTypeUnknown);
        }
    }

    pub async fn read_packet(&self) -> Result<DataPacket, RobotRadioError> {
        self.read_data(|data| {
            self.parse_data_packet(data)
        })
        .await?
    }

    pub fn read_packet_nonblocking(&self) -> Result<Option<DataPacket>, ()> {
        let res = self.read_data_nonblocking(|data| {
            self.parse_data_packet(data)
        });

        match res {
            Ok(res) => {
                match res {
                    Some(pkt) => {
                        match pkt {
                            Ok(pkt) => {
                                return Ok(Some(pkt));
                            },
                            Err(_) => {
                                // we got data that was a valid EDM DataPacket, but couldn't parse it
                                // into any known A-Team packet format
                                defmt::debug!("got EDM packet but wasn't A-Team");
                                return Err(());
                            },
                        }
                    },
                    None => {
                        return Ok(None)
                    },
                }
            },
            Err(err_res) => {
                defmt::debug!("radio in invalid state {:?}", err_res);
                // read_data_nonblocking failed because the radio was in an invalid state
                return Err(())
            },
        }
    }
}
