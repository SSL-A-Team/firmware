use ateam_common_packets::{
    BasicControl, BasicTelemetry, ErrorTelemetry, ExtendedTelemetry,
    HelloRequest, HelloResponse, ParameterCommand, RadioData, RadioHeader, RadioPacket,
    TeamColor as PacketTeamColor,
};
use ateam_common_packets::bitfields::DiscoveryFlags;
use ateam_common_packets::radio::DataPacket;
use ateam_lib_stm32::drivers::radio::odin_w26x::{
    OdinRadioError, OdinW262, PeerConnection, WifiAuth,
};
use ateam_lib_stm32::uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue};
use const_format::formatcp;
use core::fmt::Write;
use core::mem::size_of;
use credentials::WifiCredential;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::usart::{self, DataBits, Parity, StopBits};
use embassy_stm32::{uid, Peri};
use embassy_time::{Duration, Timer};
use heapless::String;

use defmt::Format;

const MULTICAST_IP: &str = "224.4.20.69";
const MULTICAST_PORT: u16 = 42069;
const LOCAL_PORT: u16 = 42069;

// Overhead of RadioData's discriminant byte + 3 bytes of union-body alignment padding.
// All RadioData variants share the same union body starting at byte offset 4.
const RADIO_DATA_TAG_PAD: usize = 4;

#[derive(Copy, Clone)]
pub enum WifiNetwork {
    Team,
    CompMain,
    CompPractice,
}

#[derive(Copy, Clone)]
pub enum TeamColor {
    Yellow,
    Blue,
}

#[derive(Clone, Copy, PartialEq, Debug, Format)]
pub enum RobotRadioError {
    ReadDataError(OdinRadioError),

    RequestTimedOut,

    ConnectUartBadStartup(OdinRadioError),
    ConnectUartBadEcho(OdinRadioError),
    ConnectUartBadRadioConfigUpdate(OdinRadioError),
    ConnectUartBadHostConfigUpdate,
    ConnectUartCannotEnterEdm(OdinRadioError),
    ConnectUartNoEdmStartup(OdinRadioError),

    ConnectWifiBadHostName(OdinRadioError),
    ConnectWifiBadConfig(OdinRadioError),
    ConnectWifiConnectionFailed(OdinRadioError),

    OpenMulticastError(OdinRadioError),

    DisconnectFailed,

    PeerMissing,

    SoftwareConnectAckHeaderInvalid,
    SoftwareHelloHeaderInvalid,
    SoftwareHelloResponseTimeout,

    ControlPacketDecodeInvalid,
    ParameterPacketDecodeInvalid,
    PacketTypeUnknown,
}

impl From<OdinRadioError> for RobotRadioError {
    fn from(err: OdinRadioError) -> Self {
        RobotRadioError::ReadDataError(err)
    }
}

unsafe impl<
        'a,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        const DEBUG_UART_QUEUES: bool,
    > Send for RobotRadio<'a, LEN_RX, LEN_TX, DEPTH_TX, DEPTH_RX, DEBUG_UART_QUEUES>
{
}

pub struct RobotRadio<
    'a,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    const DEBUG_UART_QUEUES: bool,
> {
    odin_driver: OdinW262<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, DEBUG_UART_QUEUES>,
    reset_pin: Output<'a>,
    use_flow_control: bool,
    peer: Option<PeerConnection>,
}

impl<
        'a,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        const DEBUG_UART_QUEUES: bool,
    > RobotRadio<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, DEBUG_UART_QUEUES>
{
    pub fn new(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_UART_QUEUES>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        reset_pin: Peri<'static, AnyPin>,
        use_flow_control: bool,
    ) -> RobotRadio<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, DEBUG_UART_QUEUES> {
        let reset_pin = Output::new(reset_pin, Level::High, Speed::Medium);
        let radio = OdinW262::new(read_queue, write_queue, uart);

        Self {
            odin_driver: radio,
            reset_pin,
            peer: None,
            use_flow_control,
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
        // highspeed_radio_uart_config.baudrate = 3_000_000;
        // highspeed_radio_uart_config.baudrate = 921_600;
        highspeed_radio_uart_config.stop_bits = StopBits::STOP1;
        highspeed_radio_uart_config.data_bits = DataBits::DataBits8;
        highspeed_radio_uart_config.parity = usart::Parity::ParityEven;

        highspeed_radio_uart_config
    }

    pub async fn connect_uart(&mut self) -> Result<(), RobotRadioError> {
        // were about to reset the radio, so we also need to reset the uart queue config to match the startup config
        if self
            .odin_driver
            .update_host_uart_config(self.get_startup_uart_config())
            .await
            .is_err()
        {
            defmt::debug!("failed to reset host uart to startup config.");
        }

        // reset the radio so we can listen for the startup event
        self.reset_pin.set_high();
        Timer::after(Duration::from_millis(1)).await;
        self.reset_pin.set_low();

        // wait until startup event is received
        if let Err(e) = self.odin_driver.wait_startup().await {
            defmt::debug!("error processing radio wait startup command");
            return Err(RobotRadioError::ConnectUartBadStartup(e));
        }
        defmt::trace!("increasing link speed");

        let baudrate = self.get_highspeed_uart_config().baudrate;
        if let Err(e) = self.odin_driver.set_echo(false).await {
            defmt::debug!("error disabling echo on radio");
            return Err(RobotRadioError::ConnectUartBadEcho(e));
        }

        if let Err(e) = self
            .odin_driver
            .config_uart(baudrate, self.use_flow_control, 8, true)
            .await
        {
            defmt::debug!("error increasing radio baud rate.");
            return Err(RobotRadioError::ConnectUartBadRadioConfigUpdate(e));
        }
        defmt::trace!("configured radio link speed");

        if self
            .odin_driver
            .update_host_uart_config(self.get_highspeed_uart_config())
            .await
            .is_err()
        {
            defmt::debug!("error increasing host baud rate.");
            return Err(RobotRadioError::ConnectUartBadHostConfigUpdate);
        }
        defmt::trace!("configured host link speed");

        // Datasheet says wait at least 40ms after UART config change
        Timer::after(Duration::from_millis(50)).await;

        // Datasheet says wait at least 50ms after entering data mode
        match self.odin_driver.enter_edm().await {
            Ok(got_edm_startup) => {
                defmt::trace!("entered edm at high link speed");

                if !got_edm_startup {
                    if let Err(e) = self.odin_driver.wait_edm_startup().await {
                        defmt::debug!("error waiting for EDM startup after uart baudrate increase");
                        return Err(RobotRadioError::ConnectUartNoEdmStartup(e));
                    }
                } else {
                    defmt::trace!("got EDM startup command");
                }
            }
            Err(e) => {
                defmt::debug!("error entering EDM mode after uart baudrate increase");
                return Err(RobotRadioError::ConnectUartCannotEnterEdm(e));
            }
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

    pub async fn connect_to_network(
        &mut self,
        wifi_credential: WifiCredential,
        robot_number: u8,
    ) -> Result<(), RobotRadioError> {
        // set radio hardware name enumeration
        let uid = uid::uid();
        let uid_u16 = (uid[1] as u16) << 8 | uid[0] as u16;

        let mut s = String::<25>::new();
        core::write!(
            &mut s,
            "A-Team Robot #{:02X} ({:04X})",
            robot_number,
            uid_u16
        )
        .unwrap();
        if let Err(e) = self.odin_driver.set_host_name(s.as_str()).await {
            defmt::trace!("could not set radio host name");
            return Err(RobotRadioError::ConnectWifiBadHostName(e));
        }

        // load the wifi network configuration into config slot 1
        let wifi_ssid = wifi_credential.get_ssid();
        let wifi_pass = WifiAuth::WPA {
            passphrase: wifi_credential.get_password(),
        };
        if let Err(e) = self.odin_driver.config_wifi(1, wifi_ssid, wifi_pass).await {
            defmt::trace!("could not configure wifi profile");
            return Err(RobotRadioError::ConnectWifiBadConfig(e));
        }

        // connect to config slot 1
        if let Err(e) = self.odin_driver.connect_wifi(1).await {
            defmt::trace!("could not connect to wifi");

            // can never configure a profile that "active" even when unconnected
            // we're not really in a known state with out a lot more effort
            // so ignore the result
            let _ = self.disconnect_network().await;

            return Err(RobotRadioError::ConnectWifiConnectionFailed(e));
        }

        // if we made it this far, we're connected
        Ok(())
    }

    pub async fn open_multicast(&mut self) -> Result<(), RobotRadioError> {
        match self
            .odin_driver
            .connect_peer(formatcp!(
                "udp://{MULTICAST_IP}:{MULTICAST_PORT}/?flags=1&local_port={LOCAL_PORT}"
            ))
            .await
        {
            Err(e) => {
                defmt::debug!("failed to connect peer");
                Err(RobotRadioError::OpenMulticastError(e))
            }
            Ok(peer) => {
                self.peer = Some(peer);
                Ok(())
            }
        }
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
                    Ok(ret) => Ok(Some(ret)),
                    Err(e) => {
                        defmt::error!(
                            "try read data failed after can read data reported data ready"
                        );
                        Err(RobotRadioError::ReadDataError(e))
                    }
                }
            } else {
                Ok(None)
            }
        } else {
            defmt::error!("peer was none");
            Err(RobotRadioError::PeerMissing)
        }
    }

    // ACK/NACK are encoded in _reserved[1] to occupy the same byte offset as the
    // old command_code field (byte 5 of RadioHeader).  CC_ACK=1, CC_NACK=2.
    pub async fn send_ack(&self, nack: bool) -> Result<(), RobotRadioError> {
        let header = RadioHeader {
            crc32: 0,
            _reserved: [0, if nack { 2 } else { 1 }],
            data_length: 0,
        };
        let header_bytes = unsafe {
            core::slice::from_raw_parts(
                &header as *const _ as *const u8,
                size_of::<RadioHeader>(),
            )
        };
        self.send_data(header_bytes)?;
        Ok(())
    }

    pub async fn wait_ack(&self, timeout: Duration) -> Result<bool, RobotRadioError> {
        let read_fut = self.read_data(|data| {
            if data.len() != size_of::<RadioHeader>() {
                return Err(RobotRadioError::SoftwareConnectAckHeaderInvalid);
            }
            let packet = unsafe { &*(data as *const _ as *const RadioHeader) };
            // _reserved[1] encodes the old command_code: CC_ACK=1, CC_NACK=2
            match packet._reserved[1] {
                1 => Ok(true),
                2 => Ok(false),
                _ => Err(RobotRadioError::SoftwareConnectAckHeaderInvalid),
            }
        });
        match select(read_fut, Timer::after(timeout)).await {
            Either::First(ret) => ret?,
            Either::Second(_) => Err(RobotRadioError::RequestTimedOut),
        }
    }

    pub async fn send_hello(&self, id: u8, team: TeamColor) -> Result<(), RobotRadioError> {
        use crate::git_version;

        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                _reserved: [0u8; 2],
                data_length: (RADIO_DATA_TAG_PAD + size_of::<HelloRequest>()) as u16,
            },
            data: RadioData::HelloRequest(HelloRequest {
                robot_id: id,
                color: match team {
                    TeamColor::Yellow => PacketTeamColor::Yellow,
                    TeamColor::Blue => PacketTeamColor::Blue,
                },
                flags: DiscoveryFlags::default()
                    .with_coms_repo_dirty(git_version::COMS_DIRTY)
                    .with_controls_repo_dirty(git_version::CONTROLS_DIRTY)
                    .with_firmware_repo_dirty(git_version::FIRMWARE_DIRTY),
                _reserved: [0u8; 1],
                coms_hash: git_version::COMS_HASH,
                controls_hash: git_version::CONTROLS_HASH,
                firmware_hash: git_version::FIRMWARE_HASH,
            }),
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioHeader>() + RADIO_DATA_TAG_PAD + size_of::<HelloRequest>(),
            )
        };
        self.send_data(packet_bytes)?;
        Ok(())
    }

    pub fn send_telemetry(&self, telemetry: BasicTelemetry) -> Result<(), RobotRadioError> {
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                _reserved: [0u8; 2],
                data_length: (RADIO_DATA_TAG_PAD + size_of::<BasicTelemetry>()) as u16,
            },
            data: RadioData::Telemetry(telemetry),
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioHeader>() + RADIO_DATA_TAG_PAD + size_of::<BasicTelemetry>(),
            )
        };
        self.send_data(packet_bytes)?;
        Ok(())
    }

    pub async fn send_control_debug_telemetry(
        &self,
        telemetry: ExtendedTelemetry,
    ) -> Result<(), RobotRadioError> {
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                _reserved: [0u8; 2],
                data_length: (RADIO_DATA_TAG_PAD + size_of::<ExtendedTelemetry>()) as u16,
            },
            data: RadioData::ControlDebugTelemetry(telemetry),
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioHeader>() + RADIO_DATA_TAG_PAD + size_of::<ExtendedTelemetry>(),
            )
        };
        self.send_data(packet_bytes)?;
        Ok(())
    }

    pub async fn send_parameter_response(
        &self,
        parameter_cmd: ParameterCommand,
    ) -> Result<(), RobotRadioError> {
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                _reserved: [0u8; 2],
                data_length: (RADIO_DATA_TAG_PAD + size_of::<ParameterCommand>()) as u16,
            },
            data: RadioData::RobotParameterCommand(parameter_cmd),
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioHeader>() + RADIO_DATA_TAG_PAD + size_of::<ParameterCommand>(),
            )
        };
        self.send_data(packet_bytes)?;
        Ok(())
    }

    pub async fn send_error_telemetry(
        &self,
        error_telemetry: ErrorTelemetry,
    ) -> Result<(), RobotRadioError> {
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                _reserved: [0u8; 2],
                data_length: (RADIO_DATA_TAG_PAD + size_of::<ErrorTelemetry>()) as u16,
            },
            data: RadioData::ErrorTelemetry(error_telemetry),
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioHeader>() + RADIO_DATA_TAG_PAD + size_of::<ErrorTelemetry>(),
            )
        };
        self.send_data(packet_bytes)?;
        Ok(())
    }

    pub async fn wait_hello(&self, timeout: Duration) -> Result<HelloResponse, RobotRadioError> {
        let read_fut = self.read_data(|data| {
            const PACKET_SIZE: usize =
                size_of::<RadioHeader>() + RADIO_DATA_TAG_PAD + size_of::<HelloResponse>();
            if data.len() != PACKET_SIZE {
                defmt::trace!(
                    "invalid hello response - got packet size {}, expected {}",
                    data.len(),
                    PACKET_SIZE
                );
                return Err(RobotRadioError::SoftwareHelloHeaderInvalid);
            }

            let tag_byte = data[size_of::<RadioHeader>()];
            if tag_byte != 22 {  // RadioData::HelloResponse discriminant
                defmt::trace!(
                    "invalid hello response - got tag byte {}, expected 22",
                    tag_byte
                );
                return Err(RobotRadioError::SoftwareHelloHeaderInvalid);
            }

            let mut data_copy = [0u8; size_of::<RadioPacket>()];
            data_copy[0..PACKET_SIZE].clone_from_slice(&data[0..PACKET_SIZE]);

            let packet = unsafe { &*(data_copy.as_ptr() as *const RadioPacket) };
            let RadioData::HelloResponse(resp) = packet.data else {
                return Err(RobotRadioError::SoftwareHelloHeaderInvalid);
            };
            Ok(resp)
        });

        match select(read_fut, Timer::after(timeout)).await {
            Either::First(ret) => ret?,
            Either::Second(_) => {
                defmt::trace!("software hello response timeout");
                Err(RobotRadioError::SoftwareHelloResponseTimeout)
            }
        }
    }

    pub fn parse_data_packet(&self, data: &[u8]) -> Result<DataPacket, RobotRadioError> {
        const HEADER_SIZE: usize = size_of::<RadioHeader>();
        const CONTROL_PACKET_SIZE: usize =
            HEADER_SIZE + RADIO_DATA_TAG_PAD + size_of::<BasicControl>();
        const PARAMETER_PACKET_SIZE: usize =
            HEADER_SIZE + RADIO_DATA_TAG_PAD + size_of::<ParameterCommand>();

        if data.len() < HEADER_SIZE + RADIO_DATA_TAG_PAD {
            defmt::error!("packet too short: {}", data.len());
            return Err(RobotRadioError::PacketTypeUnknown);
        }

        let tag_byte = data[HEADER_SIZE];

        match tag_byte {
            61 => {  // RadioData::Control discriminant
                if data.len() != CONTROL_PACKET_SIZE {
                    return Err(RobotRadioError::ControlPacketDecodeInvalid);
                }
                let mut data_copy = [0u8; size_of::<RadioPacket>()];
                data_copy[0..CONTROL_PACKET_SIZE].clone_from_slice(&data[0..CONTROL_PACKET_SIZE]);
                let packet = unsafe { &*(data_copy.as_ptr() as *const RadioPacket) };
                let RadioData::Control(ctrl) = packet.data else {
                    return Err(RobotRadioError::ControlPacketDecodeInvalid);
                };
                Ok(DataPacket::BasicControl(ctrl))
            }
            43 => {  // RadioData::RobotParameterCommand discriminant
                if data.len() != PARAMETER_PACKET_SIZE {
                    return Err(RobotRadioError::ParameterPacketDecodeInvalid);
                }
                let mut data_copy = [0u8; size_of::<RadioPacket>()];
                data_copy[0..PARAMETER_PACKET_SIZE].clone_from_slice(&data[0..PARAMETER_PACKET_SIZE]);
                let packet = unsafe { &*(data_copy.as_ptr() as *const RadioPacket) };
                let RadioData::RobotParameterCommand(param) = packet.data else {
                    return Err(RobotRadioError::ParameterPacketDecodeInvalid);
                };
                Ok(DataPacket::ParameterCommand(param))
            }
            _ => {
                defmt::error!("unknown packet tag: {}", tag_byte);
                Err(RobotRadioError::PacketTypeUnknown)
            }
        }
    }

    pub async fn read_packet(&self) -> Result<DataPacket, RobotRadioError> {
        self.read_data(|data| self.parse_data_packet(data)).await?
    }

    pub fn read_packet_nonblocking(&self) -> Result<Option<DataPacket>, RobotRadioError> {
        let res = self.read_data_nonblocking(|data| self.parse_data_packet(data));

        match res {
            Ok(res) => {
                match res {
                    Some(pkt) => {
                        match pkt {
                            Ok(pkt) => Ok(Some(pkt)),
                            Err(e) => {
                                // we got data that was a valid EDM DataPacket, but couldn't parse it
                                // into any known A-Team packet format
                                defmt::warn!("got EDM packet but wasn't A-Team: {}", e);
                                Err(e)
                            }
                        }
                    }
                    None => Ok(None),
                }
            }
            Err(e) => {
                defmt::error!("radio in invalid state: {}", e);
                Err(e)
            }
        }
    }
}
