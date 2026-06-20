use ateam_common_packets::bindings::{
    self, BasicControl, BasicTelemetry, CommandCode, ErrorTelemetry, ExtendedTelemetry,
    HelloRequest, HelloResponse, ParameterCommand, RadioData, RadioHeader, RadioPacket,
};
use ateam_common_packets::radio::DataPacket;
use ateam_lib_stm32::drivers::radio::nora_w36x::{
    NoraRadioError, NoraW36x, SocketConnection, WifiAuth, DataMode,
};
use ateam_lib_stm32::drivers::radio::w36x::at_protocol::SocketProtocol;
use ateam_lib_stm32::uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue};
use core::fmt::Write;
use core::mem::size_of;
use credentials::WifiCredential;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::usart::{self, DataBits, Parity, StopBits};
use embassy_stm32::{uid, Peri};
use embassy_time::{Duration, Instant, Timer};
use heapless::String;

use defmt::Format;

use super::radio_robot::TeamColor;

const MULTICAST_IP: &str = "224.4.20.69";
const MULTICAST_PORT: u16 = 42069;

#[derive(Clone, Copy, PartialEq, Debug, Format)]
pub enum RobotRadioNoraError {
    DriverError(NoraRadioError),

    RequestTimedOut,

    ConnectUartBadStartup,
    ConnectUartBadEcho,
    ConnectUartBadRadioConfigUpdate,
    ConnectUartBadHostConfigUpdate,

    ConnectWifiBadHostName,
    ConnectWifiBadConfig,
    ConnectWifiConnectionFailed,

    OpenMulticastError,
    OpenSocketError,

    DisconnectFailed,

    SocketMissing,

    SoftwareConnectAckHeaderInvalid,
    SoftwareHelloHeaderInvalid,

    ControlPacketDecodeInvalid,
    ParameterPacketDecodeInvalid,
    PacketTypeUnknown,
}

impl From<NoraRadioError> for RobotRadioNoraError {
    fn from(err: NoraRadioError) -> Self {
        RobotRadioNoraError::DriverError(err)
    }
}

unsafe impl<
        'a,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        const DEBUG_UART_QUEUES: bool,
    > Send for RobotRadioNora<'a, LEN_RX, LEN_TX, DEPTH_TX, DEPTH_RX, DEBUG_UART_QUEUES>
{
}

pub struct RobotRadioNora<
    'a,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    const DEBUG_UART_QUEUES: bool,
> {
    nora_driver: NoraW36x<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, DEBUG_UART_QUEUES>,
    reset_pin: Output<'a>,
    #[allow(dead_code)]
    use_flow_control: bool,
    socket: Option<SocketConnection>,
}

impl<
        'a,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        const DEBUG_UART_QUEUES: bool,
    > RobotRadioNora<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, DEBUG_UART_QUEUES>
{
    pub fn new(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_UART_QUEUES>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        reset_pin: Peri<'static, AnyPin>,
        use_flow_control: bool,
    ) -> RobotRadioNora<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, DEBUG_UART_QUEUES> {
        let reset_pin = Output::new(reset_pin, Level::High, Speed::Medium);
        let radio = NoraW36x::new(read_queue, write_queue, uart);

        Self {
            nora_driver: radio,
            reset_pin,
            socket: None,
            use_flow_control,
        }
    }

    pub fn get_startup_uart_config(&self) -> usart::Config {
        let mut startup_radio_uart_config = usart::Config::default();
        startup_radio_uart_config.baudrate = 3_000_000;
        startup_radio_uart_config.data_bits = DataBits::DataBits8;
        startup_radio_uart_config.stop_bits = StopBits::STOP1;
        startup_radio_uart_config.parity = Parity::ParityNone;

        startup_radio_uart_config
    }

    pub fn get_highspeed_uart_config(&self) -> usart::Config {
        let mut highspeed_radio_uart_config = usart::Config::default();
        highspeed_radio_uart_config.baudrate = 3_000_000;
        highspeed_radio_uart_config.stop_bits = StopBits::STOP1;
        highspeed_radio_uart_config.data_bits = DataBits::DataBits8;
        // NORA-W36 AT+USYUS only configures baudrate and flow control;
        // data format is fixed at 8N1, so parity must stay None
        highspeed_radio_uart_config.parity = usart::Parity::ParityNone;

        highspeed_radio_uart_config
    }

    pub async fn connect_uart(&mut self) -> Result<(), RobotRadioNoraError> {
        // reset host uart config to match the startup config before resetting the radio
        // if self
        //     .nora_driver
        //     .update_host_uart_config(self.get_startup_uart_config())
        //     .await
        //     .is_err()
        // {
        //     defmt::debug!("failed to reset host uart to startup config.");
        // }

        defmt::trace!("Will reset the radio for baseline.");
        // reset the radio so we can listen for the startup event
        // PD7 drives an inverting transistor to the radio's active-low nRESET:
        //   PD7 Low  → nRESET High (radio running)
        //   PD7 High → nRESET Low  (radio in reset)
        // Ensure a clean reset pulse: release first, then assert, then release
        self.reset_pin.set_low();
        Timer::after(Duration::from_millis(10)).await;
        self.reset_pin.set_high();
        // t_RESET is Minimum 1ms (4.2.3 in the datasheet), but we'll give it a bit more time to be safe
        Timer::after(Duration::from_millis(10)).await;
        self.reset_pin.set_low();
        defmt::trace!("Reset released, waiting for radio boot.");

        // Wait for radio to boot (t_STARTUP is 2.3s per datasheet section 4.2.3)
        defmt::trace!("Waiting for radio startup with timeout.");
        match select(
            self.nora_driver.wait_startup(),
            Timer::after(Duration::from_millis(5000)),
        )
        .await
        {
            Either::First(res) => {
                if res.is_err() {
                    defmt::debug!("radio startup packet wait failed");
                    return Err(RobotRadioNoraError::ConnectUartBadStartup);
                }
            }
            Either::Second(_) => {
                defmt::debug!("radio startup packet wait timed out");
                return Err(RobotRadioNoraError::ConnectUartBadStartup);
            }
        }

        defmt::trace!("Radio UART Startup received");

        //defmt::trace!("increasing link speed");
        //// NORA config_uart only takes baudrate and flow control (no data_bits/parity params)
        //if self
        //    .nora_driver
        //    .config_uart(baudrate, self.use_flow_control)
        //    .await
        //    .is_err()
        //{
        //    defmt::debug!("error increasing radio baud rate.");
        //    return Err(RobotRadioNoraError::ConnectUartBadRadioConfigUpdate);
        //}
        //defmt::trace!("configured radio link speed");

        // TODO Make this happen ahead of time.
        //if self
        //    .nora_driver
        //    .update_host_uart_config(self.get_highspeed_uart_config())
        //    .await
        //    .is_err()
        //{
        //    defmt::debug!("error increasing host baud rate.");
        //    return Err(RobotRadioNoraError::ConnectUartBadHostConfigUpdate);
        //}
        //defmt::trace!("configured host link speed");

        // Allow the radio time to stabilize after baud rate switch
        //Timer::after(Duration::from_millis(50)).await;

        // NORA-W36x does not use EDM mode - it stays in AT command mode.
        // Enable direct binary mode for inline data delivery in +UESODB/+UESODBF events
        // (replaces the 2-step buffered approach of +UESODA then AT+USORB).
        if self.nora_driver.set_socket_receive_mode(DataMode::DirectBinaryMode).await.is_err() {
            defmt::debug!("error setting direct binary receive mode");
            return Err(RobotRadioNoraError::ConnectUartBadRadioConfigUpdate);
        }
        defmt::trace!("configured direct binary receive mode");

        Ok(())
    }

    pub async fn disconnect_network(&mut self) -> Result<(), RobotRadioNoraError> {
        let mut had_error = false;
        if let Some(socket) = self.socket.take() {
            defmt::debug!("closing socket..");
            if self
                .nora_driver
                .close_socket(socket.socket_id)
                .await
                .is_err()
            {
                defmt::warn!("failed to close socket on network dc");
                had_error = true;
            } else {
                defmt::debug!("closed socket.")
            }
        }

        defmt::debug!("closing wifi.");
        // NORA disconnect_wifi takes no config_id parameter
        if self.nora_driver.disconnect_wifi().await.is_err() {
            defmt::warn!("failed to disconnect network.");
            had_error = true;
        } else {
            defmt::debug!("disconnected wifi.")
        }

        if had_error {
            Err(RobotRadioNoraError::DisconnectFailed)
        } else {
            Ok(())
        }
    }

    pub async fn connect_to_network(
        &mut self,
        wifi_credential: WifiCredential,
        robot_number: u8,
    ) -> Result<(), RobotRadioNoraError> {
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
        if self.nora_driver.set_host_name(s.as_str()).await.is_err() {
            defmt::trace!("could not set radio host name");
            return Err(RobotRadioNoraError::ConnectWifiBadHostName);
        }

        // Load the wifi network configuration (only supports one profile, at index 0)
        let wifi_ssid = wifi_credential.get_ssid();
        let wifi_pass = WifiAuth::WPA {
            passphrase: wifi_credential.get_password(),
        };
        if self
            .nora_driver
            .config_wifi(wifi_ssid, wifi_pass)
            .await
            .is_err()
        {
            defmt::trace!("could not configure wifi profile");
            return Err(RobotRadioNoraError::ConnectWifiBadConfig);
        }

        // Connect to config slot (only supports one, at index 0)
        if self.nora_driver.connect_wifi().await.is_err() {
            defmt::trace!("could not connect to wifi");

            let _ = self.disconnect_network().await;

            return Err(RobotRadioNoraError::ConnectWifiConnectionFailed);
        }

        // if we made it this far, we're connected
        Ok(())
    }

    pub async fn open_multicast(&mut self) -> Result<(), RobotRadioNoraError> {
        // Create an unconnected UDP socket. Connecting to the multicast peer would
        // cause the module to filter incoming packets to that peer only, dropping
        // HelloResponse from the software's unicast address. Instead, send_hello uses
        // send_data_to with an explicit multicast destination so the socket stays
        // unconnected and receives from any source.
        let socket_id = self
            .nora_driver
            .create_socket(SocketProtocol::UDP)
            .await
            .map_err(|_| RobotRadioNoraError::OpenSocketError)?;

        self.socket = Some(SocketConnection { socket_id });
        Ok(())
    }

    pub async fn open_unicast(
        &mut self,
        ipv4: [u8; 4],
        port: u16,
    ) -> Result<(), RobotRadioNoraError> {
        let mut addr = String::<16>::new();
        core::write!(&mut addr, "{}.{}.{}.{}", ipv4[0], ipv4[1], ipv4[2], ipv4[3],).unwrap();

        let socket_id = self
            .nora_driver
            .create_socket(SocketProtocol::UDP)
            .await
            .map_err(|_| RobotRadioNoraError::OpenSocketError)?;

        let socket = self
            .nora_driver
            .connect_socket(socket_id, addr.as_str(), port, SocketProtocol::UDP)
            .await?;

        self.socket = Some(socket);
        Ok(())
    }

    pub async fn close_peer(&mut self) -> Result<(), RobotRadioNoraError> {
        if let Some(socket) = &self.socket {
            self.nora_driver.close_socket(socket.socket_id).await?;
            self.socket = None;
            Ok(())
        } else {
            Err(RobotRadioNoraError::SocketMissing)
        }
    }

    pub async fn send_data(&self, data: &[u8]) -> Result<(), RobotRadioNoraError> {
        if let Some(socket) = &self.socket {
            self.nora_driver.send_data(socket.socket_id, data).await?;
            Ok(())
        } else {
            Err(RobotRadioNoraError::SocketMissing)
        }
    }

    async fn send_data_to(
        &self,
        addr: &str,
        port: u16,
        data: &[u8],
    ) -> Result<(), RobotRadioNoraError> {
        if let Some(socket) = &self.socket {
            self.nora_driver
                .send_data_to(socket.socket_id, addr, port, data)
                .await?;
            Ok(())
        } else {
            Err(RobotRadioNoraError::SocketMissing)
        }
    }

    pub async fn read_data<RET, FN>(&'a self, fn_read: FN) -> Result<RET, RobotRadioNoraError>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        if let Some(socket) = &self.socket {
            let ret = self
                .nora_driver
                .read_data(socket.socket_id, size_of::<RadioPacket>() as u16, fn_read)
                .await?;
            Ok(ret)
        } else {
            Err(RobotRadioNoraError::SocketMissing)
        }
    }

    fn read_data_nonblocking<RET, FN>(
        &'a self,
        fn_read: FN,
    ) -> Result<Option<RET>, RobotRadioNoraError>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        if self.socket.is_some() {
            if self.nora_driver.can_read_data() {
                // In direct binary mode, +UESODB/+UESODBF events contain inline data.
                // We can read and return it synchronously — no async AT+USORB needed.
                match self.nora_driver.try_read_data_binary(fn_read) {
                    Ok(Some(ret)) => Ok(Some(ret)),
                    Ok(None) => Ok(None),
                    Err(NoraRadioError::ReadLowLevelBufferEmpty) => Ok(None),
                    Err(NoraRadioError::ReadDataInvalid) => {
                        // Non-data event in the queue (e.g. connection event) — skip it
                        defmt::trace!("non-data event in queue, skipping");
                        Ok(None)
                    }
                    Err(e) => {
                        defmt::trace!("try_read_data_binary failed: {:?}", e);
                        Err(RobotRadioNoraError::DriverError(e))
                    }
                }
            } else {
                Ok(None)
            }
        } else {
            defmt::trace!("socket was none");
            Err(RobotRadioNoraError::SocketMissing)
        }
    }

    pub async fn send_ack(&self, nack: bool) -> Result<(), RobotRadioNoraError> {
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                _reserved: 0,
                command_code: if nack {
                    CommandCode::CC_NACK
                } else {
                    CommandCode::CC_ACK
                },
                data_length: 0,
            },
            data: unsafe { core::mem::zeroed() },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioData>(),
            )
        };
        self.send_data(packet_bytes).await?;

        Ok(())
    }

    pub async fn wait_ack(&self, timeout: Duration) -> Result<bool, RobotRadioNoraError> {
        let read_fut = self.read_data(|data| {
            if data.len() != size_of::<RadioPacket>() - size_of::<RadioData>() {
                return Err(RobotRadioNoraError::SoftwareConnectAckHeaderInvalid);
            }
            let packet = unsafe { &*(data as *const _ as *const RadioPacket) };

            match packet.header.command_code {
                CommandCode::CC_ACK => Ok(true),
                CommandCode::CC_NACK => Ok(false),
                _ => Err(RobotRadioNoraError::SoftwareConnectAckHeaderInvalid),
            }
        });
        match select(read_fut, Timer::after(timeout)).await {
            Either::First(ret) => ret?,
            Either::Second(_) => Err(RobotRadioNoraError::RequestTimedOut),
        }
    }

    pub async fn send_hello(&self, id: u8, team: TeamColor) -> Result<(), RobotRadioNoraError> {
        use crate::git_version;

        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                command_code: CommandCode::CC_HELLO_REQ,
                _reserved: 0,
                data_length: size_of::<HelloRequest>() as u16,
            },
            data: RadioData {
                hello_request: HelloRequest {
                    robot_id: id,
                    color: match team {
                        TeamColor::Yellow => bindings::TeamColor::TC_YELLOW,
                        TeamColor::Blue => bindings::TeamColor::TC_BLUE,
                    },
                    _bitfield_1: HelloRequest::new_bitfield_1(
                        git_version::COMS_DIRTY.into(),
                        git_version::CONTROLS_DIRTY.into(),
                        git_version::FIRMWARE_DIRTY.into(),
                        0,
                    ),
                    _bitfield_align_1: Default::default(),
                    reserved: Default::default(),
                    coms_hash: git_version::COMS_HASH,
                    controls_hash: git_version::CONTROLS_HASH,
                    firmware_hash: git_version::FIRMWARE_HASH,
                },
            },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioData>() + size_of::<HelloRequest>(),
            )
        };
        self.send_data_to(MULTICAST_IP, MULTICAST_PORT, packet_bytes).await?;

        Ok(())
    }

    pub async fn send_telemetry(
        &self,
        telemetry: BasicTelemetry,
    ) -> Result<(), RobotRadioNoraError> {
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                _reserved: 0,
                command_code: CommandCode::CC_TELEMETRY,
                data_length: size_of::<BasicTelemetry>() as u16,
            },
            data: RadioData {
                telemetry: telemetry,
            },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioData>() + size_of::<BasicTelemetry>(),
            )
        };
        self.send_data(packet_bytes).await?;

        Ok(())
    }

    pub async fn send_control_debug_telemetry(
        &self,
        telemetry: ExtendedTelemetry,
    ) -> Result<(), RobotRadioNoraError> {
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                _reserved: 0,
                command_code: CommandCode::CC_CONTROL_DEBUG_TELEMETRY,
                data_length: size_of::<ExtendedTelemetry>() as u16,
            },
            data: RadioData {
                extended_telemetry: telemetry,
            },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioData>() + size_of::<ExtendedTelemetry>(),
            )
        };
        self.send_data(packet_bytes).await?;

        Ok(())
    }

    pub async fn send_parameter_response(
        &self,
        parameter_cmd: ParameterCommand,
    ) -> Result<(), RobotRadioNoraError> {
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                _reserved: 0,
                command_code: CommandCode::CC_ROBOT_PARAMETER_COMMAND,
                data_length: size_of::<ParameterCommand>() as u16,
            },
            data: RadioData {
                robot_parameter_command: parameter_cmd,
            },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioData>() + size_of::<ParameterCommand>(),
            )
        };
        self.send_data(packet_bytes).await?;

        Ok(())
    }

    pub async fn send_error_telemetry(
        &self,
        error_telemetry: ErrorTelemetry,
    ) -> Result<(), RobotRadioNoraError> {
        let packet = RadioPacket {
            header: RadioHeader {
                crc32: 0,
                _reserved: 0,
                command_code: CommandCode::CC_ERROR_TELEMETRY,
                data_length: size_of::<ParameterCommand>() as u16,
            },
            data: RadioData { error_telemetry },
        };
        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                size_of::<RadioPacket>() - size_of::<RadioData>() + size_of::<ErrorTelemetry>(),
            )
        };
        self.send_data(packet_bytes).await?;

        Ok(())
    }

    pub async fn wait_hello(
        &self,
        timeout: Duration,
    ) -> Result<HelloResponse, RobotRadioNoraError> {
        let deadline = Instant::now() + timeout;

        loop {
            if Instant::now() >= deadline {
                return Err(RobotRadioNoraError::RequestTimedOut);
            }

            if !self.nora_driver.can_read_data() {
                Timer::after_millis(1).await;
                continue;
            }

            match self.nora_driver.try_read_data_binary(|data| {
                defmt::trace!("wait_hello read data: {:?}", data);
                const PACKET_SIZE: usize =
                    size_of::<RadioPacket>() - size_of::<RadioData>() + size_of::<HelloResponse>();
                if data.len() != PACKET_SIZE {
                    return Err(RobotRadioNoraError::SoftwareHelloHeaderInvalid);
                }

                let mut data_copy = [0u8; size_of::<RadioPacket>()];
                data_copy[0..PACKET_SIZE].clone_from_slice(&data[0..PACKET_SIZE]);

                let packet = unsafe { &*(&data_copy as *const _ as *const RadioPacket) };

                if packet.header.command_code != CommandCode::CC_HELLO_RESP {
                    return Err(RobotRadioNoraError::SoftwareHelloHeaderInvalid);
                }

                Ok(unsafe { packet.data.hello_response })
            }) {
                Ok(Some(result)) => return result,
                Ok(None) => {
                    Timer::after_millis(1).await;
                    continue;
                }
                Err(NoraRadioError::ReadLowLevelBufferEmpty) => {
                    Timer::after_millis(1).await;
                    continue;
                }
                Err(NoraRadioError::ReadDataInvalid) => {
                    defmt::trace!("wait_hello: non-data event in queue, skipping");
                    continue;
                }
                Err(e) => return Err(RobotRadioNoraError::DriverError(e)),
            }
        }
    }

    pub fn parse_data_packet(&self, data: &[u8]) -> Result<DataPacket, RobotRadioNoraError> {
        const CONTROL_PACKET_SIZE: usize =
            size_of::<RadioPacket>() - size_of::<RadioData>() + size_of::<BasicControl>();
        const PARAMERTER_PACKET_SIZE: usize =
            size_of::<RadioPacket>() - size_of::<RadioData>() + size_of::<ParameterCommand>();

        if data.len() == CONTROL_PACKET_SIZE {
            let mut data_copy = [0u8; size_of::<RadioPacket>()];
            data_copy[0..CONTROL_PACKET_SIZE].clone_from_slice(&data[0..CONTROL_PACKET_SIZE]);

            let packet = unsafe { &*(&data_copy as *const _ as *const RadioPacket) };

            if packet.header.command_code != CommandCode::CC_CONTROL {
                return Err(RobotRadioNoraError::ControlPacketDecodeInvalid);
            }

            Ok(unsafe { DataPacket::BasicControl(packet.data.control) })
        } else if data.len() == PARAMERTER_PACKET_SIZE {
            let mut data_copy = [0u8; size_of::<RadioPacket>()];
            data_copy[0..PARAMERTER_PACKET_SIZE].clone_from_slice(&data[0..PARAMERTER_PACKET_SIZE]);

            let packet = unsafe { &*(&data_copy as *const _ as *const RadioPacket) };

            if packet.header.command_code != CommandCode::CC_ROBOT_PARAMETER_COMMAND {
                return Err(RobotRadioNoraError::ParameterPacketDecodeInvalid);
            }

            Ok(unsafe { DataPacket::ParameterCommand(packet.data.robot_parameter_command) })
        } else {
            return Err(RobotRadioNoraError::PacketTypeUnknown);
        }
    }

    pub async fn read_packet(&self) -> Result<DataPacket, RobotRadioNoraError> {
        self.read_data(|data| self.parse_data_packet(data)).await?
    }

    pub fn read_packet_nonblocking(&self) -> Result<Option<DataPacket>, ()> {
        let res = self.read_data_nonblocking(|data| self.parse_data_packet(data));

        match res {
            Ok(res) => match res {
                Some(pkt) => match pkt {
                    Ok(pkt) => {
                        return Ok(Some(pkt));
                    }
                    Err(_) => {
                        defmt::debug!("got AT packet but wasn't A-Team");
                        return Err(());
                    }
                },
                None => return Ok(None),
            },
            Err(err_res) => {
                defmt::debug!("radio in invalid state {:?}", err_res);
                return Err(());
            }
        }
    }
}
