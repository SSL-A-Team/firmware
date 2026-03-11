use core::fmt::Write;
use defmt::Format;
use embassy_futures::select::select;
use embassy_stm32::usart;
use embassy_time::Timer;
use heapless::String;

use crate::queue;
use crate::uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue};

use super::at_protocol::{AtPacketError, ATEvent, ATResponse, SocketProtocol};

/// Parsed AT packet from UART — either a command response or an unsolicited result code (URC).
/// NORA-W36 is always in AT command mode (no EDM).
#[derive(defmt::Format)]
pub enum NoraPacket<'a> {
    Response(ATResponse<'a>),
    Event(ATEvent<'a>),
}

#[derive(Copy, Clone, PartialEq, Debug, Format)]
pub enum NoraRadioError {
    CommandConstructionFailed,
    AtPacketError(AtPacketError),
    SendCommandLowLevelBufferFull,
    ReadDataInvalid,
    ReadLowLevelBufferEmpty,
    ReadLowLevelBufferBusy,
    AtEventUnsupported,
    AuthModeUnsupported,
    OperationTimedOut,
    SocketCreationFailed,
    SocketConnectionFailed,
    SocketCloseFailed,
    SocketWriteFailed,
    SocketReadFailed,
}

impl From<AtPacketError> for NoraRadioError {
    fn from(err: AtPacketError) -> Self {
        NoraRadioError::AtPacketError(err)
    }
}

#[allow(dead_code)]
pub enum WifiAuth<'a> {
    Open,
    WPA { passphrase: &'a str },
    LEAP,
    PEAP,
    EAPTLS,
    // TODO (W36): W36 supports WPA3/SAE via AT+UWSSS=<handle>,<passphrase>
}

pub struct SocketConnection {
    pub socket_id: u8,
}

pub struct NoraW36x<
    'a,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    const DEBUG_UART_QUEUES: bool,
> {
    reader: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_UART_QUEUES>,
    writer: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
    uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
}

impl<
        'a,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        const DEBUG_UART_QUEUES: bool,
    > NoraW36x<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, DEBUG_UART_QUEUES>
{
    pub fn new(
        reader: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_UART_QUEUES>,
        writer: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
    ) -> Self {
        Self {
            reader,
            writer,
            uart,
        }
    }

    pub async fn update_host_uart_config(&self, config: usart::Config) -> Result<(), ()> {
        self.uart.update_uart_config(config).await
    }

    pub async fn wait_startup(&self) -> Result<(), NoraRadioError> {
        self.reader
            .dequeue(|buf| {
                if let NoraPacket::Response(ATResponse::Other("+STARTUP")) = self.parse_packet(buf)? {
                    Ok(())
                } else {
                    Err(NoraRadioError::ReadDataInvalid)
                }
            })
            .await
    }

    pub async fn set_echo(&self, echo_on: bool) -> Result<(), NoraRadioError> {
        let echo_on = if echo_on { '1' } else { '0' };
        let mut str: String<4> = String::new();
        write!(&mut str, "ATE{echo_on}").unwrap();
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        Ok(())
    }

    /// Configure UART on the NORA-W36 module.
    /// Uses AT+USYUS=<baud_rate>[,<flow_control>[,<change_after_confirm>]]
    ///   - flow_control: 0=disabled, 1=CTS/RTS (default)
    ///   - change_after_confirm: 0=change now (default), 1=change after reboot
    /// Note: data_bits, stop_bits, and parity are not configurable on W36.
    pub async fn config_uart(
        &self,
        baudrate: u32,
        flow_control: bool,
    ) -> Result<(), NoraRadioError> {
        let mut str: String<28> = String::new();
        let flow_control = if flow_control { 1 } else { 0 };
        write!(&mut str, "AT+USYUS={baudrate},{flow_control}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        Ok(())
    }

    /// Set the host name on the NORA-W36 module.
    /// Uses AT+UWHN="<host_name>"
    pub async fn set_host_name(&self, host_name: &str) -> Result<(), NoraRadioError> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UWHN=\"{host_name}\"")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        defmt::trace!("host configuration string: {}", str.as_str());
        self.send_command(str.as_str()).await?;
        defmt::trace!("sent configuration command");
        self.read_ok().await?;
        defmt::trace!("read OK");

        Ok(())
    }

    /// Configure WiFi on the NORA-W36 module.
    /// Uses separate commands per configuration aspect:
    ///   1. Set SSID:     AT+UWSCP=<config_id>,<ssid>
    ///   2. Set security:
    ///      - Open:       AT+UWSSO=<config_id>
    ///      - WPA/WPA2:   AT+UWSSW=<config_id>,<passphrase>
    pub async fn config_wifi(
        &self,
        config_id: u8,
        ssid: &str,
        auth: WifiAuth<'_>,
    ) -> Result<(), NoraRadioError> {
        // Set SSID
        let mut str: String<64> = String::new();
        write!(str, "AT+UWSCP={config_id},\"{ssid}\"")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;

        // Set authentication
        str.clear();
        match auth {
            WifiAuth::Open => {
                write!(str, "AT+UWSSO={config_id}")
                    .or(Err(NoraRadioError::CommandConstructionFailed))?;
                self.send_command(str.as_str()).await?;
                self.read_ok().await?;
            }
            WifiAuth::WPA { passphrase } => {
                write!(str, "AT+UWSSW={config_id},\"{passphrase}\"")
                    .or(Err(NoraRadioError::CommandConstructionFailed))?;
                self.send_command(str.as_str()).await?;
                self.read_ok().await?;
            }
            _ => return Err(NoraRadioError::AuthModeUnsupported),
        }

        Ok(())
    }

    /// Disconnect WiFi on the NORA-W36 module.
    /// Uses AT+UWSDC (disconnects active WiFi station connection).
    pub async fn disconnect_wifi(&self) -> Result<(), NoraRadioError> {
        self.send_command("AT+UWSDC").await?;
        self.read_ok().await?;

        match select(
            async move {
                let mut run = true;
                while run {
                    let _ = self
                        .reader
                        .dequeue(|buf| {
                            let packet = self.parse_packet(buf);
                            if let Ok(packet) = packet {
                                if let NoraPacket::Event(ATEvent::StationNetworkDown) = packet
                                {
                                    defmt::debug!("got station network down event.");
                                    run = false;
                                } else if let NoraPacket::Event(ATEvent::WifiLinkDown {
                                    wlan_handle: _,
                                    reason: _,
                                }) = packet
                                {
                                    defmt::debug!("got wifi link down event.");
                                    run = false;
                                } else if let NoraPacket::Response(ATResponse::Ok(_)) = packet {
                                    run = false;
                                } else {
                                    defmt::warn!("got unexpected packet: {}", packet);
                                }
                            } else {
                                defmt::warn!("parsed invalid packet");
                            }
                        })
                        .await;
                }
            },
            Timer::after_millis(2500),
        )
        .await
        {
            embassy_futures::select::Either::First(_) => Ok(()),
            embassy_futures::select::Either::Second(_) => {
                defmt::warn!("disconnect timed out");
                Err(NoraRadioError::OperationTimedOut)
            }
        }
    }

    /// Connect to WiFi on the NORA-W36 module.
    /// Uses AT+UWSC=<config_id> to trigger connection using previously configured params.
    pub async fn connect_wifi(&self, config_id: u8) -> Result<(), NoraRadioError> {
        let mut str: String<16> = String::new();
        write!(str, "AT+UWSC={config_id}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;

        let mut link_up = false;
        let mut network_up = false;

        while !link_up || !network_up {
            self.reader
                .dequeue(|buf| {
                    let packet = self.parse_packet(buf)?;

                    if let NoraPacket::Event(ATEvent::StationNetworkUp) = packet {
                        network_up = true;
                    } else if let NoraPacket::Event(ATEvent::WifiLinkUp {
                        wlan_handle: _,
                        bssid: _,
                        channel: _,
                    }) = packet
                    {
                        link_up = true;
                    } else {
                        return Err(NoraRadioError::AtEventUnsupported);
                    }
                    Ok(())
                })
                .await?;
        }

        Ok(())
    }

    /// Create a TCP or UDP socket.
    /// Uses AT+USOCR=<protocol> where protocol is 6 (TCP) or 17 (UDP).
    /// Returns the socket ID assigned by the module.
    pub async fn create_socket(&self, protocol: SocketProtocol) -> Result<u8, NoraRadioError> {
        let mut str: String<16> = String::new();
        let proto_num = protocol as u8;
        write!(str, "AT+USOCR={proto_num}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;

        // Response: +USOCR:<socket_id>\r\nOK\r\n
        let socket_id = self
            .reader
            .dequeue(|buf| {
                match self.parse_packet(buf)? {
                    NoraPacket::Response(ATResponse::Ok(resp)) => {
                        if let Some(i) = resp.find("+USOCR:") {
                            resp[i + 7..]
                                .trim()
                                .parse::<u8>()
                                .or(Err(NoraRadioError::SocketCreationFailed))
                        } else {
                            Err(NoraRadioError::SocketCreationFailed)
                        }
                    }
                    _ => Err(NoraRadioError::SocketCreationFailed),
                }
            })
            .await?;

        Ok(socket_id)
    }

    /// Connect a socket to a remote address and port.
    /// Uses AT+USOC=<socket_id>,"<remote_addr>",<remote_port>
    /// Waits for +UESOC URC confirming the connection.
    pub async fn connect_socket(
        &self,
        socket_id: u8,
        addr: &str,
        port: u16,
    ) -> Result<SocketConnection, NoraRadioError> {
        let mut str: String<64> = String::new();
        write!(str, "AT+USOC={socket_id},\"{addr}\",{port}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;

        // Wait for +UESOC URC confirming connection
        self.reader
            .dequeue(|buf| {
                match self.parse_packet(buf)? {
                    NoraPacket::Event(ATEvent::SocketConnected { socket_id: sid })
                        if sid == socket_id =>
                    {
                        Ok(())
                    }
                    _ => Err(NoraRadioError::SocketConnectionFailed),
                }
            })
            .await?;

        Ok(SocketConnection { socket_id })
    }

    /// Close a socket.
    /// Uses AT+USOCL=<socket_id>
    pub async fn close_socket(&self, socket_id: u8) -> Result<(), NoraRadioError> {
        let mut str: String<16> = String::new();
        write!(str, "AT+USOCL={socket_id}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;

        let mut ok = false;
        let mut socket_closed = false;

        while !ok || !socket_closed {
            self.reader
                .dequeue(|buf| {
                    match self.parse_packet(buf)? {
                        NoraPacket::Event(ATEvent::SocketClosed { socket_id: sid })
                            if sid == socket_id =>
                        {
                            socket_closed = true;
                        }
                        NoraPacket::Response(ATResponse::Ok("")) => {
                            ok = true;
                        }
                        _ => {
                            return Err(NoraRadioError::SocketCloseFailed);
                        }
                    };

                    Ok(())
                })
                .await?;
        }

        Ok(())
    }

    /// Write binary data to a socket.
    /// Uses AT+USOWB=<socket_id>,<length> followed by raw binary data after '>' prompt.
    pub async fn send_data(&self, socket_id: u8, data: &[u8]) -> Result<(), NoraRadioError> {
        let mut str: String<24> = String::new();
        write!(str, "AT+USOWB={socket_id},{}", data.len())
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;

        // TODO: wait for '>' prompt from module before sending data

        // Send raw binary data
        let res = self.writer.enqueue(|buf| {
            buf[..data.len()].copy_from_slice(data);
            data.len()
        });

        if res.is_err() {
            return Err(NoraRadioError::SendCommandLowLevelBufferFull);
        }

        self.read_ok().await?;
        Ok(())
    }

    /// Read binary data from a socket.
    /// Uses AT+USORB=<socket_id>,<length>
    /// Response: +USORB:<socket_id>,<length>,<hex_data>\r\nOK\r\n
    pub async fn read_data<RET, FN>(
        &'a self,
        socket_id: u8,
        max_length: u16,
        fn_read: FN,
    ) -> Result<RET, NoraRadioError>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        let mut str: String<24> = String::new();
        write!(str, "AT+USORB={socket_id},{max_length}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;

        self.reader
            .dequeue(|buf| {
                match self.parse_packet(buf)? {
                    NoraPacket::Response(ATResponse::Ok(resp)) => {
                        // Parse +USORB:<socket_id>,<length>,<data>
                        if let Some(i) = resp.find("+USORB:") {
                            let payload = &resp[i + 7..];
                            // Skip socket_id and length fields to get to data
                            let mut parts = payload.splitn(3, ',');
                            let _sid = parts.next(); // socket_id
                            let _len = parts.next(); // length
                            if let Some(data_str) = parts.next() {
                                Ok(fn_read(data_str.as_bytes()))
                            } else {
                                Err(NoraRadioError::SocketReadFailed)
                            }
                        } else {
                            Err(NoraRadioError::SocketReadFailed)
                        }
                    }
                    _ => Err(NoraRadioError::ReadDataInvalid),
                }
            })
            .await
    }

    pub fn can_read_data(&'a self) -> bool {
        self.reader.can_dequque()
    }

    /// Try to read a queued +UESODA URC indicating data is available.
    /// Returns the socket_id and length of available data, or an error if nothing queued.
    pub fn try_read_data_ready(&'a self) -> Result<(u8, u16), NoraRadioError> {
        match self.reader.try_dequeue() {
            Ok(buf) => {
                match self.parse_packet(buf.data()) {
                    Ok(NoraPacket::Event(ATEvent::SocketDataAvailable { socket_id, length })) => {
                        Ok((socket_id, length))
                    }
                    _ => Err(NoraRadioError::ReadDataInvalid),
                }
            }
            Err(queue::Error::QueueFull) => Err(NoraRadioError::ReadLowLevelBufferEmpty),
            Err(queue::Error::QueueEmpty) => Err(NoraRadioError::ReadLowLevelBufferEmpty),
            Err(queue::Error::InProgress) => Err(NoraRadioError::ReadLowLevelBufferBusy),
        }
    }

    /// Send an AT command string over UART with \r terminator.
    pub async fn send_command(&self, cmd: &str) -> Result<(), NoraRadioError> {
        let res = self.writer.enqueue(|buf| {
            buf[..cmd.len()].clone_from_slice(cmd.as_bytes());
            buf[cmd.len()] = b'\r';
            cmd.len() + 1
        });

        if res.is_err() {
            return Err(NoraRadioError::SendCommandLowLevelBufferFull);
        }

        Ok(())
    }

    /// Read packets until an OK or ERROR AT response is received.
    /// Non-response packets (events/URCs) are consumed and ignored.
    async fn read_ok(&self) -> Result<(), NoraRadioError> {
        let mut res = Err(NoraRadioError::ReadDataInvalid);
        loop {
            let brk = self.reader.dequeue(|buf| {
                let brk = if let Ok(packet) = self.parse_packet(buf) {
                    match packet {
                        NoraPacket::Response(at_resp) => {
                            match at_resp {
                                ATResponse::Ok(_) => {
                                    res = Ok(());
                                    true
                                },
                                ATResponse::Error => return true,
                                ATResponse::Other(_) => {
                                    defmt::debug!("unhandled ATResponse in read_ok()");
                                    false
                                },
                            }
                        },
                        NoraPacket::Event(at_event) => {
                            match at_event {
                                ATEvent::SocketConnected { socket_id: _ } => false,
                                ATEvent::SocketClosed { socket_id: _ } => false,
                                ATEvent::SocketDataAvailable { socket_id: _, length: _ } => false,
                                ATEvent::WifiLinkUp { wlan_handle: _, bssid: _, channel: _ } => false,
                                ATEvent::WifiLinkDown { wlan_handle: _, reason: _ } => false,
                                ATEvent::WifiAccessPointUp => false,
                                ATEvent::WifiAccessPointDown => false,
                                ATEvent::WifiAccessPointStationAssociated { mac_addr: _ } => false,
                                ATEvent::WifiAccessPointStationDisassociated { mac_addr: _ } => false,
                                ATEvent::StationNetworkUp => false,
                                ATEvent::StationNetworkDown => false,
                                ATEvent::AccessPointNetworkUp => false,
                                ATEvent::AccessPointNetworkDown => false,
                                _ => {
                                    defmt::debug!("ignoring ATEvent in read_ok(). event: {}", at_event);
                                    false
                                }
                            }
                        },
                    }
                } else {
                    defmt::debug!("read ok could not parse packet");
                    false
                };

                brk
            })
            .await;

            if brk {
                break;
            }
        }

        res
    }

    /// Parse a raw UART buffer into either an AT response or an AT event (URC).
    /// URCs start with \r\n+ and don't end with OK/ERROR.
    /// Responses end with OK or ERROR.
    fn parse_packet<'b>(&self, buf: &'b [u8]) -> Result<NoraPacket<'b>, AtPacketError> {
        // Try parsing as ATResponse first
        if let Ok(resp) = ATResponse::new(buf) {
            match resp {
                ATResponse::Ok(_) | ATResponse::Error => {
                    return Ok(NoraPacket::Response(resp));
                }
                ATResponse::Other(_) => {
                    // Could be a URC — try parsing as event
                    if let Ok(event) = ATEvent::new(buf) {
                        return Ok(NoraPacket::Event(event));
                    }
                    // Not a recognized event, return as Other response
                    return Ok(NoraPacket::Response(resp));
                }
            }
        }

        // Try as event (for cases where ATResponse::new fails on framing)
        if let Ok(event) = ATEvent::new(buf) {
            return Ok(NoraPacket::Event(event));
        }

        Err(AtPacketError::FramingDecodeFailed)
    }
}
