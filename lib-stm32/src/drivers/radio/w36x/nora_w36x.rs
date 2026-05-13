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
    HostNameTooLong,
    SsidTooLong,
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

    /// Drain one queued packet from the read buffer (discard it).
    pub async fn drain_one(&self) {
        self.reader.dequeue(|_buf| {}).await;
    }

    /// Send a bare "AT" command and wait for OK to verify UART communication.
    pub async fn probe(&self) -> Result<(), NoraRadioError> {
        self.send_command("AT").await?;
        self.read_ok().await?;
        Ok(())
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
    ///   - flow_control: 0=disabled (default), 1=CTS/RTS
    ///   - change_after_confirm: 0=switch after reboot (requires AT&W), 1=switch immediately after OK
    /// Note: data_bits, stop_bits, and parity are not configurable on W36.
    pub async fn config_uart(
        &self,
        baudrate: u32,
        flow_control: bool,
    ) -> Result<(), NoraRadioError> {
        let mut str: String<32> = String::new();
        let flow_control = if flow_control { 1 } else { 0 };
        write!(&mut str, "AT+USYUS={baudrate},{flow_control},1")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        Ok(())
    }

    /// Set socket receive mode for incoming data delivery.
    /// Uses AT+USORM=<receive_mode>
    ///   - 0: Buffered mode (default) — +UESODA event, then AT+USORB to read
    ///   - 2: Direct binary mode — +UESODB/+UESODBF events with inline data
    pub async fn set_socket_receive_mode(&self, mode: u8) -> Result<(), NoraRadioError> {
        let mut str: String<12> = String::new();
        write!(&mut str, "AT+USORM={mode}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        Ok(())
    }

    /// Set the host name on the NORA-W36 module.
    /// Uses AT+UWHN="<host_name>"
    pub async fn set_host_name(&self, host_name: &str) -> Result<(), NoraRadioError> {
        if host_name.len() > 40 {
            defmt::error!("host name too long: {}, must be less than or equal to 40 characters", host_name);
            return Err(NoraRadioError::HostNameTooLong);
        }

        let mut str: String<64> = String::new();
        write!(str, "AT+UWHN=\"{host_name}\"")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        defmt::trace!("host configuration string: {}", str.as_str());
        self.send_command(str.as_str()).await?;
        defmt::trace!("sent host configuration command");
        self.read_ok().await?;
        defmt::trace!("read OK");

        Ok(())
    }

    /// Configure WiFi on the NORA-W36 module.
    /// Uses separate commands per configuration aspect:
    ///   1. Set SSID:     AT+UWSCP=<wlan_handle>,<ssid>
    ///     - <wlan_handle> can only be 0.
    ///     - <ssid> must be 32 characters or fewer.
    ///   2. Set security:
    ///      - Open:       AT+UWSSO=<wlan_handle>
    ///      - WPA:        AT+UWSSW=<wlan_handle>,<passphrase>,<wpa_threshold>
    ///     - <wlan_handle> can only be 0.
    ///     - <passphrase> must be 8 to 63 characters.
    ///     - <wpa_threshold> is optional, defaults to 0 (WPA2 or up), 1 WPA3 only.
    pub async fn config_wifi(
        &self,
        ssid: &str,
        auth: WifiAuth<'_>,
    ) -> Result<(), NoraRadioError> {
        // Set SSID
        if ssid.len() > 32 {
            defmt::error!("SSID too long: {}, must be less than or equal to 32 characters", ssid);
            return Err(NoraRadioError::SsidTooLong);
        }

        // Bytes set to accommodate max command length with longest expected SSID and WPA passphrase.
        let mut str: String<76> = String::new();
        write!(str, "AT+UWSCP=0,\"{ssid}\"")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;

        self.send_command(str.as_str()).await?;
        defmt::trace!("WiFi network connection sent.");
        self.read_ok().await?;
        defmt::trace!("WiFi network connection accepted.");

        // Set authentication
        str.clear();
        match auth {
            WifiAuth::Open => {
                write!(str, "AT+UWSSO=0")
                    .or(Err(NoraRadioError::CommandConstructionFailed))?;
                self.send_command(str.as_str()).await?;
                defmt::trace!("WiFi open network authentication sent.");
                self.read_ok().await?;
                defmt::trace!("WiFi open network authentication accepted.");
            }
            WifiAuth::WPA { passphrase } => {
                if passphrase.len() < 8 || passphrase.len() > 63 {
                    defmt::error!("WPA passphrase length invalid: {}, must be between 8 and 63 characters", passphrase);
                    return Err(NoraRadioError::CommandConstructionFailed);
                }

                write!(str, "AT+UWSSW=0,\"{passphrase}\",0")
                    .or(Err(NoraRadioError::CommandConstructionFailed))?;
                self.send_command(str.as_str()).await?;
                defmt::trace!("WiFi WPA network authentication sent.");
                self.read_ok().await?;
                defmt::trace!("WiFi WPA network authentication accepted.");
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
    /// AT+UWSC=<wlan_handle>
    ///  - <wlan_handle> can only be 0.
    pub async fn connect_wifi(&self) -> Result<(), NoraRadioError> {
        let mut str: String<16> = String::new();
        write!(str, "AT+UWSC=0")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        defmt::trace!("WiFi connection request sent.");
        self.read_ok().await?;
        defmt::trace!("WiFi connection request acknowledged.");

        let mut link_up = false;
        let mut network_up = false;

        // Wait for both link up and network up events before returning success.
        while !link_up || !network_up {
            self.reader
                .dequeue(|buf| {
                    let packet = self.parse_packet(buf)?;

                    if let NoraPacket::Event(ATEvent::StationNetworkUp) = packet {
                        network_up = true;
                        defmt::trace!("WiFi network is up.");
                    } else if let NoraPacket::Event(ATEvent::WifiLinkUp {
                        wlan_handle: _,
                        bssid: _,
                        channel: _,
                    }) = packet
                    {
                        link_up = true;
                        defmt::trace!("WiFi link is up.");
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
    /// AT+USOCR=<protocol>[,<pref_ip_ver>]
    /// - <protocol>: 6 for TCP, 17 for UDP
    /// - <pref_ip_ver> is optional, 0 for IPv4 (default) or 1 for IPv6.
    /// Returns the socket ID assigned by the module.
    pub async fn create_socket(&self, protocol: SocketProtocol) -> Result<u8, NoraRadioError> {
        let mut str: String<16> = String::new();
        let proto_num = protocol as u8;
        write!(str, "AT+USOCR={proto_num}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        defmt::trace!("Socket creation request sent.");

        // Response: +USOCR:<socket_handle>\r\nOK\r\n
        let socket_handle = self
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

        defmt::trace!("Socket creation response received, socket handle: {}", socket_handle);
        Ok(socket_handle)
    }

    /// Connect a socket to a remote address and port.
    /// Uses AT+USOC=<socket_handle>,"<host_address>",<remote_port>
    /// - <socket_handle> is assigned by the module when the socket is created.
    /// - <host_address> is the IP address or domain name of the remote host. Must be 0 to 128 characters. If using a domain name, the module will perform DNS resolution before connecting.
    /// - <remote_port> is the port number on the remote host to connect to. Must be 1-65535.
    /// protocol is the socket protocol (TCP or UDP). For TCP, waits for +UESOC URC confirming the connection.
    pub async fn connect_socket(
        &self,
        socket_handle: u8,
        addr: &str,
        port: u16,
        protocol: SocketProtocol,
    ) -> Result<SocketConnection, NoraRadioError> {
        if addr.len() > 128 {
            defmt::error!("host address length invalid: {}, must be less than 128 characters", addr);
            return Err(NoraRadioError::HostNameTooLong);
        }
        else if port == 0 {
            defmt::error!("port number out of range: {}, must be between 1 and 65535", port);
            return Err(NoraRadioError::CommandConstructionFailed);
        }

        // Bytes set to accommodate max command length with longest expected address (128 chars) and port (5 chars) plus AT command syntax.
        let mut str: String<148> = String::new();
        write!(str, "AT+USOC={socket_handle},\"{addr}\",{port}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        defmt::trace!("Socket connect command sent: {}", str.as_str());
        self.read_ok().await?;
        defmt::trace!("Socket connect command acknowledged");

        // TCP requires waiting for +UESOC URC confirming connection; UDP does not.
        if protocol == SocketProtocol::TCP {
            self.reader
                .dequeue(|buf| {
                    match self.parse_packet(buf)? {
                        NoraPacket::Event(ATEvent::SocketConnected { socket_id: sid })
                            if sid == socket_handle =>
                        {
                            defmt::trace!("Socket connect command accepted for TCP");
                            Ok(())
                        }
                        _ => Err(NoraRadioError::SocketConnectionFailed),
                    }
                })
                .await?;
        } else {
            defmt::trace!("No socket connection confirmation needed for UDP");
        }

        Ok(SocketConnection { socket_id: socket_handle })
    }

    /// Close a socket.
    /// Uses AT+USOCL=<socket_handle>
    pub async fn close_socket(&self, socket_handle: u8) -> Result<(), NoraRadioError> {
        let mut str: String<16> = String::new();
        write!(str, "AT+USOCL={socket_handle}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;

        let mut ok = false;
        let mut socket_closed = false;

        while !ok || !socket_closed {
            self.reader
                .dequeue(|buf| {
                    match self.parse_packet(buf)? {
                        NoraPacket::Event(ATEvent::SocketClosed { socket_id: sid })
                            if sid == socket_handle =>
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
    /// AT+USOWB=<socket_handle>{binary_data}
    /// - <socket_handle> is assigned by the module when the socket is created.
    /// - {binary_data} uses the u-blox binary format: 0x01 (SOH), MSB of length, LSB of length,
    ///   followed by the raw data bytes. No \r terminator is used.
    /// - The response is +USOWB:<socket_handle>,<written_length>.
    pub async fn send_data(&self, socket_handle: u8, data: &[u8]) -> Result<(), NoraRadioError> {
        let mut cmd: String<24> = String::new();
        write!(cmd, "AT+USOWB={socket_handle}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;

        let data_len = data.len() as u16;

        // Send command + binary header + data in one buffer write (no \r terminator).
        let res = self.writer.enqueue(|buf| {
            let cmd_bytes = cmd.as_bytes();
            let cmd_len = cmd_bytes.len();
            buf[..cmd_len].copy_from_slice(cmd_bytes);
            buf[cmd_len] = 0x01; // SOH
            buf[cmd_len + 1] = (data_len >> 8) as u8; // MSB
            buf[cmd_len + 2] = (data_len & 0xFF) as u8; // LSB
            buf[cmd_len + 3..cmd_len + 3 + data.len()].copy_from_slice(data);
            cmd_len + 3 + data.len()
        });

        if res.is_err() {
            return Err(NoraRadioError::SendCommandLowLevelBufferFull);
        }

        // Read +USOWB:<socket_handle>,<written_length>\r\nOK\r\n response and validate written length.
        self.reader
            .dequeue(|buf| {
                match self.parse_packet(buf)? {
                    NoraPacket::Response(ATResponse::Ok(resp)) => {
                        if let Some(i) = resp.find("+USOWB:") {
                            let payload = &resp[i + 7..];
                            let mut parts = payload.splitn(2, ',');
                            let _handle = parts.next();
                            if let Some(len_str) = parts.next() {
                                if let Ok(written) = len_str.trim().parse::<usize>() {
                                    if written != data.len() {
                                        defmt::warn!(
                                            "AT+USOWB: written length {} does not match intended length {}",
                                            written,
                                            data.len()
                                        );
                                    }
                                    return Ok(());
                                }
                            }
                        }
                        Err(NoraRadioError::SocketWriteFailed)
                    }
                    _ => Err(NoraRadioError::SocketWriteFailed),
                }
            })
            .await?;

        Ok(())
    }

    /// Read binary data from a socket.
    /// Uses AT+USORB=<socket_handle>,<length>
    /// - <socket_handle> is assigned by the module when the socket is created.
    /// - <length> is the number of bytes to read, from 1 to 1000.
    /// Response: +USORB:<socket_handle>{binary_data}\r\nOK\r\n
    /// The {binary_data} uses the u-blox binary format: 0x01 (SOH), MSB of length, LSB of length,
    /// followed by the raw data bytes.
    pub async fn read_data<RET, FN>(
        &'a self,
        socket_handle: u8,
        max_length: u16,
        fn_read: FN,
    ) -> Result<RET, NoraRadioError>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        if max_length == 0 || max_length > 1000 {
            defmt::error!("read length out of range: {}, must be between 1 and 1000", max_length);
            return Err(NoraRadioError::CommandConstructionFailed);
        }

        let mut str: String<24> = String::new();
        write!(str, "AT+USORB={socket_handle},{max_length}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;

        let ret = self
            .reader
            .dequeue(|buf| {
                match self.parse_packet(buf)? {
                    NoraPacket::Response(ATResponse::Ok(resp)) => {
                        // Response is +USORB:<socket_handle>{binary_data}
                        // Find the +USORB: prefix, skip socket_handle, then decode binary header.
                        if let Some(i) = resp.find("+USORB:") {
                            let after_prefix = &resp.as_bytes()[i + 7..];
                            // Find the SOH (0x01) byte that starts the binary header
                            if let Some(soh_pos) = after_prefix.iter().position(|&b| b == 0x01) {
                                let binary = &after_prefix[soh_pos..];
                                if binary.len() >= 3 {
                                    let data_len =
                                        ((binary[1] as usize) << 8) | (binary[2] as usize);
                                    let data_start = 3;
                                    if binary.len() >= data_start + data_len {
                                        return Ok(fn_read(
                                            &binary[data_start..data_start + data_len],
                                        ));
                                    }
                                }
                            }
                        }
                        Err(NoraRadioError::SocketReadFailed)
                    }
                    _ => Err(NoraRadioError::ReadDataInvalid),
                }
            })
            .await?;

        self.read_ok().await?;
        Ok(ret)
    }

    pub fn can_read_data(&'a self) -> bool {
        self.reader.can_dequque()
    }

    /// Try to read a queued +UESODA URC indicating data is available.
    /// Returns the socket_id and length of available data, or an error if nothing queued.
    /// Used in buffered mode (AT+USORM=0).
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

    /// Try to read inline binary data from a queued +UESODB or +UESODBF event.
    /// Used in direct binary mode (AT+USORM=2).
    /// Returns the data via the provided callback, or None if no data is queued.
    pub fn try_read_data_binary<RET, FN>(&'a self, fn_read: FN) -> Result<Option<RET>, NoraRadioError>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        match self.reader.try_dequeue() {
            Ok(buf) => {
                match self.parse_packet(buf.data()) {
                    Ok(NoraPacket::Event(ATEvent::SocketDataBinary { socket_id: _, data })) => {
                        Ok(Some(fn_read(data)))
                    }
                    Ok(NoraPacket::Event(ATEvent::SocketDataBinaryFrom { socket_id: _, data, .. })) => {
                        Ok(Some(fn_read(data)))
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
                                // TODO: In direct binary mode, +UESODB/+UESODBF events
                                // arriving during AT command processing will be dropped here.
                                // This is acceptable during init (no connections yet) and
                                // during send_data (brief window). For robustness, consider
                                // a side-buffer to stash data events.
                                ATEvent::SocketDataBinary { .. } => {
                                    defmt::warn!("dropping +UESODB data event in read_ok()");
                                    false
                                },
                                ATEvent::SocketDataBinaryFrom { .. } => {
                                    defmt::warn!("dropping +UESODBF data event in read_ok()");
                                    false
                                },
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
    /// Binary data events (+UESODB/+UESODBF) are checked first since they
    /// contain raw bytes that would fail UTF-8 text parsing.
    fn parse_packet<'b>(&self, buf: &'b [u8]) -> Result<NoraPacket<'b>, AtPacketError> {
        // Check for binary data events first (before UTF-8 conversion).
        // These contain raw binary payloads that cannot be parsed as text.
        if let Some(event) = ATEvent::try_parse_binary(buf)? {
            return Ok(NoraPacket::Event(event));
        }

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
