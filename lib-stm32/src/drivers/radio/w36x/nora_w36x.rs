use core::cell::Cell;
use core::fmt::Write;
use defmt::Format;
use embassy_futures::select::select;
use embassy_stm32::usart;
use embassy_time::{Instant, Timer};
use heapless::{String, Vec};

use crate::uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue};
use ateam_lib_crossarch::queue;

use super::at_protocol::{ATEvent, ATResponse, AtPacketError, SocketProtocol};

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

#[derive(Copy, Clone, PartialEq, Debug, Format)]
#[repr(u8)]
pub enum DataMode {
    BufferedMode = 0,
    DirectBinaryMode = 2,
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
    data_mode: Cell<DataMode>,
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
            data_mode: Cell::new(DataMode::BufferedMode),
        }
    }

    pub async fn update_host_uart_config(&self, config: usart::Config) -> Result<(), ()> {
        self.uart.update_uart_config(config).await
    }

    pub async fn wait_startup(&self) -> Result<(), NoraRadioError> {
        loop {
            let found = self
                .reader
                .dequeue(|buf| {
                    defmt::debug!("wait_startup buf: {:?}", buf);
                    // Skip everything that isn't +STARTUP: pre-boot URCs, split fragments,
                    // unknown events. Outer select() in connect_uart() provides the timeout.
                    Ok::<bool, NoraRadioError>(matches!(
                        self.parse_packet(buf),
                        Ok(NoraPacket::Event(ATEvent::Startup))
                    ))
                })
                .await?;

            if found {
                return Ok(());
            }
        }
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
    ///
    /// Note: data_bits, stop_bits, and parity are not configurable on W36.
    pub async fn config_uart(
        &self,
        baudrate: u32,
        flow_control: bool,
    ) -> Result<(), NoraRadioError> {
        let mut str: String<32> = String::new();
        let flow_control = if flow_control { 1 } else { 0 };
        write!(&mut str, "AT+USYUS={baudrate},{flow_control},0")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        Ok(())
    }

    /// Set socket receive mode for incoming data delivery.
    /// Uses AT+USORM=<receive_mode>
    ///   - 0: Buffered mode (default) — +UESODA event, then AT+USORB to read
    ///   - 2: Direct binary mode — +UESODB/+UESODBF events with inline data
    pub async fn set_socket_receive_mode(&self, mode: DataMode) -> Result<(), NoraRadioError> {
        let mode_num = mode as u8;
        let mut str: String<12> = String::new();
        write!(&mut str, "AT+USORM={mode_num}").or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        self.data_mode.set(mode);
        Ok(())
    }

    pub fn data_mode(&self) -> DataMode {
        self.data_mode.get()
    }

    /// Set the host name on the NORA-W36 module.
    /// Uses AT+UWHN="<host_name>"
    pub async fn set_host_name(&self, host_name: &str) -> Result<(), NoraRadioError> {
        if host_name.len() > 40 {
            defmt::error!(
                "host name too long: {}, must be less than or equal to 40 characters",
                host_name
            );
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
    ///      - <wlan_handle> can only be 0.
    ///      - <ssid> must be 32 characters or fewer.
    ///   2. Set security:
    ///      - Open:       AT+UWSSO=<wlan_handle>
    ///      - WPA:        AT+UWSSW=<wlan_handle>,<passphrase>,<wpa_threshold>
    ///        - <wlan_handle> can only be 0.
    ///        - <passphrase> must be 8 to 63 characters.
    ///        - <wpa_threshold> is optional, defaults to 0 (WPA2 or up), 1 WPA3 only.
    pub async fn config_wifi(&self, ssid: &str, auth: WifiAuth<'_>) -> Result<(), NoraRadioError> {
        // Set SSID
        if ssid.len() > 32 {
            defmt::error!(
                "SSID too long: {}, must be less than or equal to 32 characters",
                ssid
            );
            return Err(NoraRadioError::SsidTooLong);
        }

        // Bytes set to accommodate max command length with longest expected SSID and WPA passphrase.
        let mut str: String<76> = String::new();
        write!(str, "AT+UWSCP=0,\"{ssid}\"").or(Err(NoraRadioError::CommandConstructionFailed))?;

        self.send_command(str.as_str()).await?;
        defmt::trace!("WiFi network connection sent.");
        self.read_ok().await?;
        defmt::trace!("WiFi network connection accepted.");

        // Set authentication
        str.clear();
        match auth {
            WifiAuth::Open => {
                write!(str, "AT+UWSSO=0").or(Err(NoraRadioError::CommandConstructionFailed))?;
                self.send_command(str.as_str()).await?;
                defmt::trace!("WiFi open network authentication sent.");
                self.read_ok().await?;
                defmt::trace!("WiFi open network authentication accepted.");
            }
            WifiAuth::WPA { passphrase } => {
                if passphrase.len() < 8 || passphrase.len() > 63 {
                    defmt::error!(
                        "WPA passphrase length invalid: {}, must be between 8 and 63 characters",
                        passphrase
                    );
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
                                if let NoraPacket::Event(ATEvent::StationNetworkDown) = packet {
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
                                } else if let NoraPacket::Event(
                                    ATEvent::Empty | ATEvent::Startup,
                                ) = packet
                                {
                                    // framing artifact, skip
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
        write!(str, "AT+UWSC=0").or(Err(NoraRadioError::CommandConstructionFailed))?;
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

                    match packet {
                        NoraPacket::Event(ATEvent::StationNetworkUp) => {
                            defmt::info!("nora - station network up");
                            network_up = true;
                        }
                        NoraPacket::Event(ATEvent::StationNetworkDown) => {
                            defmt::warn!("nora - station network down");
                            network_up = false;
                        }
                        NoraPacket::Event(ATEvent::WifiLinkUp {
                            wlan_handle,
                            bssid,
                            channel,
                        }) => {
                            defmt::info!("nora - wifi link up {} {} {}", wlan_handle, bssid, channel);
                            link_up = true;
                        }
                        NoraPacket::Event(ATEvent::WifiLinkDown {
                            wlan_handle,
                            reason,
                        }) => {
                            defmt::warn!("nora - got WifiLinkDown during initial connect. handle: {}, reason: {}", wlan_handle, reason);
                            link_up = false;
                        }
                        NoraPacket::Event(ATEvent::Empty | ATEvent::Startup) => {}
                        other => {
                            defmt::error!("nora - connect_wifi: unsupported packet: {}", other);
                            return Err(NoraRadioError::AtEventUnsupported);
                        }
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
    ///
    /// Returns the socket ID assigned by the module.
    pub async fn create_socket(&self, protocol: SocketProtocol) -> Result<u8, NoraRadioError> {
        let mut str: String<16> = String::new();
        let proto_num = protocol as u8;
        write!(str, "AT+USOCR={proto_num},0").or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        defmt::debug!("create_socket: command sent ({})", str.as_str());

        let socket_handle = self
            .read_response_raw::<64, _, _>(|accum| {
                if accum.windows(5).any(|w| w == b"ERROR") {
                    return Some(Err(NoraRadioError::SocketCreationFailed));
                }
                if let Some(start) = accum.windows(7).position(|w| w == b"+USOCR:") {
                    let after = &accum[start + 7..];
                    let end = after
                        .iter()
                        .position(|&b| b == b'\r' || b == b'\n')
                        .unwrap_or(after.len());
                    if end > 0 {
                        if let Ok(s) = core::str::from_utf8(&after[..end]) {
                            if let Ok(id) = s.trim().parse::<u8>() {
                                // Wait for trailing OK\r\n so it isn't left in the queue
                                if !accum.windows(4).any(|w| w == b"OK\r\n") {
                                    return None;
                                }
                                return Some(Ok(id));
                            }
                        }
                    }
                }
                None
            })
            .await?;

        defmt::debug!("create_socket: socket handle {}", socket_handle);
        Ok(socket_handle)
    }

    /// Connect a socket to a remote address and port.
    /// Uses AT+USOC=<socket_handle>,"<host_address>",<remote_port>
    /// - <socket_handle> is assigned by the module when the socket is created.
    /// - <host_address> is the IP address or domain name of the remote host. Must be 0 to 128 characters. If using a domain name, the module will perform DNS resolution before connecting.
    /// - <remote_port> is the port number on the remote host to connect to. Must be 1-65535.
    ///
    /// protocol is the socket protocol (TCP or UDP). For TCP, waits for +UESOC URC confirming the connection.
    pub async fn connect_socket(
        &self,
        socket_handle: u8,
        addr: &str,
        port: u16,
        protocol: SocketProtocol,
    ) -> Result<SocketConnection, NoraRadioError> {
        if addr.len() > 128 {
            defmt::error!(
                "host address length invalid: {}, must be less than 128 characters",
                addr
            );
            return Err(NoraRadioError::HostNameTooLong);
        } else if port == 0 {
            defmt::error!(
                "port number out of range: {}, must be between 1 and 65535",
                port
            );
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
            loop {
                let done = self
                    .reader
                    .dequeue(|buf| {
                        match self.parse_packet(buf) {
                            Ok(NoraPacket::Event(ATEvent::SocketConnected { socket_id: sid }))
                                if sid == socket_handle =>
                            {
                                defmt::trace!("Socket connect command accepted for TCP");
                                Ok(true)
                            }
                            Ok(NoraPacket::Response(ATResponse::Error)) => {
                                Err(NoraRadioError::SocketConnectionFailed)
                            }
                            Ok(NoraPacket::Event(ATEvent::Empty | ATEvent::Startup)) => Ok(false),
                            Ok(_) => Ok(false),
                            Err(_) => Ok(false), // echo fragment, skip
                        }
                    })
                    .await?;
                if done {
                    break;
                }
            }
        } else {
            defmt::trace!("No socket connection confirmation needed for UDP");
        }

        Ok(SocketConnection {
            socket_id: socket_handle,
        })
    }

    /// Bind a socket to a specific local port.
    /// AT+USOB=<socket_handle>,<local_port>
    /// Must be called after create_socket and before connect_socket or send.
    pub async fn bind_socket(&self, socket_handle: u8, local_port: u16) -> Result<(), NoraRadioError> {
        let mut str: String<24> = String::new();
        write!(str, "AT+USOB={socket_handle},{local_port}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        defmt::debug!("bind_socket: socket {} bound to port {}", socket_handle, local_port);
        self.read_ok().await
    }

    /// Close a socket.
    /// Uses AT+USOCL=<socket_handle>
    /// NORA-W36x returns OK for explicit close; +UESOCL is only unsolicited (remote peer close).
    pub async fn close_socket(&self, socket_handle: u8) -> Result<(), NoraRadioError> {
        let mut str: String<16> = String::new();
        write!(str, "AT+USOCL={socket_handle}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;

        self.read_response_raw::<64, _, _>(|accum| {
            if accum.windows(5).any(|w| w == b"ERROR") {
                return Some(Err(NoraRadioError::SocketCloseFailed));
            }
            if accum.windows(4).any(|w| w == b"OK\r\n") {
                defmt::debug!("nora - close socket - OK");
                return Some(Ok(()));
            }
            None
        })
        .await
    }

    /// Write binary data to a socket.
    /// AT+USOWB=<socket_handle>{binary_data}
    /// - <socket_handle> is assigned by the module when the socket is created.
    /// - {binary_data} uses the u-blox binary format: 0x01 (SOH), MSB of length, LSB of length,
    ///   followed by the raw data bytes. No \r terminator is used.
    /// - The response is +USOWB:<socket_handle>,<written_length>.
    pub async fn send_data(&self, socket_handle: u8, data: &[u8]) -> Result<(), NoraRadioError> {
        let data_len = data.len() as u16;

        if data_len == 0 || data_len > 1460 {
            defmt::error!(
                "data length out of range: {}, must be between 1 and 1460",
                data_len
            );
            return Err(NoraRadioError::CommandConstructionFailed);
        }

        // Prepare the command string with the socket handle.
        let mut cmd: String<24> = String::new();
        write!(cmd, "AT+USOWB={socket_handle}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;

        // Send command + binary header + data in one buffer write (no \r terminator).
        let res = self.writer.enqueue(|buf| {
            let cmd_bytes = cmd.as_bytes();
            let cmd_len = cmd_bytes.len();
            buf[..cmd_len].copy_from_slice(cmd_bytes);
            buf[cmd_len] = 0x01; // SOH
            buf[cmd_len + 1] = (data_len >> 8) as u8; // MSB
            buf[cmd_len + 2] = (data_len & 0xFF) as u8; // LSB
            buf[cmd_len + 3..cmd_len + 3 + data.len()].copy_from_slice(data);
            defmt::trace!("send_data buffer: {:?}", &buf[..cmd_len + 3 + data.len()]);
            cmd_len + 3 + data.len()
        });

        if res.is_err() {
            return Err(NoraRadioError::SendCommandLowLevelBufferFull);
        }

        let ack_t0 = Instant::now();
        let r = self.wait_send_ack(data.len()).await;
        defmt::trace!("NoraW36x - wait_send_ack took {}us", (Instant::now() - ack_t0).as_micros());
        r
    }

    /// Write binary data to a socket without waiting for the +USOWB acknowledgement.
    /// The ack arrives later and will be silently consumed by try_read_data_binary.
    /// Only safe to use when no other AT command response is expected before the ack drains
    /// (i.e. the connected data-phase loop, not during handshake or socket setup).
    pub async fn send_data_no_ack(&self, socket_handle: u8, data: &[u8]) -> Result<(), NoraRadioError> {
        let data_len = data.len() as u16;
        if data_len == 0 || data_len > 1460 {
            defmt::error!("data length out of range: {}, must be between 1 and 1460", data_len);
            return Err(NoraRadioError::CommandConstructionFailed);
        }
        let mut cmd: String<24> = String::new();
        write!(cmd, "AT+USOWB={socket_handle}").or(Err(NoraRadioError::CommandConstructionFailed))?;
        let res = self.writer.enqueue(|buf| {
            let cmd_bytes = cmd.as_bytes();
            let cmd_len = cmd_bytes.len();
            buf[..cmd_len].copy_from_slice(cmd_bytes);
            buf[cmd_len] = 0x01;
            buf[cmd_len + 1] = (data_len >> 8) as u8;
            buf[cmd_len + 2] = (data_len & 0xFF) as u8;
            buf[cmd_len + 3..cmd_len + 3 + data.len()].copy_from_slice(data);
            cmd_len + 3 + data.len()
        });
        if res.is_err() {
            return Err(NoraRadioError::SendCommandLowLevelBufferFull);
        }
        Ok(())
    }

    async fn wait_send_ack(&self, intended_len: usize) -> Result<(), NoraRadioError> {
        // Read +USOWB:<socket_handle>,<written_length>\r\nOK\r\n response and validate written length.
        // Must also see OK\r\n before returning so the trailing OK is not left in the queue.
        // 100ms timeout guards against NORA becoming unresponsive (socket failure, firmware hang).
        let ack_future = self.read_response_raw::<64, _, _>(|accum| {
            if accum.windows(5).any(|w| w == b"ERROR") {
                return Some(Err(NoraRadioError::SocketWriteFailed));
            }
            if let Some(start) = accum.windows(7).position(|w| w == b"+USOWB:") {
                let after = &accum[start + 7..];
                if let Some(comma) = after.iter().position(|&b| b == b',') {
                    let len_bytes = &after[comma + 1..];
                    let end = len_bytes
                        .iter()
                        .position(|&b| b == b'\r' || b == b'\n')
                        .unwrap_or(len_bytes.len());
                    if end > 0 {
                        if let Ok(s) = core::str::from_utf8(&len_bytes[..end]) {
                            if let Ok(written) = s.trim().parse::<usize>() {
                                if !accum.windows(4).any(|w| w == b"OK\r\n") {
                                    return None;
                                }
                                if written != intended_len {
                                    defmt::warn!(
                                        "AT+USOWB: written {} != intended {}",
                                        written,
                                        intended_len
                                    );
                                }
                                return Some(Ok(()));
                            }
                        }
                    }
                }
            }
            None
        });
        match select(ack_future, Timer::after_millis(7)).await {
            embassy_futures::select::Either::First(r) => r?,
            embassy_futures::select::Either::Second(_) => {
                defmt::warn!("NoraW36x - wait_send_ack timed out (NORA unresponsive)");
                return Err(NoraRadioError::OperationTimedOut);
            }
        }
        Ok(())
    }


    /// Read binary data from a socket.
    /// Uses AT+USORB=<socket_handle>,<length>
    /// - <socket_handle> is assigned by the module when the socket is created.
    /// - <length> is the number of bytes to read, from 1 to 1000.
    ///
    /// Response: `+USORB:<socket_handle>{binary_data}\r\nOK\r\n`
    /// The `{binary_data}` uses the u-blox binary format: 0x01 (SOH), MSB of length, LSB of length,
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
            defmt::error!(
                "read length out of range: {}, must be between 1 and 1000",
                max_length
            );
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
            Ok(buf) => match self.parse_packet(buf.data()) {
                Ok(NoraPacket::Event(ATEvent::SocketDataAvailable { socket_id, length })) => {
                    Ok((socket_id, length))
                }
                _ => Err(NoraRadioError::ReadDataInvalid),
            },
            Err(queue::Error::QueueFull) => {
                defmt::warn!("NoraW36x - ReadLowLevelBufferEmpty - QueueFull");
                Err(NoraRadioError::ReadLowLevelBufferEmpty)
            }
            Err(queue::Error::QueueEmpty) => {
                defmt::warn!("NoraW36x - ReadLowLevelBufferEmpty - QueueEmpty");
                Err(NoraRadioError::ReadLowLevelBufferEmpty)
            }
            Err(queue::Error::InProgress) => {
                defmt::error!("NoraW36x - ReadLowLevelBufferBusy");
                Err(NoraRadioError::ReadLowLevelBufferBusy)
            }
        }
    }

    /// Try to read inline binary data from a queued +UESODB or +UESODBF event.
    /// Used in direct binary mode (AT+USORM=2).
    /// Returns the data via the provided callback, or None if no data is queued.
    pub fn try_read_data_binary<RET, FN>(
        &'a self,
        fn_read: FN,
    ) -> Result<Option<RET>, NoraRadioError>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        match self.reader.try_dequeue() {
            Ok(buf) => match self.parse_packet(buf.data()) {
                Ok(NoraPacket::Event(ATEvent::SocketDataBinary { socket_id: _, data })) => {
                    Ok(Some(fn_read(data)))
                }
                Ok(NoraPacket::Event(ATEvent::SocketDataBinaryFrom {
                    socket_id: _, data, ..
                })) => Ok(Some(fn_read(data))),
                other => {
                    defmt::trace!("NoraW36x - expected binary data event, got {:?}", other);
                    Err(NoraRadioError::ReadDataInvalid)
                }
            },
            Err(queue::Error::QueueFull) => {
                defmt::warn!("NoraW36x - ReadLowLevelBufferEmpty - QueueFull");
                Err(NoraRadioError::ReadLowLevelBufferEmpty)
            }
            Err(queue::Error::QueueEmpty) => {
                defmt::warn!("NoraW36x - ReadLowLevelBufferEmpty - QueueEmpty");
                Err(NoraRadioError::ReadLowLevelBufferEmpty)
            }
            Err(queue::Error::InProgress) => {
                defmt::error!("NoraW36x - ReadLowLevelBufferBusy");
                Err(NoraRadioError::ReadLowLevelBufferBusy)
            }
        }
    }

    /// Read manufacturer identification string (AT+CGMI).
    /// Response has no prefix — typically "u-blox".
    pub async fn read_manufacturer(&self) -> Result<String<32>, NoraRadioError> {
        self.send_command("AT+CGMI").await?;
        self.read_response_raw::<128, _, _>(|accum| {
            if accum.windows(5).any(|w| w == b"ERROR") {
                return Some(Err(NoraRadioError::ReadDataInvalid));
            }
            if !accum.windows(4).any(|w| w == b"OK\r\n") {
                return None;
            }
            let s = match ATResponse::new(accum) {
                Ok(ATResponse::Ok(s)) => s,
                _ => return Some(Err(NoraRadioError::ReadDataInvalid)),
            };
            let mut out: String<32> = String::new();
            if out.push_str(s.trim().trim_matches('"')).is_err() {
                return Some(Err(NoraRadioError::CommandConstructionFailed));
            }
            Some(Ok(out))
        })
        .await
    }

    /// Read model identification string (AT+CGMM).
    /// Response has no prefix — e.g. "NORA-W36-OINK".
    pub async fn read_model(&self) -> Result<String<32>, NoraRadioError> {
        self.send_command("AT+CGMM").await?;
        self.read_response_raw::<128, _, _>(|accum| {
            if accum.windows(5).any(|w| w == b"ERROR") {
                return Some(Err(NoraRadioError::ReadDataInvalid));
            }
            if !accum.windows(4).any(|w| w == b"OK\r\n") {
                return None;
            }
            let s = match ATResponse::new(accum) {
                Ok(ATResponse::Ok(s)) => s,
                _ => return Some(Err(NoraRadioError::ReadDataInvalid)),
            };
            let mut out: String<32> = String::new();
            if out.push_str(s.trim().trim_matches('"')).is_err() {
                return Some(Err(NoraRadioError::CommandConstructionFailed));
            }
            Some(Ok(out))
        })
        .await
    }

    /// Read firmware version string (AT+CGMR).
    /// Response has no prefix — e.g. "3.4.1".
    pub async fn read_firmware_version(&self) -> Result<String<32>, NoraRadioError> {
        self.send_command("AT+CGMR").await?;
        self.read_response_raw::<128, _, _>(|accum| {
            if accum.windows(5).any(|w| w == b"ERROR") {
                return Some(Err(NoraRadioError::ReadDataInvalid));
            }
            if !accum.windows(4).any(|w| w == b"OK\r\n") {
                return None;
            }
            let s = match ATResponse::new(accum) {
                Ok(ATResponse::Ok(s)) => s,
                _ => return Some(Err(NoraRadioError::ReadDataInvalid)),
            };
            let mut out: String<32> = String::new();
            if out.push_str(s.trim().trim_matches('"')).is_err() {
                return Some(Err(NoraRadioError::CommandConstructionFailed));
            }
            Some(Ok(out))
        })
        .await
    }

    /// Read current UART settings (AT+USYUS?).
    /// Returns (baud_rate, flow_control_enabled).
    pub async fn read_uart_settings(&self) -> Result<(u32, bool), NoraRadioError> {
        self.send_command("AT+USYUS?").await?;
        self.read_response_raw::<128, _, _>(|accum| {
            if accum.windows(5).any(|w| w == b"ERROR") {
                return Some(Err(NoraRadioError::ReadDataInvalid));
            }
            if !accum.windows(4).any(|w| w == b"OK\r\n") {
                return None;
            }
            let s = match ATResponse::new(accum) {
                Ok(ATResponse::Ok(s)) => s,
                _ => return Some(Err(NoraRadioError::ReadDataInvalid)),
            };
            let payload = s.trim().trim_start_matches("+USYUS:");
            let mut parts = payload.splitn(2, ',');
            let baud: u32 = match parts.next().and_then(|x| x.trim().parse().ok()) {
                Some(v) => v,
                None => return Some(Err(NoraRadioError::ReadDataInvalid)),
            };
            let flow: u8 = match parts.next().and_then(|x| x.trim().parse().ok()) {
                Some(v) => v,
                None => return Some(Err(NoraRadioError::ReadDataInvalid)),
            };
            Some(Ok((baud, flow != 0)))
        })
        .await
    }

    /// Store current configuration to persistent memory (AT&W).
    pub async fn store_config(&self) -> Result<(), NoraRadioError> {
        self.send_command("AT&W").await?;
        self.read_ok().await
    }

    /// Read current Wi-Fi regulatory domain (AT+UWRD?).
    /// 0=World, 1=ETSI, 2=FCC, 3=IC/ISED — see AT command spec for full list.
    pub async fn read_regulatory_domain(&self) -> Result<u8, NoraRadioError> {
        self.send_command("AT+UWRD?").await?;
        self.read_response_raw::<64, _, _>(|accum| {
            if accum.windows(5).any(|w| w == b"ERROR") {
                defmt::warn!("read_regulatory_domain: AT+UWRD? returned ERROR");
                return Some(Err(NoraRadioError::ReadDataInvalid));
            }
            if !accum.windows(4).any(|w| w == b"OK\r\n") {
                return None;
            }
            let s = match ATResponse::new(accum) {
                Ok(ATResponse::Ok(s)) => s,
                _ => {
                    defmt::warn!("read_regulatory_domain: ATResponse::new failed");
                    return Some(Err(NoraRadioError::ReadDataInvalid));
                }
            };
            let val_str = s.trim().trim_start_matches("+UWRD:").trim().trim_matches('"');
            match val_str.parse::<u8>() {
                Ok(v) => Some(Ok(v)),
                Err(_) => {
                    defmt::warn!("read_regulatory_domain: parse failed on {:?}", val_str);
                    Some(Err(NoraRadioError::ReadDataInvalid))
                }
            }
        })
        .await
    }

    /// Set Wi-Fi regulatory domain (AT+UWRD=<domain>).
    /// Call store_config() to persist. Must be set before starting AP or Station.
    pub async fn set_regulatory_domain(&self, domain: u8) -> Result<(), NoraRadioError> {
        let mut cmd: String<16> = String::new();
        write!(&mut cmd, "AT+UWRD={domain}")
            .or(Err(NoraRadioError::CommandConstructionFailed))?;
        self.send_command(cmd.as_str()).await?;
        self.read_ok().await
    }

    /// Restore factory defaults (AT+USYFR).
    /// Requires store_config() + radio reboot to take effect.
    /// Resets all settings; removes certificates and BT bonding info.
    pub async fn factory_restore(&self) -> Result<(), NoraRadioError> {
        self.send_command("AT+USYFR").await?;
        self.read_ok().await
    }

    /// Accumulate raw UART bytes across multiple idle-triggered DMA transfers until the
    /// provided predicate signals completion. The predicate receives the full accumulated
    /// buffer on every new chunk and returns `Some(result)` when done, `None` to keep going.
    /// N is the accumulation buffer capacity in bytes; excess bytes are silently dropped.
    async fn read_response_raw<const N: usize, T, F>(
        &self,
        mut f: F,
    ) -> Result<T, NoraRadioError>
    where
        F: FnMut(&[u8]) -> Option<Result<T, NoraRadioError>>,
    {
        let mut accum: Vec<u8, N> = Vec::new();
        loop {
            self.reader
                .dequeue(|buf| {
                    defmt::trace!("read_response_raw: {:?}", buf);
                    let _ = accum.extend_from_slice(buf);
                })
                .await;
            if let Some(result) = f(&accum) {
                return result;
            }
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

    /// Accumulate raw bytes until `OK\r\n` or `ERROR` appears.
    /// Any interleaved URCs (WiFi events, socket events) are included in the buffer
    /// but do not affect the outcome — only the terminal OK/ERROR matters.
    async fn read_ok(&self) -> Result<(), NoraRadioError> {
        self.read_response_raw::<256, _, _>(|accum| {
            if accum.windows(5).any(|w| w == b"ERROR") {
                return Some(Err(NoraRadioError::ReadDataInvalid));
            }
            if accum.windows(4).any(|w| w == b"OK\r\n") {
                return Some(Ok(()));
            }
            None
        })
        .await
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
