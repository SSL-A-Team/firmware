use core::fmt::Write;
use defmt::Format;
use embassy_futures::select::select;
use embassy_stm32::usart;
use embassy_time::Timer;
use heapless::String;

use crate::queue;
use crate::uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue};

use super::at_protocol::{ATEvent, ATResponse, WifiLinkDisconnectedReason};
use super::edm_protocol::{EdmPacket, EdmPacketError};

#[derive(Copy, Clone, PartialEq, Debug, Format)]
pub enum OdinRadioError {
    CommandConstructionFailed,
    EdmPacketError(EdmPacketError),
    SendCommandLowLevelBufferFull,
    SendCommandCommunicationModeInvalid,
    ReadDataInvalid,
    ReadLowLevelBufferEmpty,
    ReadLowLevelBufferBusy,
    AtEventUnsupported,
    AuthModeUnsupported,
    OperationTimedOut,
    PeerConnectionReceivedInvalidResponse,
    PeerConnectionFailed,
    PeerCloseFailed,
    PeerMissing,
    EdmTransitionFailed,
}

impl From<EdmPacketError> for OdinRadioError {
    fn from(err: EdmPacketError) -> Self {
        OdinRadioError::EdmPacketError(err)
    }
}

#[allow(dead_code)]
pub enum RadioMode {
    // Unknown,
    CommandMode,
    DataMode,
    ExtendedDataMode,
}

#[allow(dead_code)]
pub enum WifiAuth<'a> {
    Open,
    WPA { passphrase: &'a str },
    LEAP,
    PEAP,
    EAPTLS,
}

#[allow(dead_code)]
pub enum ServerType {
    Disabled = 0,
    TCP = 1,
    UDP = 2,
    SPP = 3,
    DUN = 4,
    UUID = 5,
    SPS = 6,
    ATP = 8,
}

pub struct PeerConnection {
    pub peer_id: u8,
    pub channel_id: u8,
}

pub struct OdinW262<
    'a,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
> {
    reader: &'a UartReadQueue<LEN_RX, DEPTH_RX>,
    writer: &'a UartWriteQueue<LEN_TX, DEPTH_TX>,
    uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX>,
    mode: RadioMode,
}

impl<
        'a,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
    > OdinW262<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX>
{
    pub fn new(
        reader: &'a UartReadQueue<LEN_RX, DEPTH_RX>,
        writer: &'a UartWriteQueue<LEN_TX, DEPTH_TX>,
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX>,
    ) -> Self {
        Self {
            reader,
            writer,
            uart,
            mode: RadioMode::CommandMode,
        }
    }

    pub async fn update_host_uart_config(&self, config: usart::Config) -> Result<(), ()> {
        self.uart.update_uart_config(config).await
    }

    pub async fn wait_startup(&mut self) -> Result<(), OdinRadioError> {
        // if were waiting for startup, we fellback to command mode
        self.mode = RadioMode::CommandMode;

        self.reader
            .dequeue(|buf| {
                // defmt::trace!("radio dequeueing {}", buf);
                if let EdmPacket::ATResponse(ATResponse::Other("+STARTUP")) = self.to_packet(buf)? {
                    Ok(())
                } else {
                    Err(OdinRadioError::ReadDataInvalid)
                }
            })
            .await
    }

    pub async fn wait_edm_startup(&self) -> Result<(), OdinRadioError> {
        self.reader
            .dequeue(|buf| {
                if let EdmPacket::StartEvent = self.to_packet(buf)? {
                    Ok(())
                } else {
                    Err(OdinRadioError::ReadDataInvalid)
                }
            })
            .await
    }

    #[allow(dead_code)]
    pub async fn attention(&self) -> Result<(), OdinRadioError> {
        self.send_command("AT").await?;
        self.read_ok().await?;
        Ok(())
    }

    pub async fn set_echo(&self, echo_on: bool) -> Result<(), OdinRadioError> {
        let echo_on = if echo_on { '1' } else { '0' };
        let mut str: String<4> = String::new();
        write!(&mut str, "ATE{echo_on}").unwrap();
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        Ok(())
    }

    pub async fn config_uart(
        &self,
        baudrate: u32,
        flow_control: bool,
        data_bits: u8,
        parity: bool,
    ) -> Result<(), OdinRadioError> {
        let mut str: String<28> = String::new();
        let flow_control = if flow_control { '1' } else { '2' };
        let stop_bits = '1';
        let parity = if parity { '3' } else { '1' };
        write!(
            &mut str,
            "AT+UMRS={baudrate},{flow_control},{data_bits},{stop_bits},{parity},1",
        )
        .unwrap(); // TODO unwrap
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        Ok(())
    }

    pub async fn enter_edm(&mut self) -> Result<bool, OdinRadioError> {
        self.send_command("ATO2").await?;
        // if self.send_command("ATO2").await.is_err() {
        //     return Err(());
        // }

        // TODO this is getting CR LF O K CR LF [EDM START, 0, 2, 0, 113, EDM END]
        // 0, 2 decodes as payload length 2
        // 0 113 decodes as payload event EDM_START
        // this appear valid, we just captured two events at once.
        // write a new function to handle this
        // self.read_ok().await?;
        let res = self.read_ok_at_edm_transition().await;
        if res.is_err() {
            defmt::trace!("failed to enter EDM mode!");
        } else {
            self.mode = RadioMode::ExtendedDataMode;
        }

        res
    }

    pub async fn set_host_name(&self, host_name: &str) -> Result<(), OdinRadioError> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UNHN=\"{host_name}\"").or(Err(OdinRadioError::CommandConstructionFailed))?;
        defmt::trace!("host configuration string: {}", str.as_str());
        self.send_command(str.as_str()).await?;
        defmt::trace!("sent configuration command");
        self.read_ok().await?;
        defmt::trace!("read OK");

        Ok(())
    }

    pub async fn config_wifi(
        &self,
        config_id: u8,
        ssid: &str,
        auth: WifiAuth<'_>,
    ) -> Result<(), OdinRadioError> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UWSC={config_id},2,\"{ssid}\"").or(Err(OdinRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        str.clear();
        match auth {
            WifiAuth::Open => {
                write!(str, "AT+UWSC={config_id},5,1").or(Err(OdinRadioError::CommandConstructionFailed))?;
                self.send_command(str.as_str()).await?;
                self.read_ok().await?;
            }
            WifiAuth::WPA { passphrase } => {
                write!(str, "AT+UWSC={config_id},5,2").or(Err(OdinRadioError::CommandConstructionFailed))?;
                self.send_command(str.as_str()).await?;
                self.read_ok().await?;
                str.clear();
                write!(str, "AT+UWSC={config_id},8,\"{passphrase}\"").or(Err(OdinRadioError::CommandConstructionFailed))?;
                self.send_command(str.as_str()).await?;
                self.read_ok().await?;
            }
            _ => return Err(OdinRadioError::AuthModeUnsupported),
        }

        Ok(())
    }

    #[allow(dead_code)]
    pub async fn config_wifi_addr(
        &self,
        _config_id: u8,
        _static_ip: Option<&str>,
        _subnet_mask: Option<&str>,
        _default_gateway: Option<&str>,
        _dns_server1: Option<&str>,
        _dns_server2: Option<&str>,
    ) -> Result<(), ()> {
        todo!("implement if needed");
    }

    pub async fn disconnect_wifi(&self, config_id: u8) -> Result<(), OdinRadioError> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UWSCA={config_id},4").or(Err(OdinRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;

        match select(
            async move {
                let mut run = true;
                while run {
                    let _ = self.reader.dequeue(|buf| {
                        // defmt::warn!("buf {}", buf);
                        let packet = self.to_packet(buf);
                        if packet.is_err() {
                            defmt::warn!("parsed invalid packet");
                        } else {
                            let packet = packet.unwrap();
                            if let EdmPacket::ATEvent(ATEvent::NetworkDown { interface_id: 0 }) = packet {
                                defmt::debug!("got network down event.");
                                run = false;
                            } else if let EdmPacket::ATEvent(ATEvent::WifiLinkDisconnected { conn_id:_, reason: WifiLinkDisconnectedReason::NetworkDisabled }) = packet {
                                run = false;
                            } else if let EdmPacket::ATResponse(ATResponse::Ok(_)) = packet {
                                run = false;
                            } else {
                                defmt::warn!("got edm packet: {}", packet);
                            }
                        }
                        }).await;
                }
            },
            Timer::after_millis(2500)).await {
                embassy_futures::select::Either::First(_) => {
                    Ok(())
                },
                embassy_futures::select::Either::Second(_) => {
                    defmt::warn!("disconnect timed out");
                    Err(OdinRadioError::OperationTimedOut)
                },
            }
    }

    pub async fn connect_wifi(&self, config_id: u8) -> Result<(), OdinRadioError> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UWSCA={config_id},3").or(Err(OdinRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;

        let mut network_up = 0;

        while network_up < 2 {
            self.reader
                .dequeue(|buf| {
                    let packet = self.to_packet(buf)?;

                    if let EdmPacket::ATEvent(ATEvent::NetworkUp { interface_id: 0 }) = packet {
                        network_up += 1;
                    } else if let EdmPacket::ATEvent(ATEvent::WifiLinkConnected {
                        conn_id: _,
                        bssid: _,
                        channel: _,
                    }) = packet
                    {
                        // TODO
                        // self.wifiConnected = true;
                    } else {
                        return Err(OdinRadioError::AtEventUnsupported);
                    }
                    Ok(())
                })
                .await?;
        }

        Ok(())
    }

    #[allow(dead_code)]
    pub async fn config_server(
        &self,
        server_id: u8,
        server_type: ServerType,
        port: u16,
    ) -> Result<(), OdinRadioError> {
        let mut str: String<64> = String::new();
        let server_type = server_type as u8;
        write!(str, "AT+UDSC={server_id},{server_type},{port},1,0").or(Err(OdinRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        Ok(())
    }

    pub async fn connect_peer(&self, url: &str) -> Result<PeerConnection, OdinRadioError> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UDCP={url}").or(Err(OdinRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;

        let mut peer_id = None;
        let mut peer_connected_ip = false;
        let mut channel_ret = None;

        while peer_id.is_none() || !peer_connected_ip || channel_ret.is_none() {
            self.reader
                .dequeue(|buf| {
                    // defmt::trace!("connect peer buf {}", buf);
                    let pkt = self.to_packet(buf);
                    if pkt.is_err() {
                        defmt::error!("got undecodable pkt {}", buf);
                    }

                    match pkt? {
                        EdmPacket::ATEvent(ATEvent::PeerConnectedIP {
                            peer_handle: _,
                            is_ipv6: _,
                            protocol: _,
                            local_address: _,
                            local_port: _,
                            remote_address: _,
                            remote_port: _,
                        }) => {
                            // defmt::info!("AT event");

                            peer_connected_ip = true;
                        }
                        EdmPacket::ConnectEvent {
                            channel,
                            event_type: _,
                        } => {
                            // defmt::info!("connect event");

                            channel_ret = Some(channel);
                        }
                        EdmPacket::ATResponse(ATResponse::Ok(resp)) => {
                            // defmt::info!("AT resp connect event");

                            if let Some(i) = resp.find("+UDCP:") {
                                peer_id = Some(resp[i + 6..].parse::<u8>().or(Err(OdinRadioError::PeerConnectionReceivedInvalidResponse))?);
                            } else {
                                return Err(OdinRadioError::PeerConnectionFailed);
                            }
                        }
                        _ => {
                            return Err(OdinRadioError::AtEventUnsupported);
                        }
                    };

                    Ok(())
                })
                .await?;
        }
        Ok(PeerConnection {
            peer_id: peer_id.unwrap(),
            channel_id: channel_ret.unwrap(),
        })
    }

    pub async fn close_peer(&self, peer_id: u8) -> Result<(), OdinRadioError> {
        let mut str: String<12> = String::new();
        write!(str, "AT+UDCPC={peer_id}").or(Err(OdinRadioError::CommandConstructionFailed))?;
        self.send_command(str.as_str()).await?;

        let mut ok = false;
        let mut peer_disconnect = false;
        let mut disconnect = false;

        while !ok || !peer_disconnect || !disconnect {
            self.reader
                .dequeue(|buf| {
                    match self.to_packet(buf)? {
                        EdmPacket::ATEvent(ATEvent::PeerDisconnected { peer_handle: _ }) => {
                            peer_disconnect = true
                        }
                        EdmPacket::DisconnectEvent { channel: _ } => {
                            disconnect = true;
                        }
                        EdmPacket::ATResponse(ATResponse::Ok("")) => {
                            ok = true;
                        }
                        EdmPacket::DataEvent {
                            channel: _,
                            data: _,
                        } => {}
                        _ => {
                            return Err(OdinRadioError::PeerCloseFailed);
                        }
                    };

                    Ok(())
                })
                .await?;
        }

        Ok(())
    }

    pub fn send_data(&self, channel_id: u8, data: &[u8]) -> Result<(), OdinRadioError> {
        let res = self.writer.enqueue(|buf| {
                EdmPacket::DataCommand {
                    channel: channel_id,
                    data,
                }
                .write(buf).unwrap()
            });

        if res.is_err() {
            // queue was full
            return Err(OdinRadioError::SendCommandLowLevelBufferFull);
        }

        Ok(())
    }

    pub async fn read_data<RET, FN>(&'a self, fn_read: FN) -> Result<RET, OdinRadioError>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        self.reader
            .dequeue(|buf| {
                if let EdmPacket::DataEvent { channel: _, data } = self.to_packet(buf)? {
                    Ok(fn_read(data))
                } else {
                    Err(OdinRadioError::ReadDataInvalid)
                }
            }).await
    }

    pub fn can_read_data(&'a self) -> bool {
        self.reader.can_dequque()
    }

    pub fn try_read_data<RET, FN>(&'a self, fn_read: FN) -> Result<RET, OdinRadioError>
    where FN: FnOnce(&[u8]) -> RET,
    {
        match self.reader.try_dequeue() {
            Ok(buf) => {
                match self.to_packet(buf.data()) {
                    Ok(pkt) => {
                        if let EdmPacket::DataEvent { channel: _ , data } = pkt {
                            Ok(fn_read(data))
                        } else {
                            // defmt::trace!("got non data event");
                            Err(OdinRadioError::ReadDataInvalid)
                        }
                    },
                    Err(_) => {
                        // defmt::trace!("got data that wasn't an edm packet: {}", buf.data());
                        Err(OdinRadioError::ReadDataInvalid)
                    },
                }
                // we read something

            },
            Err(queue::Error::QueueFull) => {
                // nothing to read
                return Err(OdinRadioError::ReadLowLevelBufferEmpty);
            }
            Err(queue::Error::QueueEmpty) => {
                // nothing to read
                Err(OdinRadioError::ReadLowLevelBufferEmpty)
            }
            Err(queue::Error::InProgress) => {
                // you did something illegal
                Err(OdinRadioError::ReadLowLevelBufferBusy)
            },
        }
    }

    pub async fn send_command(&self, cmd: &str) -> Result<(), OdinRadioError> {
        match self.mode {
            RadioMode::CommandMode => {
                let res = self.writer.enqueue(|buf| {
                        buf[..cmd.len()].clone_from_slice(cmd.as_bytes());
                        buf[cmd.len()] = b'\r';
                        cmd.len() + 1
                    });

                if res.is_err() {
                    // queue was full
                    return Err(OdinRadioError::SendCommandLowLevelBufferFull)
                }

                Ok(())
            }
            RadioMode::ExtendedDataMode => {
                let res = self.writer.enqueue(|buf| EdmPacket::ATRequest(cmd).write(buf).unwrap());

                if res.is_err() {
                    // queue was full
                    return Err(OdinRadioError::SendCommandLowLevelBufferFull)
                }

                Ok(())
            }
            _ => Err(OdinRadioError::SendCommandCommunicationModeInvalid),
        }
    }

    #[allow(dead_code)]
    async fn get_response(&self) {}

    #[allow(dead_code)]
    pub async fn read(&self) {
        self.reader
            .dequeue(|buf| {
                defmt::info!("{:?}", self.to_packet(buf).unwrap());
            })
            .await;
    }

    async fn read_ok(&self) -> Result<(), OdinRadioError> {
        let mut res = Err(OdinRadioError::ReadDataInvalid);
        loop {
            let brk = self.reader.dequeue(|buf| {
                // defmt::warn!("buf: {}", buf);
                let brk = if let Ok(packet) = self.to_packet(buf) {
                    match packet {
                        EdmPacket::ConnectEvent { channel: _, event_type: _ } => false,
                        EdmPacket::DisconnectEvent { channel: _ } => false,
                        EdmPacket::ATResponse(at_resp) => {
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
                        EdmPacket::ATEvent(at_event) => {
                            match at_event {
                                ATEvent::PeerConnectedIP { peer_handle: _, is_ipv6: _, protocol: _, local_address: _, local_port: _, remote_address: _, remote_port: _ } => false,
                                ATEvent::PeerDisconnected { peer_handle: _ } => false,
                                ATEvent::WifiLinkConnected { conn_id: _, bssid: _, channel: _ } => false,
                                ATEvent::WifiLinkDisconnected { conn_id: _, reason: _ } => false,
                                ATEvent::WifiAccessPointUp { id: _ } => false,
                                ATEvent::WifiAccessPointDown { id: _ } => false,
                                ATEvent::WifiAccessPointStationConnected { id: _, mac_addr: _ } => false,
                                ATEvent::WifiAccessPointStationDisconnected { id: _ } => false,
                                ATEvent::NetworkUp { interface_id: _ } => false,
                                ATEvent::NetworkDown { interface_id: _ } => false,
                                ATEvent::NetworkError { interface_id: _, code: _ } => false,
                                _ => {
                                    defmt::debug!("ignoring ATEvent in read_ok(). event: {}", at_event);
                                    false
                                }
                            }
                        },
                        _ => {
                            defmt::debug!("ignoring unhandled EdmPacket variant in read_ok(), variant: {}", packet);
                            false
                        }
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

    async fn read_ok_at_edm_transition(&self) -> Result<bool, OdinRadioError> {
        let transition_buf: [u8; 12] = [13, 10, 79, 75, 13, 10, 170, 0, 2, 0, 113, 85];
        

        self.reader.dequeue(|buf| {
            if buf.len() == transition_buf.len() && buf.iter().zip(transition_buf.iter()).all(|(a,b)| a == b) {
                Ok(true)
            } else if let EdmPacket::ATResponse(ATResponse::Ok("")) = self.to_packet(buf)? {
                Ok(false)
            } else {
                Err(OdinRadioError::EdmTransitionFailed)
            }
        }).await
    }

    fn to_packet<'b>(&self, buf: &'b [u8]) -> Result<EdmPacket<'b>, EdmPacketError> {
        match self.mode {
            RadioMode::CommandMode => Ok(EdmPacket::ATResponse(ATResponse::new(buf)?)),
            RadioMode::ExtendedDataMode => EdmPacket::new(buf),
            _ => Err(EdmPacketError::PacketTypeDecodingFailed),
        }
    }
}
