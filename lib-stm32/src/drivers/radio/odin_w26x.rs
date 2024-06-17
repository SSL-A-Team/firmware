use core::fmt::Write;
use embassy_futures::select::select;
use embassy_stm32::usart;
use embassy_time::Timer;
use heapless::String;

use crate::queue;
use crate::uart::queue::{UartReadQueue, UartWriteQueue};

use super::at_protocol::{ATEvent, ATResponse, WifiLinkDisconnectedReason};
use super::edm_protocol::EdmPacket;

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
    UART: usart::BasicInstance,
    TxDma: usart::TxDma<UART>,
    RxDma: usart::RxDma<UART>,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
> {
    reader: &'a UartReadQueue<UART, RxDma, LEN_RX, DEPTH_RX>,
    writer: &'a UartWriteQueue<UART, TxDma, LEN_TX, DEPTH_TX>,
    mode: RadioMode,
}

impl<
        'a,
        UART: usart::BasicInstance,
        TxDma: usart::TxDma<UART>,
        RxDma: usart::RxDma<UART>,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
    > OdinW262<'a, UART, TxDma, RxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX>
{
    pub fn new(
        reader: &'a UartReadQueue<UART, RxDma, LEN_RX, DEPTH_RX>,
        writer: &'a UartWriteQueue<UART, TxDma, LEN_TX, DEPTH_TX>,
    ) -> Self {
        Self {
            reader,
            writer,
            mode: RadioMode::CommandMode,
        }
    }

    pub async fn update_host_uart_config(&self, config: usart::Config) -> Result<(), ()> {
        self.writer.update_uart_config(config).await
    }

    pub async fn wait_startup(&mut self) -> Result<(), ()> {
        // if were waiting for startup, we fellback to command mode
        self.mode = RadioMode::CommandMode;

        self.reader
            .dequeue(|buf| {
                defmt::info!("dequeueing {}", buf);
                if let EdmPacket::ATResponse(ATResponse::Other("+STARTUP")) = self.to_packet(buf)? {
                    Ok(())
                } else {
                    Err(())
                }
            })
            .await
    }

    pub async fn wait_edm_startup(&self) -> Result<(), ()> {
        self.reader
            .dequeue(|buf| {
                if let EdmPacket::StartEvent = self.to_packet(buf)? {
                    Ok(())
                } else {
                    Err(())
                }
            })
            .await
    }

    #[allow(dead_code)]
    pub async fn attention(&self) -> Result<(), ()> {
        self.send_command("AT").await?;
        self.read_ok().await?;
        Ok(())
    }

    pub async fn set_echo(&self, echo_on: bool) -> Result<(), ()> {
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
    ) -> Result<(), ()> {
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

    pub async fn enter_edm(&mut self) -> Result<bool, ()> {
        if self.send_command("ATO2").await.is_err() {
            return Err(());
        }

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

        return res;
    }

    pub async fn set_host_name(&self, host_name: &str) -> Result<(), ()> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UNHN=\"{host_name}\"").or(Err(()))?;
        defmt::trace!("host configuration string: {}", str.as_str());
        self.send_command(str.as_str()).await?;
        defmt::trace!("sent configuration command");
        self.read_ok().await?;
        defmt::trace!("read OK");

        Ok(())
    }

    pub async fn config_wifi<'b>(
        &self,
        config_id: u8,
        ssid: &str,
        auth: WifiAuth<'b>,
    ) -> Result<(), ()> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UWSC={config_id},2,\"{ssid}\"").or(Err(()))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        str.clear();
        match auth {
            WifiAuth::Open => {
                write!(str, "AT+UWSC={config_id},5,1").or(Err(()))?;
                self.send_command(str.as_str()).await?;
                self.read_ok().await?;
            }
            WifiAuth::WPA { passphrase } => {
                write!(str, "AT+UWSC={config_id},5,2").or(Err(()))?;
                self.send_command(str.as_str()).await?;
                self.read_ok().await?;
                str.clear();
                write!(str, "AT+UWSC={config_id},8,\"{passphrase}\"").or(Err(()))?;
                self.send_command(str.as_str()).await?;
                self.read_ok().await?;
            }
            _ => return Err(()),
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

    pub async fn disconnect_wifi(&self, config_id: u8) -> Result<(), ()> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UWSCA={config_id},4").or(Err(()))?;
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
                    return Ok(());
                },
                embassy_futures::select::Either::Second(_) => {
                    defmt::warn!("disconnect timed out");
                    return Err(());
                },
            }
    }

    pub async fn connect_wifi(&self, config_id: u8) -> Result<(), ()> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UWSCA={config_id},3").or(Err(()))?;
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
                        return Err(());
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
    ) -> Result<(), ()> {
        let mut str: String<64> = String::new();
        let server_type = server_type as u8;
        write!(str, "AT+UDSC={server_id},{server_type},{port},1,0").or(Err(()))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
        Ok(())
    }

    pub async fn connect_peer(&self, url: &str) -> Result<PeerConnection, ()> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UDCP={url}").or(Err(()))?;
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
                                peer_id = Some(resp[i + 6..].parse::<u8>().or(Err(()))?);
                            } else {
                                return Err(());
                            }
                        }
                        _ => {
                            return Err(());
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

    pub async fn close_peer(&self, peer_id: u8) -> Result<(), ()> {
        let mut str: String<12> = String::new();
        write!(str, "AT+UDCPC={peer_id}").or(Err(()))?;
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
                            return Err(());
                        }
                    };

                    Ok(())
                })
                .await?;
        }

        Ok(())
    }

    pub async fn send_data(&self, channel_id: u8, data: &[u8]) -> Result<(), ()> {
        let res = self.writer.enqueue(|buf| {
                EdmPacket::DataCommand {
                    channel: channel_id,
                    data: data,
                }
                .write(buf).unwrap()
            });

        if res.is_err() {
            // queue was full
            return Err(());
        }

        Ok(())
    }

    pub async fn read_data<RET, FN>(&'a self, fn_read: FN) -> Result<RET, ()>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        self.reader
            .dequeue(|buf| {
                if let EdmPacket::DataEvent { channel: _, data } = self.to_packet(buf)? {
                    Ok(fn_read(data))
                } else {
                    Err(())
                }
            }).await
    }

    pub fn can_read_data(&'a self) -> bool {
        self.reader.can_dequque()
    }

    pub fn try_read_data<RET, FN>(&'a self, fn_read: FN) -> Result<RET, ()>
    where FN: FnOnce(&[u8]) -> RET,
    {
        match self.reader.try_dequeue() {
            Ok(buf) => {
                match self.to_packet(buf.data()) {
                    Ok(pkt) => {
                        if let EdmPacket::DataEvent { channel: _ , data } = pkt {
                            return Ok(fn_read(data))
                        } else {
                            // defmt::trace!("got non data event");
                            return Err(());
                        }
                    },
                    Err(_) => {
                        // defmt::trace!("got data that wasn't an edm packet: {}", buf.data());
                        return Err(());
                    },
                }
                // we read something

            },
            Err(queue::Error::QueueFullEmpty) => {
                // nothing to read
                return Err(());
            }
            Err(queue::Error::InProgress) => {
                // you did something illegal
                return Err(());
            },
        }
    }

    pub async fn send_command(&self, cmd: &str) -> Result<(), ()> {
        match self.mode {
            RadioMode::CommandMode => {
                let res = self.writer.enqueue(|buf| {
                        buf[..cmd.len()].clone_from_slice(cmd.as_bytes());
                        buf[cmd.len()] = b'\r';
                        cmd.len() + 1
                    });

                if res.is_err() {
                    // queue was full
                    return Err(())
                }
                
                Ok(())
            }
            RadioMode::ExtendedDataMode => {
                let res = self.writer.enqueue(|buf| EdmPacket::ATRequest(cmd).write(buf).unwrap());

                if res.is_err() {
                    // queue was full
                    return Err(())
                }

                Ok(())
            }
            _ => Err(()),
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

    async fn read_ok(&self) -> Result<(), ()> {
        let mut res = Err(());
        loop {
            let brk = self.reader.dequeue(|buf| {
                // defmt::warn!("buf: {}", buf);
                let brk = if let Ok(packet) = self.to_packet(buf) {
                    match packet {
                        EdmPacket::ConnectEvent { channel, event_type } => false,
                        EdmPacket::DisconnectEvent { channel } => false,
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
                                ATEvent::PeerConnectedIP { peer_handle, is_ipv6, protocol, local_address, local_port, remote_address, remote_port } => false,
                                ATEvent::PeerDisconnected { peer_handle } => false,
                                ATEvent::WifiLinkConnected { conn_id, bssid, channel } => false,
                                ATEvent::WifiLinkDisconnected { conn_id, reason } => false,
                                ATEvent::WifiAccessPointUp { id } => false,
                                ATEvent::WifiAccessPointDown { id } => false,
                                ATEvent::WifiAccessPointStationConnected { id, mac_addr } => false,
                                ATEvent::WifiAccessPointStationDisconnected { id } => false,
                                ATEvent::NetworkUp { interface_id } => false,
                                ATEvent::NetworkDown { interface_id } => false,
                                ATEvent::NetworkError { interface_id, code } => false,
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

                return brk;
            })
            .await;

            if brk {
                break;
            }
            
        }

        return res;
    }

    // async fn read_ok(&self) -> Result<(), ()> {
    //     self.reader
    //         .dequeue(|buf| {
    //             if let EdmPacket::ATResponse(ATResponse::Ok("")) = self.to_packet(buf)? {
    //                 Ok(())
    //             } else {
    //                 Err(())
    //             }
    //         })
    //         .await
    // }

    async fn read_ok_at_edm_transition(&self) -> Result<bool, ()> {
        let transition_buf: [u8; 12] = [13, 10, 79, 75, 13, 10, 170, 0, 2, 0, 113, 85];
        let res = self.reader.dequeue(|buf| {
            if buf.len() == transition_buf.len() && buf.iter().zip(transition_buf.iter()).all(|(a,b)| a == b) {
                Ok(true)
            } else {
                if let EdmPacket::ATResponse(ATResponse::Ok("")) = self.to_packet(buf)? {
                    Ok(false)
                } else {
                    Err(())
                }
            }
        }).await;

        return res;
    }

    fn to_packet<'b>(&self, buf: &'b [u8]) -> Result<EdmPacket<'b>, ()> {
        match self.mode {
            RadioMode::CommandMode => Ok(EdmPacket::ATResponse(ATResponse::new(buf)?)),
            RadioMode::ExtendedDataMode => EdmPacket::new(buf),
            _ => Err(()),
        }
    }
}
