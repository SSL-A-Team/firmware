use core::future::Future;

use core::fmt::Write;
use embassy_stm32::usart;
use heapless::String;

use crate::uart_queue::{UartReadQueue, UartWriteQueue};

use super::at_protocol::{ATEvent, ATResponse};
use super::edm_protocol::EdmPacket;

// use crate::uart_queue::{Reader, Writer};

// pub trait Reader<'a> {
//     type F<RET, FN: FnOnce(&[u8]) -> RET>: Future<Output = Result<RET, ()>>
//     where
//         Self: 'a;

//     fn read<RET, FN: FnOnce(&[u8]) -> RET>(&'a self, fn_read: FN) -> Self::F<RET, FN>;
// }

// pub trait Writer<'a> {
//     type F<FN: FnOnce(&mut [u8]) -> usize>: Future<Output = Result<(), ()>>
//     where
//         Self: 'a;

//     fn write<FN: FnOnce(&mut [u8]) -> usize>(&'a self, fn_write: FN) -> Self::F<FN>;
// }

pub enum RadioMode {
    // Unknown,
    CommandMode,
    DataMode,
    ExtendedDataMode,
}

pub enum WifiAuth<'a> {
    Open,
    WPA { passphrase: &'a str },
    LEAP,
    PEAP,
    EAPTLS,
}

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

// pub struct Radio<'a, R: Reader<'a>, W: Writer<'a>> {
pub struct Radio<
    'a,
    UART: usart::BasicInstance,
    TxDma: usart::TxDma<UART>,
    RxDma: usart::RxDma<UART>,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
> {
    reader: &'a UartReadQueue<'a, UART, RxDma, LEN_RX, DEPTH_RX>,
    writer: &'a UartWriteQueue<'a, UART, TxDma, LEN_TX, DEPTH_TX>,
    mode: RadioMode,
    wifi_connected: bool,
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
    > Radio<'a, UART, TxDma, RxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX>
{
    pub fn new(
        reader: &'a UartReadQueue<'a, UART, RxDma, LEN_RX, DEPTH_RX>,
        writer: &'a UartWriteQueue<'a, UART, TxDma, LEN_TX, DEPTH_TX>,
    ) -> Self {
        Self {
            reader,
            writer,
            mode: RadioMode::CommandMode,
            wifi_connected: false,
        }
    }

    pub async fn wait_startup(&self) -> Result<(), ()> {
        self.reader
            .dequeue(|buf| {
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

    pub async fn enter_edm(&mut self) -> Result<(), ()> {
        self.send_command("ATO2").await?;
        self.read_ok().await?;
        self.mode = RadioMode::ExtendedDataMode;
        Ok(())
    }

    pub async fn set_host_name(&self, host_name: &str) -> Result<(), ()> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UNHN=\"{host_name}\"").or(Err(()))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;
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

    pub async fn config_wifi_addr(
        &self,
        config_id: u8,
        static_ip: Option<&str>,
        subnet_mask: Option<&str>,
        default_gateway: Option<&str>,
        dns_server1: Option<&str>,
        dns_server2: Option<&str>,
    ) -> Result<(), ()> {
        todo!("implement if needed");
    }

    pub async fn connect_wifi(&self, config_id: u8) -> Result<(), ()> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UWSCA={config_id},3").or(Err(()))?;
        self.send_command(str.as_str()).await?;
        self.read_ok().await?;

        let mut networkUp = 0;

        while networkUp < 2 {
            self.reader
                .dequeue(|buf| {
                    let packet = self.to_packet(buf)?;

                    if let EdmPacket::ATEvent(ATEvent::NetworkUp { interface_id: 0 }) = packet {
                        networkUp += 1;
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
                    match self.to_packet(buf)? {
                        EdmPacket::ATEvent(ATEvent::PeerConnectedIP {
                            peer_handle: _,
                            is_ipv6: _,
                            protocol: _,
                            local_address: _,
                            local_port: _,
                            remote_address: _,
                            remote_port: _,
                        }) => {
                            peer_connected_ip = true;
                        }
                        EdmPacket::ConnectEvent {
                            channel,
                            event_type: _,
                        } => {
                            channel_ret = Some(channel);
                        }
                        EdmPacket::ATResponse(ATResponse::Ok(resp)) => {
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
        self.writer
            .enqueue(|buf| {
                EdmPacket::DataCommand {
                    channel: channel_id,
                    data: data,
                }
                .write(buf)
                .unwrap()
            }); // TODO: unwrap
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

    pub async fn send_command(&self, cmd: &str) -> Result<(), ()> {
        match self.mode {
            RadioMode::CommandMode => {
                self.writer
                    .enqueue(|buf| {
                        buf[..cmd.len()].clone_from_slice(cmd.as_bytes());
                        buf[cmd.len()] = b'\r';
                        cmd.len() + 1
                    });
                Ok(())
            }
            RadioMode::ExtendedDataMode => {
                self.writer
                    .enqueue(|buf| EdmPacket::ATRequest(cmd).write(buf).unwrap());
                Ok(())
            }
            _ => Err(()),
        }
    }

    async fn get_response(&self) {}

    pub async fn read(&self) {
        self.reader
            .dequeue(|buf| {
                defmt::info!("{:?}", self.to_packet(buf).unwrap());
            })
            .await;
    }

    async fn read_ok(&self) -> Result<(), ()> {
        self.reader
            .dequeue(|buf| {
                if let EdmPacket::ATResponse(ATResponse::Ok("")) = self.to_packet(buf)? {
                    Ok(())
                } else {
                    Err(())
                }
            })
            .await
    }

    fn to_packet<'b>(&self, buf: &'b [u8]) -> Result<EdmPacket<'b>, ()> {
        match self.mode {
            RadioMode::CommandMode => Ok(EdmPacket::ATResponse(ATResponse::new(buf)?)),
            RadioMode::ExtendedDataMode => EdmPacket::new(buf),
            _ => Err(()),
        }
    }
}

// impl<
//         'a,
//         UART: usart::BasicInstance,
//         Dma: usart::RxDma<UART>,
//         const LEN: usize,
//         const DEPTH: usize,
//     > Reader<'a> for ateam_control_board::uart_queue::UartReadQueue<'a, UART, Dma, LEN, DEPTH>
// {
//     type F<RET, FN: FnOnce(&[u8]) -> RET> = impl Future<Output = Result<RET, ()>> where Self: 'a;

//     fn read<RET, FN: FnOnce(&[u8]) -> RET>(&'a self, fn_read: FN) -> Self::F<RET, FN> {
//         async { Ok(self.dequeue(|buf| fn_read(buf)).await) }
//     }
// }

// impl<
//         'a,
//         UART: usart::BasicInstance,
//         Dma: usart::TxDma<UART>,
//         const LEN: usize,
//         const DEPTH: usize,
//     > Writer<'a> for ateam_control_board::uart_queue::UartWriteQueue<'a, UART, Dma, LEN, DEPTH>
// {
//     type F<FN: FnOnce(&mut [u8]) -> usize> = impl Future<Output = Result<(), ()>> where Self: 'a;

//     fn write<FN: FnOnce(&mut [u8]) -> usize>(&'a self, fn_write: FN) -> Self::F<FN> {
//         async { self.enqueue(|buf| fn_write(buf)).or(Err(())) }
//     }
// }
