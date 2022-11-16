use core::future::Future;

use core::fmt::Write;
use embassy_stm32::usart;
use heapless::String;

use crate::{
    at_protocol::{ATEvent, ATResponse},
    edm_protocol::EdmPacket,
};

pub trait Reader<'a> {
    type F<RET, FN: FnOnce(&[u8]) -> RET>: Future<Output = Result<RET, ()>>
    where
        Self: 'a;

    fn read<RET, FN: FnOnce(&[u8]) -> RET>(&'a self, fn_read: FN) -> Self::F<RET, FN>;
}

pub trait Writer<'a> {
    type F<FN: FnOnce(&mut [u8]) -> usize>: Future<Output = Result<(), ()>>
    where
        Self: 'a;

    fn write<FN: FnOnce(&mut [u8]) -> usize>(&'a self, fn_write: FN) -> Self::F<FN>;
}

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

pub struct Radio<'a, R: Reader<'a>, W: Writer<'a>> {
    reader: &'a R,
    writer: &'a W,
    mode: RadioMode,
}

impl<'a, R: Reader<'a>, W: Writer<'a>> Radio<'a, R, W> {
    pub fn new(reader: &'a R, writer: &'a W) -> Self {
        Self {
            reader,
            writer,
            mode: RadioMode::CommandMode,
        }
    }

    pub async fn wait_startup(&self) -> Result<(), ()> {
        self.reader
            .read(|buf| {
                if let EdmPacket::ATResponse(ATResponse::Other("+STARTUP")) = self.to_packet(buf)? {
                    Ok(())
                } else {
                    Err(())
                }
            })
            .await?
    }

    pub async fn wait_edm_startup(&self) -> Result<(), ()> {
        self.reader
            .read(|buf| {
                if let EdmPacket::StartEvent = self.to_packet(buf)? {
                    Ok(())
                } else {
                    Err(())
                }
            })
            .await?
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
        Ok(())
    }

    async fn send_command(&self, cmd: &str) -> Result<(), ()> {
        match self.mode {
            RadioMode::CommandMode => {
                self.writer
                    .write(|buf| {
                        buf[..cmd.len()].clone_from_slice(cmd.as_bytes());
                        buf[cmd.len()] = b'\r';
                        cmd.len() + 1
                    })
                    .await?;
                Ok(())
            }
            RadioMode::ExtendedDataMode => {
                self.writer
                    .write(|buf| EdmPacket::ATRequest(cmd).write(buf).unwrap())
                    .await?; // TODO: unwrap
                Ok(())
            }
            _ => Err(()),
        }
    }

    async fn get_response(&self ) {

    }

    pub async fn read(&self) {
        self.reader
            .read(|buf| {
                defmt::info!("{:?}", self.to_packet(buf).unwrap());
            })
            .await.unwrap();
    }

    async fn read_ok(&self) -> Result<(), ()> {
        self.reader
            .read(|buf| {
                if let EdmPacket::ATResponse(ATResponse::Ok("")) = self.to_packet(buf)? {
                    Ok(())
                } else {
                    Err(())
                }
            })
            .await?
    }

    fn to_packet<'b>(&self, buf: &'b [u8]) -> Result<EdmPacket<'b>, ()> {
        match self.mode {
            RadioMode::CommandMode => Ok(EdmPacket::ATResponse(ATResponse::new(buf)?)),
            RadioMode::ExtendedDataMode => EdmPacket::new(buf),
            _ => Err(()),
        }
    }
}

impl<
        'a,
        UART: usart::BasicInstance,
        Dma: usart::RxDma<UART>,
        const LEN: usize,
        const DEPTH: usize,
    > Reader<'a> for motor_embassy::uart_queue::UartReadQueue<'a, UART, Dma, LEN, DEPTH>
{
    type F<RET, FN: FnOnce(&[u8]) -> RET> = impl Future<Output = Result<RET, ()>> where Self: 'a;

    fn read<RET, FN: FnOnce(&[u8]) -> RET>(&'a self, fn_read: FN) -> Self::F<RET, FN> {
        async { Ok(self.dequeue(|buf| fn_read(buf)).await) }
    }
}

impl<
        'a,
        UART: usart::BasicInstance,
        Dma: usart::TxDma<UART>,
        const LEN: usize,
        const DEPTH: usize,
    > Writer<'a> for motor_embassy::uart_queue::UartWriteQueue<'a, UART, Dma, LEN, DEPTH>
{
    type F<FN: FnOnce(&mut [u8]) -> usize> = impl Future<Output = Result<(), ()>> where Self: 'a;

    fn write<FN: FnOnce(&mut [u8]) -> usize>(&'a self, fn_write: FN) -> Self::F<FN> {
        async { self.enqueue(|buf| fn_write(buf)).or(Err(())) }
    }
}
