use super::radio::{PeerConnection, Radio, WifiAuth};
use crate::uart_queue::{UartReadQueue, UartWriteQueue};
use ateam_common_packets::bindings_radio::{
    self, BasicControl, CommandCode, HelloRequest, HelloResponse, RadioPacket, RadioPacket_Data, BasicTelemetry,
};
use const_format::formatcp;
use core::fmt::Write;
use core::mem::size_of;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::{Level, OutputOpenDrain, Pin, Pull, Speed, Output};
use embassy_stm32::pac;
use embassy_stm32::usart;
use embassy_stm32::Peripheral;
use embassy_time::{Duration, Timer};
use heapless::String;

const MULTICAST_IP: &str = "224.4.20.69";
const MULTICAST_PORT: u16 = 42069;
const LOCAL_PORT: u16 = 42069;
const TEAM_WIFI_SSID: &str = "A-Team Field";
const COMP_MAIN_WIFI_SSID: &str = "T3_SSL_RBC23";
const COMP_PRACTICE_WIFI_SSID: &str = "T1_SSL_RBC23";
// const WIFI_SSID: &str = "PROMISED_LAN_DC_DEVEL";

const TEAM_WIFI_PASS: &str = "plancomestogether";
const COMP_MAIN_WIFI_PASS: &str = "1fNrzxtSHG5o9";
const COMP_PRACTICE_WIFI_PASS: &str = "e568Cwg0PjwcI";
// const WIFI_PASS: &str = "plddevel";

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

fn get_uuid() -> u16 {
    unsafe { *(0x1FF1_E800 as *const u16) }
}

unsafe impl<
        'a,
        UART: usart::BasicInstance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        PIN: Pin,
    > Send for RobotRadio<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_TX, DEPTH_RX, PIN>
{
}

pub struct RobotRadio<
    'a,
    UART: usart::BasicInstance,
    DmaRx: usart::RxDma<UART>,
    DmaTx: usart::TxDma<UART>,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    PIN: Pin,
> {
    radio: Radio<
        'a,
        UART,
        DmaTx,
        DmaRx,
        LEN_TX,
        LEN_RX,
        DEPTH_TX,
        DEPTH_RX,
    >,
    reset_pin: Output<'a, PIN>,
    peer: Option<PeerConnection>,
}

impl<
        'a,
        UART: usart::BasicInstance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        PIN: Pin,
    > RobotRadio<'a, UART, DmaRx, DmaTx, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, PIN>
{
    pub async fn new(
        read_queue: &'a UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_RX>,
        write_queue: &'a UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_TX>,
        reset_pin: impl Peripheral<P = PIN> + 'a,
    ) -> Result<RobotRadio<'a, UART, DmaRx, DmaTx, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, PIN>, ()>
    {
        let mut reset_pin = Output::new(reset_pin, Level::High, Speed::Medium);
        Timer::after(Duration::from_micros(100)).await;
        reset_pin.set_low();

        let mut radio = Radio::new(read_queue, write_queue);
        radio.wait_startup().await?;

        let baudrate = 5_250_000;
        radio.set_echo(false).await?;
        radio.config_uart(baudrate, false, 8, true).await?;

        let div = (UART::frequency().0 + (baudrate / 2)) / baudrate * UART::MULTIPLIER;
        unsafe {
            let r = UART::regs();
            r.cr1().modify(|w| {
                w.set_ue(false);
            });
            r.brr().modify(|w| {
                w.set_brr(div);
            });
            r.cr1().modify(|w| {
                w.set_ue(true);
                w.set_m0(pac::lpuart::vals::M0::BIT9);
                w.set_pce(true);
                w.set_ps(pac::lpuart::vals::Ps::EVEN);
            });
        };

        // Datasheet says wait at least 40ms after UART config change
        Timer::after(Duration::from_millis(50)).await;

        // Datasheet says wait at least 50ms after entering data mode
        radio.enter_edm().await?;
        radio.wait_edm_startup().await?;
        Timer::after(Duration::from_millis(50)).await;

        Ok(Self {
            radio,
            reset_pin,
            peer: None,
        })
    }

    pub async fn connect_to_network(&mut self, wifi_network: WifiNetwork) -> Result<(), ()> {
        let mut s = String::<17>::new();
        core::write!(&mut s, "A-Team Robot {:04X}", get_uuid()).unwrap();
        self.radio.set_host_name(s.as_str()).await?;
        let wifi_ssid = match wifi_network {
            WifiNetwork::Team => TEAM_WIFI_SSID,
            WifiNetwork::CompMain => COMP_MAIN_WIFI_SSID,
            WifiNetwork::CompPractice => COMP_PRACTICE_WIFI_SSID
        };

        let wifi_pass = match wifi_network {
            WifiNetwork::Team => TEAM_WIFI_PASS,
            WifiNetwork::CompMain => COMP_MAIN_WIFI_PASS,
            WifiNetwork::CompPractice => COMP_PRACTICE_WIFI_PASS
        };

        self.radio
            .config_wifi(
                1,
                wifi_ssid,
                WifiAuth::WPA {
                    passphrase: wifi_pass,
                },
            )
            .await?;
        self.radio.connect_wifi(1).await?;
        Ok(())
    }

    pub async fn open_multicast(&mut self) -> Result<(), ()> {
        let peer = self
            .radio
            .connect_peer(formatcp!(
                "udp://{MULTICAST_IP}:{MULTICAST_PORT}/?flags=1&local_port={LOCAL_PORT}"
            ))
            .await?;
        self.peer = Some(peer);
        Ok(())
    }

    pub async fn open_unicast(&mut self, ipv4: [u8; 4], port: u16) -> Result<(), ()> {
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
        let peer = self.radio.connect_peer(s.as_str()).await?;
        self.peer = Some(peer);
        Ok(())
    }

    pub async fn close_peer(&mut self) -> Result<(), ()> {
        if let Some(peer) = &self.peer {
            self.radio.close_peer(peer.peer_id).await?;
            self.peer = None;
            Ok(())
        } else {
            Err(())
        }
    }

    pub async fn send_data(&self, data: &[u8]) -> Result<(), ()> {
        if let Some(peer) = &self.peer {
            self.radio.send_data(peer.channel_id, data).await
        } else {
            Err(())
        }
    }

    pub async fn read_data<RET, FN>(&'a self, fn_read: FN) -> Result<RET, ()>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        if self.peer.is_some() {
            self.radio.read_data(fn_read).await
        } else {
            Err(())
        }
    }

    pub async fn send_ack(&self, nack: bool) -> Result<(), ()> {
        let packet = RadioPacket {
            crc32: 0,
            major_version: bindings_radio::kProtocolVersionMajor,
            minor_version: bindings_radio::kProtocolVersionMinor,
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
        self.send_data(packet_bytes).await?;

        Ok(())
    }

    pub async fn wait_ack(&self, timeout: Duration) -> Result<bool, ()> {
        let read_fut = self.read_data(|data| {
            if data.len() != size_of::<RadioPacket>() - size_of::<RadioPacket_Data>() {
                return Err(());
            }
            let packet = unsafe { &*(data as *const _ as *const RadioPacket) };

            match packet.command_code {
                CommandCode::CC_ACK => Ok(true),
                CommandCode::CC_NACK => Ok(false),
                _ => Err(()),
            }
        });
        match select(read_fut, Timer::after(timeout)).await {
            Either::First(ret) => ret?,
            Either::Second(_) => Err(()),
        }
    }

    pub async fn send_hello(&self, id: u8, team: TeamColor) -> Result<(), ()> {
        let packet = RadioPacket {
            crc32: 0,
            major_version: bindings_radio::kProtocolVersionMajor,
            minor_version: bindings_radio::kProtocolVersionMinor,
            command_code: CommandCode::CC_HELLO_REQ,
            data_length: size_of::<HelloRequest>() as u16,
            data: RadioPacket_Data {
                hello_request: HelloRequest {
                    robot_id: id,
                    color: match team {
                        TeamColor::Yellow => bindings_radio::TeamColor::TC_YELLOW,
                        TeamColor::Blue => bindings_radio::TeamColor::TC_BLUE,
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
        self.send_data(packet_bytes).await?;

        Ok(())
    }

    pub async fn send_telemetry(&self, telemetry: BasicTelemetry) -> Result<(), ()> {
        let packet = RadioPacket {
            crc32: 0,
            major_version: bindings_radio::kProtocolVersionMajor,
            minor_version: bindings_radio::kProtocolVersionMinor,
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
        self.send_data(packet_bytes).await?;

        Ok(())
    }

    pub async fn wait_hello(&self, timeout: Duration) -> Result<HelloResponse, ()> {
        let read_fut = self.read_data(|data| {
            const PACKET_SIZE: usize = size_of::<RadioPacket>() - size_of::<RadioPacket_Data>()
                + size_of::<HelloResponse>();
            if data.len() != PACKET_SIZE {
                return Err(());
            }

            let mut data_copy = [0u8; PACKET_SIZE];
            data_copy.clone_from_slice(&data[0..PACKET_SIZE]);

            let packet = unsafe { &*(&data_copy as *const _ as *const RadioPacket) };

            if packet.command_code != CommandCode::CC_HELLO_RESP {
                return Err(());
            }
            // TODO: handle nack

            Ok(unsafe { packet.data.hello_response })
        });

        match select(read_fut, Timer::after(timeout)).await {
            Either::First(ret) => ret?,
            Either::Second(_) => Err(()),
        }
    }

    pub async fn read_control(&self) -> Result<BasicControl, ()> {
        self.read_data(|data| {
            const PACKET_SIZE: usize = size_of::<RadioPacket>() - size_of::<RadioPacket_Data>()
                + size_of::<BasicControl>();
            if data.len() != PACKET_SIZE {
                return Err(());
            }

            let mut data_copy = [0u8; PACKET_SIZE];
            data_copy.clone_from_slice(&data[0..PACKET_SIZE]);

            let packet = unsafe { &*(&data_copy as *const _ as *const RadioPacket) };

            if packet.command_code != CommandCode::CC_CONTROL {
                return Err(());
            }

            Ok(unsafe { packet.data.control })
        })
        .await?
    }
}
