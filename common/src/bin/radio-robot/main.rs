#![feature(async_fn_in_trait)]
#![feature(type_alias_impl_trait)]

use std::net;

use ateam_common::{
    radio::radio::{Ipv4Addr, Radio, UdpSocket},
    radio::robot_radio::{RobotRadio, RobotRadioTask, TeamColor},
    task::TaskStorage,
};
use ateam_common_packets::bindings_radio::{BasicControl, BasicTelemetry};
use embassy_time::{Duration, Ticker, Timer};
use futures_util::StreamExt;

struct RadioMock;
struct RobotMock;

struct UdpSockermMock {
    addr: net::SocketAddr,
    socket: async_net::UdpSocket,
}

impl UdpSocket for UdpSockermMock {
    async fn send(&self, buf: &[u8]) -> Result<usize, ()> {
        let res = self.socket.send_to(buf, self.addr).await;
        res.or(Err(()))
    }
    async fn recv(&self, buf: &mut [u8]) -> Result<usize, ()> {
        let res = self.socket.recv(buf).await;
        res.or(Err(()))
    }
}

impl Radio for RadioMock {
    type UdpSocket = UdpSockermMock;

    async fn connect_to_network(
        &self,
        ssid: &str,
        pass: Option<&str>,
        hostname: Option<&str>,
    ) -> Result<(), ()> {
        Ok(())
    }
    async fn open_udp(&self, addr: Ipv4Addr, port: u16) -> Result<UdpSockermMock, ()> {
        let addr_local = net::SocketAddr::from(([0, 0, 0, 0], 42070));
        let addr = net::SocketAddr::from((addr.octets, port));
        println!("{}", addr);
        let socket = async_net::UdpSocket::bind(addr_local).await.unwrap();

        Ok(UdpSockermMock { addr, socket })
    }
}
use core::fmt::Write;

impl RobotRadio for RobotMock {
    const WIFI_SSID: &'static str = "Network 123";
    const WIFI_PASS: Option<&'static str> = None;
    const MULTICAST_ADDR: Ipv4Addr = Ipv4Addr {
        octets: [224, 4, 20, 69],
    };
    const MULTICAST_PORT: u16 = 42069;
    const CONTROL_TIMEOUT: Duration = Duration::from_millis(1000);
    const HELLO_RATE: Duration = Duration::from_millis(1000);

    fn get_robot_id(&self) -> u8 {
        // log::info!("get robot id");
        0
    }
    fn get_robot_color(&self) -> TeamColor {
        TeamColor::Blue
    }
    fn get_robot_name(&self) -> Option<heapless::String<30>> {
        let mut s = heapless::String::<30>::new();
        core::write!(&mut s, "A-Team Robot {:04X}", 0).unwrap();
        Some(s)
    }
}
static ROBOT: RobotMock = RobotMock;
static RADIO: RadioMock = RadioMock;
static RADIO_TASK: TaskStorage<RobotRadioTask<RobotMock, RadioMock>> = TaskStorage::new();

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let robot_radio = RobotRadioTask::new(&ROBOT, &RADIO);
    spawner.spawn(RADIO_TASK.spawn(robot_radio)).unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        ticker.next().await;
    }
}
