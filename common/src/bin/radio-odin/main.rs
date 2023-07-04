////////////////////////////
//  TODO: this is broken  //
////////////////////////////

#![feature(async_fn_in_trait)]
#![feature(type_alias_impl_trait)]

// use core::slice::SlicePattern;
use std::{collections::VecDeque, net};

use ateam_common::{
    radio::radio::{Ipv4Addr, Radio, UdpSocket},
    radio::{
        edm_protocol::EdmPacket,
        odin_radio::{OdinRadio, RadioInterfaceControl, RadioMode},
        robot_radio::{RobotRadio, RobotRadioTask, TeamColor},
    },
    task::TaskStorage,
    transfer::{DataRefReadTrait, DataRefWriteTrait, Reader, Reader2, Writer, Writer2},
};
use ateam_common_packets::bindings_radio::{BasicControl, BasicTelemetry};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
use futures_util::StreamExt;

// struct RadioMock;
struct RobotMock;

// struct UdpSockermMock {
//     addr: net::SocketAddr,
//     socket: async_net::UdpSocket,
// }

// impl UdpSocket for UdpSockermMock {
//     async fn send(&self, buf: &[u8]) -> Result<usize, ()> {
//         let res = self.socket.send_to(buf, self.addr).await;
//         res.or(Err(()))
//     }
//     async fn recv(&self, buf: &mut [u8]) -> Result<usize, ()> {
//         let res = self.socket.recv(buf).await;
//         res.or(Err(()))
//     }
// }

// impl<'a> Radio<'a> for RadioMock {
//     type UdpSocket = UdpSockermMock;

//     async fn connect_to_network(&self) {}
//     async fn open_udp(&self, addr: Ipv4Addr, port: u16) -> UdpSockermMock {
//         let addr_local = net::SocketAddr::from(([0, 0, 0, 0], 42070));
//         let addr = net::SocketAddr::from((addr.octets, port));
//         println!("{}", addr);
//         let socket = async_net::UdpSocket::bind(addr_local).await.unwrap();

//         UdpSockermMock { addr, socket }
//     }
// }

impl RobotRadio for RobotMock {
    const WIFI_SSID: &'static str = "Network 123";
    const WIFI_PASS: Option<&'static str> = Some("WIFI PASS");
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
}

struct RadioMockInternal {
    mode: RadioMode,
    responses: VecDeque<Vec<u8>>,
}

struct RadioInterfaceMock {
    data: Mutex<CriticalSectionRawMutex, RadioMockInternal>,
}

const STARTUP_STR: &str = "\r\n+STARTUP\r\n";
const OK_STR: &str = "\r\nOK\r\n";

impl RadioInterfaceMock {
    fn new() -> Self {
        let mut responses = VecDeque::new();
        responses.push_back(STARTUP_STR.into());
        Self {
            data: Mutex::new(RadioMockInternal {
                mode: RadioMode::CommandMode,
                responses,
            }),
        }
    }
}

struct VecDataRefRead {
    vec: Vec<u8>,
}

impl DataRefReadTrait for VecDataRefRead {
    fn data(&self) -> &[u8] {
        self.vec.as_slice()
    }
    fn cancel(self) {}
}

impl Reader2 for RadioInterfaceMock {
    type DataRefRead = VecDataRefRead;

    async fn read<'a>(&'a self) -> Result<Self::DataRefRead, ()> {
        let mut ticker = Ticker::every(Duration::from_millis(500));
        loop {
            {
                println!("r");
                let data = &mut *self.data.lock().await;
                let response = data.responses.pop_front();
                if let Some(response) = response {
                    println!("read {:?}", &response);
                    return Ok(VecDataRefRead {
                        vec: response
                    });
                }
            }
            ticker.next().await;
        }
    }
}

struct VecDataRefWrite<'a> {
    data: &'a Mutex<CriticalSectionRawMutex, RadioMockInternal>,
    vec: Vec<u8>,
    len: usize,
}

impl<'a> DataRefWriteTrait for VecDataRefWrite<'a> {
    fn data(&mut self) -> &mut [u8] {
        self.vec.as_mut_slice()
    }
    fn len(&mut self) -> &mut usize {
        &mut self.len
    }
    fn cancel(self) {}
}

impl Writer2 for RadioInterfaceMock {
    type DataRefWrite = VecDataRefWrite<'static>;

    async fn write(&self) -> Result<Self::DataRefWrite, ()> {
        let mut vec = Vec::new();
        unsafe { vec.set_len(100) };
        return Ok(VecDataRefWrite { data: &self.data, vec, len: 0 });
    }
}

impl Drop for VecDataRefWrite {
    fn drop(&mut self) {
        let mut buf = [0u8; 100];
        // let size = fn_write(&mut buf).unwrap();
        let data = &mut *self.data.lock().await;
        match data.mode {
            RadioMode::CommandMode => {
                let s = core::str::from_utf8(&buf[..size]).or(Err(()))?;
                println!("write {:?}", s);
                data.responses.push_back(OK_STR.into());

                if s == "ATO2\r" {
                    data.mode = RadioMode::ExtendedDataMode;
                    data.responses
                        .push_back(vec![0xAA, 0x00, 0x02, 0x00, 0x71, 0x55]);
                }
            }
            RadioMode::ExtendedDataMode => {
                let packet = EdmPacket::new(&buf[..size]);

                println!("write edm {:?}", packet);
                let mut resp = vec![0xAA, 0x00, 2 + OK_STR.len() as u8, 0x00, 0x45];
                resp.extend_from_slice(OK_STR.as_bytes());
                resp.push(0x55);
                data.responses.push_back(resp);

                if let Ok(EdmPacket::ATRequest(request)) = &packet {
                    if request.starts_with("AT+UWSCA") {
                        data.responses.push_back(create_edm_event("\r\n+UUNU:0\r\n"));
                        data.responses.push_back(create_edm_event("\r\n+UUNU:0\r\n"));
                        data.responses.push_back(create_edm_event("\r\n+UUWLE:1,0,0\r\n"));
                        // TODO: network up events
                    }
                }
            }
            RadioMode::DataMode => return Err(()),
        }
        Ok(())
    }
}

// impl Reader for RadioInterfaceMock {
//     async fn read<RET, FN: FnOnce(&[u8]) -> Result<RET, ()>>(
//         &self,
//         fn_read: FN,
//     ) -> Result<RET, ()> {
//         // Timer::after(Duration::from_millis(500)).await;
//         let mut ticker = Ticker::every(Duration::from_millis(500));
//         loop {
//             {
//                 println!("r");
//                 let data = &mut *self.data.lock().await;
//                 let response = data.responses.pop_front();
//                 if let Some(response) = response {
//                     println!("read {:?}", &response);
//                     return Ok(fn_read(&response).unwrap());
//                 }
//             }
//             ticker.next().await;
//         }
//     }
// }

fn create_edm_event(event: &str) -> Vec<u8> {
    let mut resp = vec![0xAA, 0x00, 2 + event.len() as u8, 0x00, 0x41];
    resp.extend_from_slice(event.as_bytes());
    resp.push(0x55);
    resp
}

// impl Writer for RadioInterfaceMock {
//     async fn write<FN: FnOnce(&mut [u8]) -> Result<usize, ()>>(
//         &self,
//         fn_write: FN,
//     ) -> Result<(), ()> {
//         let mut buf = [0u8; 100];
//         let size = fn_write(&mut buf).unwrap();
//         let data = &mut *self.data.lock().await;
//         match data.mode {
//             RadioMode::CommandMode => {
//                 let s = core::str::from_utf8(&buf[..size]).or(Err(()))?;
//                 println!("write {:?}", s);
//                 data.responses.push_back(OK_STR.into());

//                 if s == "ATO2\r" {
//                     data.mode = RadioMode::ExtendedDataMode;
//                     data.responses
//                         .push_back(vec![0xAA, 0x00, 0x02, 0x00, 0x71, 0x55]);
//                 }
//             }
//             RadioMode::ExtendedDataMode => {
//                 let packet = EdmPacket::new(&buf[..size]);

//                 println!("write edm {:?}", packet);
//                 let mut resp = vec![0xAA, 0x00, 2 + OK_STR.len() as u8, 0x00, 0x45];
//                 resp.extend_from_slice(OK_STR.as_bytes());
//                 resp.push(0x55);
//                 data.responses.push_back(resp);

//                 if let Ok(EdmPacket::ATRequest(request)) = &packet {
//                     if request.starts_with("AT+UWSCA") {
//                         data.responses.push_back(create_edm_event("\r\n+UUNU:0\r\n"));
//                         data.responses.push_back(create_edm_event("\r\n+UUNU:0\r\n"));
//                         data.responses.push_back(create_edm_event("\r\n+UUWLE:1,0,0\r\n"));
//                         // TODO: network up events
//                     }
//                 }
//             }
//             RadioMode::DataMode => return Err(()),
//         }
//         Ok(())
//     }
// }
impl RadioInterfaceControl for RadioInterfaceMock {
    async fn reset_radio(&self) {}
    async fn config_uart(&self, baudrate: u32, flow_control: bool, data_bits: u8, parity: bool) {}
}

static ROBOT: RobotMock = RobotMock;
static RADIO_TASK: TaskStorage<RobotRadioTask<RobotMock, OdinRadio<'static, RadioInterfaceMock>>> =
    TaskStorage::new();
static ODIN_TASK: TaskStorage<OdinRadio<'static, RadioInterfaceMock>> = TaskStorage::new();

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let interface = RadioInterfaceMock::new();
    let interface = unsafe { core::mem::transmute(&interface) };
    let odin_radio = OdinRadio::<'static, RadioInterfaceMock>::new(interface);
    let odin_radio = unsafe { core::mem::transmute(&odin_radio) };
    let robot_radio = RobotRadioTask::new(&ROBOT, odin_radio);

    spawner.spawn(ODIN_TASK.spawn(odin_radio)).unwrap();
    spawner.spawn(RADIO_TASK.spawn(robot_radio)).unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        ticker.next().await;
    }
}
