use core::{mem::size_of, slice};

use crate::{
    radio::radio::{Ipv4Addr, Radio, UdpSocket},
    task::Task,
};
use ateam_common_packets::bindings_radio::{
    self, BasicControl, BasicTelemetry, CommandCode, HelloRequest, HelloResponse, RadioHeader,
};
use embassy_futures::select;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{with_timeout, Duration, Instant, Ticker};
use futures_util::StreamExt;

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TeamColor {
    Yellow,
    Blue,
}

pub trait RobotRadio {
    const WIFI_SSID: &'static str;
    const WIFI_PASS: Option<&'static str> = None;
    const MULTICAST_ADDR: Ipv4Addr;
    const MULTICAST_PORT: u16;
    const CONTROL_TIMEOUT: Duration;
    const HELLO_RATE: Duration;

    fn get_robot_id(&self) -> u8;
    fn get_robot_color(&self) -> TeamColor;
    fn get_robot_name(&self) -> Option<heapless::String<30>> {
        None
    }

    // fn recieved_control(&self, control: &BasicControl);
    // async fn wait_send_telemetry(&self) -> BasicTelemetry;
    // async fn shutdown(&self);

    // async fn read_control(&self) -> BasicControl;
    // fn send_telemetry(&self, telemetry: &BasicTelemetry);
    // fn shutdown(&self);
}

// pub struct RobotRadioTask<Robot: RobotRadio + 'static, R: Radio<'static> + 'static> {
//     pub robot: &'static Robot,
//     pub radio: &'static R,
// }

pub struct RobotRadioTask<Robot: RobotRadio + 'static, R: Radio + 'static> {
    robot: &'static Robot,
    radio: &'static R,
    latest_control: Mutex<CriticalSectionRawMutex, Option<BasicControl>>,
    telemetry: Signal<CriticalSectionRawMutex, BasicTelemetry>,
    shutdown: Signal<CriticalSectionRawMutex, ()>,
}

// unsafe impl<Robot: RobotRadio + 'static, R: Radio<'static> + 'static> Sync for RobotRadioTask<Robot, R> {}
// unsafe impl<Robot: RobotRadio + 'static, R: Radio<'static> + 'static> Send for RobotRadioTask<Robot, R> {}


impl<Robot: RobotRadio + 'static, R: Radio + 'static> RobotRadioTask<Robot, R> {
    pub fn new(robot: &'static Robot, radio: &'static R) -> Self {
        Self {
            robot,
            radio,
            latest_control: Mutex::new(None),
            telemetry: Signal::new(),
            shutdown: Signal::new(),
        }
    }

    pub fn take_latest_control(&self) -> Option<BasicControl> {
        let mut latest_control = self.latest_control.try_lock();
        if let Ok(latest_control) = &mut latest_control {
            latest_control.take()
        } else {
            None
        }
    }

    pub fn send_telemetry(&self, telemetry: &BasicTelemetry) {
        self.telemetry.signal(*telemetry);
    }

    pub fn shutdown(&self) {
        self.shutdown.signal(());
    }
}

// impl<Robot: RobotRadio + 'static, R: Radio<'static> + 'static> Task for (Robot, R) {
//     type Data = (&'static Robot, &'static R);

//     async fn task((robot, radio): Self::Data) {
impl<Robot: RobotRadio + 'static, R: Radio + 'static> Task for RobotRadioTask<Robot, R> {
    type Data = RobotRadioTask<Robot, R>;

    async fn task(data: Self::Data) {
        let robot = data.robot;
        let radio = data.radio;
        {
            let hostname = robot.get_robot_name();
            let hostname = hostname.as_ref().map(|h| h.as_str());
            radio
                .connect_to_network(Robot::WIFI_SSID, Robot::WIFI_PASS, hostname)
                .await.unwrap();
        }

        loop {
            let multicast_socket = radio
                .open_udp(Robot::MULTICAST_ADDR, Robot::MULTICAST_PORT)
                .await.unwrap();

            let ((addr, port), robot_id, robot_color) = {
                let mut ticker = Ticker::every(Robot::HELLO_RATE);
                loop {
                    const MAX_SIZE: usize = size_of::<RadioHeader>()
                        + core::cmp::max(size_of::<HelloResponse>(), size_of::<HelloRequest>());
                    let mut buf = [0; MAX_SIZE];
                    let robot_id = robot.get_robot_id();
                    let robot_color = robot.get_robot_color();
                    let buf_len = write_hello_req(&mut buf, robot_id, robot_color);
                    info!(
                        "Sending hello to {}:{} (id: {}, color: {:?})",
                        Robot::MULTICAST_ADDR,
                        Robot::MULTICAST_PORT,
                        robot_id,
                        robot_color,
                    );
                    let res = multicast_socket.send(&buf[0..buf_len]).await;
                    if let Ok(_) = res {
                        let res =
                            with_timeout(Robot::HELLO_RATE, multicast_socket.recv(&mut buf)).await;
                        if let Ok(Ok(size)) = res {
                            let resp = read_hello_resp(&buf[0..size]);
                            if let Ok(addr) = resp {
                                info!("Recieved hello resp from {}:{}", addr.0, addr.1);
                                break (addr, robot_id, robot_color);
                            }
                        }
                    }
                    ticker.next().await;
                }
            };

            multicast_socket.disconnect().await;
            drop(multicast_socket);

            let unicast_socket = radio.open_udp(addr, port).await.unwrap();

            let either = select::select4(
                data.shutdown.wait(),
                async {
                    let mut ticker = Ticker::every(Robot::HELLO_RATE);
                    loop {
                        let new_robot_id = robot.get_robot_id();
                        let new_robot_color = robot.get_robot_color();
                        if robot_id != new_robot_id || robot_color != new_robot_color {
                            info!("Robot id/color changed");
                            break;
                        }
                        ticker.next().await;
                    }
                },
                async {
                    loop {
                        let mut buf = [0; size_of::<RadioHeader>() + size_of::<BasicTelemetry>()];
                        let telemetry = data.telemetry.wait().await;
                        write_telemetry(&mut buf, &telemetry);
                        trace!("Sending telemetry");
                        let res = unicast_socket.send(&buf).await;
                        if let Err(_err) = res {}
                    }
                },
                async {
                    let mut last_control = Instant::now();
                    loop {
                        let mut buf = [0; size_of::<RadioHeader>() + size_of::<BasicControl>()];
                        let res =
                            with_timeout(Robot::CONTROL_TIMEOUT, unicast_socket.recv(&mut buf))
                                .await;
                        let now = Instant::now();
                        if let Ok(Ok(size)) = res {
                            // TODO: really don't want to clone this
                            let header = unsafe { &*(&buf as *const _ as *const RadioHeader) }.clone();
                            if size == size_of::<RadioHeader>()
                                && header.command_code == CommandCode::CC_GOODBYE
                            {
                                info!("Recieved Goodbye");
                                break;
                            } else if size == size_of::<RadioHeader>() + size_of::<BasicControl>()
                                && header.command_code == CommandCode::CC_CONTROL
                            {
                                trace!("Recieved Control");
                                // TODO: don't clone
                                let control = unsafe {
                                    &*(&buf[size_of::<RadioHeader>()] as *const _
                                        as *const BasicControl)
                                }.clone();
                                (*data.latest_control.lock().await).replace(control);
                                last_control = now;
                            } else {
                                if size >= size_of::<RadioHeader>() {
                                    info!(
                                        "Recieved other packet, size: {}, code: {}",
                                        size, header.command_code
                                    );
                                } else {
                                    info!("Recieved other packet, size: {}", size);
                                }
                            }
                        }

                        if now - last_control > Robot::CONTROL_TIMEOUT {
                            info!("Radio Control timeout");
                            break;
                        }
                    }
                },
            )
            .await;

            let mut buf = [0; size_of::<RadioHeader>()];
            write_goodbye(&mut buf);
            info!("Sending Goodbye");
            let res = unicast_socket.send(&buf).await;
            if let Err(_err) = res {}

            unicast_socket.disconnect().await;
            drop(unicast_socket);

            match either {
                select::Either4::First(_) => return,
                _ => {}
            }
        }
    }
}

fn write_header(buf: &mut [u8], command_code: u8, data_len: usize) {
    let header_len = size_of::<RadioHeader>();
    assert!(buf.len() >= header_len + data_len);
    let header = unsafe { &mut *(buf as *mut _ as *mut RadioHeader) };

    *header = RadioHeader {
        crc32: 0,
        major_version: bindings_radio::kProtocolVersionMajor,
        minor_version: bindings_radio::kProtocolVersionMinor,
        command_code: command_code,
        data_length: data_len as u16,
    };
}

fn write_hello_req(buf: &mut [u8], id: u8, team: TeamColor) -> usize {
    let header_len = size_of::<RadioHeader>();
    let data_len = size_of::<HelloRequest>();
    write_header(buf, CommandCode::CC_HELLO_REQ, data_len);
    let hello = unsafe { &mut *(&mut buf[header_len] as *mut _ as *mut HelloRequest) };

    *hello = HelloRequest {
        robot_id: id,
        color: match team {
            TeamColor::Yellow => bindings_radio::TeamColor::TC_YELLOW,
            TeamColor::Blue => bindings_radio::TeamColor::TC_BLUE,
        },
    };

    header_len + data_len
}

fn write_goodbye(buf: &mut [u8]) -> usize {
    write_header(buf, CommandCode::CC_GOODBYE, 0);

    size_of::<RadioHeader>()
}

fn write_telemetry(buf: &mut [u8], telemetry: &BasicTelemetry) -> usize {
    let header_len = size_of::<RadioHeader>();
    let data_len = size_of::<BasicTelemetry>();
    write_header(buf, CommandCode::CC_TELEMETRY, data_len);

    let data = unsafe { slice::from_raw_parts(telemetry as *const _ as *const u8, data_len) };
    buf[header_len..header_len + data_len].clone_from_slice(data);

    header_len + data_len
}

fn read_hello_resp(buf: &[u8]) -> Result<(Ipv4Addr, u16), ()> {
    let header_len = size_of::<RadioHeader>();
    let data_len = size_of::<HelloResponse>();
    if buf.len() != header_len + data_len {
        return Err(());
    }

    let header = unsafe { &*(buf as *const _ as *const RadioHeader) };
    if header.command_code != CommandCode::CC_HELLO_RESP {
        return Err(());
    }

    let hello = unsafe { &*(&buf[header_len] as *const _ as *const HelloResponse) };

    Ok((Ipv4Addr { octets: hello.ipv4 }, hello.port))
}
