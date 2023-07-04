use std::{
    mem::size_of,
    net::{IpAddr, Ipv4Addr, UdpSocket},
    str::FromStr,
    time::Duration,
};

use ateam_common_packets::bindings_radio::{
    self, BasicControl, CommandCode, HelloRequest, HelloResponse, KickRequest, RadioPacket,
    RadioPacket_Data,
};
use local_ip_address::local_ip;

fn main() -> std::io::Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:42069")?;

    // let local_ip = if let IpAddr::V4(ip) = local_ip().unwrap() {
    //     ip
    // } else {
    //     panic!("ipv6");
    // };

    let local_ip = Ipv4Addr::new(172, 16, 1, 240);

    socket.join_multicast_v4(
        &Ipv4Addr::from_str("224.4.20.71").unwrap(),
        &Ipv4Addr::UNSPECIFIED,
    )?;

    let mut buf = [0; size_of::<RadioPacket>()];
    let mut src = loop {
        let (len, src) = socket.recv_from(&mut buf)?;
        if len == size_of::<RadioPacket>() - size_of::<RadioPacket_Data>() + size_of::<HelloRequest>()
        {
            let packet = unsafe { &*(buf.as_ptr() as *const RadioPacket) };
            if packet.command_code == CommandCode::CC_HELLO_REQ {
                println!("Source: {src}");
                println!("{:?}", unsafe { packet.data.hello_request });
                break src;
            }
        }
    };

    let packet = RadioPacket {
        crc32: 0,
        major_version: bindings_radio::kProtocolVersionMajor,
        minor_version: bindings_radio::kProtocolVersionMinor,
        command_code: CommandCode::CC_HELLO_RESP,
        data_length: size_of::<HelloResponse>() as u16,
        data: RadioPacket_Data {
            hello_response: HelloResponse {
                ipv4: local_ip.octets(),
                port: 42069,
            },
        },
    };
    let packet_bytes = unsafe {
        core::slice::from_raw_parts(
            &packet as *const _ as *const u8,
            size_of::<RadioPacket>() - size_of::<RadioPacket_Data>()
                + core::mem::size_of::<HelloResponse>(),
        )
    };
    socket.send_to(packet_bytes, src)?;

    let mut packet = RadioPacket {
        crc32: 0,
        major_version: bindings_radio::kProtocolVersionMajor,
        minor_version: bindings_radio::kProtocolVersionMinor,
        command_code: CommandCode::CC_CONTROL,
        data_length: size_of::<BasicControl>() as u16,
        data: RadioPacket_Data {
            control: BasicControl {
                vel_x_linear: 0.,
                vel_y_linear: 0.,
                vel_z_angular: 0.,
                kick_vel: 0.,
                dribbler_speed: 0.,
                kick_request: KickRequest::KR_DISABLE,
            },
        },
    };
// 0.001
    let mut vel = 0.;
    // let max = 0.001;
    let max = 1.0;
    let mut up = true;
    loop {
        // packet.data.control.vel_x_linear = vel;
        packet.data.control.vel_z_angular = vel;

        let packet_bytes = unsafe {
            core::slice::from_raw_parts(
                &packet as *const _ as *const u8,
                core::mem::size_of::<RadioPacket>() - core::mem::size_of::<RadioPacket_Data>()
                    + core::mem::size_of::<BasicControl>(),
            )
        };
        socket.send_to(packet_bytes, src)?;
        std::thread::sleep(Duration::from_millis(100));

        if up {
            vel += max/200.;
        } else {
            vel -= max/200.;
        }
        if vel >= max {
            up = false;
        }
        if vel <= -max {
            up = true;
        }
    }
}
