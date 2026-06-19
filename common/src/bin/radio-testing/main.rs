use std::{
    mem::size_of,
    net::{IpAddr, Ipv4Addr, UdpSocket},
    str::FromStr,
    time::Duration,
};

use ateam_common_packets::bindings::{
    BasicControl, BodyControlCommand, BodyControlMode, CommandCode, DribblerCommand, HelloRequest,
    HelloResponse, KickRequest, LocalVelocityCommand, RadioData, RadioHeader, RadioPacket,
};
use local_ip_address::local_ip;

fn main() -> std::io::Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:42069")?;

    let local_ip = if let IpAddr::V4(ip) = local_ip().unwrap() {
        ip
    } else {
        panic!("ipv6");
    };

    socket.join_multicast_v4(
        &Ipv4Addr::from_str("224.4.20.69").unwrap(),
        &Ipv4Addr::UNSPECIFIED,
    )?;

    let mut buf = [0; size_of::<RadioPacket>()];
    let src = loop {
        let (len, src) = socket.recv_from(&mut buf)?;
        if len == size_of::<RadioPacket>() - size_of::<RadioData>() + size_of::<HelloRequest>() {
            let packet = unsafe { &*(buf.as_ptr() as *const RadioPacket) };
            if packet.header.command_code == CommandCode::CC_HELLO_REQ {
                println!("Source: {src}");
                println!("{:?}", unsafe { packet.data.hello_request });
                break src;
            }
        }
    };

    let packet = RadioPacket {
        header: RadioHeader {
            crc32: 0,
            _reserved: 0,
            command_code: CommandCode::CC_HELLO_RESP,
            data_length: size_of::<HelloResponse>() as u16,
        },
        data: RadioData {
            hello_response: HelloResponse {
                ipv4: local_ip.octets(),
                port: 42069,
            },
        },
    };
    let packet_bytes = unsafe {
        core::slice::from_raw_parts(
            &packet as *const _ as *const u8,
            size_of::<RadioPacket>() - size_of::<RadioData>()
                + core::mem::size_of::<HelloResponse>(),
        )
    };
    socket.send_to(packet_bytes, src)?;

    let mut packet = RadioPacket {
        header: RadioHeader {
            crc32: 0,
            _reserved: 0,
            command_code: CommandCode::CC_CONTROL,
            data_length: size_of::<BasicControl>() as u16,
        },
        data: RadioData {
            control: BasicControl {
                _bitfield_1: Default::default(),
                _bitfield_align_1: Default::default(),
                vision_position_update: [0.0, 0.0, 0.0],

                body_control_mode: BodyControlMode::BCM_LOCAL_VELOCITY,
                kick_request: KickRequest::KR_ARM,
                play_song: 0,
                dribbler_mode: DribblerCommand::DC_DISABLE,

                kick_vel: 0.,
                dribbler_setpoint: 0.,

                cmd: BodyControlCommand {
                    local_vel: LocalVelocityCommand {
                        local_xd: 0.0,
                        local_yd: 0.0,
                        local_omega: 0.0,
                        max_linear_acc: 0.0,
                        max_angular_acc: 0.0,
                    },
                },
            },
        },
    };
    let packet_bytes = unsafe {
        core::slice::from_raw_parts(
            &packet as *const _ as *const u8,
            core::mem::size_of::<RadioPacket>() - core::mem::size_of::<RadioData>()
                + core::mem::size_of::<BasicControl>(),
        )
    };

    // std::thread::sleep(Duration::from_millis(500));

    // for _ in 0..5 {
    //     packet.data.control.kick_request = KickRequest::KR_ARM;
    //     socket.send_to(packet_bytes, src)?;
    //     std::thread::sleep(Duration::from_millis(100));
    // }

    // std::thread::sleep(Duration::from_millis(1000));

    // // packet.data.control.kick_request = KickRequest::KR_KICK_NOW;
    // // packet.data.control.kick_vel = 0.5;
    // // socket.send_to(packet_bytes, src)?;

    // // std::thread::sleep(Duration::from_millis(1000));

    // loop {
    //     packet.data.control.kick_vel = 0.5;
    //     packet.data.control.kick_request = KickRequest::KR_KICK_TOUCH;
    //     // packet.data.control.vel_z_angular = 0.1;
    //     socket.send_to(packet_bytes, src)?;
    //     std::thread::sleep(Duration::from_millis(1000));
    // }

    // 0.001
    let mut vel = 0.;
    // let max = 0.001;
    let max = 1.0;
    let mut up = true;
    loop {
        // packet.data.control.x_linear_cmd = vel;
        packet.data.control.cmd.local_vel.local_omega = 0.0;
        socket.send_to(packet_bytes, src)?;
        std::thread::sleep(Duration::from_millis(10));

        if up {
            vel += max / 200.;
        } else {
            vel -= max / 200.;
        }
        if vel >= max {
            up = false;
        }
        if vel <= -max {
            up = true;
        }
    }
}
