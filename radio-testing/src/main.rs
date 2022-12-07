use std::{net::{UdpSocket, Ipv4Addr}, str::FromStr};

fn main() -> std::io::Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:42069")?;
    
    socket.join_multicast_v4(
        &Ipv4Addr::from_str("224.4.20.69").unwrap(),
        &Ipv4Addr::UNSPECIFIED,
    )?;
    loop {
        let mut buf = [0; 10];
        let (len, src) = socket.recv_from(&mut buf)?;
        println!("{src}");
        println!("{:?}", &buf[..len]);
        socket.send_to(&[1, 3, 5], src)?;
    }
}
