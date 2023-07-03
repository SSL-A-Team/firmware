use core::fmt;

pub struct Ipv4Addr {
    pub octets: [u8; 4],
}

// TODO: template radio lifetime instead static
pub trait Radio {
    type UdpSocket: UdpSocket;

    async fn connect_to_network(&self, ssid: &str, pass: Option<&str>, hostname: Option<&str>) -> Result<(), ()>;
    async fn open_udp(&'static self, addr: Ipv4Addr, port: u16) -> Result<Self::UdpSocket, ()>;
}

pub trait UdpSocket {
    async fn send(&self, buf: &[u8]) -> Result<usize, ()>;
    async fn recv(&self, buf: &mut [u8]) -> Result<usize, ()>;

    async fn disconnect(&self) {}
}

impl fmt::Display for Ipv4Addr {
    fn fmt(&self, fmt: &mut fmt::Formatter<'_>) -> fmt::Result {
        let octets = self.octets;
        write!(
            fmt,
            "{}.{}.{}.{}",
            octets[0], octets[1], octets[2], octets[3]
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Ipv4Addr {
    fn format(&self, fmt: defmt::Formatter) {
        let octets = self.octets;
        defmt::write!(
            fmt,
            "{}.{}.{}.{}",
            octets[0],
            octets[1],
            octets[2],
            octets[3]
        )
    }
}
