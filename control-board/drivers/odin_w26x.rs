pub trait WifiRadio {
    fn network_connect(ssid: &str, password: &str);
    fn network_disconnect();
    fn network_connected() -> Option<&str>; // return SSID

    fn socket_udp_open(port: u16) -> u8;
    fn socket_udp_close(sock_num: u8);

    fn has_packet() -> bool;
    fn get_packet() -> None;
    fn can_send_packet() -> bool;
    fn send_packet(pkt: None) -> None; // result
}