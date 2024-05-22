
// pub enum ATRequest {
//     Attention,
//     ManufacturerID,
//     ModelID,
// }

#[derive(defmt::Format)]
pub enum WifiLinkDisconnectedReason {
    Unknown = 0,
    RemoteClose = 1,
    OutOfRange = 2,
    Roaming = 3,
    SecurityProblems = 4,
    NetworkDisabled = 5,
}

impl TryFrom<u8> for WifiLinkDisconnectedReason {
    type Error = ();

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::Unknown),
            1 => Ok(Self::RemoteClose),
            2 => Ok(Self::OutOfRange),
            3 => Ok(Self::Roaming),
            4 => Ok(Self::SecurityProblems),
            5 => Ok(Self::NetworkDisabled),
            _ => Err(()),
        }
    }
}

#[derive(defmt::Format)]
pub enum PeerConnectedProfile {
    TCP,
    UDP,
    MQTT,
}

impl TryFrom<u8> for PeerConnectedProfile {
    type Error = ();

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            1 => Ok(Self::TCP),
            2 => Ok(Self::UDP),
            6 => Ok(Self::MQTT),
            _ => Err(()),
        }
    }
}

#[allow(dead_code)]
#[derive(defmt::Format)]
pub enum ATEvent<'a> {
    DataAvailable {
        peer: u16,
        length: u16,
    }, // UUDATA
    PeerConnectedBluetooth {
        peer_handle: u16,
        profile: u8, // TODO: enum
        address: &'a str,
        frame_size: u16,
    }, // UUDPC
    PeerConnectedIP {
        peer_handle: u16,
        is_ipv6: bool,
        protocol: PeerConnectedProfile,
        local_address: &'a str,
        local_port: u16,
        remote_address: &'a str,
        remote_port: u16,
    }, // UUDPC
    PeerDisconnected {
        peer_handle: u16,
    }, // UUDPD
    BondEvent {
        bd_addr: &'a str,
        status: u8, // TODO: enum
    }, // UUBTB
    UserConfirmation {
        bd_addr: &'a str,
        value: u32,
    }, // UUBTUC
    UserPasskeyDisplay {
        bd_addr: &'a str,
        passkey: u32,
    }, // UUBTUPD
    UserPasskeyEntry {
        bd_addr: &'a str,
    }, // UUBTUPE
    ACLConnected {
        conn_handle: u16,
        type_: u8,
    }, // UUBTACLC
    ACLDisconnected {
        conn_handle: u16,
    }, // UUBTACLD
    WifiLinkConnected {
        conn_id: u16,
        bssid: &'a str,
        channel: u16,
    }, // UUWLE
    WifiLinkDisconnected {
        conn_id: u16,
        reason: WifiLinkDisconnectedReason,
    }, // UUWLD
    WifiAccessPointUp {
        id: u16,
    }, // UUWAPU
    WifiAccessPointDown {
        id: u16,
    }, // UUWAPD
    WifiAccessPointStationConnected {
        id: u16,
        mac_addr: &'a str,
    }, // UUWAPSTAC
    WifiAccessPointStationDisconnected {
        id: u16,
    }, // UUWAPSTAD
    EthernetLinkUp,   // UUETHLU
    EthernetLinkDown, // UUETHLD
    RemoteServiceConnected {
        handle: u16,
        local_url: &'a str,
        remote_url: &'a str,
    }, // UUDRSC
    RemoteServiceDisconnected {
        handle: u16,
    }, // UUDRSD
    NetworkUp {
        interface_id: u16,
    }, // UUNU
    NetworkDown {
        interface_id: u16,
    }, // UUND
    NetworkError {
        interface_id: u16,
        code: u16,
    }, // UUNERR
    GATTRequestToRead {
        conn_handle: u16,
        char_handle: u16,
    }, // UUBTGRR
    GATTRequestToWrite {
        conn_handle: u16,
        char_handle: u16,
        value: &'a str,
        options: u8,
    }, // UUBTGRW
    GATTIndicationConfirmation {
        conn_handle: u16,
        char_handle: u16,
    }, // UUBTGIC
    GATTNotification {
        conn_handle: u16,
        value_handle: u16,
        hex_data: &'a str,
    }, // UUBTGN
    GATTIndication {
        conn_handle: u16,
        value_handle: u16,
        hex_data: &'a str,
    }, // UUBTGI
    HTTPResponseEvent {
        peer_handle: u16,
        status_code: u16,
        length: u16,
        content_type: &'a str,
        content: &'a [u8],
    }, // UUDHTTP
    NFCReadEvent,     // UUNFCRD
}

impl<'b> ATEvent<'b> {
    const CR_LF: &'static str = "\r\n";

    const PEER_CONNECTED: &'static str = "+UUDPC";
    const PEER_DISCONNECTED: &'static str = "+UUDPD";
    const WIFI_LINK_CONNECTED: &'static str = "+UUWLE";
    const WIFI_LINK_DISCONNECTED: &'static str = "+UUWLD";
    const WIFI_ACCESS_POINT_UP: &'static str = "+UUWAPU";
    const WIFI_ACCESS_POINT_DOWN: &'static str = "+UUWAPD";
    const WIFI_AP_STATION_CONNECTED: &'static str = "+UUWAPSTAC";
    const WIFI_AP_STATION_DISCONNECTED: &'static str = "+UUWAPSTAD";
    const ETHERNET_LINK_UP: &'static str = "+UUETHLU";
    const ETHERNET_LINK_DOWN: &'static str = "+UUETHLD";
    const NETWORK_UP: &'static str = "+UUNU";
    const NETWORK_DOWN: &'static str = "+UUND";
    const NETWORK_ERROR: &'static str = "UUNERR";

    const PEER_CONNECTED_BLUETOOTH: &'static str = "1";
    const PEER_CONNECTED_IPV4: &'static str = "2";
    const PEER_CONNECTED_IPV6: &'static str = "3";

    pub fn new<'a>(buf: &'a [u8]) -> Result<ATEvent<'a>, ()> {
        let s = core::str::from_utf8(buf).or(Err(()))?;
        let s = &s[Self::CR_LF.len()..];
        if !s.ends_with(Self::CR_LF) {
            return Err(());
        }
        let s = &s[..s.len() - Self::CR_LF.len()];

        let d = s.find(':').unwrap_or(s.len());
        let event = &s[..d];
        let mut params = s[d + 1..].split(',');

        match event {
            Self::PEER_CONNECTED => {
                let peer_handle = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;
                let type_ = params.next().ok_or(())?;
                match type_ {
                    Self::PEER_CONNECTED_IPV4 | Self::PEER_CONNECTED_IPV6 => {
                        let protocol = params
                            .next()
                            .ok_or(())?
                            .parse::<u8>()
                            .or(Err(()))?
                            .try_into()?;
                        let local_address = params.next().ok_or(())?;
                        let local_port = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;
                        let remote_address = params.next().ok_or(())?;
                        let remote_port = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;

                        Ok(ATEvent::PeerConnectedIP {
                            peer_handle,
                            is_ipv6: type_ == Self::PEER_CONNECTED_IPV6,
                            protocol,
                            local_address,
                            local_port,
                            remote_address,
                            remote_port,
                        })
                    }
                    Self::PEER_CONNECTED_BLUETOOTH => Err(()),
                    _ => Err(()),
                }
            }
            Self::PEER_DISCONNECTED => {
                let peer_handle = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;
                Ok(ATEvent::PeerDisconnected { peer_handle })
            }
            Self::WIFI_LINK_CONNECTED => {
                let conn_id = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;
                let bssid = params.next().ok_or(())?;
                let channel = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;

                Ok(ATEvent::WifiLinkConnected {
                    conn_id,
                    bssid,
                    channel,
                })
            }
            Self::WIFI_LINK_DISCONNECTED => {
                let conn_id = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;
                let reason = params
                    .next()
                    .ok_or(())?
                    .parse::<u8>()
                    .or(Err(()))?
                    .try_into()?;

                Ok(ATEvent::WifiLinkDisconnected { conn_id, reason })
            }
            Self::WIFI_ACCESS_POINT_UP => {
                let id = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;

                Ok(ATEvent::WifiAccessPointUp { id })
            }
            Self::WIFI_ACCESS_POINT_DOWN => {
                let id = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;

                Ok(ATEvent::WifiAccessPointDown { id })
            }
            Self::WIFI_AP_STATION_CONNECTED => {
                let id = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;
                let mac_addr = params.next().ok_or(())?;

                Ok(ATEvent::WifiAccessPointStationConnected { id, mac_addr })
            }
            Self::WIFI_AP_STATION_DISCONNECTED => {
                let id = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;

                Ok(ATEvent::WifiAccessPointStationDisconnected { id })
            }
            Self::ETHERNET_LINK_UP => Ok(ATEvent::EthernetLinkUp),
            Self::ETHERNET_LINK_DOWN => Ok(ATEvent::EthernetLinkDown),
            Self::NETWORK_UP => {
                let interface_id = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;
                Ok(ATEvent::NetworkUp { interface_id })
            }
            Self::NETWORK_DOWN => {
                let interface_id = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;
                Ok(ATEvent::NetworkDown { interface_id })
            }
            Self::NETWORK_ERROR => {
                let interface_id = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;
                let code = params.next().ok_or(())?.parse::<u16>().or(Err(()))?;

                Ok(ATEvent::NetworkError { interface_id, code })
            }
            _ => Err(()),
        }
    }
}

#[derive(defmt::Format)]
pub enum ATResponse<'a> {
    Ok(&'a str),
    Error,
    Other(&'a str),
}

impl<'b> ATResponse<'b> {
    const CR_LF: &'static str = "\r\n";

    const RESP_OK: &'static str = "OK";
    const RESP_ERROR: &'static str = "ERROR";

    pub fn new<'a>(buf: &'a [u8]) -> Result<ATResponse<'a>, ()> {
        // TODO: error handling in this function is bad
        let s = core::str::from_utf8(buf).or(Err(()))?;
        
        let i_echo = s.find(Self::CR_LF).ok_or(())?;

        let s = &s[i_echo + Self::CR_LF.len()..];
        if !s.ends_with(Self::CR_LF) {
            return Err(());
        }
        let s = &s[..s.len() - Self::CR_LF.len()];

        if s == Self::RESP_OK {
            Ok(ATResponse::Ok(""))
        } else if s == Self::RESP_ERROR {
            Ok(ATResponse::Error)
        } else if let Some(i) = s.rfind(Self::CR_LF) {
            if &s[i + 2..] == Self::RESP_OK {
                Ok(ATResponse::Ok(&s[..i]))
            } else {
                Ok(ATResponse::Other(s))
            }
        } else {
            Ok(ATResponse::Other(s))
        }
    }
}
