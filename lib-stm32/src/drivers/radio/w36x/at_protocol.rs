use defmt::Format;

#[derive(Copy, Clone, PartialEq, Debug, Format)]
pub enum AtPacketError {
    Utf8DecodeFailed,
    FramingDecodeFailed,
    TypeDecodeUnsupported,
    TypeDecodeParameterMissing,
    TypeDecodeParameterDataTypeInvalid,
    EventUnknown,
}

/// Socket protocol type for AT+USOCR
#[derive(Copy, Clone, PartialEq, Debug, defmt::Format)]
pub enum SocketProtocol {
    TCP = 6,
    UDP = 17,
}

/// AT unsolicited result codes (URCs) for the NORA-W36.
/// See https://github.com/u-blox/u-connectXpress/blob/main/NORA-W36/3.2.0/at_commands.md
#[allow(dead_code)]
#[derive(defmt::Format)]
pub enum ATEvent<'a> {
    // Socket events (Section 10.2)
    SocketConnected {
        socket_id: u8,
    }, // +UESOC
    SocketClosed {
        socket_id: u8,
    }, // +UESOCL
    SocketDataAvailable {
        socket_id: u8,
        length: u16,
    }, // +UESODA

    // WiFi events (Section 9.2)
    WifiLinkUp {
        wlan_handle: u16,
        bssid: &'a str,
        channel: u16,
    }, // +UEWLU
    WifiLinkDown {
        wlan_handle: u16,
        reason: u16,
    }, // +UEWLD (reason is standard 802.11 reason code)
    WifiAccessPointUp,          // +UEWAPU (no parameters)
    WifiAccessPointDown,        // +UEWAPD (no parameters)
    WifiAccessPointStationAssociated {
        mac_addr: &'a str,
    }, // +UEWAPSA
    WifiAccessPointStationDisassociated {
        mac_addr: &'a str,
    }, // +UEWAPSDA

    // Network events (Section 9.2)
    StationNetworkUp,           // +UEWSNU (no parameters)
    StationNetworkDown,         // +UEWSND (no parameters)
    AccessPointNetworkUp,       // +UEWAPNU (no parameters)
    AccessPointNetworkDown,     // +UEWAPND (no parameters)
}

impl ATEvent<'_> {
    const CR_LF: &'static str = "\r\n";

    // Socket URCs
    const SOCKET_CONNECTED: &'static str = "+UESOC";
    const SOCKET_CLOSED: &'static str = "+UESOCL";
    const SOCKET_DATA_AVAILABLE: &'static str = "+UESODA";

    // WiFi URCs (Section 9.2)
    const WIFI_LINK_UP: &'static str = "+UEWLU";
    const WIFI_LINK_DOWN: &'static str = "+UEWLD";
    const WIFI_ACCESS_POINT_UP: &'static str = "+UEWAPU";
    const WIFI_ACCESS_POINT_DOWN: &'static str = "+UEWAPD";
    const WIFI_AP_STATION_ASSOCIATED: &'static str = "+UEWAPSA";
    const WIFI_AP_STATION_DISASSOCIATED: &'static str = "+UEWAPSDA";

    // Network URCs (Section 9.2)
    const STATION_NETWORK_UP: &'static str = "+UEWSNU";
    const STATION_NETWORK_DOWN: &'static str = "+UEWSND";
    const ACCESS_POINT_NETWORK_UP: &'static str = "+UEWAPNU";
    const ACCESS_POINT_NETWORK_DOWN: &'static str = "+UEWAPND";

    pub fn new(buf: &[u8]) -> Result<ATEvent<'_>, AtPacketError> {
        let s = core::str::from_utf8(buf).or(Err(AtPacketError::Utf8DecodeFailed))?;
        let s = &s[Self::CR_LF.len()..];
        if !s.ends_with(Self::CR_LF) {
            return Err(AtPacketError::FramingDecodeFailed);
        }
        let s = &s[..s.len() - Self::CR_LF.len()];

        let d = s.find(':').unwrap_or(s.len());
        let event = &s[..d];

        match event {
            Self::SOCKET_CONNECTED => {
                let mut params = s[d + 1..].split(',');
                let socket_id = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?
                    .parse::<u8>()
                    .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;
                Ok(ATEvent::SocketConnected { socket_id })
            }
            Self::SOCKET_CLOSED => {
                let mut params = s[d + 1..].split(',');
                let socket_id = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?
                    .parse::<u8>()
                    .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;
                Ok(ATEvent::SocketClosed { socket_id })
            }
            Self::SOCKET_DATA_AVAILABLE => {
                let mut params = s[d + 1..].split(',');
                let socket_id = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?
                    .parse::<u8>()
                    .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;
                let length = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?
                    .parse::<u16>()
                    .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;
                Ok(ATEvent::SocketDataAvailable { socket_id, length })
            }
            Self::WIFI_LINK_UP => {
                let mut params = s[d + 1..].split(',');
                let wlan_handle = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?
                    .parse::<u16>()
                    .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;
                let bssid = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?;
                let channel = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?
                    .parse::<u16>()
                    .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;

                Ok(ATEvent::WifiLinkUp {
                    wlan_handle,
                    bssid,
                    channel,
                })
            }
            Self::WIFI_LINK_DOWN => {
                let mut params = s[d + 1..].split(',');
                let wlan_handle = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?
                    .parse::<u16>()
                    .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;
                let reason = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?
                    .parse::<u16>()
                    .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;

                Ok(ATEvent::WifiLinkDown { wlan_handle, reason })
            }
            Self::WIFI_ACCESS_POINT_UP => {
                Ok(ATEvent::WifiAccessPointUp)
            }
            Self::WIFI_ACCESS_POINT_DOWN => {
                Ok(ATEvent::WifiAccessPointDown)
            }
            Self::WIFI_AP_STATION_ASSOCIATED => {
                let mut params = s[d + 1..].split(',');
                let mac_addr = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?;

                Ok(ATEvent::WifiAccessPointStationAssociated { mac_addr })
            }
            Self::WIFI_AP_STATION_DISASSOCIATED => {
                let mut params = s[d + 1..].split(',');
                let mac_addr = params
                    .next()
                    .ok_or(AtPacketError::TypeDecodeParameterMissing)?;

                Ok(ATEvent::WifiAccessPointStationDisassociated { mac_addr })
            }
            Self::STATION_NETWORK_UP => {
                Ok(ATEvent::StationNetworkUp)
            }
            Self::STATION_NETWORK_DOWN => {
                Ok(ATEvent::StationNetworkDown)
            }
            Self::ACCESS_POINT_NETWORK_UP => {
                Ok(ATEvent::AccessPointNetworkUp)
            }
            Self::ACCESS_POINT_NETWORK_DOWN => {
                Ok(ATEvent::AccessPointNetworkDown)
            }
            _ => Err(AtPacketError::EventUnknown),
        }
    }
}

#[derive(defmt::Format)]
pub enum ATResponse<'a> {
    Ok(&'a str),
    Error,
    Other(&'a str),
}

impl ATResponse<'_> {
    const CR_LF: &'static str = "\r\n";

    const RESP_OK: &'static str = "OK";
    const RESP_ERROR: &'static str = "ERROR";

    pub fn new(buf: &[u8]) -> Result<ATResponse<'_>, AtPacketError> {
        // TODO: error handling in this function is bad
        let s = core::str::from_utf8(buf).or(Err(AtPacketError::Utf8DecodeFailed))?;

        let i_echo = s
            .find(Self::CR_LF)
            .ok_or(AtPacketError::FramingDecodeFailed)?;

        let s = &s[i_echo + Self::CR_LF.len()..];
        if !s.ends_with(Self::CR_LF) {
            return Err(AtPacketError::FramingDecodeFailed);
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
