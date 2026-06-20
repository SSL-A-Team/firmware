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
    // Framing / system events
    Empty,   // standalone \r\n — UART idle artifact, not a real URC
    Startup, // +STARTUP — module boot, no leading \r\n prefix

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
    /// Direct binary mode: incoming TCP socket data with inline binary payload.
    /// Requires AT+USORM=2 to enable.
    SocketDataBinary {
        socket_id: u8,
        data: &'a [u8],
    }, // +UESODB
    /// Direct binary mode: incoming UDP socket data with inline binary payload
    /// and source address info. Requires AT+USORM=2 to enable.
    SocketDataBinaryFrom {
        socket_id: u8,
        remote_ip: &'a str,
        remote_port: u16,
        data: &'a [u8],
    }, // +UESODBF

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
    WifiAccessPointUp,   // +UEWAPU (no parameters)
    WifiAccessPointDown, // +UEWAPD (no parameters)
    WifiAccessPointStationAssociated {
        mac_addr: &'a str,
    }, // +UEWAPSA
    WifiAccessPointStationDisassociated {
        mac_addr: &'a str,
    }, // +UEWAPSDA

    // Network events (Section 9.2)
    StationNetworkUp,       // +UEWSNU (no parameters)
    StationNetworkDown,     // +UEWSND (no parameters)
    AccessPointNetworkUp,   // +UEWAPNU (no parameters)
    AccessPointNetworkDown, // +UEWAPND (no parameters)
}

impl ATEvent<'_> {
    const CR_LF: &'static str = "\r\n";

    // Socket URCs
    const SOCKET_CONNECTED: &'static str = "+UESOC";
    const SOCKET_CLOSED: &'static str = "+UESOCL";
    const SOCKET_DATA_AVAILABLE: &'static str = "+UESODA";

    // Binary socket event prefixes (byte-level, for binary-aware parsing)
    const UESODB_PREFIX: &'static [u8] = b"\r\n+UESODB:";
    const UESODBF_PREFIX: &'static [u8] = b"\r\n+UESODBF:";
    /// Binary data start marker byte (0x01)
    const BINARY_START_MARKER: u8 = 0x01;

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

    // System URCs
    const STARTUP: &'static str = "+STARTUP";

    pub fn new(buf: &[u8]) -> Result<ATEvent<'_>, AtPacketError> {
        let s = core::str::from_utf8(buf).or(Err(AtPacketError::Utf8DecodeFailed))?;

        // Strip optional leading \r\n. Most URCs are framed as \r\n<URC>\r\n, but
        // +STARTUP is sent bare on boot with no prefix.
        let s = s.strip_prefix(Self::CR_LF).unwrap_or(s);

        // Standalone \r\n (now empty after strip) or empty buffer: UART idle artifact.
        if s.is_empty() || s == Self::CR_LF {
            return Ok(ATEvent::Empty);
        }

        if !s.ends_with(Self::CR_LF) {
            return Err(AtPacketError::FramingDecodeFailed);
        }
        let s = &s[..s.len() - Self::CR_LF.len()];

        // Handle compound URC buffers (multiple events in one UART read).
        // Only parse the first event; remaining events in the buffer are discarded.
        let s = if let Some(pos) = s.find(Self::CR_LF) {
            defmt::warn!("compound URC buffer detected, parsing first event only");
            &s[..pos]
        } else {
            s
        };

        let d = s.find(':').unwrap_or(s.len());
        let event = &s[..d];

        match event {
            Self::STARTUP => Ok(ATEvent::Startup),
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

                Ok(ATEvent::WifiLinkDown {
                    wlan_handle,
                    reason,
                })
            }
            Self::WIFI_ACCESS_POINT_UP => Ok(ATEvent::WifiAccessPointUp),
            Self::WIFI_ACCESS_POINT_DOWN => Ok(ATEvent::WifiAccessPointDown),
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
            Self::STATION_NETWORK_UP => Ok(ATEvent::StationNetworkUp),
            Self::STATION_NETWORK_DOWN => Ok(ATEvent::StationNetworkDown),
            Self::ACCESS_POINT_NETWORK_UP => Ok(ATEvent::AccessPointNetworkUp),
            Self::ACCESS_POINT_NETWORK_DOWN => Ok(ATEvent::AccessPointNetworkDown),
            _ => Err(AtPacketError::EventUnknown),
        }
    }

    /// Try to parse a binary data event from raw bytes.
    /// Binary events (+UESODB, +UESODBF) contain raw binary data that cannot
    /// be parsed as UTF-8. This must be called BEFORE text-based parsing.
    ///
    /// Binary data format: `<01><length_high><length_low><data>`
    /// where length is a 2-byte big-endian value.
    pub fn try_parse_binary(buf: &[u8]) -> Result<Option<ATEvent<'_>>, AtPacketError> {
        if buf.starts_with(Self::UESODB_PREFIX) {
            // +UESODB:<socket_id><01><len_hi><len_lo><data>
            let after_prefix = &buf[Self::UESODB_PREFIX.len()..];

            // Find the binary start marker (0x01)
            let marker_pos = after_prefix
                .iter()
                .position(|&b| b == Self::BINARY_START_MARKER)
                .ok_or(AtPacketError::FramingDecodeFailed)?;

            // Parse socket_id from ASCII digits before the marker
            let socket_id_bytes = &after_prefix[..marker_pos];
            let socket_id_str =
                core::str::from_utf8(socket_id_bytes).or(Err(AtPacketError::Utf8DecodeFailed))?;
            let socket_id = socket_id_str
                .parse::<u8>()
                .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;

            // Extract length and data after marker
            let after_marker = &after_prefix[marker_pos + 1..];
            if after_marker.len() < 2 {
                return Err(AtPacketError::FramingDecodeFailed);
            }
            let data_len = ((after_marker[0] as u16) << 8) | (after_marker[1] as u16);
            let data_start = marker_pos + 1 + 2;
            let data_end = data_start + data_len as usize;

            if data_end > after_prefix.len() {
                return Err(AtPacketError::FramingDecodeFailed);
            }

            let data = &after_prefix[data_start..data_end];
            Ok(Some(ATEvent::SocketDataBinary { socket_id, data }))
        } else if buf.starts_with(Self::UESODBF_PREFIX) {
            // +UESODBF:<socket_id>,<remote_ip>,<remote_port><01><len_hi><len_lo><data>
            let after_prefix = &buf[Self::UESODBF_PREFIX.len()..];

            // Find the binary start marker (0x01)
            let marker_pos = after_prefix
                .iter()
                .position(|&b| b == Self::BINARY_START_MARKER)
                .ok_or(AtPacketError::FramingDecodeFailed)?;

            // Parse text metadata before the marker as UTF-8
            let text_part = core::str::from_utf8(&after_prefix[..marker_pos])
                .or(Err(AtPacketError::Utf8DecodeFailed))?;

            let mut parts = text_part.splitn(3, ',');
            let socket_id = parts
                .next()
                .ok_or(AtPacketError::TypeDecodeParameterMissing)?
                .parse::<u8>()
                .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;
            let remote_ip = parts
                .next()
                .ok_or(AtPacketError::TypeDecodeParameterMissing)?;
            let remote_port = parts
                .next()
                .ok_or(AtPacketError::TypeDecodeParameterMissing)?
                .parse::<u16>()
                .or(Err(AtPacketError::TypeDecodeParameterDataTypeInvalid))?;

            // Extract length and data after marker
            let after_marker = &after_prefix[marker_pos + 1..];
            if after_marker.len() < 2 {
                return Err(AtPacketError::FramingDecodeFailed);
            }
            let data_len = ((after_marker[0] as u16) << 8) | (after_marker[1] as u16);
            let data_start = marker_pos + 1 + 2;
            let data_end = data_start + data_len as usize;

            if data_end > after_prefix.len() {
                return Err(AtPacketError::FramingDecodeFailed);
            }

            let data = &after_prefix[data_start..data_end];
            Ok(Some(ATEvent::SocketDataBinaryFrom {
                socket_id,
                remote_ip,
                remote_port,
                data,
            }))
        } else {
            Ok(None)
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
        let s = core::str::from_utf8(buf).or(Err(AtPacketError::Utf8DecodeFailed))?;

        // Fast path: bare OK/ERROR with optional leading \r\n.
        // Handles "OK\r\n", "\r\nOK\r\n" etc. that arrive when echo is off or the
        // UART idle timer splits the echo from the response into separate buffers.
        let bare = s.strip_prefix(Self::CR_LF).unwrap_or(s);
        let bare = bare.strip_suffix(Self::CR_LF).unwrap_or(bare);
        if bare == Self::RESP_OK {
            return Ok(ATResponse::Ok(""));
        }
        if bare == Self::RESP_ERROR {
            return Ok(ATResponse::Error);
        }

        // Full response with echo or standalone URC body: <echo>\r\n[<body>\r\n]<OK|ERROR>\r\n
        let i_echo = s
            .find(Self::CR_LF)
            .ok_or(AtPacketError::FramingDecodeFailed)?;

        let after = &s[i_echo + Self::CR_LF.len()..];

        // Standalone URC with no following content (e.g. "+USOCR:0\r\n" split from its OK).
        // The body is everything before the \r\n terminator.
        if after.is_empty() {
            let body = &s[..i_echo];
            return if body == Self::RESP_OK {
                Ok(ATResponse::Ok(""))
            } else if body == Self::RESP_ERROR {
                Ok(ATResponse::Error)
            } else {
                Ok(ATResponse::Other(body))
            };
        }

        let s = after;
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
