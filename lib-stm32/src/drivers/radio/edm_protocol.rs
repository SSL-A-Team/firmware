use core::mem::{self};
use defmt::Format;

use super::at_protocol::*;

// Data to/from radio is in big endian mode, ensure proper conversions are done

#[derive(Copy, Clone, PartialEq, Debug, Format)]
pub enum EdmPacketError {
    PacketTypeDecodingFailed,
    PacketTypeDecodeLengthMismatch,
    PacketDecodeFramingInvalid,
    PacketDecodeTargetMemorySizeInvalid,
    PacketEncodePayloadSizeInvalid,
    PacketEncodeTypeUnknown,
    ConnectEventDecodeError,
    ConnectEventUnsupported,
    DisconnectEventDecodeError,
    AtRequestEventDecodeError,
    AtPacketError(AtPacketError),
    PayloadTypeUnknown,
}

impl From<AtPacketError> for EdmPacketError {
    fn from(err: AtPacketError) -> Self {
        EdmPacketError::AtPacketError(err)
    }
}

#[repr(C)]
#[derive(Debug)]
struct EdmPacketRaw<P: ?Sized = [u8]> {
    payload_length: u16, // actually only 12bits, 4 bits reserved
    identifier: u16,
    payload: P,
}

impl EdmPacketRaw {
    const START: u8 = 0xAA;
    const END: u8 = 0x55;

    const CONNECT_EVENT: u16 = 0x0011;
    const DISCONNECT_EVENT: u16 = 0x0021;
    const DATA_EVENT: u16 = 0x0031;
    const DATA_COMMAND: u16 = 0x0036;
    const AT_REQUEST: u16 = 0x0044;
    const AT_RESPONSE: u16 = 0x0045;
    const AT_EVENT: u16 = 0x0041;
    const RESEND_CONNECT_EVENTS: u16 = 0x0056;
    const START_EVENT: u16 = 0x0071;

    fn new(data: &[u8]) -> Result<&EdmPacketRaw, EdmPacketError> {
        if data.len() >= 2 && data[0] == Self::START && data[data.len() - 1] == Self::END {
            let packet = data_to_ref::<EdmPacketRaw, EdmPacketRaw<()>>(&data[1..data.len() - 1])?;
            if (u16::from_be(packet.payload_length) & 0x0FFF) as usize == packet.payload.len() + 2 {
                Ok(packet)
            } else {
                Err(EdmPacketError::PacketTypeDecodeLengthMismatch)
            }
        } else {
            Err(EdmPacketError::PacketDecodeFramingInvalid)
        }
    }

    fn write(buf: &mut [u8], packet: &EdmPacket) -> Result<usize, EdmPacketError> {
        let buf_len = buf.len();
        let buf_packet = data_to_mut::<EdmPacketRaw, EdmPacketRaw<()>>(&mut buf[1..buf_len - 1])?;
        let len = match packet {
            EdmPacket::DataCommand { channel, data } => {
                let payload_size = mem::size_of::<DataCommand<()>>() + data.len();
                if buf_packet.payload.len() < payload_size {
                    return Err(EdmPacketError::PacketEncodePayloadSizeInvalid);
                }
                buf_packet.identifier = u16::to_be(Self::DATA_COMMAND);
                let command = data_to_mut::<DataCommand, DataCommand<()>>(&mut buf_packet.payload)?;
                command.channel_id = *channel;
                command.payload[..data.len()].copy_from_slice(data);
                payload_size
            }
            EdmPacket::ATRequest(req) => {
                let payload_size = req.len() + 1;
                if buf_packet.payload.len() < payload_size {
                    return Err(EdmPacketError::PacketEncodePayloadSizeInvalid);
                }
                buf_packet.identifier = u16::to_be(Self::AT_REQUEST);
                buf_packet.payload[..req.len()].copy_from_slice(req.as_bytes());
                buf_packet.payload[req.len()] = b'\r';

                payload_size
            }
            EdmPacket::ResendConnectEvents => {
                buf_packet.identifier = u16::to_be(Self::RESEND_CONNECT_EVENTS);
                0
            }
            _ => return Err(EdmPacketError::PacketEncodeTypeUnknown),
        };
        // Payload length is (payload size + identifier field size)
        buf_packet.payload_length = u16::to_be(2 + len as u16);
        let total_len = 1 + mem::size_of::<EdmPacketRaw<()>>() + len + 1;
        buf[0] = Self::START;
        buf[total_len - 1] = Self::END;

        Ok(total_len)
    }

    fn get_payload(&'_ self) -> Result<EdmPacket<'_>, EdmPacketError> {
        match u16::from_be(self.identifier) {
            Self::CONNECT_EVENT => {
                let event = data_to_ref::<ConnectEvent, ConnectEvent<()>>(&self.payload)?;
                let event_payload = event.get_payload()?;
                Ok(EdmPacket::ConnectEvent {
                    channel: event.channel_id,
                    event_type: event_payload,
                })
            }
            Self::DISCONNECT_EVENT => {
                if self.payload.len() == mem::size_of::<DisconnectEvent>() {
                    let event = unsafe { &*(self.payload.as_ptr() as *const DisconnectEvent) };
                    Ok(EdmPacket::DisconnectEvent {
                        channel: event.channel_id,
                    })
                } else {
                    Err(EdmPacketError::DisconnectEventDecodeError)
                }
            }
            Self::DATA_EVENT => {
                let event = data_to_ref::<DataEvent, DataEvent<()>>(&self.payload)?;
                Ok(EdmPacket::DataEvent {
                    channel: event.channel_id,
                    data: &event.payload,
                })
            }
            Self::DATA_COMMAND => {
                let event = data_to_ref::<DataEvent, DataEvent<()>>(&self.payload)?;
                Ok(EdmPacket::DataEvent {
                    channel: event.channel_id,
                    data: &event.payload,
                })
            }
            Self::AT_REQUEST => Ok(EdmPacket::ATRequest(
                core::str::from_utf8(&self.payload)
                    .or(Err(EdmPacketError::AtRequestEventDecodeError))?,
            )),
            Self::AT_RESPONSE => Ok(EdmPacket::ATResponse(ATResponse::new(&self.payload)?)),
            Self::AT_EVENT => Ok(EdmPacket::ATEvent(ATEvent::new(&self.payload)?)),
            Self::RESEND_CONNECT_EVENTS => Ok(EdmPacket::ResendConnectEvents),
            Self::START_EVENT => Ok(EdmPacket::StartEvent),

            _ => Err(EdmPacketError::PayloadTypeUnknown),
        }
    }
}

#[derive(defmt::Format)]
pub enum EdmPacket<'a> {
    ConnectEvent {
        channel: u8,
        event_type: ConnectEventType,
    },
    DisconnectEvent {
        channel: u8,
    },
    DataEvent {
        channel: u8,
        data: &'a [u8],
    },
    DataCommand {
        channel: u8,
        data: &'a [u8],
    },
    ATRequest(&'a str),
    ATResponse(ATResponse<'a>),
    ATEvent(ATEvent<'a>),
    ResendConnectEvents,
    StartEvent,
}

impl EdmPacket<'_> {
    pub fn new(data: &'_ [u8]) -> Result<EdmPacket<'_>, EdmPacketError> {
        EdmPacketRaw::new(data)?.get_payload()
    }

    pub fn write(&self, buf: &mut [u8]) -> Result<usize, EdmPacketError> {
        EdmPacketRaw::write(buf, self)
    }

    // pub fn create(data: &mut [u8]) {
    //     let d = [0, 0, 0];
    //     let a = EdmPacket::DataCommand {
    //         channel: 0,
    //         data: &d,
    //     };
    // }
}

///////////////////
// Connect Event //
///////////////////

#[repr(C)]
struct ConnectEvent<P: ?Sized = [u8]> {
    pub channel_id: u8,
    connect_type: u8,
    payload: P,
}

#[derive(defmt::Format)]
pub enum ConnectEventType {
    // ConnectEventBluetooth(ConnectEventBluetooth),
    ConnectEventIPv4(ConnectEventIPv4),
    // ConnectEventIPv6(ConnectEventIPv6),
}

#[derive(defmt::Format)]
#[repr(C)]
pub struct ConnectEventBluetooth {
    pub profile: u8,
    pub bd_address: [u8; 6],
    pub frame_size: u16,
}
#[derive(defmt::Format)]
#[repr(C)]
pub struct ConnectEventIPv4 {
    pub protocol: u8,
    pub remote_addr: [u8; 4],
    pub remote_port: u16,
    pub local_addr: [u8; 4],
    pub local_port: u16,
}
#[derive(defmt::Format)]
#[repr(C)]
pub struct ConnectEventIPv6 {
    pub protocol: u8,
    pub remote_addr: [u8; 16],
    pub remote_port: u16,
    pub local_addr: [u8; 16],
    pub local_port: u16,
}

impl ConnectEvent {
    fn get_payload(&self) -> Result<ConnectEventType, EdmPacketError> {
        match self.connect_type {
            0x02 => {
                // IPv4 Connect Event
                if self.payload.len() == 13 {
                    Ok(ConnectEventType::ConnectEventIPv4(ConnectEventIPv4 {
                        protocol: self.payload[0],
                        remote_addr: [
                            self.payload[1],
                            self.payload[2],
                            self.payload[3],
                            self.payload[4],
                        ],
                        remote_port: u16::from_be_bytes([self.payload[5], self.payload[6]]),
                        local_addr: [
                            self.payload[7],
                            self.payload[8],
                            self.payload[9],
                            self.payload[10],
                        ],
                        local_port: u16::from_be_bytes([self.payload[11], self.payload[12]]),
                    }))
                } else {
                    Err(EdmPacketError::ConnectEventDecodeError)
                }
            }
            _ => Err(EdmPacketError::ConnectEventUnsupported),
        }
    }
}

//////////////////////
// Disconnect Event //
//////////////////////

#[repr(C)]
struct DisconnectEvent {
    channel_id: u8,
}

//////////////////
// Data Packets //
//////////////////

#[repr(C)]
struct DataEvent<P: ?Sized = [u8]> {
    channel_id: u8,
    payload: P,
}

#[repr(C)]
struct DataCommand<P: ?Sized = [u8]> {
    channel_id: u8,
    payload: P,
}

// misc

fn data_to_ref<T, BASE>(data: &[u8]) -> Result<&T, EdmPacketError>
where
    T: core::ptr::Pointee<Metadata = usize> + ?Sized,
{
    if data.len() >= mem::size_of::<BASE>() {
        Ok(unsafe {
            &*core::ptr::from_raw_parts(
                data.as_ptr() as *const (),
                data.len() - mem::size_of::<BASE>(),
            )
        })
    } else {
        Err(EdmPacketError::PacketDecodeTargetMemorySizeInvalid)
    }
}

fn data_to_mut<T, BASE>(data: &mut [u8]) -> Result<&mut T, EdmPacketError>
where
    T: core::ptr::Pointee<Metadata = usize> + ?Sized,
{
    if data.len() >= mem::size_of::<BASE>() {
        Ok(unsafe {
            &mut *core::ptr::from_raw_parts_mut(
                data.as_ptr() as *mut (),
                data.len() - mem::size_of::<BASE>(),
            )
        })
    } else {
        Err(EdmPacketError::PacketDecodeTargetMemorySizeInvalid)
    }
}
