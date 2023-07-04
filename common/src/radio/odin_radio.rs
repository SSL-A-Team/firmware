use core::{
    cell::UnsafeCell,
    future::poll_fn,
    mem::MaybeUninit,
    task::{Poll, Waker},
};

use core::fmt::Write;
use embassy_futures::join::join;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;

use crate::{
    radio::radio::{Ipv4Addr, Radio, UdpSocket},
    task::Task,
    transfer::{Reader, Reader2, Writer, Writer2},
};

use super::{
    at_protocol::{ATEvent, ATResponse},
    edm_protocol::EdmPacket,
};

use crate::transfer::DataRefReadTrait;
use crate::transfer::DataRefWriteTrait;

// struct Test1 {}
// struct Test2<'a> {
//     test1: &'a Test1
// }

// trait Test3 {
//     type Item<'a> where Self: 'a;

//     async fn test<'a>(&'a mut self) -> Self::Item<'a>;
// }

// impl Test3 for Test1 {
//     type Item<'a> = Test2<'a>;

//     async fn test(&mut self) -> Test2 {
//         Test2 {
//             test1: self
//         }
//     }
// }

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(core::fmt::Debug)]
pub enum RadioMode {
    CommandMode,
    DataMode,
    ExtendedDataMode,
}

pub enum WifiAuth<'a> {
    Open,
    WPA { passphrase: &'a str },
    LEAP,
    PEAP,
    EAPTLS,
}

pub enum ServerType {
    Disabled = 0,
    TCP = 1,
    UDP = 2,
    SPP = 3,
    DUN = 4,
    UUID = 5,
    SPS = 6,
    ATP = 8,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(core::fmt::Debug)]
pub struct PeerConnection {
    pub peer_id: u8,
    pub channel_id: u8,
}

pub trait RadioInterfaceControl: Reader2 + Writer2 {
    async fn reset_radio(&self);
    async fn config_uart(&self, baudrate: u32, flow_control: bool, data_bits: u8, parity: bool);
}

#[derive(Clone, Copy)]
struct Status {
    mode: RadioMode,
    connected: bool,
}

// TODO: can CriticalSectionRawMutex be changed to CriticalSectionRawMutex
struct OpenPeerConnection {
    channel_id: u8,
    is_read: bool,
    is_write: bool,
    waker: Option<Waker>,
}

pub struct OdinRadio<'a, RadioInterface: RadioInterfaceControl> {
    interface: &'a RadioInterface,
    initialized: Signal<CriticalSectionRawMutex, ()>,
    status: Mutex<CriticalSectionRawMutex, Status>,
    // mode: Mutex<CriticalSectionRawMutex, RadioMode>,
    // initialized: bool,
    // connected: bool,
    // commandInProgress: UnsafeCell<bool>,
    command_mutex: Mutex<CriticalSectionRawMutex, ()>,
    // write in progress
    // read in progress
    packet_buffer: UnsafeCell<[MaybeUninit<u8>; 100]>, // TODO: length
    packet_buffer_len: UnsafeCell<usize>,
    packet: Mutex<CriticalSectionRawMutex, Option<EdmPacket<'a>>>,
    response_waker: UnsafeCell<Option<Waker>>,
    packet_complete: Signal<CriticalSectionRawMutex, ()>,

    // socket_read_wakers: UnsafeCell<[Option<Socket>; 1]>,
    // channels: Mutex<CriticalSectionRawMutex, [Option<Channel>; 1]>,
    peers: Mutex<CriticalSectionRawMutex, heapless::LinearMap<u8, OpenPeerConnection, 2>>,
    // packet_waker: UnsafeCell<[Option<Waker>; WAKER_COUNT]>,
    // _phantom: PhantomData<&'a str>,
}

unsafe impl<'a, RadioInterface: RadioInterfaceControl> Sync for OdinRadio<'a, RadioInterface> {}
unsafe impl<'a, RadioInterface: RadioInterfaceControl> Send for OdinRadio<'a, RadioInterface> {}

pub struct OdinSocket<'a, RadioInterface: RadioInterfaceControl> {
    radio: &'a OdinRadio<'a, RadioInterface>,
    peer_id: u8,
}

impl<RadioInterface: RadioInterfaceControl> Radio for OdinRadio<'static, RadioInterface> {
    type UdpSocket = OdinSocket<'static, RadioInterface>;

    async fn connect_to_network(
        &self,
        ssid: &str,
        pass: Option<&str>,
        hostname: Option<&str>,
    ) -> Result<(), ()> {
        info!("Waiting for radio initialization");
        self.initialized.wait().await;

        // TODO: handle if connected already

        if let Some(hostname) = hostname {
            self.set_host_name(hostname).await?;
        }

        if let Some(pass) = pass {
            self.config_wifi(1, ssid, WifiAuth::WPA { passphrase: pass })
                .await?;
        } else {
            self.config_wifi(1, ssid, WifiAuth::Open).await?;
        }

        self.connect_wifi(1).await?;
        info!("Netork {} Connected as {}", ssid, hostname.unwrap_or("?"));
        Ok(())
    }
    async fn open_udp(
        &'static self,
        ipv4: Ipv4Addr,
        port: u16,
        local_port: Option<u16>,
    ) -> Result<Self::UdpSocket, ()> {
        // let socket_index = critical_section::with(|_| unsafe {
        //     for (i, socket) in (*self.socket_read_wakers.get()).iter_mut().enumerate() {
        //         if let None = socket {
        //             *socket = Some(Socket { read_waker: None });
        //             return Ok(i);
        //         }
        //     }
        //     Err(())
        // })?;

        // TODO: is this right?
        // let local_port = port;
        // // // or this?
        // let local_port = 42070;
        let local_port = local_port.unwrap_or(0);

        // TODO: flags conditional?
        let mut url = String::<50>::new();
        core::write!(
            &mut url,
            // "udp://{}.{}.{}.{}:{}/?local_port={local_port}",
            "udp://{}.{}.{}.{}:{}/?flags=1&local_port={local_port}",
            ipv4.octets[0],
            ipv4.octets[1],
            ipv4.octets[2],
            ipv4.octets[3],
            port
        )
        .unwrap();

        let peer_connection = self.connect_peer(&url).await?;
        info!("{:?}", peer_connection);

        Ok(OdinSocket {
            radio: self,
            peer_id: peer_connection.peer_id,
        })
    }
}

impl<'a, RadioInterface: RadioInterfaceControl> OdinRadio<'a, RadioInterface> {
    pub fn new(interface: &'a RadioInterface) -> Self {
        let packet_complete = Signal::new();
        packet_complete.signal(());
        Self {
            interface,
            initialized: Signal::new(),
            status: Mutex::new(Status {
                mode: RadioMode::CommandMode,
                connected: false,
            }),
            command_mutex: Mutex::new(()),
            packet_buffer: UnsafeCell::new(MaybeUninit::uninit_array()),
            packet_buffer_len: UnsafeCell::new(0),
            packet: Mutex::new(None),
            packet_complete,
            response_waker: UnsafeCell::new(None),
            peers: Mutex::new(heapless::LinearMap::new()),
            // channels: Mutex::new([None; 1]),
            // channel_wakers: [Mutex::new(None)],
            // socket_read_wakers: UnsafeCell::new([None; 1]),
        }
    }

    pub async fn initialize(&self) -> Result<(), ()> {
        // Reset radio and configure UART to factory default of radio
        // TODO: flow control off default?
        self.interface.reset_radio().await;
        self.interface.config_uart(115_200, false, 8, false).await;

        // Wait for startup event to proceed
        self.wait_startup().await?;

        self.set_echo(false).await?;
        self.config_uart(5_250_000, false, 8, true).await?;
        // Timer::after(Duration::from_millis(5)).await;
        // info!("uart update");
        self.interface.config_uart(5_250_000, false, 8, true).await;
        // Datasheet says wait at least 40ms after UART config change
        Timer::after(Duration::from_millis(50)).await;

        // Enter extended data mode
        self.enter_edm().await?;
        // Datasheet says wait at least 50ms after entering data mode
        Timer::after(Duration::from_millis(50)).await;

        // self.send_command("AT+CGMR", |resp| {
        //     info!("{:?}", resp);
        // }).await;

        // self.close_peer(0).await?;
        // self.list_peers().await;

        self.initialized.signal(());
        info!("Radio Initialized");
        Ok(())
    }

    pub async fn read_packet(&self) -> Result<(), ()> {
        self.packet_complete.wait().await;
        // info!("reading");

        // TODO: try adding wait
        // Timer::after(Duration::from_millis(10)).await;
        // Timer::after(Duration::from_millis(150)).await;

        // self.interface
        //     .read(|buf| {
        //         let _ = self.packet.try_lock().or(Err(()))?;
        //         let data =
        //             unsafe { MaybeUninit::slice_assume_init_mut(&mut *self.packet_buffer.get()) };
        //         data[..buf.len()].clone_from_slice(buf);
        //         unsafe { *self.packet_buffer_len.get() = buf.len() };
        //         Ok(())
        //     })
        //     .await?;

        let data_ref = self.interface.read().await?;
        let mut packet = self.packet.lock().await;
        let data = unsafe { MaybeUninit::slice_assume_init_mut(&mut *self.packet_buffer.get()) };
        let len = data_ref.data().len();
        data[..len].clone_from_slice(data_ref.data());
        unsafe { *self.packet_buffer_len.get() = len };
        let status = { *self.status.lock().await };
        // let peers = { *self.peers.lock().await };
        // info!("len {}", len);
        match status.mode {
            RadioMode::CommandMode => {
                let event = ATEvent::new(&data[..len]);
                if let Ok(event) = event {
                    *packet = Some(EdmPacket::ATEvent(event));
                } else {
                    let response = ATResponse::new(&data[..len]);
                    if let Ok(response) = response {
                        *packet = Some(EdmPacket::ATResponse(response));
                    }
                }
            }
            RadioMode::ExtendedDataMode => {
                *packet = Some(EdmPacket::new(&data[..len])?);
            }
            _ => return Err(()),
        }

        critical_section::with(|_| unsafe {
            if let Some(waker) = &*self.response_waker.get() {
                waker.wake_by_ref();
            };
        });
        let peers = self.peers.lock().await;
        for peer in &*peers {
            if let Some(EdmPacket::DataEvent { channel, data: _ }) = &*packet {
                if *channel == peer.1.channel_id && !peer.1.is_read {
                    *packet = None;
                    self.packet_complete.signal(());
                    info!("reset");
                    break;
                }
            }

            if let Some(waker) = &peer.1.waker {
                waker.wake_by_ref();
            }
        }
        drop(peers);
        
        if let Some(_) = &*packet {
            self.packet_complete.reset();
        }
        drop(packet);

        Ok(())
    }

    pub async fn wait_startup(&self) -> Result<(), ()> {
        let lock = self.command_mutex.try_lock().or(Err(()))?;
        poll_fn(|cx| {
            let lock = self.packet.try_lock();
            match lock {
                Ok(mut lock) => {
                    if let Some(packet) = &*lock {
                        if let EdmPacket::ATResponse(ATResponse::Other("+STARTUP")) = &packet {
                            *lock = None;
                            self.packet_complete.signal(());
                            return Poll::Ready(());
                        }
                    }
                }
                Err(_) => {}
            }
            critical_section::with(|_| unsafe {
                *self.response_waker.get() = Some(cx.waker().clone());
            });
            Poll::Pending
        })
        .await;
        drop(lock);

        Ok(())
    }

    async fn _send_command(&self, cmd: &str) -> Result<(), ()> {
        let status = { *self.status.lock().await };
        // Send at request
        let mut data_ref = self.interface.write().await?;
        let data = data_ref.data();
        match status.mode {
            RadioMode::CommandMode => {
                // self.interface
                //     .write(|buf| {
                //         buf[..cmd.len()].clone_from_slice(cmd.as_bytes());
                //         buf[cmd.len()] = b'\r';
                //         Ok(cmd.len() + 1)
                //     })
                //     .await?;
                data[..cmd.len()].clone_from_slice(cmd.as_bytes());
                data[cmd.len()] = b'\r';
                *data_ref.len() = cmd.len() + 1;

                Ok(())
            }
            RadioMode::ExtendedDataMode => {
                // self.interface
                //     .write(|buf| EdmPacket::ATRequest(cmd).write(buf))
                //     .await?;
                let len = EdmPacket::ATRequest(cmd).write(data);
                if let Ok(len) = len {
                    *data_ref.len() = len;
                } else {
                    data_ref.cancel();
                    return Err(());
                }
                Ok(())
            }
            _ => Err(()),
        }?;
        drop(data_ref);
        Ok(())
    }

    pub async fn send_command(
        &self,
        cmd: &str,
        fn_resp: impl FnOnce(&ATResponse),
    ) -> Result<(), ()> {
        // Error if at command in progress
        let command_lock = self.command_mutex.try_lock().or(Err(()))?;
        self._send_command(cmd).await?;
        // Timer::after(Duration::from_millis(2)).await;

        // Wait for at response
        let mut packet = poll_fn(|cx| {
            let lock = self.packet.try_lock();
            match lock {
                Ok(lock) => {
                    if let Some(packet) = &*lock {
                        if let EdmPacket::ATResponse(_) = &packet {
                            return Poll::Ready(lock);
                        }
                    }
                }
                Err(_) => {}
            }
            critical_section::with(|_| unsafe {
                *self.response_waker.get() = Some(cx.waker().clone());
            });
            Poll::Pending
        })
        .await;
        if let EdmPacket::ATResponse(response) = &*packet.as_ref().unwrap() {
            fn_resp(response);
            *packet = None;
            self.packet_complete.signal(());
        }
        drop(packet);
        drop(command_lock);

        Ok(())
    }

    pub async fn send_command_ok(&self, cmd: &str) -> Result<(), ()> {
        let mut command_error = Ok(());
        // info!("send");
        self.send_command(cmd, |response| {
            // info!("ok");
            if let ATResponse::Error = &response {
                command_error = Err(());
            }
        })
        .await?;
        command_error?;
        Ok(())
    }

    pub async fn set_echo(&self, echo_on: bool) -> Result<(), ()> {
        let echo_on = if echo_on { '1' } else { '0' };
        let mut str: String<4> = String::new();
        write!(&mut str, "ATE{echo_on}").unwrap();
        self.send_command_ok(str.as_str()).await?;

        Ok(())
    }

    pub async fn config_uart(
        &self,
        baudrate: u32,
        flow_control: bool,
        data_bits: u8,
        parity: bool,
    ) -> Result<(), ()> {
        let mut str: String<28> = String::new();
        let flow_control = if flow_control { '1' } else { '2' };
        let stop_bits = '1';
        let parity = if parity { '3' } else { '1' };
        write!(
            &mut str,
            "AT+UMRS={baudrate},{flow_control},{data_bits},{stop_bits},{parity},1",
        )
        .or(Err(()))?;
        self.send_command_ok(str.as_str()).await?;

        Ok(())
    }

    pub async fn enter_edm(&self) -> Result<(), ()> {
        self.send_command_ok("ATO2").await?;
        // TODO: wait for extended data mode event
        (*self.status.lock().await).mode = RadioMode::ExtendedDataMode;

        let command_lock = self.command_mutex.try_lock().or(Err(()))?;
        poll_fn(|cx| {
            let lock = self.packet.try_lock();
            match lock {
                Ok(mut lock) => {
                    if let Some(packet) = &*lock {
                        if let EdmPacket::StartEvent = &packet {
                            *lock = None;
                            self.packet_complete.signal(());
                            return Poll::Ready(());
                        }
                    }
                }
                Err(_) => {}
            }
            critical_section::with(|_| unsafe {
                *self.response_waker.get() = Some(cx.waker().clone());
            });
            Poll::Pending
        })
        .await;
        drop(command_lock);

        Ok(())
    }

    pub async fn set_host_name(&self, host_name: &str) -> Result<(), ()> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UNHN=\"{host_name}\"").or(Err(()))?;

        self.send_command_ok(str.as_str()).await?;
        Ok(())
    }

    pub async fn config_wifi<'b>(
        &self,
        config_id: u8,
        ssid: &str,
        auth: WifiAuth<'b>,
    ) -> Result<(), ()> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UWSC={config_id},2,\"{ssid}\"").or(Err(()))?;

        self.send_command_ok(str.as_str()).await?;
        str.clear();
        match auth {
            WifiAuth::Open => {
                write!(str, "AT+UWSC={config_id},5,1").or(Err(()))?;
                self.send_command_ok(str.as_str()).await?;
            }
            WifiAuth::WPA { passphrase } => {
                write!(str, "AT+UWSC={config_id},5,2").or(Err(()))?;
                self.send_command_ok(str.as_str()).await?;
                str.clear();
                write!(str, "AT+UWSC={config_id},8,\"{passphrase}\"").or(Err(()))?;
                self.send_command_ok(str.as_str()).await?;
            }
            _ => return Err(()),
        }

        Ok(())
    }

    pub async fn connect_wifi(&self, config_id: u8) -> Result<(), ()> {
        let mut str: String<64> = String::new();
        write!(str, "AT+UWSCA={config_id},3").or(Err(()))?;
        let command_lock = self.command_mutex.try_lock().or(Err(()))?;
        self._send_command(str.as_str()).await?;

        let mut command_response = false;
        let mut network_up = 0;
        let mut wifi_connected = false;
        poll_fn(|cx| {
            let lock = self.packet.try_lock();
            if let Ok(mut lock) = lock {
                if let Some(packet) = &*lock {
                    match packet {
                        EdmPacket::ATResponse(response) => {
                            if let ATResponse::Ok(_) = response {
                                command_response = true;
                                *lock = None;
                                self.packet_complete.signal(());
                            } else {
                                *lock = None;
                                self.packet_complete.signal(());
                                return Poll::Ready(Err(()));
                            }
                        }
                        EdmPacket::ATEvent(event) => match event {
                            ATEvent::NetworkUp { interface_id: 0 } => {
                                network_up += 1;
                                *lock = None;
                                self.packet_complete.signal(());
                            }
                            ATEvent::WifiLinkConnected {
                                conn_id: _,
                                bssid: _,
                                channel: _,
                            } => {
                                wifi_connected = true;
                                *lock = None;
                                self.packet_complete.signal(());
                            }
                            _ => {}
                        },
                        _ => {}
                    }
                }
            }

            if command_response && network_up >= 2 && wifi_connected {
                return Poll::Ready(Ok(()));
            }
            critical_section::with(|_| unsafe {
                *self.response_waker.get() = Some(cx.waker().clone());
            });
            Poll::Pending
        })
        .await?;

        drop(command_lock);

        // TODO: mark connected

        Ok(())
    }

    pub async fn connect_peer(&self, url: &str) -> Result<PeerConnection, ()> {
        let mut cmd: String<68> = String::new();
        write!(cmd, "AT+UDCP={url}").or(Err(()))?;
        let command_lock = self.command_mutex.try_lock().or(Err(()))?;
        self._send_command(&cmd).await?;
        Timer::after(Duration::from_millis(20)).await;

        let mut peer_id = None;
        let mut peer_connected_ip = false;
        let mut channel_id = None;

        info!("cmd");
        poll_fn(|cx| {
            let lock = self.packet.try_lock();
            if let Ok(mut lock) = lock {
                if let Some(packet) = &*lock {
                    info!("{:?}", packet);
                    info!("{:?}", packet);
                    info!("{:?}", packet);
                    info!("{:?}", packet);
                    match packet {
                        EdmPacket::ATResponse(response) => {
                            if let ATResponse::Ok(resp) = response {
                                if let Some(i) = resp.find("+UDCP:") {
                                    peer_id = Some(resp[i + 6..].parse::<u8>().or(Err(()))?);
                                    *lock = None;
                                    self.packet_complete.signal(());
                                } else {
                                    *lock = None;
                                    self.packet_complete.signal(());
                                    return Poll::Ready(Err(()));
                                }
                            } else {
                                *lock = None;
                                self.packet_complete.signal(());
                                return Poll::Ready(Err(()));
                            }
                        }
                        EdmPacket::ATEvent(ATEvent::PeerConnectedIP {
                            peer_handle: _,
                            is_ipv6: _,
                            protocol: _,
                            local_address: _,
                            local_port: _,
                            remote_address: _,
                            remote_port: _,
                        }) => {
                            peer_connected_ip = true;
                            *lock = None;
                            self.packet_complete.signal(());
                        }
                        EdmPacket::ConnectEvent {
                            channel,
                            event_type: _,
                        } => {
                            channel_id = Some(*channel);
                            *lock = None;
                            self.packet_complete.signal(());
                        }
                        // EdmPacket::DisconnectEvent { channel: _ } => {
                        //     *lock = None;
                        //     self.packet_complete.signal(());
                        //     return Poll::Ready(Err(()))
                        // }
                        _ => {}
                    }
                }
            }

            if peer_id.is_some() && peer_connected_ip && channel_id.is_some() {
                return Poll::Ready(Ok(()));
            }
            critical_section::with(|_| unsafe {
                *self.response_waker.get() = Some(cx.waker().clone());
            });
            Poll::Pending
        })
        .await?;
        info!("cmd end");

        drop(command_lock);

        let channel_id = channel_id.unwrap();
        let peer_id = peer_id.unwrap();

        {
            let mut peers = self.peers.lock().await;
            if peers.contains_key(&peer_id) {
                return Err(());
            }
            let _ = peers.insert(
                peer_id,
                OpenPeerConnection {
                    channel_id,
                    is_read: false,
                    is_write: false,
                    waker: None,
                },
            );
        }

        Ok(PeerConnection {
            peer_id,
            channel_id,
        })
    }

    // pub async fn list_peers<const LEN: usize>(&self) -> heapless::Vec<, > {
    pub async fn list_peers(&self) {
        self.send_command("AT+UDLP?", |response| {
            info!("{:?}", response);
        }).await.unwrap();
    }

    pub async fn close_peer(&self, peer_id: u8) -> Result<(), ()> {
        let mut cmd: String<12> = String::new();
        write!(cmd, "AT+UDCPC={peer_id}").or(Err(()))?;
        let command_lock = self.command_mutex.try_lock().or(Err(()))?;

        {
            let mut peers = self.peers.lock().await;
            let peer = peers.get(&peer_id);
            if let Some(peer) = peer {
                if peer.is_read || peer.is_write {
                    return Err(());
                }
            } else {
                return Err(());
            }
            peers.remove(&peer_id);
        }

        self._send_command(&cmd).await?;
        Timer::after(Duration::from_millis(1000)).await;

        let mut ok = false;
        let mut peer_disconnect = false;
        let mut disconnect = false;

        poll_fn(|cx| {
            let lock = self.packet.try_lock();
            if let Ok(mut lock) = lock {
                if let Some(packet) = &*lock {
                    match packet {
                        EdmPacket::ATResponse(response) => {
                            if let ATResponse::Ok(_) = response {
                                ok = true;
                                *lock = None;
                                self.packet_complete.signal(());
                            } else {
                                *lock = None;
                                self.packet_complete.signal(());
                                return Poll::Ready(Err(()));
                            }
                        }
                        EdmPacket::ATEvent(ATEvent::PeerDisconnected { peer_handle: _ }) => {
                            peer_disconnect = true;
                            *lock = None;
                            self.packet_complete.signal(());
                        }
                        EdmPacket::DataEvent {
                            channel: _,
                            data: _,
                        } => {
                            *lock = None;
                            self.packet_complete.signal(());
                        }
                        EdmPacket::DisconnectEvent { channel: _ } => {
                            disconnect = true;
                            *lock = None;
                            self.packet_complete.signal(());
                        }
                        _ => {}
                    }
                }
            }

            if ok && peer_disconnect && disconnect {
                return Poll::Ready(Ok(()));
            }
            critical_section::with(|_| unsafe {
                *self.response_waker.get() = Some(cx.waker().clone());
            });
            Poll::Pending
        })
        .await?;

        drop(command_lock);

        Timer::after(Duration::from_millis(50)).await;

        Ok(())
    }

    pub async fn write_data(&self, peer_id: u8, buf: &[u8]) -> Result<(), ()> {
        let channel_id = {
            let mut peers = self.peers.lock().await;
            let peer = peers.get_mut(&peer_id).ok_or(())?;
            if peer.is_write {
                return Err(());
            };
            peer.is_write = true;
            peer.channel_id
        };

        let mut data_ref = self.interface.write().await?;
        let len = EdmPacket::DataCommand {
            channel: channel_id,
            data: buf,
        }
        .write(data_ref.data());
        if let Ok(len) = len {
            *data_ref.len() = len;
        } else {
            data_ref.cancel();
            return Err(());
        }

        drop(data_ref);

        // self.interface
        //     .write(|b| {
        //         EdmPacket::DataCommand {
        //             channel: channel_id,
        //             data: buf,
        //         }
        //         .write(b)
        //     })
        //     .await?;

        {
            let mut peers = self.peers.lock().await;
            let peer = peers.get_mut(&peer_id).unwrap();
            peer.is_write = false;
        }

        Ok(())
    }

    pub async fn read_data(&self, peer_id: u8, buf: &mut [u8]) -> Result<usize, ()> {
        let channel_id = {
            let mut peers = self.peers.lock().await;
            let peer = peers.get_mut(&peer_id).ok_or(())?;
            // TODO: set is_read = false on dropped future
            // if peer.is_read {
            //     return Err(());
            // };
            peer.is_read = true;
            peer.channel_id
        };

        let size = poll_fn(|cx| {
            let lock = self.packet.try_lock();
            match lock {
                Ok(mut lock) => {
                    if let Some(EdmPacket::DataEvent { channel, data }) = &*lock {
                        if *channel == channel_id {
                            let size = data.len();
                            if buf.len() < size {
                                *lock = None;
                                self.packet_complete.signal(());
                                return Poll::Ready(Err(()))
                            }
                            buf[..size].copy_from_slice(data);
                            *lock = None;
                            self.packet_complete.signal(());
                            return Poll::Ready(Ok(size));
                        }
                    }
                }
                Err(_) => {}
            }

            {
                let peers = self.peers.try_lock();
                if let Ok(mut peers) = peers {
                    let peer = peers.get_mut(&peer_id).unwrap();
                    peer.waker = Some(cx.waker().clone());
                }
                // TODO: else?
            }

            Poll::Pending
        })
        .await?;

        {
            let mut peers = self.peers.lock().await;
            let peer = peers.get_mut(&peer_id).unwrap();
            peer.is_read = false;
        }

        Ok(size)
    }
}

// struct OdinRadioTask<'a, RadioInterface: RadioInterfaceControl + 'a> {
//     radio: &'a OdinRadio<'a, RadioInterface>,
// }

impl<'a, RadioInterface: RadioInterfaceControl + 'a> Task for OdinRadio<'a, RadioInterface> {
    type Data = &'a Self;
    async fn task(radio: Self::Data) {
        join(
            (async || loop {
                let _ = radio.read_packet().await;
            })(),
            (async || {
                radio.initialize().await.unwrap();
            })(),
        )
        .await;
    }
}

impl<'a, RadioInterface: RadioInterfaceControl> UdpSocket for OdinSocket<'a, RadioInterface> {
    async fn send(&self, buf: &[u8]) -> Result<usize, ()> {
        self.radio.write_data(self.peer_id, buf).await?;
        Ok(buf.len())
    }
    async fn recv(&self, buf: &mut [u8]) -> Result<usize, ()> {
        self.radio.read_data(self.peer_id, buf).await
    }

    async fn disconnect(&self) {
        let _ = self.radio.close_peer(self.peer_id).await;
    }
}

// impl<'a, RadioInterface: RadioInterfaceControl> Drop for OdinSocket<'a, RadioInterface> {
//     fn drop(&mut self) {
//         // self.radio.close_peer(self.peer_id).await;
//         // TODO:
//     }
// }
