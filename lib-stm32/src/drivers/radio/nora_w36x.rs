use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use embassy_stm32::{gpio::Output, usart::{Config, Parity, StopBits}};
use embassy_time::Timer;

use crate::uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue};


enum WifiConnectionState {
    CONNECTED,
    UNCONNECTED,
}

enum DataState {
    TRANSPARENT,
    OPAQUE
}

pub enum NoraW36xError {
    UartQueueError(crate::queue::Error),
}

impl From<crate::queue::Error> for NoraW36xError {
    fn from(value: crate::queue::Error) -> Self {
        NoraW36xError::UartQueueError(value)
    }
}

pub fn get_factory_uart_settings() -> Config {
    let mut uart_config = Config::default();
    uart_config.baudrate = 115_200;
    uart_config.stop_bits = StopBits::STOP1;
    uart_config.parity = Parity::ParityNone;

    uart_config
}

pub fn get_highest_datarate_uart_settings() -> Config {
    let mut uart_config = Config::default();
    uart_config.baudrate = 4_000_000;
    uart_config.stop_bits = StopBits::STOP1;
    uart_config.parity = Parity::ParityNone;

    uart_config
}

pub fn get_highest_datarate_with_parity_uart_settings() -> Config {
    let mut uart_config = Config::default();
    uart_config.baudrate = 4_000_000;
    uart_config.stop_bits = StopBits::STOP1;
    uart_config.parity = Parity::ParityEven;

    uart_config
}

pub struct SocketHandle {
    open: AtomicBool,
    net_connected: AtomicBool,
    buffered: bool,
    has_data: bool,
    nora_socket_handle: AtomicUsize,
}

impl SocketHandle {
    pub fn is_open(&self) -> bool {
        self.open.load(Ordering::Acquire)
    }

    pub fn network_connected(&self) -> bool {
        self.net_connected.load(Ordering::Acquire)
    }

    pub fn can_send(&self) -> bool {
        self.network_connected() && self.can_send()
    }
}

pub struct NoraW36x<
    'a,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    const DEBUG_UART_QUEUES: bool,
> {
    reader: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_UART_QUEUES>,
    writer: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
    uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
    reset_pin: Output<'static>,
    
    wifi_connection_state: WifiConnectionState,
    data_state: DataState,
}

impl<
    'a,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    const DEBUG_UART_QUEUES: bool,
> NoraW36x<'a, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, DEBUG_UART_QUEUES> {
    pub fn new(
        reader: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_UART_QUEUES>,
        writer: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        reset_pin: Output<'static>,
    ) -> Self {
        NoraW36x {
            reader,
            writer,
            uart,
            reset_pin,
            wifi_connection_state: WifiConnectionState::UNCONNECTED,
            data_state: DataState::OPAQUE,
        }
    }

    fn link_write_data(&mut self, buf: &[u8]) -> Result<(), NoraW36xError> {
        self.writer.enqueue_copy(buf)?;
        Ok(())
    }

    async fn link_write_data_blocking(&mut self) {

    }

    fn link_read_data(&mut self) {

    }

    async fn link_read_data_blocking(&mut self) {

    }

    pub fn reset_state(&mut self) {
        self.wifi_connection_state = WifiConnectionState::UNCONNECTED;
        self.data_state = DataState::OPAQUE;
    }

    pub async fn sys_hard_reset(&mut self) {
        self.reset_pin.set_high();
        Timer::after_millis(1).await;
        self.reset_pin.set_low();
        Timer::after_millis(1).await;

        self.reset_state();

        // TODO wait for +STARTUP
    }

    pub fn sys_soft_reset(&mut self) {
        // TODO: send AT+CPWROFF

        // TODO: wait for +STARTUP
    }

    pub fn sys_verify_device(&self) -> Result<(), ()> {
        // TODO: send CGMI -> "u-blox"
        // TODO: send CGMM -> 

        Err(())
    }

    pub fn sys_firmware_version(&self) {
        // TODO: send CGMR -> 
    }

    pub fn sys_update_radio_uart_config(&self, _store: bool) -> Result<(), NoraW36xError> {
        Ok(())
    }

    pub fn wifi_discover_available_networks(&mut self) -> Result<(), NoraW36xError> {
        Ok(())
    }

    pub fn wifi_network_present(&mut self, _update: bool) -> Result<(), NoraW36xError> {
        Ok(())
    }

    pub fn wifi_connect(&mut self) -> Result<(), NoraW36xError> {
        self.wifi_connection_state = WifiConnectionState::CONNECTED;
        Ok(())
    }

    pub fn wifi_disconnect(&mut self) -> Result<(), NoraW36xError> {
        self.wifi_connection_state = WifiConnectionState::UNCONNECTED;
        Ok(())
    }

    pub fn tcp_open_socket(&mut self) -> Result<SocketHandle, NoraW36xError> {
        unimplemented!()
    }

    pub fn udp_open_socket(&mut self) -> Result<SocketHandle, NoraW36xError> {
        unimplemented!()
    }

    pub fn tcp_close_socket(&mut self, socket_handle: SocketHandle) -> Result<(), NoraW36xError> {
        unimplemented!()
    }

    pub fn socket_send_data(&mut self, socket_handle: SocketHandle) -> Result<(), NoraW36xError> {
        unimplemented!()
    } 

    pub async fn socket_send_data_blocking(&mut self, socket_handle: SocketHandle) -> Result<(), NoraW36xError> {
        unimplemented!()
    }

    pub fn socket_read_data_from_handle(&mut self, socket_handle: SocketHandle, buf: &mut [u8]) -> Result<(), NoraW36xError> {
        unimplemented!()

        // if socket is in direct mode, this isn't possible
        // if multiple sockets exist all of their data will be queued together
        // and you must read it in order, optionally discarding from other handles
        // if you need to be able to quert a specific socket read at a specific time
        // you must use buffered mode
    }

    pub fn socket_read_data(&mut self) -> Option<Result<(), NoraW36xError>> {
        None
    }
}