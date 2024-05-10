use core::future::Future;

use embassy_executor::{raw::TaskStorage, SendSpawner, SpawnToken};
use embassy_futures::join::join;
use embassy_stm32::{gpio::Pin, usart};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::Uart;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};

use defmt::*;

use ateam_lib_stm32::make_uart_queues;
use ateam_lib_stm32::uart::queue::{UartReadQueue, UartWriteQueue};

use ateam_common_packets::bindings_radio::{BasicControl, BasicTelemetry, ControlDebugTelemetry, ParameterCommand};
use ateam_common_packets::radio::DataPacket;

use crate::drivers::radio::{RobotRadio, TeamColor, WifiNetwork};
use crate::pins::*;

type ControlTaskFuture<
    UART: usart::BasicInstance + Send,
    RxDma: usart::RxDma<UART> + Send,
    TxDma: usart::TxDma<UART> + Send,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    ResetPin: Pin + Send + Sync,
> where
    UART::Interrupt: Send,
= impl Future + Sync;

/*
 * Uart tx (Radio)
 *   mut - write task
 * Uart rx (Radio)
 *   mut - read task
 * Write Queue
 *   [mut] - write task
 *   [mut] - control task (through radio)
 * Read Queue
 *   [mut] - read task
 *   [mut] - radio read task
 * Radio State
 *   mut - radio read task
 * Latest Control
 *   mut - radio read task
 *   mut - control task
 */

pub struct RadioTest<
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    UART: usart::BasicInstance + Send,
    RxDma: usart::RxDma<UART> + Send,
    TxDma: usart::TxDma<UART> + Send,
    ResetPin: Pin + Send + Sync,
> where
    UART::Interrupt: Send,
{
    queue_tx: &'static UartWriteQueue<UART, TxDma, LEN_TX, DEPTH_TX>,
    queue_rx: &'static UartReadQueue<UART, RxDma, LEN_RX, DEPTH_RX>,
    task: TaskStorage<
        ControlTaskFuture<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>,
    >,
    latest_control: Mutex<CriticalSectionRawMutex, Option<BasicControl>>,
    latest_params: Mutex<CriticalSectionRawMutex, Option<ParameterCommand>>,
    radio: Option<RobotRadio<'static, UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX>>,
}

impl<
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        UART: usart::BasicInstance + Send,
        RxDma: usart::RxDma<UART> + Send,
        TxDma: usart::TxDma<UART> + Send,
        ResetPin: Pin + Send + Sync,
    > RadioTest<LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, UART, RxDma, TxDma, ResetPin>
where
    UART::Interrupt: Send,
{
    pub const fn new(
        tx_queue: &'static UartWriteQueue<UART, TxDma, LEN_TX, DEPTH_TX>,
        rx_queue: &'static UartReadQueue<UART, RxDma, LEN_RX, DEPTH_RX>,
    ) -> Self {
        RadioTest {
            queue_rx: rx_queue,
            queue_tx: tx_queue,
            task: TaskStorage::new(),
            latest_control: Mutex::new(None),
            latest_params: Mutex::new(None),
            radio: None,
        }
    }

    // setup uart
    // setup tasks
    pub async fn setup(
        &'static mut self,
        spawner: &SendSpawner,
        uart: usart::Uart<'static, UART, Async>,
        reset_pin: impl Pin,
        id: u8,
        team: TeamColor,
        wifi_network: WifiNetwork,
    ) -> SpawnToken<impl Sized> {
        let (tx, rx) = Uart::split(uart);

        spawner.spawn(self.queue_rx.spawn_task(rx)).unwrap();
        spawner.spawn(self.queue_tx.spawn_task(tx)).unwrap();

        let mut radio = RobotRadio::new(&self.queue_rx, &self.queue_tx, reset_pin)
            .await
            .unwrap();

        info!("radio created");
        defmt::panic!("radio temporarily left unconnected");
        // radio.connect_to_network(wifi_network).await.unwrap();
        info!("radio connected");

        radio.open_multicast().await.unwrap();
        info!("multicast open");

        loop {
            info!("sending hello");
            radio.send_hello(id, team).await.unwrap();
            let hello = radio.wait_hello(Duration::from_millis(1000)).await;

            match hello {
                Ok(hello) => {
                    info!(
                        "recieved hello resp to: {}.{}.{}.{}:{}",
                        hello.ipv4[0], hello.ipv4[1], hello.ipv4[2], hello.ipv4[3], hello.port
                    );
                    radio.close_peer().await.unwrap();
                    info!("multicast peer closed");
                    radio.open_unicast(hello.ipv4, hello.port).await.unwrap();
                    info!("unicast open");
                    break;
                }
                Err(_) => {}
            }
        }

        {
            self.radio = Some(radio);
        }

        self.task
            .spawn(|| Self::control_task(self.radio.as_ref().unwrap(), &self.latest_control, &self.latest_params))
    }

    fn control_task(
        radio: &'static RobotRadio<
            'static,
            UART,
            RxDma,
            TxDma,
            LEN_TX,
            LEN_RX,
            DEPTH_TX,
            DEPTH_RX
        >,
        latest_control: &'static Mutex<CriticalSectionRawMutex, Option<BasicControl>>,
        latest_param_cmd: &'static Mutex<CriticalSectionRawMutex, Option<ParameterCommand>>,
    ) -> ControlTaskFuture<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin> {
        let last_control: Mutex<CriticalSectionRawMutex, bool> = Mutex::new(true);
        async move {
            join(
                (async || loop {
                    let data_packet = radio.read_packet().await;
                    if let Ok(data_packet) = data_packet {
                        if let DataPacket::BasicControl(control) = data_packet {
                            let mut latest_control = latest_control.lock().await;
                            *latest_control = Some(control);

                            // feed the disconnected watch-dog because we got a packet
                            {
                                let mut last_control = last_control.lock().await;
                                *last_control = true;
                            }
                        } else if let DataPacket::ParameterCommand(param_cmd) = data_packet {
                            let mut latest_param_cmd = latest_param_cmd.lock().await;
                            *latest_param_cmd = Some(param_cmd);
                        }
                    }
                })(),
                (async || {
                    // TODO: parameterize this timeout
                    let mut no_packet_timeout = Ticker::every(Duration::from_millis(3000));
                    loop {
                        {
                            // TODO this can be an AtomicBool
                            let mut last_control = last_control.lock().await;
                            if !*last_control {
                                cortex_m::peripheral::SCB::sys_reset();
                            }
                            *last_control = false;
                        }
                        no_packet_timeout.next().await;
                    }
                })(),
            )
            .await;
        }
    }

    // write telemetry to queue
    pub async fn send_telemetry(&self, telemetry: BasicTelemetry) {
        self.radio
            .as_ref()
            .unwrap()
            .send_telemetry(telemetry)
            .await
            .unwrap();
    }

    pub async fn send_control_debug_telemetry(&self, control_debug_telemetry: ControlDebugTelemetry) {
        self.radio
            .as_ref()
            .unwrap()
            .send_control_debug_telemetry(control_debug_telemetry)
            .await
            .unwrap();
    }

    pub async fn send_parameter_response(&self, parameter_command: ParameterCommand) {
        self.radio
        .as_ref()
        .unwrap()
        .send_parameter_response(parameter_command)
        .await
        .unwrap();
    }

    // fetch latest stored control value
    pub fn get_latest_control(&self) -> Option<BasicControl> {
        let mut latest_control = self.latest_control.try_lock();
        if let Ok(latest_control) = &mut latest_control {
            latest_control.take()
        } else {
            None
        }
    }

    // fetch latest stored parameters packet
    pub fn get_latest_params_cmd(&self) -> Option<ParameterCommand> {
        let mut latest_param_cmd = self.latest_params.try_lock();
        if let Ok(latest_param_cmd) = &mut latest_param_cmd {
            latest_param_cmd.take()
        } else {
            None
        }
    }
}
