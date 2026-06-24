#![no_std]
#![no_main]
#![feature(sync_unsafe_cell)]

// Radio round-trip latency profiler — W36x (Nora) variant.
//
// Connects to WiFi via the standard hello handshake, then enters a raw
// echo loop: any UDP payload received is immediately sent back to the host.
// No A-Team packet parsing is performed.
//
// Payload size: 512 bytes.
// Constraint: RobotRadioNora::read_data uses AT+USORB with a fixed
// read length of size_of::<RadioPacket>() = 524 bytes. Sending packets
// larger than 524 bytes would split across multiple reads. Use 512 bytes
// as a clean power-of-two that fits comfortably within this limit.
//
// Companion script: scripts/radio/radio_latency.py --payload-size 512

use ateam_control_board::{
    drivers::radio_robot::TeamColor,
    drivers::radio_robot_nora::RobotRadioNora,
    get_system_config, SystemIrqs, DEBUG_RADIO_UART_QUEUES,
};
use ateam_lib_stm32::{
    idle_buffered_uart_read_task, idle_buffered_uart_write_task, static_idle_buffered_uart,
};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    interrupt, pac::Interrupt,
    usart::{DataBits, Parity, StopBits, Uart},
};
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Timer};

#[cfg(not(feature = "no-private-credentials"))]
use credentials::private_credentials::wifi::wifi_credentials;
#[cfg(feature = "no-private-credentials")]
use credentials::public_credentials::wifi::wifi_credentials;

use defmt_rtt as _;

// Match DIP switch 3 default (true = flow control enabled).
// Set false if DIP switch 3 is off on your board.
pub const USE_FLOW_CONTROL: bool = true;

// 512-byte payload; AT protocol framing overhead for TX is ~15 bytes,
// for RX (AT+USORB response) is ~20 bytes. Use 600 for comfortable margin.
pub const PROFILE_PAYLOAD_SIZE: usize = 512;
pub const PROFILE_BUF_SIZE: usize = 600;
pub const PROFILE_BUF_DEPTH: usize = 4;

static_idle_buffered_uart!(
    RADIO_NORA_PROF,
    PROFILE_BUF_SIZE,
    PROFILE_BUF_DEPTH,
    PROFILE_BUF_SIZE,
    PROFILE_BUF_DEPTH,
    DEBUG_RADIO_UART_QUEUES,
    #[link_section = ".axisram.buffers"]
);

static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    cortex_m::asm::delay(100_000_000);
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::main]
async fn main(_main_spawner: embassy_executor::Spawner) {
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("profile-radio-w36 (Nora) starting");

    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CEC,
        embassy_stm32::interrupt::Priority::P5,
    );
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    let mut radio_uart_config = embassy_stm32::usart::Config::default();
    radio_uart_config.baudrate = 3_000_000;
    radio_uart_config.data_bits = DataBits::DataBits8;
    radio_uart_config.stop_bits = StopBits::STOP1;
    radio_uart_config.parity = Parity::ParityNone;

    let radio_uart = if USE_FLOW_CONTROL {
        Uart::new_with_rtscts(
            p.USART2,
            p.PD6,
            p.PD5,
            SystemIrqs,
            p.PD4,
            p.PD3,
            p.DMA2_CH0,
            p.DMA2_CH1,
            radio_uart_config,
        )
        .unwrap()
    } else {
        Uart::new(
            p.USART2,
            p.PD6,
            p.PD5,
            SystemIrqs,
            p.DMA2_CH0,
            p.DMA2_CH1,
            radio_uart_config,
        )
        .unwrap()
    };
    let (radio_uart_tx, radio_uart_rx) = Uart::split(radio_uart);

    RADIO_NORA_PROF_IDLE_BUFFERED_UART.init();
    uart_queue_spawner
        .spawn(idle_buffered_uart_read_task!(RADIO_NORA_PROF, radio_uart_rx))
        .unwrap();
    uart_queue_spawner
        .spawn(idle_buffered_uart_write_task!(RADIO_NORA_PROF, radio_uart_tx))
        .unwrap();

    let mut radio = RobotRadioNora::new(
        &RADIO_NORA_PROF_IDLE_BUFFERED_UART,
        RADIO_NORA_PROF_IDLE_BUFFERED_UART.get_uart_read_queue(),
        RADIO_NORA_PROF_IDLE_BUFFERED_UART.get_uart_write_queue(),
        p.PD7.into(),
        USE_FLOW_CONTROL,
    );

    defmt::info!("UART queues started, entering connection loop");

    'uart: loop {
        // --- UART ---
        defmt::info!("connecting UART...");
        loop {
            match select(radio.connect_uart(), Timer::after_millis(5000)).await {
                Either::First(Ok(_)) => break,
                Either::First(Err(e)) => {
                    defmt::warn!("UART connect failed: {}, retrying in 1s", e);
                    Timer::after_millis(1000).await;
                }
                Either::Second(_) => {
                    defmt::warn!("UART connect timed out, retrying...");
                }
            }
        }
        defmt::info!("UART connected");

        // Network session loop: WiFi failures and echo failures stay here;
        // only a hard UART-level fault breaks to 'uart.
        'net: loop {
            // --- WiFi ---
            defmt::info!("connecting WiFi...");
            loop {
                let cred = wifi_credentials[1];
                match radio.connect_to_network(cred, 0).await {
                    Ok(_) => break,
                    Err(e) => {
                        defmt::warn!("WiFi connect failed: {}, retrying in 2s", e);
                        Timer::after_millis(2000).await;
                    }
                }
            }
            defmt::info!("WiFi connected");

            // --- Multicast discovery ---
            if let Err(e) = radio.open_multicast().await {
                defmt::warn!("multicast open failed: {}", e);
                let _ = radio.disconnect_network().await;
                continue 'net;
            }
            defmt::info!("multicast open, sending hello...");

            // --- Software hello ---
            if let Err(e) = radio.send_hello(0, TeamColor::Yellow).await {
                defmt::warn!("send_hello failed: {}", e);
                let _ = radio.disconnect_network().await;
                continue 'net;
            }

            let hello = match radio.wait_hello(Duration::from_millis(5000)).await {
                Ok(h) => h,
                Err(e) => {
                    defmt::warn!("wait_hello failed: {}", e);
                    let _ = radio.disconnect_network().await;
                    continue 'net;
                }
            };
            defmt::info!(
                "hello from {}.{}.{}.{}:{}",
                hello.ipv4[0], hello.ipv4[1], hello.ipv4[2], hello.ipv4[3], hello.port
            );

            // --- Switch to unicast ---
            if let Err(e) = radio.close_peer().await {
                defmt::warn!("close_peer failed: {}", e);
                let _ = radio.disconnect_network().await;
                continue 'net;
            }
            if let Err(e) = radio.open_unicast(hello.ipv4, hello.port).await {
                defmt::warn!("open_unicast failed: {}", e);
                let _ = radio.disconnect_network().await;
                continue 'net;
            }
            defmt::info!("unicast open, entering echo loop");

            // --- Echo loop ---
            // Radio is in direct binary mode (AT+USORM=2): data arrives as +UESODBF URCs.
            // read_data uses AT+USORB (buffered mode) and will fail. Poll read_data_nonblocking
            // instead, which reads from the direct-binary event queue.
            loop {
                let result = loop {
                    match radio.read_data_nonblocking(|data| {
                        let mut buf = [0u8; PROFILE_PAYLOAD_SIZE];
                        let n = data.len().min(PROFILE_PAYLOAD_SIZE);
                        buf[..n].copy_from_slice(&data[..n]);
                        (buf, n)
                    }) {
                        Ok(Some(val)) => break Ok(val),
                        Ok(None) => Timer::after_millis(1).await,
                        Err(e) => break Err(e),
                    }
                };

                match result {
                    Ok((buf, n)) => {
                        defmt::trace!("rx {} bytes, echoing", n);
                        if let Err(e) = radio.send_data(&buf[..n]).await {
                            defmt::warn!("send_data failed: {}", e);
                        }
                    }
                    Err(e) => {
                        defmt::warn!("read_data failed: {}", e);
                    }
                }
            }
        }
    }
}
