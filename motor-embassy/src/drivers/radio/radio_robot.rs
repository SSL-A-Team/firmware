use super::radio::{Radio, WifiAuth};
use crate::uart_queue::{UartReadQueue, UartWriteQueue};
use core::fmt::Write;
use embassy_stm32::gpio::{Level, OutputOpenDrain, Pin, Pull, Speed};
use embassy_stm32::pac;
use embassy_stm32::usart;
use embassy_stm32::Peripheral;
use embassy_time::{Duration, Timer};
use heapless::String;

fn get_uuid() -> u16 {
    unsafe { *(0x1FF1_E800 as *const u16) }
}

pub struct RobotRadio<
    'a,
    UART: usart::BasicInstance,
    DmaRx: usart::RxDma<UART>,
    DmaTx: usart::TxDma<UART>,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    PIN: Pin,
> {
    radio: Radio<
        'a,
        UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_TX>,
        UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_RX>,
    >,
    reset_pin: OutputOpenDrain<'a, PIN>,
    channel: Option<u8>,
}

impl<
        'a,
        UART: usart::BasicInstance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        PIN: Pin,
    > RobotRadio<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_TX, DEPTH_RX, PIN>
{
    pub async fn new(
        read_queue: &'a UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_TX>,
        write_queue: &'a UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_RX>,
        reset_pin: impl Peripheral<P = PIN> + 'a,
    ) -> Result<RobotRadio<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_TX, DEPTH_RX, PIN>, ()>
    {
        let mut reset_pin = OutputOpenDrain::new(reset_pin, Level::Low, Speed::Medium, Pull::None);
        Timer::after(Duration::from_micros(100)).await;
        reset_pin.set_high();

        let mut radio = Radio::new(read_queue, write_queue);
        radio.wait_startup().await?;

        let baudrate = 5250000;
        radio.set_echo(false).await?;
        radio.config_uart(baudrate, false, 8, true).await?;

        let div = (UART::frequency().0 + (baudrate / 2)) / baudrate * UART::MULTIPLIER;
        unsafe {
            let r = UART::regs();
            r.cr1().modify(|w| {
                w.set_ue(false);
            });
            r.brr().modify(|w| {
                w.set_brr(div);
            });
            r.cr1().modify(|w| {
                w.set_ue(true);
                w.set_m0(pac::lpuart::vals::M0::BIT9);
                w.set_pce(true);
                w.set_ps(pac::lpuart::vals::Ps::EVEN);
            });
        };

        // Datasheet says wait at least 40ms after UART config change
        Timer::after(Duration::from_millis(50)).await;

        // Datasheet says wait at least 50ms after entering data mode
        radio.enter_edm().await?;
        radio.wait_edm_startup().await?;
        Timer::after(Duration::from_millis(50)).await;

        Ok(Self {
            radio,
            reset_pin,
            channel: None,
        })
    }

    pub async fn connect_to_network(&mut self) -> Result<(), ()> {
        let mut s = String::<17>::new();
        core::write!(&mut s, "A-Team Robot {:04X}", get_uuid()).unwrap();
        self.radio.set_host_name(s.as_str()).await?;
        self.radio
            .config_wifi(
                1,
                "PROMISED_LAN_DC_DEVEL",
                WifiAuth::WPA {
                    passphrase: "plddevel",
                },
            )
            .await?;
        self.radio.connect_wifi(1).await?;
        let channel = self
            .radio
            .connect_peer("udp://224.4.20.69:42069/?flags=1&local_port=42069")
            .await?;
        self.channel = Some(channel);
        Ok(())
    }

    pub async fn send_data(&self, data: &[u8]) -> Result<(), ()> {
        if let Some(channel) = self.channel {
            self.radio.send_data(channel, data).await
        } else {
            Err(())
        }
    }

    pub async fn read_data<RET, FN>(&'a self, fn_read: FN) -> Result<RET, ()>
    where
        FN: FnOnce(&[u8]) -> RET,
    {
        if self.channel.is_some() {
            self.radio.read_data(fn_read).await
        } else {
            Err(())
        }
    }
}
