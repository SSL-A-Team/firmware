use embedded_hal::digital::v2::OutputPin;
use stm32h7xx_hal::{prelude::*, rcc, serial};

use super::{stm32_bootloader::StmBootloader, timeout};

// TODO: remove this:
// #[link_section = ".axisram_prog.stspin"]
// static STSPIN_BINARY: [u8; include_bytes!("../../steval-spin3201-6step.bin").len()] =
//     *include_bytes!("../../steval-spin3201-6step.bin");

pub struct STSpin<USART, BOOT, RST> {
    boot: BOOT,
    rst: RST,
    serial: serial::Serial<USART>,
}

impl<USART: serial::SerialExt, BOOT: OutputPin, RST: OutputPin> STSpin<USART, BOOT, RST>
where
// serial::Serial<USART>: trait_ext::Serial<USART>,
// serial::Tx<USART>: trait_ext::SerialTx<USART>,
// USART: serial::SerialExt,
// serial::Serial<USART>: embedded_hal::serial::Read<u8>,
// serial::Serial<USART>: embedded_hal::serial::Write<u8>,
// <serial::Serial<USART> as embedded_hal::serial::Read<u8>>::Error: core::fmt::Debug,
{
    pub fn init<PINS: serial::Pins<USART>>(
        serial: USART,
        uart_pins: PINS,
        prec: USART::Rec,
        clocks: &rcc::CoreClocks,
        boot: BOOT,
        rst: RST,
    ) -> STSpin<USART, BOOT, RST> {
        let serial = serial
            .serial(uart_pins, 115_200.bps(), prec, clocks)
            .unwrap();
        STSpin { serial, boot, rst }
    }

    // pub fn program(&mut self) -> Result<(), ()> {
    //     self.boot.set_high().or(Err(()))?;
    //     self.rst.set_high().or(Err(()))?;
    //     timeout::delay_us(20000);
    //     let mut bootloader = StmBootloader::new(&mut self.serial)?;
    //     bootloader.write_flash(&STSPIN_BINARY[..])?;
    //     self.boot.set_low().or(Err(()))?;
    //     self.rst.set_low().or(Err(()))?;
    //     self.rst.set_high().or(Err(()))?;

    //     Ok(())
    // }

    // pub fn start_program<STREAM>(self, stream: STREAM)
    // where
    //     STREAM: DoubleBufferedStream + Stream<Config = DmaConfig>,
    //     serial::Tx<USART>: TargetAddress<MemoryToPeripheral>,
    //     &'static [u8]:
    //         ReadBuffer<Word = <serial::Tx<USART> as TargetAddress<MemoryToPeripheral>>::MemSize>,
    // {
    //     let (tx, rx) = self.serial.split();
    //     let config = DmaConfig::default().memory_increment(true);
    //     let mut tx_transfer =
    //         dma::Transfer::init_const(stream, tx, &STSPIN_BINARY[..], None, config);
    //     tx_transfer.start(|serial| {
    //         serial.enable_dma_tx();
    //     });
    //     while !tx_transfer.get_transfer_complete_flag() {}

    //     let (_stream, tx, _, _) = tx_transfer.free();
    //     let st = STSpin {
    //         serial: serial::Serial::join(tx, rx),
    //     };
    // }
}
