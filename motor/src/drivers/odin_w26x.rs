use stm32h7xx_hal::{serial, dma};

use crate::peripherals::uart::{UartDma, UartDmaStreamType};

pub struct OdinW26X<USART, RxDmaStream, TxDmaStream, const RX_BUF_SIZE: usize = 512, const RX_BUF_DEPTH: usize = 4, const TX_BUF_SIZE: usize = 512, const TX_BUF_DEPTH: usize = 2> 
    where USART: serial::SerialExt,
          RxDmaStream: UartDmaStreamType,
          TxDmaStream: UartDmaStreamType,
          serial::Tx<USART>: dma::traits::TargetAddress<dma::MemoryToPeripheral>,
          serial::Rx<USART>: dma::traits::TargetAddress<dma::PeripheralToMemory>, {
    uart_dma: UartDma<USART, RxDmaStream, TxDmaStream, RX_BUF_SIZE, RX_BUF_DEPTH, TX_BUF_SIZE, TX_BUF_DEPTH>
}

impl<USART, RxDmaStream, TxDmaStream, const RX_BUF_SIZE: usize, const RX_BUF_DEPTH: usize, const TX_BUF_SIZE: usize, const TX_BUF_DEPTH: usize> OdinW26X<USART, RxDmaStream, TxDmaStream, RX_BUF_SIZE, RX_BUF_DEPTH, TX_BUF_SIZE, TX_BUF_DEPTH> 
    where USART: serial::SerialExt,
        RxDmaStream: UartDmaStreamType,
        TxDmaStream: UartDmaStreamType,
        serial::Tx<USART>: dma::traits::TargetAddress<dma::MemoryToPeripheral>,
        serial::Rx<USART>: dma::traits::TargetAddress<dma::PeripheralToMemory>, {
    fn init(uart_dma: UartDma<USART, RxDmaStream, TxDmaStream, RX_BUF_SIZE, RX_BUF_DEPTH, TX_BUF_SIZE, TX_BUF_DEPTH>) -> OdinW26X<USART, RxDmaStream, TxDmaStream, RX_BUF_SIZE, RX_BUF_DEPTH, TX_BUF_SIZE, TX_BUF_DEPTH> {
        OdinW26X {
            uart_dma: uart_dma
        }
    }
}

// unsafe impl<USART, RxDmaStream, TxDmaStream, const BUF_SIZE: usize> 
// Send for OdinW26X<USART, RxDmaStream, TxDmaStream, BUF_SIZE> 
//     where USART: serial::SerialExt,
//             RxDmaStream: UartDmaStreamType,
//             TxDmaStream: UartDmaStreamType,
//             serial::Tx<USART>: dma::traits::TargetAddress<dma::MemoryToPeripheral>,
//             serial::Rx<USART>: dma::traits::TargetAddress<dma::PeripheralToMemory> {}

// unsafe impl<USART, RxDmaStream, TxDmaStream, const BUF_SIZE: usize> 
// Sync for OdinW26X<USART, RxDmaStream, TxDmaStream, BUF_SIZE> 
//     where USART: serial::SerialExt,
//             RxDmaStream: UartDmaStreamType,
//             TxDmaStream: UartDmaStreamType,
//             serial::Tx<USART>: dma::traits::TargetAddress<dma::MemoryToPeripheral>,
//             serial::Rx<USART>: dma::traits::TargetAddress<dma::PeripheralToMemory> {}