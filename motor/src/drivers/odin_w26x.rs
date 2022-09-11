use stm32h7xx_hal::{serial, dma};

use crate::peripherals::uart::{UartDma, UartDmaStreamType};

pub struct OdinW26X<USART, RxDmaStream, TxDmaStream, const RX_BUF_SIZE: usize = 512, const TX_BUF_SIZE: usize = 512> 
    where USART: serial::SerialExt,
          RxDmaStream: UartDmaStreamType,
          TxDmaStream: UartDmaStreamType,
          serial::Tx<USART>: dma::traits::TargetAddress<dma::MemoryToPeripheral>,
          serial::Rx<USART>: dma::traits::TargetAddress<dma::PeripheralToMemory>, {
    uart_dma: UartDma<USART, RxDmaStream, TxDmaStream, RX_BUF_SIZE, TX_BUF_SIZE>
}

impl<USART, RxDmaStream, TxDmaStream, const RX_BUF_SIZE: usize, const TX_BUF_SIZE: usize> OdinW26X<USART, RxDmaStream, TxDmaStream, RX_BUF_SIZE, TX_BUF_SIZE> 
    where USART: serial::SerialExt,
        RxDmaStream: UartDmaStreamType,
        TxDmaStream: UartDmaStreamType,
        serial::Tx<USART>: dma::traits::TargetAddress<dma::MemoryToPeripheral>,
        serial::Rx<USART>: dma::traits::TargetAddress<dma::PeripheralToMemory>, {
    fn init(uart_dma: UartDma<USART, RxDmaStream, TxDmaStream, RX_BUF_SIZE, TX_BUF_SIZE>) -> OdinW26X<USART, RxDmaStream, TxDmaStream, RX_BUF_SIZE, TX_BUF_SIZE> {
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