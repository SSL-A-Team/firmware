use embassy_stm32::usart;
use core::future::Future;

pub mod queue;

// pub type ReadTaskFuture<
//     UART: usart::Instance,
//     DMA: usart::RxDma<UART>,
//     const LENGTH: usize,
//     const DEPTH: usize,
// > = impl Future;