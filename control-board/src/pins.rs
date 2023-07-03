// use embassy_stm32::{peripherals::*, usart, Peripheral};

pub mod MotorFR {
    use embassy_stm32::peripherals::*;
    pub type Uart = USART1;
    pub type Tx = PB15;
    pub type Rx = PB14;
    pub type DmaTx = DMA1_CH0;
    pub type DmaRx = DMA1_CH1;
    pub type Boot = PD8;
    pub type Reset = PD9;
}

// pub struct MotorPins<'a, Uart: usart::BasicInstance, DmaTx, DmaRx> {
//     pub uart: usart::Uart<'a, Uart, DmaTx, DmaRx>,
// }

// trait MotorPinsTrait {
//     type Uart: usart::BasicInstance;
//     type Tx: Peripheral;
//     type Rx: Peripheral;
//     type DmaTx;
//     type DmaRx;
//     type Boot: Peripheral;
//     type Reset: Peripheral;
// }

// // impl<'a, Uart: usart::BasicInstance, DmaTx, DmaRx> MotorPins<'a, Uart, DmaTx, DmaRx> {
// //     pub type DmaTx = DmaTx;
// //     pub type DmaRx = DmaRx;
// // }

// pub type MotorFR<'a> = MotorPins<'a, USART1, DMA1_CH0, DMA1_CH1>;
// impl<'a> MotorPinsTrait for MotorFR<'a> {
//     type Uart = USART1;
//     type Tx = PB15;
//     type Rx = PB14;
//     type DmaTx = DMA1_CH0;
//     type DmaRx = DMA1_CH1;
//     type Boot = PD8;
//     type Reset = PD9;
// }
