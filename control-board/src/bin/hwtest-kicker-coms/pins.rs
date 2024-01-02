#![allow(dead_code)]

use embassy_stm32::peripherals::*;

pub type RadioUART = USART10;
pub type RadioTxDMA = DMA2_CH0;
pub type RadioRxDMA = DMA2_CH1;
pub type RadioReset = PC13;

pub type MotorFRUart = USART1;
pub type MotorFLUart = UART4;
pub type MotorBLUart = UART7;
pub type MotorBRUart = UART8;
pub type MotorDUart = UART5;
pub type KickerUart = USART6;

pub type MotorFRDmaTx = DMA1_CH0;
pub type MotorFRDmaRx = DMA1_CH1;
pub type MotorFLDmaTx = DMA1_CH2;
pub type MotorFLDmaRx = DMA1_CH3;
pub type MotorBLDmaTx = DMA1_CH4;
pub type MotorBLDmaRx = DMA1_CH5;
pub type MotorBRDmaTx = DMA1_CH6;
pub type MotorBRDmaRx = DMA1_CH7;
pub type MotorDDmaTx = DMA2_CH2;
pub type MotorDDmaRx = DMA2_CH3;
pub type KickerDmaTx = DMA2_CH4;
pub type KickerDmaRx = DMA2_CH5;

pub type MotorFRBootPin = PD8;
pub type MotorFLBootPin = PC1;
pub type MotorBLBootPin = PF8;
pub type MotorBRBootPin = PB9;
pub type MotorDBootPin = PD13;
pub type KickerBootPin = PA8;

pub type MotorFRResetPin = PD9;
pub type MotorFLResetPin = PC0;
pub type MotorBLResetPin = PF9;
pub type MotorBRResetPin = PB8;
pub type MotorDResetPin = PD12;
pub type KickerResetPin = PA9;


pub type PowerStatePin = PF5;
pub type PowerStateExti = EXTI5;
pub type ShutdownCompletePin = PF4;