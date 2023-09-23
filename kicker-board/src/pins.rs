use embassy_stm32::peripherals::{
    PA0, PA1, PA2, PA3, PA8, PA9, PA10, PA11, PA12,
    PB0, PB1, PB3, PB4, PB5,
    USART1,
    DMA1_CH2, DMA1_CH3,
};

pub type KickPin = PB0;
pub type ChipPin = PB1;
pub type ChargePin = PB3;
pub type RegulatorDonePin = PB4;
pub type RegulatorFaultPin = PB5;

pub type HighVoltageReadPin = PA0;
pub type BatteryVoltageReadPin = PA1;
pub type BreakbeamTxPin = PA3;
pub type BreakbeamRxPin = PA2;

pub type BlueStatusLedPin = PA8;
pub type GreenStatusLedPin = PA11;
pub type RedStatusLedPin = PA12;

pub type ComsUartModule = USART1;
pub type ComsUartTxPin = PA9;
pub type ComsUartRxPin = PA10;
pub type ComsUartTxDma = DMA1_CH2;
pub type ComsUartRxDma = DMA1_CH3;