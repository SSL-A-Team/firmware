use embassy_stm32::peripherals::*;

pub type ChargePin = PE4;
pub type KickPin = PE5;
pub type ChipPin = PE6;

pub type PowerRailAdc = ADC1;
pub type PowerRail200vReadPin = PC0;
pub type PowerRail12vReadPin = PC1;
pub type PowerRail6v2ReadPin = PC3;
pub type PowerRail5v0ReadPin = PC2;

pub type BreakbeamTxPin = PA3;
pub type BreakbeamRxPin = PA2;

pub type GreenStatusLedPin = PE0;
pub type RedStatusLedPin = PE1;
pub type BlueStatusLed1Pin = PE2;
pub type BlueStatusLed2Pin = PE3;

pub type DotstarSpi = SPI1;
pub type DotstarSpiSckPin = PA5;
pub type DotstarSpiMosiPin = PA7;
pub type DotstarSpiTxDma = DMA2_CH3;

pub type UserBtnPin = PD4;
pub type PowerBtnIntPin = PD5;
pub type PowerBtnIntExti = EXTI5;
pub type PowerKillPin = PD6;

pub type ComsUartModule = USART1;
pub type ComsUartTxPin = PA9;
pub type ComsUartRxPin = PA10;
pub type ComsUartTxDma = DMA2_CH7;
pub type ComsUartRxDma = DMA2_CH2;