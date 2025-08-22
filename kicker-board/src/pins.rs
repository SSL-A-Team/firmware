use embassy_stm32::peripherals::*;

////////////////////
//  HV Functions  //
////////////////////

pub type ChargePin = PB15;
pub type KickPin = PD9;
pub type ChipPin = PD8;

////////////////////////////
//  Voltage Measurements  //
////////////////////////////

pub type PowerRailAdc = ADC1;
pub type PowerRail200vReadPin = PC3;
pub type PowerRailVBattReadPin = PA0;
pub type PowerRailVswReadPin = PA1;
pub type PowerRail5v0ReadPin = PA2;
pub type PowerRail3v3ReadPin = PA3;
pub type PowerRailAdcDma = DMA1_CH7;

//////////////////////
//  Breakbeam Pins  //
//////////////////////

pub type BreakbeamLeftI2c = I2C1;
pub type BreakbeamLeftAgpioPin = PC0;
pub type BreakbeamLeftSdaPin = PB7;
pub type BreakbeamLeftSclPin = PA15;
pub type BreakbeamLeftNdetPin = PC11;
pub type BreakbeamLeftRstPin = PC10;
pub type BreakbeamLeftIntPin = PC12;
pub type BreakbeamLeftI2cRxDma = DMA2_CH1;
pub type BreakbeamLeftI2cTxDma = DMA2_CH2;

pub type BreakbeamRightI2c = I2C4;
pub type BreakbeamRightAgpioPin = PC2;
pub type BreakbeamRightSdaPin = PC7;
pub type BreakbeamRightSclPin = PC6;
pub type BreakbeamRightNdetPin = PD11;
pub type BreakbeamRightRstPin = PC12;
pub type BreakbeamRightIntPin = PD10;
pub type BreakbeamRightI2cRxDma = DMA2_CH3;
pub type BreakbeamRightI2cTxDma = DMA2_CH4;

///////////////
//  User IO  //
///////////////

pub type GreenStatusLedPin = PB9;
pub type RedStatusLedPin = PE0;
pub type BlueStatusLedPin = PE1;

pub type DotstarSpi = SPI4;
pub type DotstarSpiSckPin = PE2;
pub type DotstarSpiMosiPin = PE6;
pub type DotstarSpiRxDma = DMA2_CH7;
pub type DotstarSpiTxDma = DMA2_CH8;

pub type UserBtnPin = PB5;

pub type Dip0Pin = PD0;
pub type Dip1Pin = PD1;
pub type Dip2Pin = PD2;
pub type Dip3Pin = PE4;

pub type TrimPot0Pin = PE8;
pub type TrimPot1Pin = PE9;

pub type SwdSwclkPin = PA14;
pub type SwdSwdioPin = PA13;

//////////////////////////////
//  Control Communications  //
//////////////////////////////

pub type ComsUartModule = USART1;
pub type ComsUartTxPin = PA9;
pub type ComsUartRxPin = PA10;
pub type ComsUartCtsPin = PA11;
pub type ComsUartRtsPin = PA12;
pub type ComsUartTxDma = DMA1_CH1;
pub type ComsUartRxDma = DMA1_CH2;

///////////////////
//  Peripherals  //
///////////////////

pub type FlashSpi = SPI1;
pub type FlashSpiSckPin = PA5;
pub type FlashSpiMosiPin = PA7;
pub type FlashSpiMisoPin = PA6;
pub type FlashSpiNssPin = PA4;

pub type BallSenseUart = USART2;
pub type BallSenseUartRxPin = PD6;
pub type BallSenseUartTxPin = PD5;
pub type BallSenseUartCtsPin = PD3;
pub type BallSenseUartRtsPin = PD4;
pub type BallSenseBoot0Pin = PD7;
pub type BallSenseRstPin = PB3;
pub type BallSenseUartRxDma = DMA1_CH3;
pub type BallSenseUartTxDma = DMA1_CH4;

pub type DribblerUart = USART3;
pub type DribblerUartRxPin = PE15;
pub type DribblerUartTxPin = PB10;
pub type DribblerUartCtsPin = PB13;
pub type DribblerUartRtsPin = PB14;
pub type DribblerBoot0Pin = PE13;
pub type DribblerRstPin = PE14;
pub type DribblerUartRxDma = DMA1_CH5;
pub type DribblerUartTxDma = DMA1_CH6;

pub type TempProbeReadPin = PC4;
