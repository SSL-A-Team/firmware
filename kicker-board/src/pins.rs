use embassy_stm32::peripherals::*;

////////////////////
//  HV Functions  //
////////////////////

pub type ChargePin = PE4;
pub type KickPin = PE5;
pub type ChipPin = PE6;


////////////////////////////
//  Voltage Measurements  //
////////////////////////////

pub type PowerRailAdc = ADC1;
pub type PowerRail200vReadPin = PC0;
pub type PowerRailVBattReadPin = PA6;
pub type PowerRail12vReadPin = PC1;
pub type PowerRail6v2ReadPin = PC3;
pub type PowerRail5v0ReadPin = PC2;
pub type PowerRail3v3ReadPin = PA4;
pub type PowerRailAdcDma = DMA2_CH4;


//////////////////////
//  Breakbeam Pins  //
//////////////////////

pub type BreakbeamLeftI2c = I2C3;
pub type BreakbeamLeftAgpioPin = PA1;
pub type BreakbeamLeftSdaPin = PC9;
pub type BreakbeamLeftSclPin = PA8;
pub type BreakbeamLeftNdetPin = PC6;
pub type BreakbeamLeftRstPin = PC7;
pub type BreakbeamLeftIntPin = PC8;
pub type BreakbeamLeftI2cRxDma = DMA1_CH2;
pub type BreakbeamLeftI2cTxDma = DMA1_CH4;

pub type BreakbeamCenterI2c = I2C2;
pub type BreakbeamCenterAgpioPin = PC5;
pub type BreakbeamCenterSdaPin = PB11;
pub type BreakbeamCenterSclPin = PB10;
pub type BreakbeamCenterNdetPin = PE13;
pub type BreakbeamCenterRstPin = PE14;
pub type BreakbeamCenterIntPin = PE15;

pub type BreakbeamRightI2c = I2C1;
pub type BreakbeamRightAgpioPin = PA0;
pub type BreakbeamRightSdaPin = PB7;
pub type BreakbeamRightSclPin = PB6;
pub type BreakbeamRightNdetPin = PB3;
pub type BreakbeamRightRstPin = PB4;
pub type BreakbeamRightIntPin = PB5;
pub type BreakbeamRightI2cRxDma = DMA1_CH0;
pub type BreakbeamRightI2cTxDma = DMA1_CH7;


///////////////
//  User IO  //
///////////////

pub type GreenStatusLedPin = PE0;
pub type RedStatusLedPin = PE1;
pub type BlueStatusLed1Pin = PE2;
pub type BlueStatusLed2Pin = PE3;

pub type DotstarSpi = SPI1;
pub type DotstarSpiSckPin = PA5;
pub type DotstarSpiMosiPin = PA7;
pub type DotstarSpiRxDma = DMA2_CH0;
pub type DotstarSpiTxDma = DMA2_CH3;

pub type UserBtnPin = PD4;
pub type PowerBtnIntPin = PD5;
pub type PowerBtnIntExti = EXTI5;
pub type PowerKillPin = PD6;

pub type Dip0Pin = PE9;
pub type Dip1Pin = PE10;
pub type Dip2Pin = PE11;
pub type Dip3Pin = PE12;

pub type TrimPot0Pin = PB0;
pub type TrimPot1Pin = PB1;

pub type DbgHdr1Pin = PB8;
pub type DbgHdr2Pin = PB9;

pub type SwdSwclkPin = PA14;
pub type SwdSwdioPin = PA13; 


//////////////////////////////
//  Control Communications  //
//////////////////////////////

pub type ComsUartModule = USART1;
pub type ComsUartTxPin = PA9;
pub type ComsUartRxPin = PA10;
pub type ComsUartTxDma = DMA2_CH7;
pub type ComsUartRxDma = DMA2_CH2;


///////////////////
//  Peripherals  //
///////////////////

pub type UsbOtgFsDNPin = PA11;
pub type UsbOtgFsDPPin = PA12;

pub type FlashSpi = SPI2;
pub type FlashSpiSckPin = PB13;
pub type FlashSpiMosiPin = PB15;
pub type FlashSpiMisoPin = PB14;
pub type FlashSpiNssPin = PB12;

pub type AuxSpi = SPI3;
pub type AuxSpiSckPin = PC10;
pub type AuxSpiMosiPin = PC12;
pub type AuxSpiMisoPin = PC11;
pub type AuxSpiNssPin = PA15;

pub type BallSenseUart = USART2;
pub type BallSenseUartRxPin = PA3;
pub type BallSenseUartTxPin = PA2;
pub type BallSenseBoot0Pin = PE7;
pub type BallSenseRstPin = PE8;
pub type BallSenseNdetPin = PB2;
pub type BallSenseUartRxDma = DMA1_CH5;
pub type BallSenseUartTxDma = DMA1_CH6;

pub type DribblerUart = USART3;
pub type DribblerUartRxPin = PD9;
pub type DribblerUartTxPin = PD8;
pub type DribblerBoot0Pin = PD10;
pub type DribblerRstPin = PD11;
pub type DribblerNdetPin = PD12;
pub type DribblerUartRxDma = DMA1_CH1;
pub type DribblerUartTxDma = DMA1_CH3;

pub type TempProbeReadPin = PC4;

