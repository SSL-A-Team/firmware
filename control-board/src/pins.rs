#![allow(dead_code)]

use ateam_common_packets::radio::{DataPacket, TelemetryPacket};
use embassy_stm32::{dma::NoDma, peripherals::*};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    pubsub::{PubSubChannel, Publisher, Subscriber}};

use crate::tasks::dotstar_task::ControlBoardLedCommand;

/////////////////////
//  Pub Sub Types  //
/////////////////////

const COMMANDS_PUBSUB_DEPTH: usize = 4;
const TELEMETRY_PUBSUB_DEPTH: usize = 4;
const GYRO_DATA_PUBSUB_DEPTH: usize = 1;
const ACCEL_DATA_PUBSUB_DEPTH: usize = 1;
const BATTERY_VOLT_PUBSUB_DEPTH: usize = 1;

pub type CommandsPubSub = PubSubChannel<ThreadModeRawMutex, DataPacket, COMMANDS_PUBSUB_DEPTH, 2, 1>;
pub type CommandsPublisher = Publisher<'static, ThreadModeRawMutex, DataPacket, COMMANDS_PUBSUB_DEPTH, 2, 1>;
pub type CommandsSubscriber = Subscriber<'static, ThreadModeRawMutex, DataPacket, COMMANDS_PUBSUB_DEPTH, 2, 1>;

pub type TelemetryPubSub = PubSubChannel<ThreadModeRawMutex, TelemetryPacket, TELEMETRY_PUBSUB_DEPTH, 1, 1>;
pub type TelemetryPublisher = Publisher<'static, ThreadModeRawMutex, TelemetryPacket, COMMANDS_PUBSUB_DEPTH, 1, 1>;
pub type TelemetrySubcriber = Subscriber<'static, ThreadModeRawMutex, TelemetryPacket, TELEMETRY_PUBSUB_DEPTH, 1, 1>;

pub type GyroDataPubSub = PubSubChannel<ThreadModeRawMutex, nalgebra::Vector3<f32>, GYRO_DATA_PUBSUB_DEPTH, 1, 1>;
pub type GyroDataPublisher = Publisher<'static, ThreadModeRawMutex, nalgebra::Vector3<f32>, GYRO_DATA_PUBSUB_DEPTH, 1, 1>;
pub type GyroDataSubscriber = Subscriber<'static, ThreadModeRawMutex, nalgebra::Vector3<f32>, GYRO_DATA_PUBSUB_DEPTH, 1, 1>;
pub type AccelDataPubSub = PubSubChannel<ThreadModeRawMutex, nalgebra::Vector3<f32>, ACCEL_DATA_PUBSUB_DEPTH, 1, 1>;
pub type AccelDataPublisher = Publisher<'static, ThreadModeRawMutex, nalgebra::Vector3<f32>, ACCEL_DATA_PUBSUB_DEPTH, 1, 1>;
pub type AccelDataSubscriber = Subscriber<'static, ThreadModeRawMutex, nalgebra::Vector3<f32>, ACCEL_DATA_PUBSUB_DEPTH, 1, 1>;

pub type BatteryVoltPubSub = PubSubChannel<ThreadModeRawMutex, f32, BATTERY_VOLT_PUBSUB_DEPTH, 1, 1>;
pub type BatteryVoltPublisher = Publisher<'static, ThreadModeRawMutex, f32, BATTERY_VOLT_PUBSUB_DEPTH, 1, 1>;
pub type BatteryVoltSubscriber = Subscriber<'static, ThreadModeRawMutex, f32, BATTERY_VOLT_PUBSUB_DEPTH, 1, 1>;

const LED_COMMAND_PUBSUB_DEPTH: usize = 5;
const LED_COMMAND_PUBSUB_NUM_PUBS: usize = 2;
const LED_COMMAND_PUBSUB_NUM_SUBS: usize = 1;
pub type LedCommandPubSub = PubSubChannel<ThreadModeRawMutex, ControlBoardLedCommand, LED_COMMAND_PUBSUB_DEPTH, LED_COMMAND_PUBSUB_NUM_SUBS, LED_COMMAND_PUBSUB_NUM_PUBS>;
pub type LedCommandPublisher = Publisher<'static, ThreadModeRawMutex, ControlBoardLedCommand, LED_COMMAND_PUBSUB_DEPTH, LED_COMMAND_PUBSUB_NUM_SUBS, LED_COMMAND_PUBSUB_NUM_PUBS>;
pub type LedCommandSubscriber = Subscriber<'static, ThreadModeRawMutex, ControlBoardLedCommand, LED_COMMAND_PUBSUB_DEPTH, LED_COMMAND_PUBSUB_NUM_SUBS, LED_COMMAND_PUBSUB_NUM_PUBS>;

/////////////
//  Radio  //
/////////////

pub type RadioUART = USART2;
pub type RadioUartRxPin = PD6;
pub type RadioUartTxPin = PD5;
pub type RadioUartCtsPin = PD3;
pub type RadioUartRtsPin = PD4;
pub type RadioTxDMA = DMA2_CH0;
pub type RadioRxDMA = DMA2_CH1;
pub type RadioResetPin = PD7;
pub type RadioBoot0Pin = PG9;
pub type RadioNDetectPin = PA15;


//////////////
//  Motors  //
//////////////

pub type MotorFLUart = UART7;
pub type MotorFLUartRxPin = PF6;
pub type MotorFLUartTxPin = PF7;
pub type MotorFLUartCtsPin = PF9;
pub type MotorFLUartRtsPin = PF8;
pub type MotorFLDmaTx = DMA1_CH0;
pub type MotorFLDmaRx = DMA1_CH1;
pub type MotorFLBootPin = PF5;
pub type MotorFLResetPin = PF4;

pub type MotorBLUart = USART10;
pub type MotorBLUartRxPin = PE2;
pub type MotorBLUartTxPin = PE3;
pub type MotorBLUartCtsPin = PG13;
pub type MotorBLUartRtsPin = PG14;
pub type MotorBLDmaTx = DMA1_CH2;
pub type MotorBLDmaRx = DMA1_CH3;
pub type MotorBLBootPin = PE5;
pub type MotorBLResetPin = PE4;

pub type MotorBRUart = USART6;
pub type MotorBRUartRxPin = PC7;
pub type MotorBRUartTxPin = PC6;
pub type MotorBRUartCtsPin = PG15;
pub type MotorBRUartRtsPin = PG12;
pub type MotorBRDmaTx = DMA1_CH4;
pub type MotorBRDmaRx = DMA1_CH5;
pub type MotorBRBootPin = PG7;
pub type MotorBRResetPin = PG8;

pub type MotorFRUart = USART3;
pub type MotorFRUartRxPin = PD9;
pub type MotorFRUartTxPin = PD8;
pub type MotorFRUartCtsPin = PD11;
pub type MotorFRUartRtsPin = PF12;
pub type MotorFRDmaTx = DMA1_CH6;
pub type MotorFRDmaRx = DMA1_CH7;
pub type MotorFRBootPin = PB12;
pub type MotorFRResetPin = PB13;


//////////////
//  Kicker  //
//////////////

pub type KickerUart = UART8;
pub type KickerUartRxPin = PE0;
pub type KickerUartTxPin = PE1;
pub type KickerUartCtsPin = PD14;
pub type KickerUartRtsPin = PD15;
pub type KickerRxDma = DMA2_CH2;
pub type KickerTxDma = DMA2_CH3;
pub type KickerBootPin = PG2;
pub type KickerResetPin = PG3;


///////////////////
//  Power Board  //
///////////////////

pub type PowerUart = UART9;
pub type PowerUartRxPin = PG0;
pub type PowerUartTxPin = PG1;
// pub type PowerUartCtsPin = None;
// pub type PowerUartRtsPin = None;
pub type PowerRxDma = DMA2_CH4;
pub type PowerTxDma = DMA2_CH5;


////////////////////
//  Optical Flow  //
////////////////////

pub type OpticalFlowUart = UART4;
pub type OpticalFlowUartRxPin = PB8;
pub type OpticalFlowUartTxPin = PB9;
pub type OpticalFlowUartCtsPin = PB15;
pub type OpticalFlowUartRtsPin = PB14;
pub type OpticalFlowDmaRx = NoDma;
pub type OpticalFlowDmaTx = NoDma;
pub type OpticalFlowBootPin = PB7;
pub type OpticalFlowResetPin = PB6;


//////////////////
//  LCD Screen  //
//////////////////

pub type ScreenUart = USART1;
pub type ScreenUartRxPin = PA10;
pub type ScreenUartTxPin = PA9;
pub type ScreenDmaRx = NoDma;
pub type ScreenDmaTx = NoDma;
pub type ScreenResetPin = PA8;


///////////
//  USB  //
///////////

pub type UsbDpPin = PA12;
pub type UsbDmPin = PA11;

/////////////
//  SDMMC  //
/////////////

pub type SdCardSdmmc = SDMMC1;
pub type SdCardD0Pin = PC8;
pub type SdCardD1Pin = PC9;
pub type SdCardD2Pin = PC10;
pub type SdCardD3Pin = PC11;
pub type SdCardCkPin = PC12;
pub type SdCardCmdPIn = PD2;

///////////////
//  Sensors  //
///////////////

pub type ImuSpi = SPI1;
pub type ImuSpiSckPin = PA5;
pub type ImuSpiMosiPin = PA7;
pub type ImuSpiMisoPin = PA6;
pub type ImuSpiNss0Pin = PA4;
pub type ExtImuSpiNss1Pin = PA3;
pub type ExtImuSpiNss2Pin = PC4;
pub type ExtImuSpiNss3Pin = PC5;
pub type ImuSpiRxDma = DMA2_CH6;
pub type ImuSpiTxDma = DMA2_CH7;
pub type ImuSpiInt1Pin = PB0;
pub type ImuSpiInt2Pin = PB1;
pub type ImuSpiInt1Exti = EXTI0;
pub type ImuSpiInt2Exti = EXTI1;
pub type ExtImuNDetPin = PB2;

pub type ShellDetectI2c = I2C2;
pub type ShellDetectSdaPin = PF0;
pub type ShellDetectSclPin = PF1;
pub type ShellDetectDmaRx = NoDma;
pub type ShellDetectDmaTx = NoDma;

pub type BatteryAdcPin = PA0;
pub type VoltageMon5v0Pin = PA1;
pub type VoltageMon3v3Pin = PA2;
pub type BatteryAdc = ADC1;
pub type VrefIntAdc = ADC3;


///////////////
//  User IO  //
///////////////

pub type UsrDipDebugMode = PC13;

pub type UsrDip0Pin = PF11;
pub type UsrDip1Pin = PF12;
pub type UsrDip2Pin = PF13;
pub type UsrDip3Pin = PF14;
pub type UsrDip4Pin = PF15;
pub type UsrDip5Pin = PE7;
pub type UsrDip6Pin = PE8;
pub type UsrDip7Pin = PE9;

pub type UsrDipTeamIsBluePin = PB10;
pub type UsrDipBotIdSrcSelect = PB11;

pub type RobotIdSelector0Pin = PG11;
pub type RobotIdSelector1Pin = PG10;
pub type RobotIdSelector2Pin = PD1;
pub type RobotIdSelector3Pin = PD0;

pub type UsrBtnBackPin = PE10;
pub type UsrBtnEnterPin = PE11;
pub type UsrBtnLeftPin = PE12;
pub type UsrBtnRightPin = PE13;
pub type UsrBtnUpPin = PE14;
pub type UsrBtnDownPin = PE15;
pub type UsrBtnBackExti = EXTI10;
pub type UsrBtnEnterExti = EXTI11;
pub type UsrBtnLeftExti = EXTI12;
pub type UsrBtnRightExti = EXTI13;
pub type UsrBtnUpExti = EXTI14;
pub type UsrBtnDownExti = EXTI15;

pub type UsrLed0Pin = PG6;
pub type UsrLed1Pin = PG5;
pub type UsrLed2Pin = PG4;
pub type UsrLed3Pin = PD13;

pub type RobotIdSrcDisagree = PD10;
pub type RobotIdIndicator0FlPin = PC0;
pub type RobotIdIndicator1BlPin = PC2;
pub type RobotIdIndicator2BrPin = PC3;
pub type RobotIdIndicator3FrPin = PC1;
pub type RobotIdIndicator4TeamIsBluePin = PF10;

pub type DotstarSpi = SPI6;
pub type DotstarSpiSck = PB3;
pub type DotstarSpiMosi = PB5;
pub type DotstarTxDma = BDMA_CH0;

pub type BuzzerPin = PE6;
pub type BuzzerTimer = TIM15; // ch2 
