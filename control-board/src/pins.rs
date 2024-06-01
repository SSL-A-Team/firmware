#![allow(dead_code)]

use ateam_common_packets::radio::{DataPacket, TelemetryPacket};
use embassy_stm32::peripherals::*;
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    pubsub::{PubSubChannel, Publisher, Subscriber}};

/////////////////////
//  Pub Sub Types  //
/////////////////////

const COMMANDS_PUBSUB_DEPTH: usize = 4;
const TELEMETRY_PUBSUB_DEPTH: usize = 4;
const GYRO_DATA_PUBSUB_DEPTH: usize = 1;
const ACCEL_DATA_PUBSUB_DEPTH: usize = 1;

pub type CommandsPubSub = PubSubChannel<ThreadModeRawMutex, DataPacket, COMMANDS_PUBSUB_DEPTH, 1, 1>;
pub type CommandsPublisher = Publisher<'static, ThreadModeRawMutex, DataPacket, COMMANDS_PUBSUB_DEPTH, 1, 1>;
pub type CommandsSubscriber = Subscriber<'static, ThreadModeRawMutex, DataPacket, COMMANDS_PUBSUB_DEPTH, 1, 1>;

pub type TelemetryPubSub = PubSubChannel<ThreadModeRawMutex, TelemetryPacket, TELEMETRY_PUBSUB_DEPTH, 1, 1>;
pub type TelemetryPublisher = Publisher<'static, ThreadModeRawMutex, TelemetryPacket, COMMANDS_PUBSUB_DEPTH, 1, 1>;
pub type TelemetrySubcriber = Subscriber<'static, ThreadModeRawMutex, TelemetryPacket, TELEMETRY_PUBSUB_DEPTH, 1, 1>;

pub type GyroDataPubSub = PubSubChannel<ThreadModeRawMutex, nalgebra::Vector3<f32>, GYRO_DATA_PUBSUB_DEPTH, 1, 1>;
pub type GyroDataPublisher = Publisher<'static, ThreadModeRawMutex, nalgebra::Vector3<f32>, GYRO_DATA_PUBSUB_DEPTH, 1, 1>;
pub type GyroDataSubscriber = Subscriber<'static, ThreadModeRawMutex, nalgebra::Vector3<f32>, GYRO_DATA_PUBSUB_DEPTH, 1, 1>;
pub type AccelDataPubSub = PubSubChannel<ThreadModeRawMutex, nalgebra::Vector3<f32>, ACCEL_DATA_PUBSUB_DEPTH, 1, 1>;
pub type AccelDataPublisher = Publisher<'static, ThreadModeRawMutex, nalgebra::Vector3<f32>, ACCEL_DATA_PUBSUB_DEPTH, 1, 1>;
pub type AccelDataSubscriber = Subscriber<'static, ThreadModeRawMutex, nalgebra::Vector3<f32>, ACCEL_DATA_PUBSUB_DEPTH, 1, 1>;

/////////////
//  Radio  //
/////////////

pub type RadioUART = USART10;
pub type RadioUartRxPin = PE2;
pub type RadioUartTxPin = PE3;
pub type RadioUartCtsPin = PG13;
pub type RadioUartRtsPin = PG14;
pub type RadioTxDMA = DMA2_CH0;
pub type RadioRxDMA = DMA2_CH1;
pub type RadioResetPin = PC13;
pub type RadioNDetectPin = PE4;


///////////////
//  Sensors  //
///////////////

pub type ImuSpi = SPI1;
pub type ImuSpiSckPin = PA5;
pub type ImuSpiMosiPin = PA7;
pub type ImuSpiMisoPin = PA6;
pub type ImuSpiNss0Pin = PA4;
pub type ExtImuSpiNss1Pin = PC4;
pub type ExtImuSpiNss2Pin = PC5;
pub type ExtImuSpiNss3Pin = PB0;
pub type ImuSpiRxDma = DMA2_CH6;
pub type ImuSpiTxDma = DMA2_CH7;
pub type ImuSpiInt1Pin = PB1;
pub type ImuSpiInt2Pin = PB2;
pub type ImuSpiInt1Exti = EXTI1;
pub type ImuSpiInt2Exti = EXTI2;
pub type ExtImuNDetPin = PF11;


//////////////
//  Kicker  //
//////////////

pub type KickerUart = USART6;
pub type KickerUartRxPin = PC7;
pub type KickerUartTxPin = PC6;
pub type KickerRxDma = DMA2_CH5;
pub type KickerTxDma = DMA2_CH4;
pub type KickerPowerOnPin = PG8;
pub type KickerBootPin = PA8;
pub type KickerResetPin = PA9;
// pub type KickerNDetPin = ;



//////////////
//  Motors  //
//////////////

pub type MotorFLUart = UART4;
pub type MotorFLUartRxPin = PA1;
pub type MotorFLUartTxPin = PA0;
pub type MotorFLDmaTx = DMA1_CH2;
pub type MotorFLDmaRx = DMA1_CH3;
pub type MotorFLBootPin = PC1;
pub type MotorFLResetPin = PC0;

pub type MotorBLUart = UART7;
pub type MotorBLUartRxPin = PF6;
pub type MotorBLUartTxPin = PF7;
pub type MotorBLDmaTx = DMA1_CH4;
pub type MotorBLDmaRx = DMA1_CH5;
pub type MotorBLBootPin = PF8;
pub type MotorBLResetPin = PF9;

pub type MotorBRUart = UART8;
pub type MotorBRUartRxPin = PE0;
pub type MotorBRUartTxPin = PE1;
pub type MotorBRDmaTx = DMA1_CH6;
pub type MotorBRDmaRx = DMA1_CH7;
pub type MotorBRBootPin = PB9;
pub type MotorBRResetPin = PB8;

pub type MotorFRUart = USART1;
pub type MotorFRUartRxPin = PB15;
pub type MotorFRUartTxPin = PB14;
pub type MotorFRDmaTx = DMA1_CH0;
pub type MotorFRDmaRx = DMA1_CH1;
pub type MotorFRBootPin = PD8;
pub type MotorFRResetPin = PD9;

pub type MotorDUart = UART5;
pub type MotorDUartRxPin = PB12;
pub type MotorDUartTxPin = PB13;
pub type MotorDDmaTx = DMA2_CH2;
pub type MotorDDmaRx = DMA2_CH3;
pub type MotorDBootPin = PD13;
pub type MotorDResetPin = PD12;


/////////////////////
//  Power Control  //
/////////////////////

pub type PowerBtnPressedIntPin = PE11;
pub type PowerBtnPressedIntExti = EXTI11;
pub type PowerKillPin = PE10;
pub type ShutdownInitiatedLedPin = PF4;


///////////////
//  User IO  //
///////////////

pub type UsrBtn0Pin = PD6;
pub type UsrBtn1Pin = PD5;
pub type UsrBtn0Exti = EXTI6;
pub type UsrBtn1Exti = EXTI5;

pub type UsrDip1Pin = PD15;
pub type UsrDip2Pin = PG2;
pub type UsrDip3Pin = PG3;
pub type UsrDip4Pin = PG4;
pub type UsrDip5Pin = PG5;
pub type UsrDip6Pin = PG6;
pub type UsrDip7IsBluePin = PG7;

pub type RobotIdSelector0Pin = PG9;
pub type RobotIdSelector1Pin = PG10;
pub type RobotIdSelector2Pin = PG11;
pub type RobotIdSelector3Pin = PG12;


pub type UsrLed0Pin = PF3;
pub type UsrLed1Pin = PF2;
pub type UsrLed2Pin = PF1;
pub type UsrLed3Pin = PF0;

pub type RobotIdIndicator0FlPin = PD0;
pub type RobotIdIndicator1BlPin = PD1;
pub type RobotIdIndicator2BrPin = PD3;
pub type RobotIdIndicator3FrPin = PD4;
pub type RobotIdIndicator4TeamIsBluePin = PD14;

pub type DotstarSpi = SPI6;
pub type DotstarSpiSck = PB3;
pub type DotstarSpiMosi = PB5;
pub type DotstarTxDma = BDMA_CH0;

pub type BuzzerPin = PE6;
pub type BuzzerTimer = TIM15; // ch2 

