#![allow(dead_code)]

use ateam_common_packets::radio::{DataPacket, TelemetryPacket};
use embassy_stm32::peripherals::*;
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    pubsub::{Publisher, Subscriber}};

/////////////////////
//  Pub Sub Types  //
/////////////////////

const COMMANDS_PUBSUB_DEPTH: usize = 1;
const TELEMETRY_PUBSUB_DEPTH: usize = 3;
pub type CommandsPublisher = Publisher<'static, ThreadModeRawMutex, DataPacket, COMMANDS_PUBSUB_DEPTH, 1, 1>;
pub type TelemetrySubcriber = Subscriber<'static, ThreadModeRawMutex, TelemetryPacket, TELEMETRY_PUBSUB_DEPTH, 1, 1>;

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


//////////////
//  Kicker  //
//////////////

pub type KickerUart = USART6;
pub type KickerRxDma = DMA2_CH5;
pub type KickerTxDma = DMA2_CH4;


//////////////
//  Motors  //
//////////////

pub type MotorFRUart = USART1;
pub type MotorFRDmaTx = DMA1_CH0;
pub type MotorFRDmaRx = DMA1_CH1;
pub type MotorFRBootPin = PD8;
pub type MotorFRResetPin = PD9;

pub type MotorFLUart = UART4;
pub type MotorFLDmaTx = DMA1_CH2;
pub type MotorFLDmaRx = DMA1_CH3;
pub type MotorFLBootPin = PC1;
pub type MotorFLResetPin = PC0;

pub type MotorBLUart = UART7;
pub type MotorBLDmaTx = DMA1_CH4;
pub type MotorBLDmaRx = DMA1_CH5;
pub type MotorBLBootPin = PF8;
pub type MotorBLResetPin = PF9;

pub type MotorBRUart = UART8;
pub type MotorBRDmaTx = DMA1_CH6;
pub type MotorBRDmaRx = DMA1_CH7;
pub type MotorBRBootPin = PB9;
pub type MotorBRResetPin = PB8;

pub type MotorDUart = UART5;
pub type MotorDDmaTx = DMA2_CH2;
pub type MotorDDmaRx = DMA2_CH3;
pub type MotorDBootPin = PD13;
pub type MotorDResetPin = PD12;


//////////////////////////
//  Power Control Pins  //
//////////////////////////

pub type PowerStatePin = PF5;
pub type PowerStateExti = EXTI5;
pub type ShutdownCompletePin = PF4;


///////////////
//  User IO  //
///////////////

pub type DotstarSpi = SPI3;
pub type DotstarTxDma = DMA2_CH6;