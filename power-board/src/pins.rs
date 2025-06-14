use ateam_common_packets::bindings::BatteryInfo;
use ateam_lib_stm32::audio::AudioCommand;
use embassy_stm32::peripherals::*;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, pubsub::{PubSubChannel, Publisher, Subscriber}};

/////////////////////
//  power control  //
/////////////////////

pub type PowerRegulator12v0EnablePin = PB6;
pub type PowerRegulator5v0EnablePin = PB7;
pub type PowerRegulator3v3EnablePin = PB8;

pub type LoadSwitchPowerGoodPin = PD3;
pub type LoadSwitchPowerBtnInt = PB15;
pub type LoadSwitchPowerKill = PA8;

///////////////////////
//  voltage monitor  //
///////////////////////

pub type BatteryPreLoadSwitchVoltageMonitorPin = PB10;
pub type BatteryVoltageMonitorPin = PB2;

pub type BatteryCell1VoltageMonitorPin = PA0;
pub type BatteryCell2VoltageMonitorPin = PA1;
pub type BatteryCell3VoltageMonitorPin = PA2;
pub type BatteryCell4VoltageMonitorPin = PA3;
pub type BatteryCell5VoltageMonitorPin = PA4;
pub type BatteryCell6VoltageMonitorPin = PA5;

pub type Power12v0VoltageMonitorPin = PB1;
pub type Power5v0VoltageMonitorPin = PA7;
pub type Power3v3VoltageMonitorPin = PA6;

pub type PowerAdc = ADC1;
pub type PowerAdcDma = DMA1_CH1;

///////////////
//  User IO  //
///////////////

pub type UserLedRedTimer = TIM16; // ch1
pub type UserLedRedPin = PD0;
pub type UserLedGreenTimer = TIM17; // ch1
pub type UserLedGreenPin = PD1;
pub type ShutdownAcknowledgeLedPin = PA15;

pub type BuzzerTimer = TIM3; // ch1
pub type BuzzerPwmPin = PC6;

pub type DotstarSpi = SPI1;
pub type DotstarSpiSckPin = PB3;
pub type DotstarSpiMosiPin = PB5;
pub type DotstarSpiDmaTx = DMA1_CH2;

pub type ComsUart = USART1;
pub type ComsUartTxPin = PA9;
pub type ComsUartRxPin = PA10;

pub type ComsI2C = I2C2;
pub type ComsI2cScl = PA11;
pub type ComsI2cSda = PA12;

pub type ComsSpi = SPI2;
pub type ComsSpiSck = PB13;
pub type ComsSpiMosi = PB11;
pub type ComsSpiMiso = PB14;
pub type ComsSpiNss = PB12;
pub type ComsDmaTx = DMA1_CH4;
pub type ComsDmaRx = DMA1_CH5;

//////////////////////////////
//  Communication Channels  //
//////////////////////////////

const AUDIO_CHANNEL_DEPTH: usize = 3;
const AUDIO_CHANNEL_NUM_SUBS: usize = 1;
const AUDIO_CHANNEL_NUM_PUBS: usize = 3;
const TELEMETRY_CHANNEL_DEPTH: usize = 3;
const TELEMETRY_CHANNEL_NUM_SUBS: usize = 1;
const TELEMETRY_CHANNEL_NUM_PUBS: usize = 1;
pub type AudioPubSub = PubSubChannel<ThreadModeRawMutex, AudioCommand, AUDIO_CHANNEL_DEPTH, AUDIO_CHANNEL_NUM_SUBS, AUDIO_CHANNEL_NUM_PUBS>;
pub type AudioPublisher = Publisher<'static, ThreadModeRawMutex, AudioCommand, AUDIO_CHANNEL_DEPTH, AUDIO_CHANNEL_NUM_SUBS, AUDIO_CHANNEL_NUM_PUBS>;
pub type AudioSubscriber = Subscriber<'static, ThreadModeRawMutex, AudioCommand, AUDIO_CHANNEL_DEPTH, AUDIO_CHANNEL_NUM_SUBS, AUDIO_CHANNEL_NUM_PUBS>;
pub type TelemetryPubSub = PubSubChannel<ThreadModeRawMutex, BatteryInfo, TELEMETRY_CHANNEL_DEPTH, TELEMETRY_CHANNEL_NUM_SUBS, TELEMETRY_CHANNEL_NUM_PUBS>;
pub type TelemetryPublisher = Publisher<'static, ThreadModeRawMutex, BatteryInfo, TELEMETRY_CHANNEL_DEPTH, TELEMETRY_CHANNEL_NUM_SUBS, TELEMETRY_CHANNEL_NUM_PUBS>;
pub type TelemetrySubscriber = Subscriber<'static, ThreadModeRawMutex, BatteryInfo, TELEMETRY_CHANNEL_DEPTH, TELEMETRY_CHANNEL_NUM_SUBS, TELEMETRY_CHANNEL_NUM_PUBS>;