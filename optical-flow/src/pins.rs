use embassy_stm32::peripherals::*;

pub type OpticalFlow1Spi = SPI1;
pub type OpticalFlow1Nss = PA4;
pub type OpticalFlow1Sck = PA5;
pub type OpticalFlow1Miso = PA6;
pub type OpticalFlow1Mosi = PA7;
pub type OpticalFlow1Int = PC4;
pub type OpticalFlow1TxDMA = DMA1_CH1;
pub type OpticalFlow1RxDMA = DMA1_CH2;

pub type OpticalFlow2Spi = SPI2;
pub type OpticalFlow2Nss = PA8;
pub type OpticalFlow2Sck = PB13;
pub type OpticalFlow2Miso = PB14;
pub type OpticalFlow2Mosi = PB15;
pub type OpticalFlow2Int = EXTI9;
pub type OpticalFlow2IntPin = PA9;
pub type OpticalFlow2TxDMA = DMA1_CH3;
pub type OpticalFlow2RxDMA = DMA1_CH4;
