use embassy_stm32::{peripherals::{
    DMA1_CH0, DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, 
    DMA2_CH0, DMA2_CH1, DMA2_CH2, DMA2_CH3, 
    PA3, PA4,
    PB1, PB2,
    PC0, PC2,
    PD7, PD14, PD15,
    PF4,
    PG0, PG1, PG2, PG3,
    UART4, UART5, UART7,
    USART2, USART3, USART6,
    EXTI15,
}, interrupt};

pub type RadioUART = USART2;
pub type RadioTxDMA = DMA2_CH0;
pub type RadioRxDMA = DMA2_CH1;
pub type RadioReset = PC0;

pub type MotorFRUart = UART5;
pub type MotorFLUart = UART7;
pub type MotorBLUart = UART4;
pub type MotorBRUart = USART3;
pub type MotorDUart = USART6;

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

pub type MotorFRBootPin = PB1;
pub type MotorFLBootPin = PG2;
pub type MotorBLBootPin = PG0;
pub type MotorBRBootPin = PF4;
pub type MotorDBootPin = PC2;

pub type MotorFRResetPin = PB2;
pub type MotorFLResetPin = PG3;
pub type MotorBLResetPin = PG1;
pub type MotorBRResetPin = PA3;
pub type MotorDResetPin = PD7;


pub type PowerStatePin = PD15;
pub type PowerStateExti = EXTI15;
pub type ShutdownCompletePin = PD14;