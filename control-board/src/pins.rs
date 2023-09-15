use apa102_spi::Apa102;
use embassy_stm32::{
    bind_interrupts,
    dma::NoDma,
    gpio::{Level, Output, Speed},
    peripherals::{self, *},
    spi::{self, Spi},
    time::hz,
    usart::{self, Uart},
    Peripherals,
};
use smart_leds::{SmartLedsWrite, RGB8};

use crate::{
    drivers::{rotary::Rotary, shell_indicator::ShellIndicator},
    stm32_interface::get_bootloader_uart_config,
};

pub type RadioUART = USART10;

pub type MotorFRUart = USART1;
pub type MotorFLUart = UART4;
pub type MotorBLUart = UART7;
pub type MotorBRUart = UART8;
pub type MotorDUart = UART5;

pub type RadioTxDMA = DMA2_CH0;
pub type RadioRxDMA = DMA2_CH1;
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

pub type MotorFRBoot = PD8;
pub type MotorFRReset = PD9;

pub type MotorFLBoot = PC1;
pub type MotorFLReset = PC0;

pub type MotorBLBoot = PF8;
pub type MotorBLReset = PF9;

pub type MotorBRBoot = PB9;
pub type MotorBRReset = PB8;

pub type MotorDBoot = PD13;
pub type MotorDReset = PD12;

pub type RadioReset = PC13;

pub type ImuSpi = SPI6;
pub type ImuDmaTx = BDMA_CH0;
pub type ImuDmaRx = BDMA_CH1;
pub type ImuCsAccel = PC4;
pub type ImuCsGyro = PC5;

pub type Rotary0 = PG9;
pub type Rotary1 = PG10;
pub type Rotary2 = PG11;
pub type Rotary3 = PG12;

pub type DotstarSpi = SPI3;

pub type Dip1 = PG7;
pub type Dip2 = PG6;
pub type Dip3 = PG5;
pub type Dip4 = PG4;
pub type Dip5 = PG3;
pub type Dip6 = PG2;
pub type Dip7 = PG15;

pub type KickerDet = PG8;
pub type KickerBoot0 = PA8;
pub type KickerReset = PA9;

pub type ShellIndicator0 = PD0;
pub type ShellIndicator1 = PD1;
pub type ShellIndicator2 = PD3;
pub type ShellIndicator3 = PD4;

pub struct RobotPeripherals<'a> {
    pub radio_usart: Uart<'a, RadioUART, RadioTxDMA, RadioRxDMA>,
    pub radio_reset: Output<'a, RadioReset>,

    pub imu_spi: Spi<'a, ImuSpi, NoDma, NoDma>,
    pub imu_cs_accel: Output<'a, ImuCsAccel>,
    pub imu_cs_gyro: Output<'a, ImuCsGyro>,

    pub motor_fr_usart: Uart<'a, MotorFRUart, MotorFRDmaTx, MotorFRDmaRx>,
    pub motor_fr_reset: Output<'a, MotorFRReset>,
    pub motor_fr_boot0: Output<'a, MotorFRBoot>,

    pub motor_fl_usart: Uart<'a, MotorFLUart, MotorFLDmaTx, MotorFLDmaRx>,
    pub motor_fl_reset: Output<'a, MotorFLReset>,
    pub motor_fl_boot0: Output<'a, MotorFLBoot>,

    pub motor_bl_usart: Uart<'a, MotorBLUart, MotorBLDmaTx, MotorBLDmaRx>,
    pub motor_bl_reset: Output<'a, MotorBLReset>,
    pub motor_bl_boot0: Output<'a, MotorBLBoot>,

    pub motor_br_usart: Uart<'a, MotorBRUart, MotorBRDmaTx, MotorBRDmaRx>,
    pub motor_br_reset: Output<'a, MotorBRReset>,
    pub motor_br_boot0: Output<'a, MotorBRBoot>,

    pub motor_d_usart: Uart<'a, MotorDUart, MotorDDmaTx, MotorDDmaRx>,
    pub motor_d_reset: Output<'a, MotorDReset>,
    pub motor_d_boot0: Output<'a, MotorDBoot>,

    pub rotary: Rotary<'a, Rotary0, Rotary1, Rotary2, Rotary3>,
    pub dotstar: Apa102<Spi<'a, DotstarSpi, NoDma, NoDma>>,
    pub shell_indicator: ShellIndicator<'a, PD0, PD1, PD3, PD4>,
}

// struct Test<T: usart::BasicInstance, INT: embassy_stm32::interrupt::typelevel::Handler<T::Interrupt>> {
//     _phantom: core::marker::PhantomData<(T, INT)>,
// }

// impl<T: usart::BasicInstance, INT: embassy_stm32::interrupt::typelevel::Handler<T::Interrupt>> embassy_stm32::interrupt::typelevel::Handler<T::Interrupt> for Test<T, INT> {
//     unsafe fn on_interrupt() {
//         INT::on_interrupt()
//     }
// }

struct Irqs2;

pub static mut PIN_TOGGLE: Option<Output<'static, PF3>> = None;

#[allow(non_snake_case)]
#[no_mangle]
unsafe extern "C" fn USART10() {
    if let Some(pin) = &mut PIN_TOGGLE {
        pin.toggle();
    }
    <usart::InterruptHandler<RadioUART> as embassy_stm32::interrupt::typelevel::Handler<
        embassy_stm32::interrupt::typelevel::USART10,
    >>::on_interrupt();
}

unsafe impl
    embassy_stm32::interrupt::typelevel::Binding<
        embassy_stm32::interrupt::typelevel::USART10,
        usart::InterruptHandler<RadioUART>,
    > for Irqs2
{
}

bind_interrupts!(struct Irqs {
    // USART10 => usart::InterruptHandler<RadioUART>;
    // USART10 => Test<RadioUART, usart::InterruptHandler<RadioUART>>;
    USART1 => usart::InterruptHandler<MotorFRUart>;
    UART4 => usart::InterruptHandler<MotorFLUart>;
    UART7 => usart::InterruptHandler<MotorBLUart>;
    UART8 => usart::InterruptHandler<MotorBRUart>;
    UART5 => usart::InterruptHandler<MotorDUart>;
});

impl<'a> RobotPeripherals<'a> {
    pub fn new(p: Peripherals) -> Self {
        unsafe {
            PIN_TOGGLE = Some(Output::new(p.PF3, Level::Low, Speed::Medium));
        }

        let radio_usart = Uart::new(
            p.USART10,
            p.PE2,
            p.PE3,
            Irqs2,
            p.DMA2_CH0,
            p.DMA2_CH1,
            usart::Config::default(),
        );
        let radio_reset = Output::new(p.PC13, Level::Low, Speed::Medium);

        let motor_fr_usart = Uart::new(
            p.USART1,
            p.PB15,
            p.PB14,
            Irqs,
            p.DMA1_CH0,
            p.DMA1_CH1,
            get_bootloader_uart_config(),
        );
        let motor_fl_usart = Uart::new(
            p.UART4,
            p.PA1,
            p.PA0,
            Irqs,
            p.DMA1_CH2,
            p.DMA1_CH3,
            get_bootloader_uart_config(),
        );
        let motor_bl_usart = Uart::new(
            p.UART7,
            p.PF6,
            p.PF7,
            Irqs,
            p.DMA1_CH4,
            p.DMA1_CH5,
            get_bootloader_uart_config(),
        );
        let motor_br_usart = Uart::new(
            p.UART8,
            p.PE0,
            p.PE1,
            Irqs,
            p.DMA1_CH6,
            p.DMA1_CH7,
            get_bootloader_uart_config(),
        );
        let motor_d_usart = Uart::new(
            p.UART5,
            p.PB12,
            p.PB13,
            Irqs,
            p.DMA2_CH2,
            p.DMA2_CH3,
            get_bootloader_uart_config(),
        );

        let motor_fr_boot0 = Output::new(p.PD8, Level::Low, Speed::Medium);
        let motor_fl_boot0 = Output::new(p.PC1, Level::Low, Speed::Medium);
        let motor_bl_boot0 = Output::new(p.PF8, Level::Low, Speed::Medium);
        let motor_br_boot0 = Output::new(p.PB9, Level::Low, Speed::Medium);
        let motor_d_boot0 = Output::new(p.PD13, Level::Low, Speed::Medium);

        let motor_fr_reset = Output::new(p.PD9, Level::Low, Speed::Medium);
        let motor_fl_reset = Output::new(p.PC0, Level::Low, Speed::Medium);
        let motor_bl_reset = Output::new(p.PF9, Level::Low, Speed::Medium);
        let motor_br_reset = Output::new(p.PB8, Level::Low, Speed::Medium);
        let motor_d_reset = Output::new(p.PD12, Level::Low, Speed::Medium);

        let imu_spi = Spi::new(
            p.SPI6,
            p.PA5,
            p.PA7,
            p.PA6,
            NoDma,
            NoDma,
            hz(1_000_000),
            spi::Config::default(),
        );
        let imu_cs_accel = Output::new(p.PC4, Level::High, Speed::VeryHigh);
        let imu_cs_gyro = Output::new(p.PC5, Level::High, Speed::VeryHigh);

        let rotary = Rotary::new(p.PG9, p.PG10, p.PG11, p.PG12);

        let dotstar_spi = spi::Spi::new_txonly(
            p.SPI3,
            p.PB3,
            p.PB5,
            NoDma,
            NoDma,
            hz(1_000_000),
            spi::Config::default(),
        );
        let dotstar = Apa102::new(dotstar_spi);

        let shell_indicator = ShellIndicator::new(p.PD0, p.PD1, p.PD3, p.PD4);

        Self {
            radio_usart,
            radio_reset,
            imu_spi,
            imu_cs_accel,
            imu_cs_gyro,
            motor_fr_usart,
            motor_fl_usart,
            motor_bl_usart,
            motor_br_usart,
            motor_d_usart,
            motor_fr_boot0,
            motor_fl_boot0,
            motor_bl_boot0,
            motor_br_boot0,
            motor_d_boot0,
            motor_fr_reset,
            motor_fl_reset,
            motor_bl_reset,
            motor_br_reset,
            motor_d_reset,
            rotary,
            dotstar,
            shell_indicator,
        }
    }
}

// TODO: move this somewhere better?
pub trait DotstarRobotWrite {
    fn write(&mut self, color1: RGB8, color2: RGB8) -> Result<(), ()>;
}

impl<T: SmartLedsWrite> DotstarRobotWrite for T
where
    <T as SmartLedsWrite>::Color: From<RGB8>,
{
    fn write(&mut self, color1: RGB8, color2: RGB8) -> Result<(), ()> {
        let color1 = RGB8 {
            r: color1.r / 10,
            g: color1.g / 10,
            b: color1.b / 10,
        };
        let color2 = RGB8 {
            r: color2.r / 10,
            g: color2.g / 10,
            b: color2.b / 10,
        };
        <Self as SmartLedsWrite>::write(self, [color1, color2].iter().cloned()).or(Err(()))
    }
}
