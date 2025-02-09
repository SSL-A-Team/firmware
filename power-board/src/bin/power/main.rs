#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{adc::{Adc, AdcChannel, SampleTime},
    gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed},
    rcc::Hse, timer::{low_level::CountingMode, simple_pwm::SimplePwm}, 
    Config};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use ateam_lib_stm32::drivers::led::apa102::{Apa102, Apa102Anim};
use smart_leds::colors::{BLUE, WHITE, BLACK};

static mut DOTSTAR_SPI_BUFFER_CELL: [u8; 16] = [0; 16];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut led_red = Output::new(p.PD0, Level::High, Speed::Low);
    let mut led_grn = Output::new(p.PD1, Level::High, Speed::Low);

    let mut en_3v3 = Output::new(p.PB7, Level::High, Speed::Low);
    let mut en_5v0 = Output::new(p.PB8, Level::High, Speed::Low);

    let pwr_btn = Input::new(p.PB15, Pull::None);
    let mut shutdown_ind = Output::new(p.PA15, Level::High, Speed::Low);
    let mut kill_sig = OutputOpenDrain::new(p.PA8, Level::High, Speed::Low);

    // let buzzer = SimplePwm::new(p.TIM, ch1, ch2, ch3, ch4, embassy_stm32::time::Hertz(1000), CountingMode::)

    let mut adc = Adc::new(p.ADC1);
    let mut adc_dma = p.DMA1_CH1;
    let mut cell0_adc_pin = p.PA0.degrade_adc();
    let mut cell1_adc_pin = p.PA1.degrade_adc();
    let mut cell2_adc_pin = p.PA2.degrade_adc();
    let mut cell3_adc_pin = p.PA3.degrade_adc();
    let mut cell4_adc_pin = p.PA4.degrade_adc();
    let mut cell5_adc_pin = p.PA5.degrade_adc();
    let mut vrefint_channel = adc.enable_vrefint().degrade_adc();

    // Use the pin constant directly
    // Get the pins from the schematic 
    let dotstar_spi_buf: &'static mut [u8; 16] = unsafe { &mut DOTSTAR_SPI_BUFFER_CELL };
    // Dotstar SPI, SCK, MOSI, and TX_DMA
    let dotstars = Apa102::<2>::new_from_pins(p.SPI1, p.PB3, p.PB5, p.DMA1_CH1, dotstar_spi_buf.into());
    dotstars.set_drv_str_all(32);

    let mut adc_buf: [u16; 7] = [0; 7];

    loop {
        adc.read(
            &mut adc_dma,
            [
                (&mut cell0_adc_pin, SampleTime::CYCLES160_5),
                (&mut cell1_adc_pin, SampleTime::CYCLES160_5),
                (&mut cell2_adc_pin, SampleTime::CYCLES160_5),
                (&mut cell3_adc_pin, SampleTime::CYCLES160_5),
                (&mut cell4_adc_pin, SampleTime::CYCLES160_5),
                (&mut cell5_adc_pin, SampleTime::CYCLES160_5),
                (&mut vrefint_channel, SampleTime::CYCLES160_5),

            ].into_iter(),
            &mut adc_buf
        ).await;

        let vrefint_sample = adc_buf[6];
        let convert_to_millivolts = |sample: u16| {
            // From https://www.st.com/resource/en/datasheet/stm32g031g8.pdf
            // 6.3.3 Embedded internal reference voltage
            const VREFINT_MV: u32 = 1212; // mV
    
            (u32::from(sample) * VREFINT_MV / u32::from(vrefint_sample)) as u16
        };

        info!("cell0: {}, cell1: {}, cell2: {}, cell3: {}, cell4: {}, cell5: {}", 
            convert_to_millivolts(adc_buf[0]),
            convert_to_millivolts(adc_buf[1]),
            convert_to_millivolts(adc_buf[2]),
            convert_to_millivolts(adc_buf[3]),
            convert_to_millivolts(adc_buf[4]),
            convert_to_millivolts(adc_buf[5])
        );

        info!("high");
        led_red.set_high();
        led_grn.set_high();

        info!("white!");
        dotstars.set_color(WHITE, 0);
        dotstars.set_color(WHITE, 1);
        // en_3v3.set_high();
        // en_5v0.set_high();
        Timer::after_millis(1000).await;

        info!("low");
        led_red.set_low();
        led_grn.set_low();

        info!("blue!");
        dotstars.set_color(BLUE, 0);
        dotstars.set_color(BLUE, 1);
        // en_3v3.set_low();
        // en_5v0.set_low();
        Timer::after_millis(1000).await;

        if pwr_btn.is_low() {
            dotstars.set_color(BLACK, 0);
            dotstars.set_color(BLACK, 1);
            shutdown_ind.set_low();
            warn!("power down requested...");
            Timer::after_millis(1000).await;
            warn!("powering down!");
            kill_sig.set_low();
        }
    }
}
