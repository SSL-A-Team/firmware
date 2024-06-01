
use core::ops::Range;

use embassy_stm32::{mode::Async, spi::{self, Config, MosiPin, SckPin, Spi}, time::mhz, Peripheral};
use embassy_time::{Duration, Instant};
use smart_leds::RGB8;

use crate::anim::{AnimInterface, Blink, CompositeAnimation, Lerp};

pub struct Apa102<'a, 'buf, SpiPeri: spi::Instance, const NUM_LEDS: usize>
where [(); (NUM_LEDS * 4) + 8]: {
    spi: spi::Spi<'a, SpiPeri, Async>,
    spi_buf: &'buf mut [u8; (NUM_LEDS * 4) + 8],
    animation_buf: [Option<CompositeAnimation<u8, RGB8, 5>>; NUM_LEDS],
}

impl<'a, 'buf, SpiPeri: spi::Instance, const NUM_LEDS: usize> Apa102<'a, 'buf, SpiPeri, NUM_LEDS> 
where [(); (NUM_LEDS * 4) + 8]: {
    pub fn new(
        spi: spi::Spi<'a, SpiPeri, Async>, 
        spi_buf: &'buf mut [u8; (NUM_LEDS * 4) + 8],
    ) -> Self {
        // set start frame
        spi_buf[0] = 0x00;
        spi_buf[1] = 0x00;
        spi_buf[2] = 0x00;
        spi_buf[3] = 0x00;

        // set end frame
        spi_buf[8 + (NUM_LEDS * 4) - 4] = 0xFF;
        spi_buf[8 + (NUM_LEDS * 4) - 3] = 0xFF;
        spi_buf[8 + (NUM_LEDS * 4) - 2] = 0xFF;
        spi_buf[8 + (NUM_LEDS * 4) - 1] = 0xFF;

        Apa102 { 
            spi: spi,
            spi_buf: spi_buf,
            animation_buf: [None; NUM_LEDS],
        }
    }

    pub fn new_from_pins(
        peri: impl Peripheral<P = SpiPeri> + 'a,
        sck_pin: impl Peripheral<P = impl SckPin<SpiPeri>> + 'a,
        mosi_pin: impl Peripheral<P = impl MosiPin<SpiPeri>> + 'a,
        tx_dma: impl Peripheral<P = impl spi::TxDma<SpiPeri>> + 'a,
        spi_buf: &'buf mut [u8; (NUM_LEDS * 4) + 8],
    ) -> Self {
        let mut dotstar_spi_config = Config::default();
        dotstar_spi_config.frequency = mhz(1);

        let spi = Spi::new_txonly(
            peri,
            sck_pin,
            mosi_pin,
            tx_dma,
            dotstar_spi_config,
        );

        Self::new(spi, spi_buf)
    }

    const fn l2d(led_index: usize) -> usize {
        4 + (led_index * 4) + 0
    }

    const fn l2r(led_index: usize) -> usize {
        4 + (led_index * 4) + 3
    }

    const fn l2g(led_index: usize) -> usize {
        4 + (led_index * 4) + 2
    }

    const fn l2b(led_index: usize) -> usize {
        4 + (led_index * 4) + 1
    }

    pub fn set_drv_str(&mut self, str: u8, led_index: usize) {
        assert!(led_index < NUM_LEDS);

        let str = ((str >> 3) & 0x1F) | 0xE0;
        self.spi_buf[Self::l2d(led_index)] = str;
    }

    pub fn set_drv_str_range(&mut self, str: u8, led_index_range: Range<usize>) {
        for i in led_index_range {
            self.set_drv_str(str, i)
        }
    }

    pub fn set_drv_str_all(&mut self, str: u8) {
        self.set_drv_str_range(str, 0..NUM_LEDS)
    }

    pub fn set_color(&mut self, color: RGB8, led_index: usize) {
        assert!(led_index < NUM_LEDS);

        self.spi_buf[Self::l2r(led_index)] = color.r;
        self.spi_buf[Self::l2g(led_index)] = color.g;
        self.spi_buf[Self::l2b(led_index)] = color.b;
    }

    pub fn set_color_range(&mut self, color: RGB8, led_index_range: Range<usize>) {
        for i in led_index_range {
            self.set_color(color, i)
        }
    }

    pub fn set_color_all(&mut self, color: RGB8) {
        self.set_color_range(color, 0..NUM_LEDS)
    }

    pub fn set_animation(&mut self, anim: Lerp<u8, RGB8>, led_index: usize) {
        assert!(led_index < NUM_LEDS);
        self.animation_buf[led_index] = Some(anim);
    }

    pub fn clear_animation(&mut self, led_index: usize) {
        assert!(led_index < NUM_LEDS);
        self.animation_buf[led_index] = None;
    }

    pub async fn update(&mut self) {
        // update animations
        for anim in self.animation_buf.iter_mut() {
            if let Some(anim) = anim {
                anim.update();
            }
        }

        // set colors from animations
        for (i, anim) in self.animation_buf.iter().enumerate() {
            if let Some(anim) = anim {
                self.set_color(anim.get_value(), i);
            }
        }

        let _ = self.spi.write(self.spi_buf).await;
    }
}

