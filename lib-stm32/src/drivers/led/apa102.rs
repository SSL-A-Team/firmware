
use core::ops::Range;

use embassy_stm32::{mode::Async, spi::{self, Config, MosiPin, SckPin, Spi}, time::mhz, Peripheral};
use embassy_time::{Duration, Instant};
use smart_leds::RGB8;

pub struct Apa102<'a, 'buf, SpiPeri: spi::Instance, const NUM_LEDS: usize>
where [(); (NUM_LEDS * 4) + 8]: {
    spi: spi::Spi<'a, SpiPeri, Async>,
    spi_buf: &'buf mut [u8; (NUM_LEDS * 4) + 8],
    animation_buf: [Option<Apa102Blink>; NUM_LEDS],
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

    pub fn set_animation(&mut self, anim: Apa102Blink, led_index: usize) {
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
        for (i, anim) in self.animation_buf.into_iter().enumerate() {
            if let Some(anim) = anim {
                self.set_color(anim.get_color(), i);
            }
        }

        let _ = self.spi.write(self.spi_buf).await;
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Apa102AnimationRepeat {
    NoRepeat,
    RepeatForever,
    RepeatFixed(usize),
}

pub trait Apa102AnimationTrait: Sized {
    fn start_animation(&mut self);
    fn stop_animation(&mut self);
    fn animation_running(&self) -> bool;
    fn update(&mut self);
    fn get_color(&self) -> RGB8;
}

// #[derive(Clone, Copy, Debug)]
// pub enum Apa102Animation {
//     Blink(Apa102Blink),
// }

// impl Apa102AnimationTrait for Apa102Animation {
//     fn start_animation(&mut self) {
//         match self {
//             Apa102Animation::Blink(anim), Apa102Animation::Blink2(anim) => {

//             },
//         }
//     }

//     fn stop_animation(&mut self) {
//         todo!()
//     }

//     fn animation_running(&self) -> bool {
//         todo!()
//     }

//     fn update(&mut self) -> RGB8 {
//         todo!()
//     }
// }

// pub struct Apa102AnimationSequence<const NUM_ANIM: usize> {
//     animations: [Apa102Animation; NUM_ANIM],
//     repeat_style: Apa102AnimationRepeat,

//     animation_running: bool,
//     repeat_counter: usize,
//     animation_index: usize,
// }

// impl<const NUM_ANIM: usize> Apa102AnimationSequence<NUM_ANIM> {
//     pub fn new(anim_sequence: [Apa102Animation; NUM_ANIM], repeat_style: Apa102AnimationRepeat) -> Self {
//         Apa102AnimationSequence {
//             animations: anim_sequence,
//             repeat_style: repeat_style,

//             animation_running: false,
//             repeat_counter: 0,
//             animation_index: 0,
//         }
//     }
// }

// impl<const NUM_ANIM: usize> Apa102AnimationTrait for Apa102AnimationSequence<NUM_ANIM> {
//     fn start_animation(&mut self) {
//         self.animation_index = 0;
//         self.animation_running = true;

//         if let Apa102AnimationRepeat::RepeatFixed(num) = self.repeat_style {
//             self.repeat_counter = num;
//         }
//     }

//     fn stop_animation(&mut self) {
//         self.animation_running = false;
//     }

//     fn animation_running(&self) -> bool {
//         return self.animation_running;
//     }

//     fn update(&mut self) -> RGB8 {

//     }
// }

#[derive(Clone, Copy, Debug)]
pub struct Apa102Blink {
    color_one: RGB8,
    color_two: RGB8,
    c1_time: u64,
    c2_time: u64,
    repeat_style: Apa102AnimationRepeat,

    animation_running: bool,
    repeat_counter: usize,
    start_time: Instant,
    last_color: RGB8,
}

impl Apa102Blink {
    pub fn new(color_one: RGB8, color_two: RGB8, c1_time: Duration, c2_time: Duration, repeat_style: Apa102AnimationRepeat) -> Self {
        Apa102Blink { 
            color_one: color_one,
            color_two: color_two,
            c1_time: c1_time.as_millis(),
            c2_time: c2_time.as_millis(),
            repeat_style: repeat_style,

            animation_running: false,
            repeat_counter: 0,
            start_time: Instant::now(),
            last_color: color_one,
        }
    }
}

impl Apa102AnimationTrait for Apa102Blink {
    fn start_animation(&mut self) {
        self.start_time = Instant::now();
        self.animation_running = true;

        if let Apa102AnimationRepeat::RepeatFixed(num) = self.repeat_style {
            self.repeat_counter = num;
        }
    }

    fn stop_animation(&mut self) {
        self.animation_running = false;
    }

    fn animation_running(&self) -> bool {
        self.animation_running
    }

    fn update(&mut self) {
        if !self.animation_running {
            return;
        }

        let now = Instant::now();
        let elapsed_time = (now - self.start_time).as_millis();

        if elapsed_time <= self.c1_time {
            self.last_color = self.color_one;
        } else if self.c1_time < elapsed_time && elapsed_time <= self.c1_time + self.c2_time {
            self.last_color = self.color_two;
        } else if elapsed_time > self.c1_time + self.c2_time {
            match self.repeat_style {
                Apa102AnimationRepeat::NoRepeat => {
                    self.animation_running = false;
                },
                Apa102AnimationRepeat::RepeatForever => {
                    self.start_time = now;
                    self.last_color = self.color_one;
                },
                Apa102AnimationRepeat::RepeatFixed(_) => {
                    if self.repeat_counter == 0 {
                        self.animation_running = false;
                    } else {
                        self.repeat_counter = self.repeat_counter - 1;
                        self.start_time = now;
                        self.last_color = self.color_one;
                    }
                },
            }
        }
    }
    
    fn get_color(&self) -> RGB8 {
        self.last_color
    }
}