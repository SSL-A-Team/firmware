
use core::ops::Range;

use embassy_stm32::{mode::Async, spi::{self, Config, MosiPin, SckPin, Spi}, time::mhz, Peripheral};
use smart_leds::RGB8;

use crate::anim::{AnimInterface, CompositeAnimation};

const START_FRAME_SIZE: usize = 4;
const COLOR_FRAME_SIZE: usize = 4;
const END_FRAME_SIZE: usize = 4;
const HEADER_FRAME_SIZE: usize = START_FRAME_SIZE + END_FRAME_SIZE;


pub struct Apa102<'a, 'buf, const NUM_LEDS: usize>
where [(); (NUM_LEDS * COLOR_FRAME_SIZE) + HEADER_FRAME_SIZE]: {
    spi: spi::Spi<'a, Async>,
    spi_buf: &'buf mut [u8; (NUM_LEDS * COLOR_FRAME_SIZE) + HEADER_FRAME_SIZE],
}

impl<'a, 'buf, const NUM_LEDS: usize> Apa102<'a, 'buf, NUM_LEDS> 
where [(); (NUM_LEDS * COLOR_FRAME_SIZE) + HEADER_FRAME_SIZE]: {
    pub fn new(
        spi: spi::Spi<'a, Async>, 
        spi_buf: &'buf mut [u8; (NUM_LEDS * COLOR_FRAME_SIZE) + HEADER_FRAME_SIZE],
    ) -> Self {
        // set start frame
        spi_buf[0] = 0x00;
        spi_buf[1] = 0x00;
        spi_buf[2] = 0x00;
        spi_buf[3] = 0x00;

        // set end frame
        spi_buf[HEADER_FRAME_SIZE + (NUM_LEDS * COLOR_FRAME_SIZE) - 4] = 0xFF;
        spi_buf[HEADER_FRAME_SIZE + (NUM_LEDS * COLOR_FRAME_SIZE) - 3] = 0xFF;
        spi_buf[HEADER_FRAME_SIZE + (NUM_LEDS * COLOR_FRAME_SIZE) - 2] = 0xFF;
        spi_buf[HEADER_FRAME_SIZE + (NUM_LEDS * COLOR_FRAME_SIZE) - 1] = 0xFF;

        Apa102 { 
            spi: spi,
            spi_buf: spi_buf,
        }
    }

    pub fn new_from_pins<SpiPeri: spi::Instance>(
        peri: impl Peripheral<P = SpiPeri> + 'a,
        sck_pin: impl Peripheral<P = impl SckPin<SpiPeri>> + 'a,
        mosi_pin: impl Peripheral<P = impl MosiPin<SpiPeri>> + 'a,
        tx_dma: impl Peripheral<P = impl spi::TxDma<SpiPeri>> + 'a,
        spi_buf: &'buf mut [u8; (NUM_LEDS * COLOR_FRAME_SIZE) + HEADER_FRAME_SIZE],
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
        START_FRAME_SIZE + (led_index * COLOR_FRAME_SIZE) + 0
    }

    // Calculates the frame index for red based on LED index.
    const fn l2r(led_index: usize) -> usize {
        START_FRAME_SIZE + (led_index * COLOR_FRAME_SIZE) + 3
    }

    // Calculates the frame index for green based on LED index.
    const fn l2g(led_index: usize) -> usize {
        START_FRAME_SIZE + (led_index * COLOR_FRAME_SIZE) + 2
    }

    // Calculates the frame index for blue based on LED index.
    const fn l2b(led_index: usize) -> usize {
        START_FRAME_SIZE + (led_index * COLOR_FRAME_SIZE) + 1
    }

    pub fn set_drv_str(&mut self, strength: u8, led_index: usize) {
        assert!(led_index < NUM_LEDS);
        // The top 3 bits always high (0xE0)
        // Bottom 5 bits are the intensity.
        let strength_shift = ((strength >> 3) & 0x1F) | 0xE0;
        self.spi_buf[Self::l2d(led_index)] = strength_shift;
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

    pub async fn update(&mut self) {
        let _ = self.spi.write(self.spi_buf).await;
    }
}

pub struct Apa102Anim<'a, 'buf, 'ca, const NUM_LEDS: usize> 
where [(); (NUM_LEDS * COLOR_FRAME_SIZE) + HEADER_FRAME_SIZE]: {
    apa102_driver: Apa102<'a, 'buf, NUM_LEDS>,
    active_animation: [usize; NUM_LEDS],
    animation_playbook_buf: [Option<&'ca mut [CompositeAnimation<'ca, u8, RGB8>]>; NUM_LEDS],
    // animation_buf: [Option<&'ca mut [(u8, bool, &'ca mut CompositeAnimation<'ca, u8, RGB8>)]>; NUM_LEDS]
    // animation_buf: [Option<&'ca mut CompositeAnimation<'ca, u8, RGB8>>; NUM_LEDS],
}

impl<'a, 'buf, 'ca, const NUM_LEDS: usize> Apa102Anim<'a, 'buf, 'ca, NUM_LEDS> 
where [(); (NUM_LEDS * COLOR_FRAME_SIZE) + HEADER_FRAME_SIZE]: {
    pub fn new(apa102: Apa102<'a, 'buf, NUM_LEDS>, anim_playbook_buf: [Option<&'ca mut [CompositeAnimation<'ca, u8, RGB8>]>; NUM_LEDS]) -> Self {
        Apa102Anim {
            apa102_driver: apa102,
            active_animation: [0; NUM_LEDS],
            animation_playbook_buf: anim_playbook_buf,
        }
    }

    pub fn next_valid_anim(search_start_ind: usize, anim_playbook: &[CompositeAnimation<'ca, u8, RGB8>]) -> usize {
        if anim_playbook.len() == 1 {
            return 0;
        }
        
        let mut cur_index = search_start_ind;
        loop {
            // inc and loop the index
            cur_index = cur_index + 1 % anim_playbook.len();

            // we incremented and found an enabled animation
            // return the index
            if anim_playbook[cur_index].enabled() {
                return cur_index;
            }

            // we've searched everything and made it all the way around
            // return the search start index
            if cur_index == search_start_ind {
                return search_start_ind;
            }
        }
    }

    pub async fn update(&mut self) {
        // update animations
        for (led_index, anim_arr_opt) in self.animation_playbook_buf.iter_mut().enumerate() {
            let mut led_color = RGB8 { r: 0, g: 0, b: 0 };

            // see if a playbook is loaded
            if let Some(anim_playbook) = anim_arr_opt {
                // check if any animation are enabled
                if anim_playbook.iter().any(|comp_anim| comp_anim.enabled()) {
                    // confirm the current animation is enabled
                    let mut cur_active_anim = self.active_animation[led_index];
                    if !anim_playbook[cur_active_anim].enabled() {
                        // select and start the next animation
                        let next_valid_anim = Self::next_valid_anim(cur_active_anim, anim_playbook);
                        self.active_animation[led_index] = next_valid_anim;
                        anim_playbook[next_valid_anim].start_animation();

                        cur_active_anim = next_valid_anim;
                    };

                    // update the selected animation
                    anim_playbook[cur_active_anim].update();
                    // check if animation is completed
                    if anim_playbook[cur_active_anim].animation_completed() {
                        // reset the animation
                        anim_playbook[cur_active_anim].reset_animation();

                        // select and start the next animation
                        let next_active_anim = Self::next_valid_anim(cur_active_anim, anim_playbook);
                        anim_playbook[next_active_anim].start_animation();
                        self.active_animation[led_index] = next_active_anim;

                        cur_active_anim = next_active_anim;
                    }

                    // update led color
                    led_color = anim_playbook[cur_active_anim].get_value();
                }
            }

            // set the color
            self.apa102_driver.set_color(led_color, led_index);
        }

        self.apa102_driver.update().await;
    }

    fn set_animation_enabled(&mut self, led_index: usize, anim_id: usize, enable: bool) -> Result<(), ()> {
        if led_index >= NUM_LEDS {
            return Err(())
        }

        if let Some(animation_playbook) = self.animation_playbook_buf[led_index].as_deref_mut() {
            for composite_animation in animation_playbook.iter_mut() {
                //defmt::info!("evaluating animation {}", composite_animation.get_id());

                if composite_animation.get_id() == anim_id {
                    if enable {
                        //defmt::info!("enabling animation");
                        composite_animation.enable();
                        composite_animation.start_animation();
                    } else {
                        composite_animation.reset_animation();
                        composite_animation.disable();
                    }
                }
            }
        }

        Ok(())
    }

    pub fn enable_animation(&mut self, led_index: usize, anim_id: usize) -> Result<(), ()>  {
        self.set_animation_enabled(led_index, anim_id, true)
    }

    pub fn disable_animation(&mut self, led_index: usize, anim_id: usize) -> Result<(), ()>  {
        self.set_animation_enabled(led_index, anim_id, false)
    }
}

