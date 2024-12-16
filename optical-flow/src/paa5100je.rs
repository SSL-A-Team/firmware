/*
 * Driver for the PAA5100JE-Q Optical Flow Sensor.
 *
 *
 */

use core::{cmp::min, cmp::max};

use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Level, Output, Pin, Speed, Pull},
    mode::Async,
    pac::exti::Exti,
    spi::{self, MisoPin, MosiPin, SckPin},
    time::hz,
    Peripheral
};
use embassy_futures::select::{select, Either};
use embassy_time::{Timer, Duration, with_timeout};

use defmt::*;

pub const SPI_MIN_BUF_LEN: usize = 8;

/// SPI driver for the Pixart PAA5100JE-Q Optical Flow
pub struct Paa5100je<'a, 'buf> {
    spi: spi::Spi<'a, Async>,
    cs: Output<'a>,
    motion_int: ExtiInput<'a>,
    spi_buf: &'buf mut [u8; SPI_MIN_BUF_LEN],
    rotation: Rotation,
    ran_init: bool
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
enum Paa5100jeRegisters {
    ProductId = 0x00,
    RevisionId = 0x01,
    Motion = 0x02,
    DeltaXL = 0x03,
    DeltaXH = 0x04,
    DeltaYL = 0x05,
    DeltaYH = 0x06,
    Squal = 0x07,
    RawDataSum = 0x08,
    MaximumRawData = 0x09,
    MinimumRawData = 0x0A,
    ShutterLower = 0x0B,
    ShutterUpper = 0x0C,
    Observation = 0x15,
    MotionBurst = 0x16,
    PowerUpReset = 0x3A,
    Shutdown = 0x3B,
    Resolution = 0x4E,
    RawDataGrab = 0x58,
    RawDataGrabStatus = 0x59,
    Orientation = 0x5B,
    InverseProductId = 0x5F
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum Rotation {
    Deg0,
    Deg90,
    Deg180,
    Deg270
}

const PRODUCT_ID: u8 = 0x49;
const INVERSE_PRODUCT_ID: u8 = 0xB6;
const BIT_MASK_8_TH: u8 = 0b1000_0000;
const SWAP_X_Y_BIT: u8 = 0b1000_0000;
const INVERT_Y_BIT: u8 = 0b0100_0000;
const INVERT_X_BIT: u8 = 0b0010_0000;

impl<'a, 'buf> Paa5100je<'a, 'buf> {
    /// Creates a new PAA5100JE instance from a pre-existing Spi peripheral
    pub fn new_from_spi(
        spi: spi::Spi<'a, Async>,
        cs: Output<'a>,
        motion_int: ExtiInput<'a>,
        spi_buf: &'buf mut [u8; SPI_MIN_BUF_LEN],
        rotation: Rotation
    ) -> Self {
        Paa5100je {
            spi: spi,
            cs: cs,
            motion_int: motion_int,
            spi_buf: spi_buf,
            rotation: rotation,
            ran_init: false,
        }
    }

    /// Creates a new PAA5100JE instance from uninitialized pins
    pub fn new_from_pins<SpiPeri: spi::Instance, IntPin: Pin>(
        peri: impl Peripheral<P = SpiPeri> + 'a,
        cs: impl Pin,
        sck: impl Peripheral<P = impl SckPin<SpiPeri>> + 'a,
        mosi: impl Peripheral<P = impl MosiPin<SpiPeri>> + 'a,
        miso: impl Peripheral<P = impl MisoPin<SpiPeri>> + 'a,
        tx_dma: impl Peripheral<P = impl spi::TxDma<SpiPeri>> + 'a,
        rx_dma: impl Peripheral<P = impl spi::RxDma<SpiPeri>> + 'a,
        motion_int_pin: IntPin,
        motion_int: impl Peripheral<P = <IntPin as Pin>::ExtiChannel> + 'a,
        spi_buf: &'buf mut [u8; SPI_MIN_BUF_LEN],
        rotation: Rotation
    ) -> Self {

        let mut spi_config = spi::Config::default();
        spi_config.frequency = hz(2_000_000);

        let spi = spi::Spi::new(
            peri,
            sck,
            mosi,
            miso,
            tx_dma,
            rx_dma,
            spi_config
        );

        let cs = Output::new(cs, Level::High, Speed::VeryHigh);
        let motion_int = ExtiInput::new(motion_int_pin, motion_int, Pull::None);

        Paa5100je {
            spi: spi,
            cs: cs,
            motion_int: motion_int,
            spi_buf: spi_buf,
            rotation: rotation,
            ran_init: false
        }
    }

    fn select(&mut self) {
        // CS is active low.
        self.cs.set_low();
    }

    fn deselect(&mut self) {
        self.cs.set_high();
    }

    async fn read(&mut self, reg: Paa5100jeRegisters) -> u8 {
        // Translate the register enum to a u8 and read it.
        // Always selects and deselects for a full read.
        return self.read_raw(reg as u8, true).await;
    }

    // Sequentially reads from the start register to the end of the buffer.
    async fn read_sequential(&mut self, start_reg: Paa5100jeRegisters, buf: &mut [u8]) {
        let start_reg = start_reg as u8;
        // Since reading sequential registers, only select once for the start address.
        self.select();

        for i in 0..buf.len() {
            buf[i] = self.read_raw(start_reg + i as u8, false).await;
        }

        self.deselect();
    }

    async fn read_raw(&mut self, reg: u8, select: bool) -> u8 {
        // If select is true, full read is a single byte from a single register.
        if select {
            self.select();
        }

        // Mask the write bit to ensure we're reading.
        self.spi_buf[0] = reg & !BIT_MASK_8_TH;
        // The first byte is the register address, the second byte is the read payload.
        let res = self.spi.transfer_in_place(&mut self.spi_buf[..2]).await;
        if res.is_err() {
            warn!("SPI read error: {:?}", res);
        }

        if select {
            self.deselect();
        }

        return self.spi_buf[1];
    }

    async fn write(&mut self, reg: Paa5100jeRegisters, val: u8) {
        // Translate the register enum to a u8 and write to it.
        // Always selects and deselects for a full write.
        return self.write_raw(reg as u8, val, true).await;
    }

    async fn write_raw(&mut self, reg: u8, val: u8, select: bool) {
        // If select is true, full write is a single byte to a single register.
        if select {
            self.select();
        }

        // Set the write bit to ensure we're writing.
        self.spi_buf[0] = reg | BIT_MASK_8_TH;
        self.spi_buf[1] = val;
        // The first byte is the register address, the second byte is the write payload.
        let res = self.spi.transfer_in_place(&mut self.spi_buf[..2]).await;

        if res.is_err() {
            warn!("SPI write error: {:?}", res);
        }

        if select {
            self.deselect();
        }
    }

    async fn write_multi(&mut self, reg_buf: &[u8], val_buf: &[u8]) {
        // Ensure the register and value buffers are the same length.
        if reg_buf.len() != val_buf.len() {
            defmt::error!("Register and value buffers must be the same length!");
        }

        for i in 0..reg_buf.len() {
            self.write_raw(reg_buf[i], val_buf[i], true).await;
        }
    }

    async fn power_up(&mut self) {
        self.select();
        // TODO Unsure if required for this duration or at all.
        Timer::after(Duration::from_millis(50)).await;
        self.deselect();

        self.write(Paa5100jeRegisters::PowerUpReset, 0x5A).await;
        // TODO Unsure if required for this duration or at all.
        Timer::after(Duration::from_millis(20)).await;

        // TODO: Unsure if needed
        let mut buf: [u8; 5] = [0; 5];
        self.read_sequential(Paa5100jeRegisters::Motion, &mut buf).await;
    }

    async fn calibration(&mut self) {
        // First block of calibration values.
        let reg_buf_1: [u8; 5] = [
            0x7F,
            0x55,
            0x50,

            0x7F,
            0x43
        ];
        let val_buf_1: [u8; 5] = [
            0x00,
            0x01,
            0x07,

            0x0E,
            0x10
        ];
        self.write_multi(&reg_buf_1, &val_buf_1).await;

        // A conditional calibration value.
        let calibration_value_1 = self.read_raw(0x67, true).await;
        if calibration_value_1 & BIT_MASK_8_TH > 0 {
            self.write_raw(0x48, 0x04, true).await;
        } else {
            self.write_raw(0x48, 0x02, true).await;
        }

        // Second block of calibration values.
        let reg_buf_2: [u8; 5] = [
            0x7F,
            0x51,
            0x50,
            0x55,
            0x7F
        ];
        let val_buf_2: [u8; 5] = [
            0x00,
            0x7B,
            0x00,
            0x00,
            0x0E
        ];
        self.write_multi(&reg_buf_2, &val_buf_2).await;

        // Another conditional calibration value.
        let calibration_value_2 = self.read_raw(0x73, true).await;
        if calibration_value_2 == 0x00 {
            let mut c1 = self.read_raw(0x70, true).await;
            let mut c2 = self.read_raw(0x71, true).await;
            if c1 <= 28 {
                c1 += 14;
            } else if c1 > 28 {
                c1 += 11;
            }
            c1 = max(0, min(0x3F, c1));
            c2 = c2 * 45;

            let reg_buf: [u8; 6] = [
                0x7F,
                0x61,
                0x51,
                0x7F,
                0x70,
                0x71
            ];
            let val_buf: [u8; 6] = [
                0x00,
                0xAD,
                0x70,
                0x0E,
                c1,
                c2
            ];
            self.write_multi(&reg_buf, &val_buf).await;
        }

        // Third block of calibration values.
        let reg_buf_3: [u8; 67] = [
            0x7F,
            0x61,

            0x7F,
            0x40,

            0x7F,
            0x41,
            0x43,
            0x45,

            0x5F,
            0x7B,
            0x5E,
            0x5B,
            0x6D,
            0x45,
            0x70,
            0x71,

            0x7F,
            0x44,
            0x40,
            0x4E,

            0x7F,
            0x66,
            0x65,
            0x6A,
            0x61,
            0x62,

            0x7F,
            0x4F,
            0x5F,
            0x48,
            0x49,
            0x57,
            0x60,
            0x61,
            0x62,
            0x63,

            0x7F,
            0x45,

            0x7F,
            0x4D,
            0x55,
            0x74,
            0x75,
            0x4A,
            0x4B,
            0x44,

            0x45,
            0x64,
            0x65,

            0x7F,
            0x65,
            0x66,
            0x63,
            0x6F,

            0x7F,
            0x48,

            0x7F,
            0x41,
            0x43,
            0x4B,
            0x45,
            0x44,
            0x4C,

            0x7F,
            0x5B,

            0x7F,
            0x40,
        ];

        let val_buf_3: [u8; 67] = [
            0x00,
            0xAD,
            0x03,
            0x00,
            0x05,
            0xB3,
            0xF1,
            0x14,
            0x34,
            0x08,
            0x34,
            0x11,
            0x11,
            0x17,
            0xE5,
            0xE5,
            0x06,
            0x1B,
            0xBF,
            0x3F,
            0x08,
            0x44,
            0x20,
            0x3A,
            0x05,
            0x05,
            0x09,
            0xAF,
            0x40,
            0x80,
            0x80,
            0x77,
            0x78,
            0x78,
            0x08,
            0x50,
            0x0A,
            0x60,
            0x00,
            0x11,
            0x80,
            0x21,
            0x1F,
            0x78,
            0x78,
            0x08,
            0x50,
            0xFF,
            0x1F,
            0x14,
            0x67,
            0x08,
            0x70,
            0x1C,
            0x15,
            0x48,
            0x07,
            0x0D,
            0x14,
            0x0E,
            0x0F,
            0x42,
            0x80,
            0x10,
            0x02,
            0x07,
            0x41,
        ];
        self.write_multi(&reg_buf_3, &val_buf_3).await;

        // Need to wait for some reason.
        Timer::after(Duration::from_millis(10)).await;

        // Fourth block of calibration values.
        let reg_buf_4: [u8; 16] = [
            0x7F,
            0x32,
            0x7F,
            0x40,
            0x7F,
            0x68,
            0x69,
            0x7F,
            0x48,
            0x6F,
            0x7F,
            0x5B,
            0x4E,
            0x5A,
            0x40,
            0x73
        ];

        let val_buf_4: [u8; 16] = [
            0x00,
            0x00,
            0x07,
            0x40,
            0x06,
            0xF0,
            0x00,
            0x0D,
            0xC0,
            0xD5,
            0x00,
            0xA0,
            0xA8,
            0x90,
            0x80,
            0x1F
        ];
        self.write_multi(&reg_buf_4, &val_buf_4).await;

        // Need to wait for some reason.
        Timer::after(Duration::from_millis(10)).await;

        self.write_raw(0x73, 0x00, true).await;
    }

    async fn set_rotation(&mut self, rotation: Rotation) {
        let orientation: u8;
        match rotation {
            Rotation::Deg0 => {
                orientation = INVERT_X_BIT | INVERT_Y_BIT | SWAP_X_Y_BIT;
            }
            Rotation::Deg90 => {
                orientation = INVERT_Y_BIT;
            }
            Rotation::Deg180 => {
                orientation = SWAP_X_Y_BIT;
            }
            Rotation::Deg270 => {
                orientation = INVERT_X_BIT;
            }
        };
        self.write(Paa5100jeRegisters::Orientation, orientation).await;
    }

    pub async fn init(&mut self) {
        defmt::info!("PAA5100JE-Q begin startup.");

        'config_loop:
        loop {
            self.ran_init = false;

            self.power_up().await;
            self.calibration().await;

            let read_product_id = self.read(Paa5100jeRegisters::ProductId).await;
            let read_inverse_product_id = self.read(Paa5100jeRegisters::InverseProductId).await;

            if read_product_id != PRODUCT_ID || read_inverse_product_id != INVERSE_PRODUCT_ID {
                defmt::error!("Product ID mismatch. Read: {:X} {:X}, Expected: {:X} {:X}",
                    read_product_id,
                    read_inverse_product_id,
                    PRODUCT_ID,
                    INVERSE_PRODUCT_ID);
                Timer::after_millis(1000).await;
                continue 'config_loop;
            }

            self.set_rotation(self.rotation).await;

            self.ran_init = true;

            // TODO Move this to a separate function.

            'data_loop:
            loop {
                // Block on motion interrupt, active low. Or timeout after 1 second.
                match select(self.motion_int.wait_for_low(), Timer::after_millis(1000)).await {
                    Either::First(_) => {
                        // Got an interrupt, so Optical Flow should be working.
                        // robot_state.set_imu_inop(false);

                        // Read Burst Optical Flow Data
                        let mut buf: [u8; 13] = [0; 13];
                        self.read_sequential(Paa5100jeRegisters::MotionBurst, &mut buf).await;

                        // buf[0] is unused.
                        // buf[1] is the data ready.
                        if buf[1] & BIT_MASK_8_TH == 0 {
                            // Weird if interrupt goes low and the data is not ready.
                            defmt::error!("Motion data not ready! Will wait again...");
                            continue 'data_loop;
                        }

                        // buf[7] is Squal
                        // TODO: Quality of the data?
                        if buf[7] > 0x19 {
                            defmt::error!("Squal too low! Will wait again...");
                            continue 'data_loop;
                        }

                        // buf[11] is Shutter Upper.
                        // TODO Not sure what this means.
                        if buf[11] != 0x1F {
                            defmt::error!("Shutter Upper not equal! Will wait again...");
                            continue 'data_loop;
                        }

                        // buf[2] is obs.
                        // buf[8] is Raw Data Sum.
                        // buf[9] is Maximum Raw Data.
                        // buf[10] is Minimum Raw Data.
                        // buf[12] is Shutter Lower.

                        // buf[3] and buf[4] are X motion.
                        let delta_x = ((buf[4] as i16) << 8) | buf[3] as i16;
                        // buf[5] and buf[6] are Y motion.
                        let delta_y = ((buf[6] as i16) << 8) | buf[5] as i16;

                        defmt::info!("Motion: ({}, {})", delta_x, delta_y);
                    }
                    Either::Second(_) => {
                        // If it's timed out, not moving!
                        defmt::info!("Motion: ({}, {})", 0, 0);
                    }
                };
            }
        }
    }

    pub async fn read_motion_polling(&mut self) -> (i16, i16) {
        if !self.ran_init {
            defmt::error!("Did not run init!");
            return (0, 0);
        }

        let mut buf: [u8; 5] = [0; 5];
        self.read_sequential(Paa5100jeRegisters::MotionBurst, &mut buf).await;

        // TODO Implement a check for data ready bit.
        // TODO Implement a timeout for the read.
        let x = ((buf[2] as i16) << 8) | buf[1] as i16;
        let y = ((buf[4] as i16) << 8) | buf[3] as i16;

        return (x, y);
    }
}
