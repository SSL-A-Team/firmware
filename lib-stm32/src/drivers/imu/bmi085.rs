/*
 * Driver for the Bosch BMI085 IMU.
 * 
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi085-ds001.pdf
 * 
 */

use core::{cmp::min, f32::consts::PI};

use embassy_stm32::{
    gpio::{Level, Output, Pin, Speed},
    mode::Async,
    spi::{self, MisoPin, MosiPin, SckPin},
    time::hz,
    Peripheral
};
use embassy_time::{Timer, Duration};

use defmt::*;


pub const SPI_MIN_BUF_LEN: usize = 8;

/// SPI driver for the Bosch BMI085 IMU: Accel + Gyro
pub struct Bmi085<'a, 'buf, SpiPeri: spi::Instance> {
    spi: spi::Spi<'a, SpiPeri, Async>,
    accel_cs: Output<'a>,
    gyro_cs: Output<'a>,
    spi_buf: &'buf mut [u8; SPI_MIN_BUF_LEN],
    accel_range: AccelRange,
    gyro_range: GyroRange,
}

#[repr(u8)]
#[allow(non_camel_case_types, dead_code)]
#[derive(Clone, Copy, Debug)]
enum AccelRegisters {
    ACC_CHIP_ID = 0x00,
    ACC_ERR_REG = 0x02,
    ACC_STATUS = 0x03,
    ACC_X_LSB = 0x12,
    ACC_X_MSB = 0x13,
    ACC_Y_LSB = 0x14,
    ACC_Y_MSB = 0x15,
    ACC_Z_LSB = 0x16,
    ACC_Z_MSB = 0x17,
    SENSORTIME_0 = 0x18,
    SENSORTIME_1 = 0x19,
    SENSORTIME_2 = 0x1A,
    ACC_INT_STAT_1 = 0x1D,
    TEMP_MSB = 0x22,
    TEMP_LSB = 0x23,
    ACC_CONF = 0x40,
    ACC_RANGE = 0x41,
    INT1_IO_CTRL = 0x53,
    INT2_IO_CTRL = 0x54,
    INT_MAP_DATA = 0x58,
    ACC_SELF_TEST = 0x6D,
    ACC_PWR_CONF = 0x7C,
    ACC_PWR_CTRL = 0x7D,
    ACC_SOFTRESET = 0x7E,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum AccelRange {
    Range2g = 0x00,
    Range4g = 0x01,
    Range8g = 0x02,
    Range16g = 0x03,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum AccelConfBwp {
    OverSampling4Fold = 0x08,
    OverSampling2Fold = 0x09,
    OverSamplingNormal = 0x0A,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum AccelConfOdr {
    OutputDataRate12p5 = 0x05,
    OutputDataRate25p0 = 0x06,
    OutputDataRate50p0 = 0x07,
    OutputDataRate100p0 = 0x08,
    OutputDataRate200p0 = 0x09,
    OutputDataRate400p0 = 0x0A,
    OutputDataRate800p0 = 0x0B,
    OutputDataRate1600p0 = 0x0C,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
enum AccelSelfTestReg {
    Off = 0x00,
    PositiveSelfTest = 0x0D,
    NegativeSelfTest = 0x09,
}

const ACCEL_CHIP_ID: u8 = 0x1F;

#[repr(u8)]
#[allow(non_camel_case_types, dead_code)]
#[derive(Clone, Copy, Debug)]
enum GyroRegisters {
    GYRO_CHIP_ID = 0x00,
    RATE_X_LSB = 0x02,
    RATE_X_MSB = 0x03,
    RATE_Y_LSB = 0x04,
    RATE_Y_MSB = 0x05,
    RATE_Z_LSB = 0x06,
    RATE_Z_MSB = 0x07,
    GYRO_INT_STAT_1 = 0x0A,
    GYRO_RANGE = 0x0F,
    GYRO_BANDWIDTH = 0x10,
    GYRO_LPM1 = 0x11,
    GYRO_SOFTRESET = 0x14,
    GYRO_INT_CONTROL = 0x15,
    INT3_INT4_IO_CONF = 0x16,
    INT3_INT4_IO_MAP = 0x18,
    GYRO_SELF_TEST = 0x3C,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
enum GyroSelfTestReg {
    RateOk = 0b10000,
    BistFail = 0b00100,
    BistRdy = 0b00010,
    TrigBist = 0b00001,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum GyroRange {
    PlusMinus2000DegPerSec = 0x00,
    PlusMinus1000DegPerSec = 0x01,
    PlusMinus500DegPerSec = 0x02,
    PlusMinus250DegPerSec = 0x03,
    PlusMinus125DegPerSec = 0x04,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum GyroBandwidth {
    FilterBw532Hz = 0x00,
    FilterBw230Hz = 0x01,
    FilterBw116Hz = 0x02,
    FilterBw47Hz = 0x03,
    FilterBw23Hz = 0x04,
    FilterBw12Hz = 0x05,
    FilterBw64Hz = 0x06,
    FilterBw32Hz = 0x07,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
enum GyroIntCtrl {
    InterruptOff = 0x00,
    InterruptOnNewData = 0x80,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum GyroIntPinMode {
    PushPull = 0b0,
    OpenDrain = 0b1,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum GyroIntPinActiveState {
    ActiveLow = 0b0,
    ActiveHigh = 0b1,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum GyroIntMap {
    NotMapped = 0x00,
    Int3 = 0x01,
    Int4 = 0x80,
    Int3AndInt4 = 0x81,
}

const GYRO_CHIP_ID: u8 = 0x0F;

const READ_BIT: u8 = 0x80;

impl<'a, 'buf, SpiPeri: spi::Instance> 
    Bmi085<'a, 'buf, SpiPeri> {
    /// creates a new BMI085 instance from a pre-existing Spi peripheral
    pub fn new_from_spi(
        spi: spi::Spi<'a, SpiPeri, Async>, 
        accel_cs: Output<'a>, 
        gyro_cs: Output<'a>,
        spi_buf: &'buf mut [u8; SPI_MIN_BUF_LEN],
    ) -> Self {
        Bmi085 {
            spi: spi,
            accel_cs: accel_cs,
            gyro_cs: gyro_cs,
            spi_buf: spi_buf,
            accel_range: AccelRange::Range4g,
            gyro_range: GyroRange::PlusMinus2000DegPerSec,
        }
    }

    ///t creates a new BMI085 instance from uninitialized pins
    pub fn new_from_pins(
        peri: impl Peripheral<P = SpiPeri> + 'a,
        sck: impl Peripheral<P = impl SckPin<SpiPeri>> + 'a,
        mosi: impl Peripheral<P = impl MosiPin<SpiPeri>> + 'a,
        miso: impl Peripheral<P = impl MisoPin<SpiPeri>> + 'a,
        txdma: impl Peripheral<P = impl spi::TxDma<SpiPeri>> + 'a,
        rxdma: impl Peripheral<P = impl spi::RxDma<SpiPeri>> + 'a,
        accel_cs: impl Pin,
        gyro_cs: impl Pin,
        spi_buf: &'buf mut [u8; SPI_MIN_BUF_LEN],
    ) -> Self {
        let mut spi_config = spi::Config::default();
        spi_config.frequency = hz(1_000_000);

        let imu_spi = spi::Spi::new(
            peri,
            sck,
            mosi,
            miso,
            txdma,
            rxdma,
            spi_config
        );

        let accel_cs = Output::new(accel_cs, Level::High, Speed::VeryHigh);
        let imu_cs = Output::new(gyro_cs, Level::High, Speed::VeryHigh);

        Bmi085 { 
            spi: imu_spi,
            accel_cs: accel_cs,
            gyro_cs: imu_cs,
            spi_buf: spi_buf,
            accel_range: AccelRange::Range4g,
            gyro_range: GyroRange::PlusMinus2000DegPerSec,
        }
    }

    fn select_accel(&mut self) {
        self.gyro_cs.set_high();
        self.accel_cs.set_low();
    }

    fn select_gyro(&mut self) {
        self.accel_cs.set_high();
        self.gyro_cs.set_low();
    }

    fn deselect(&mut self) {
        self.accel_cs.set_high();
        self.gyro_cs.set_high();
    }

    async fn read(&mut self, reg: u8) -> u8 {
        self.spi_buf[0] = reg | READ_BIT;
        let _ = self.spi.transfer_in_place(&mut self.spi_buf[..2]).await;
        self.spi_buf[1]
    }

    async fn accel_burst_read(&mut self, reg: AccelRegisters, dest: &mut [u8]) {
        // the transaction length is either the dest buf size + 2 
        // (the start addr + N data bytes) + 1 spurious byte for data to settle (or something)
        // OR upper bounded by internal length of the buffer.
        let trx_len = min(dest.len() + 2, self.spi_buf.len());

        self.spi_buf[0] = reg as u8 | READ_BIT;
        self.spi_buf[1] = 0x00;
        let _ = self.spi.transfer_in_place(&mut self.spi_buf[..trx_len]).await;
        dest.copy_from_slice(&self.spi_buf[2..trx_len]);
    }

    async fn gyro_burst_read(&mut self, reg: GyroRegisters, dest: &mut [u8]) {
        // the transaction length is either the dest buf size + 1 
        // (the start addr + N data bytes)
        // OR upper bounded by internal length of the buffer.
        let trx_len = min(dest.len() + 1, self.spi_buf.len());

        self.spi_buf[0] = reg as u8 | READ_BIT;
        let _ = self.spi.transfer_in_place(&mut self.spi_buf[..trx_len]).await;
        dest.copy_from_slice(&self.spi_buf[1..trx_len]);
    }

    async fn accel_read(&mut self, reg: AccelRegisters) -> u8 {
        self.select_accel();
        self.read(reg as u8).await
    }

    async fn gyro_read(&mut self, reg: GyroRegisters) -> u8 {
        self.select_gyro();
        self.read(reg as u8).await
    }

    async fn write(&mut self, reg: u8, val: u8) {
        self.spi_buf[0] = reg & !READ_BIT;
        self.spi_buf[1] = val;
        let _ = self.spi.transfer_in_place(&mut self.spi_buf[..2]).await;
    }

    async fn accel_write(&mut self, reg: AccelRegisters, val: u8) {
        self.select_accel();
        self.write(reg as u8, val).await;
    }

    async fn gyro_write(&mut self, reg: GyroRegisters, val: u8) {
        self.select_gyro();
        self.write(reg as u8, val).await;
    }

    pub async fn init(&mut self) {
        // imu accel needs at least one dummy read to set the data mode to SPI from POn I2C
        // second read should be valid here for sanity purposes

        let _ = self.accel_read(AccelRegisters::ACC_CHIP_ID).await;
        let _ = self.accel_read(AccelRegisters::ACC_CHIP_ID).await;
    }

    async fn accel_self_test(&mut self) -> Result<(), ()> {
        // test is non-trivial, e.g. not self contained
        // procedure is described in section 4.6.1 of the datasheet

        self.accel_set_range(AccelRange::Range16g).await;
        self.accel_set_bandwidth(AccelConfBwp::OverSamplingNormal, AccelConfOdr::OutputDataRate1600p0).await;
        Timer::after(Duration::from_millis(2 + 1)).await;

        self.accel_write(AccelRegisters::ACC_SELF_TEST, AccelSelfTestReg::PositiveSelfTest as u8).await;
        Timer::after(Duration::from_millis(50 + 1)).await;
        let positive_test_data = self.accel_get_data_mg().await;

        self.accel_write(AccelRegisters::ACC_SELF_TEST, AccelSelfTestReg::NegativeSelfTest as u8).await;
        Timer::after(Duration::from_millis(50 + 1)).await;
        let negative_test_data = self.accel_get_data_mg().await;

        self.accel_write(AccelRegisters::ACC_SELF_TEST, AccelSelfTestReg::Off as u8).await;
        Timer::after(Duration::from_millis(50 + 1)).await;

        let x_diff = positive_test_data[0] - negative_test_data[0];
        let y_diff = positive_test_data[1] - negative_test_data[1];
        let z_diff = positive_test_data[2] - negative_test_data[2];

        let mut result = Ok(());
        if x_diff < 1000.0 {
            trace!("accel self test failed x-axis needed polarity difference >=1000 got {}", x_diff);
            result = Err(())
        }

        if y_diff < 1000.0 {
            trace!("accel self test failed y-axis needed polarity difference >=1000 got {}", y_diff);
            result = Err(())
        }

        if z_diff < 500.0 {
            trace!("accel self test failed z-axis needed polarity difference >=500 got {}", z_diff);
            result = Err(())
        }

        if result.is_err() {
            warn!("accel failed BIST. (one or more axis diverged)");
        }

        result
    }

    async fn gyro_self_test(&mut self) -> Result<(), ()> {
        self.gyro_write(GyroRegisters::GYRO_SELF_TEST, GyroSelfTestReg::TrigBist as u8).await;
        let mut try_ct = 0;
        loop {
            let strreg_val = self.gyro_read(GyroRegisters::GYRO_SELF_TEST).await;
            if (strreg_val & GyroSelfTestReg::BistRdy as u8) != 0 {
                if (strreg_val & GyroSelfTestReg::BistFail as u8) != 0 {
                    warn!("gyro failed BIST. (the bist fail bit is high)");
                    return Err(());
                } else {
                    debug!("gyro passed self test.");
                    return Ok(());
                }
            }

            Timer::after(Duration::from_millis(10)).await;
            try_ct += 1;

            if try_ct > 10 {
                warn!("gyro BIST did not converge.");
                return Err(());
            }
        }
    }

    pub async fn self_test(&mut self) -> Result<(), ()> {
        let mut has_self_test_error = Ok(());

        self.deselect();

        let _ = self.accel_read(AccelRegisters::ACC_CHIP_ID).await;
        let acc_chip_id = self.accel_read(AccelRegisters::ACC_CHIP_ID).await;
        if acc_chip_id != ACCEL_CHIP_ID {
            warn!("read accel ID (0x{:x}) does not match expected BMI085 accel ID (0x{:x})", acc_chip_id, ACCEL_CHIP_ID);
            has_self_test_error = Err(());
        } else {
            debug!("accel id verified: 0x{:x}", acc_chip_id);
        }

        let _ = self.gyro_read(GyroRegisters::GYRO_CHIP_ID).await;
        let gyro_chip_id = self.gyro_read(GyroRegisters::GYRO_CHIP_ID).await;
        if gyro_chip_id != GYRO_CHIP_ID {
            warn!("read gyro ID (0x{:x}) does not match expected BMI085 gyro ID (0x{:x})", gyro_chip_id, GYRO_CHIP_ID);
            has_self_test_error = Err(());
        } else {
            debug!("gyro id verified: 0x{:x}", gyro_chip_id);
        }
        
        if (self.accel_self_test().await).is_err() {
            has_self_test_error = Err(());
        }

        if (self.gyro_self_test().await).is_err() {
            has_self_test_error = Err(());
        }

        has_self_test_error
    }

    const fn read_pair_to_i16(&self, lsb: u8, msb: u8) -> i16 {
        (msb as u16 * 256 + lsb as u16) as i16
    }
    
    pub async fn accel_get_raw_data(&mut self) -> [i16; 3] {
        let mut buf: [u8; 6] = [0; 6];
        self.accel_burst_read(AccelRegisters::ACC_X_LSB, &mut buf).await;

        [self.read_pair_to_i16(buf[0], buf[1]),
            self.read_pair_to_i16(buf[2], buf[3]),
            self.read_pair_to_i16(buf[4], buf[5])]
    }

    pub fn accel_range_mg(&self) -> f32 {
        match self.accel_range {
            AccelRange::Range2g => 2000.0,
            AccelRange::Range4g => 4000.0,
            AccelRange::Range8g => 8000.0,
            AccelRange::Range16g => 16000.0,
        }
    }

    pub fn convert_accel_raw_sample_mg(&self, raw_sample: i16) -> f32 {
        let conversion_num = self.accel_range_mg();

        raw_sample as f32 * (conversion_num / i16::MAX as f32)
    }

    pub fn convert_accel_raw_sample_mps(&self, raw_sample: i16) -> f32 {
        self.convert_accel_raw_sample_mg(raw_sample) / 1000.0 * 9.80665
    }

    pub async fn accel_get_data_mg(&mut self) -> [f32; 3] {
        let raw_data = self.accel_get_raw_data().await;

        return [self.convert_accel_raw_sample_mg(raw_data[0]),
            self.convert_accel_raw_sample_mg(raw_data[1]),
            self.convert_accel_raw_sample_mg(raw_data[2])]
    }

    pub async fn accel_get_data_mps(&mut self) -> [f32; 3] {
        let raw_data = self.accel_get_raw_data().await;

        return [self.convert_accel_raw_sample_mps(raw_data[0]),
            self.convert_accel_raw_sample_mps(raw_data[1]),
            self.convert_accel_raw_sample_mps(raw_data[2])]
    }

    pub async fn accel_set_bandwidth(&mut self, oversampling_mode: AccelConfBwp, output_data_rate: AccelConfOdr) {
        self.accel_write(AccelRegisters::ACC_CONF, (oversampling_mode as u8) << 4 | output_data_rate as u8 ).await;
    }

    pub async fn accel_set_range(&mut self, range: AccelRange) {
        self.accel_write(AccelRegisters::ACC_RANGE, range as u8).await;

        self.accel_range = range;
    }

    pub async fn gyro_get_raw_data(&mut self) -> [i16; 3] {
        let mut buf: [u8; 6] = [0; 6];
        self.gyro_burst_read(GyroRegisters::RATE_X_LSB, &mut buf).await;

        [self.read_pair_to_i16(buf[0], buf[1]),
            self.read_pair_to_i16(buf[2], buf[3]),
            self.read_pair_to_i16(buf[4], buf[5])]
    }

    pub fn dps_to_rads(dps: f32) -> f32 {
        (dps / 360.0) * 2.0 * PI
    }

    fn gyro_rate(&self) -> f32 {
        match self.gyro_range {
            GyroRange::PlusMinus2000DegPerSec => 2000.0,
            GyroRange::PlusMinus1000DegPerSec => 1000.0,
            GyroRange::PlusMinus500DegPerSec => 500.0,
            GyroRange::PlusMinus250DegPerSec => 250.0,
            GyroRange::PlusMinus125DegPerSec => 125.0,
        }
    }

    pub fn max_dps(&self) -> f32 {
        self.gyro_rate()
    }

    pub fn max_rads(&self) -> f32 {
        Self::dps_to_rads(self.max_dps())
    }

    pub fn convert_raw_gyro_sample_dps(&self, raw_sample: i16) -> f32 {
        let conversion_num = self.gyro_rate();

        raw_sample as f32 * (conversion_num / i16::MAX as f32)
    }

    pub fn convert_raw_gyro_sample_rads(&self, raw_sample: i16) -> f32 {
        Self::dps_to_rads(self.convert_raw_gyro_sample_dps(raw_sample))
    }

    pub async fn gyro_get_data_dps(&mut self) -> [f32; 3] {
        let raw_data = self.gyro_get_raw_data().await;

        return [self.convert_raw_gyro_sample_dps(raw_data[0]),
            self.convert_raw_gyro_sample_dps(raw_data[1]),
            self.convert_raw_gyro_sample_dps(raw_data[2])]
    }

    pub async fn gyro_get_data_rads(&mut self) -> [f32; 3] {
        let raw_data = self.gyro_get_raw_data().await;

        return [self.convert_raw_gyro_sample_rads(raw_data[0]),
            self.convert_raw_gyro_sample_rads(raw_data[1]),
            self.convert_raw_gyro_sample_rads(raw_data[2])]
    }

    pub async fn gyro_set_range(&mut self, range: GyroRange) {
        self.gyro_write(GyroRegisters::GYRO_RANGE, range as u8).await;

        self.gyro_range = range;
    }

    pub async fn gyro_set_bandwidth(&mut self, bandwidth: GyroBandwidth) {
        self.gyro_write(GyroRegisters::GYRO_BANDWIDTH, bandwidth as u8).await;
    }

    pub async fn gyro_enable_interrupts(&mut self) {
        self.gyro_write(GyroRegisters::GYRO_INT_CONTROL, GyroIntCtrl::InterruptOnNewData as u8).await;
    }

    pub async fn gyro_disable_interrupts(&mut self) {
        self.gyro_write(GyroRegisters::GYRO_INT_CONTROL, GyroIntCtrl::InterruptOff as u8).await;
    }

    pub async fn gyro_set_int_config(&mut self,
        int3_active_state: GyroIntPinActiveState,
        int3_mode: GyroIntPinMode,
        int4_active_state: GyroIntPinActiveState,
        int4_mode: GyroIntPinMode) {
        let reg_val = (int4_mode as u8) << 3 | (int4_active_state as u8) << 2 | (int3_mode as u8) << 1 | int3_active_state as u8;
        self.gyro_write(GyroRegisters::INT3_INT4_IO_CONF, reg_val).await;
    }

    pub async fn gyro_set_int_map(&mut self, map: GyroIntMap) {
        self.gyro_write(GyroRegisters::INT3_INT4_IO_MAP, map as u8).await;
    }
}