/*
 * Driver for the Bosch BMI323 IMU.
 *
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi323-ds000.pdf *
 */

use core::{cmp::min, f32::consts::PI};

use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed},
    mode::Async,
    spi::{self, MisoPin, MosiPin, SckPin},
    time::hz,
    Peri,
};

pub const SPI_MIN_BUF_LEN: usize = 14;

/// SPI driver for the Bosch BMI085 IMU: Accel + Gyro
pub struct Bmi323<'a, 'buf> {
    spi: spi::Spi<'a, Async>,
    spi_cs: Output<'a>,
    spi_buf: &'buf mut [u8; SPI_MIN_BUF_LEN],
    accel_mode: AccelMode,
    accel_range: AccelRange,
    accel_odr: OutputDataRate,
    accel_bw_mode: Bandwidth3DbCutoffFreq,
    accel_avg_window: DataAveragingWindow,
    gyro_mode: GyroMode,
    gyro_range: GyroRange,
    gyro_odr: OutputDataRate,
    gyro_bw_mode: Bandwidth3DbCutoffFreq,
    gyro_avg_window: DataAveragingWindow,
}

#[repr(u8)]
#[allow(non_camel_case_types, dead_code, clippy::upper_case_acronyms)]
#[derive(Clone, Copy, Debug)]
enum ImuRegisters {
    CHIP_ID = 0x00,
    ERR_REG = 0x01,
    STATUS = 0x02,
    ACC_DATA_X = 0x03,
    ACC_DATA_Y = 0x04,
    ACC_DATA_Z = 0x05,
    GYR_DATA_X = 0x06,
    GYR_DATA_Y = 0x07,
    GYR_DATA_Z = 0x08,
    TEMP_DATA = 0x09,
    SENSORTIME_0 = 0x0A,
    SENSORTIME_1 = 0x0B,
    SAT_FLAGS = 0x0C,
    INT_STATUS_INT1 = 0x0D,
    INT_STATUS_INT2 = 0x0E,
    INT_STATUS_IBI = 0x0F,
    FEATURE_IO0 = 0x10,
    FEATURE_IO1 = 0x11,
    FEATURE_IO2 = 0x12,
    FEATURE_IO3 = 0x13,
    FEATURE_IO_STATUS = 0x14,
    FIFO_FILL_LEVEL = 0x15,
    FIFO_DATA = 0x16,
    // 0x17 - 0x19 reserved
    ACC_CONF = 0x20,
    GYR_CONF = 0x21,
    // 0x22 - 0x27 reserved
    ALT_ACC_CONF = 0x28,
    ALT_GYR_CONF = 0x29,
    ALT_CONF = 0x2A,
    ALT_STATUS = 0x2B,
    // 0x2B - 0x34 reserved
    FIFO_WATERMARK = 0x35,
    FIFO_CONF = 0x36,
    FIFO_CTRL = 0x37,
    IO_INT_CTRL = 0x38,
    INT_CONF = 0x39,
    INT_MAP1 = 0x3A,
    INT_MAP2 = 0x3B,
    // 0x3C - 0x3F reserved
    FEATURE_CTRL = 0x40,
    FEATURE_DATA_ADDR = 0x41,
    FEATURE_DATA_TX = 0x42,
    FEATURE_DATA_STATUS = 0x43,
    // 0x44 reserved
    FEATURE_ENGINE_STATUS = 0x45,
    // 0x46 reserved
    FEATURE_EVENT_EXT = 0x47,
    // 0x48 - 0x4E reserved
    IO_PDN_CTRL = 0x4F,
    IO_SPI_IF = 0x50,
    IO_PAD_STRENGTH = 0x51,
    IO_I2C_IF = 0x52,
    IO_ODR_DEVIATION = 0x53,
    // 0x54 - 0x5F reserved
    ACC_DP_OFF_X = 0x60,
    ACC_DP_DGAIN_X = 0x61,
    ACC_DP_OFF_Y = 0x62,
    ACC_DP_DGAIN_Y = 0x63,
    ACC_DP_OFF_Z = 0x64,
    ACC_DP_DGAIN_Z = 0x65,
    GYR_DP_OFF_X = 0x66,
    GYR_DP_DGAIN_X = 0x67,
    GYR_DP_OFF_Y = 0x68,
    GYR_DP_DGAIN_Y = 0x69,
    GYR_DP_OFF_Z = 0x6A,
    GYR_DP_DGAIN_Z = 0x6B,
    // 0x6C - 0x6F reserved
    I3C_TC_SYNC_TPH = 0x70,
    I3C_TC_SYNC_TU = 0x71,
    I3C_TC_SYNC_ODR = 0x72,
    // 0x73 - 0x7D reserved
    CMD = 0x7E,
    CFG_RES = 0x7F,
}

#[repr(u16)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum OutputDataRate {
    Odr0p78125 = 0x01,
    Odr1p5625 = 0x02,
    Odr3p125 = 0x03,
    Odr6p25 = 0x04,
    Odr12p5 = 0x05,
    Odr25p0 = 0x06,
    Odr50p0 = 0x07,
    Odr100p0 = 0x08,
    Odr200p0 = 0x09,
    Odr400p0 = 0x0A,
    Odr800p0 = 0x0B,
    Odr1600p0 = 0x0C,
    Odr3200p0 = 0x0D,
    Odr6400p0 = 0x0E,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum DataAveragingWindow {
    NoFiltering = 0x0,
    Average2Samples = 0x1,
    Average4Samples = 0x2,
    Average8Samples = 0x3,
    Average16Samples = 0x4,
    Average32Samples = 0x5,
    Average64Samples = 0x6,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum Bandwidth3DbCutoffFreq {
    AccOdrOver2 = 0x0,
    AccOdrOver4 = 0x1,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum IntPinLevel {
    ActiveLow = 0x0,
    ActiveHigh = 0x1,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum IntPinDriveMode {
    PushPull = 0x0,
    OpenDrain = 0x1,
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
pub enum AccelMode {
    Disabled = 0x0,
    DutyCycling = 0x3,
    ReducedCurrent = 0x4,
    ContinuousHighPerformance = 0x7,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum InterruptMode {
    Disabled = 0x0,
    MappedToInt1 = 0x1,
    MappedToInt2 = 0x2,
    MappedToI3cIbi = 0x3,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum GyroRange {
    PlusMinus125DegPerSec = 0x0,
    PlusMinus250DegPerSec = 0x1,
    PlusMinus500DegPerSec = 0x2,
    PlusMinus1000DegPerSec = 0x3,
    PlusMinus2000DegPerSec = 0x4,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub enum GyroMode {
    Disabled = 0x0,
    DisabledGyroDriveEnabled = 0x1,
    DutyCycling = 0x3,
    ContinuousReducedCurrent = 0x4,
    ContinuousHighPerformance = 0x7,
}

const BMI323_CHIP_ID: u16 = 0x0043;
const READ_BIT: u8 = 0x80;

impl<'a, 'buf> Bmi323<'a, 'buf> {
    /// creates a new BMI323 instance from a pre-existing Spi peripheral
    pub fn new_from_spi(
        spi: spi::Spi<'a, Async>,
        spi_cs: Output<'a>,
        spi_buf: &'buf mut [u8; SPI_MIN_BUF_LEN],
    ) -> Self {
        Bmi323 {
            spi,
            spi_cs,
            spi_buf,
            accel_mode: AccelMode::Disabled,
            accel_range: AccelRange::Range8g,
            accel_bw_mode: Bandwidth3DbCutoffFreq::AccOdrOver2,
            accel_odr: OutputDataRate::Odr100p0,
            accel_avg_window: DataAveragingWindow::NoFiltering,
            gyro_mode: GyroMode::Disabled,
            gyro_range: GyroRange::PlusMinus2000DegPerSec,
            gyro_bw_mode: Bandwidth3DbCutoffFreq::AccOdrOver2,
            gyro_odr: OutputDataRate::Odr100p0,
            gyro_avg_window: DataAveragingWindow::NoFiltering,
        }
    }

    ///t creates a new BMI085 instance from uninitialized pins
    pub fn new_from_pins<SpiPeri: spi::Instance>(
        peri: Peri<'a, SpiPeri>,
        sck_pin: Peri<'a, impl SckPin<SpiPeri>>,
        mosi_pin: Peri<'a, impl MosiPin<SpiPeri>>,
        miso_pin: Peri<'a, impl MisoPin<SpiPeri>>,
        tx_dma: Peri<'a, impl spi::TxDma<SpiPeri>>,
        rx_dma: Peri<'a, impl spi::RxDma<SpiPeri>>,
        spi_cs_pin: Peri<'a, AnyPin>,
        spi_buf: &'buf mut [u8; SPI_MIN_BUF_LEN],
    ) -> Self {
        let mut spi_config = spi::Config::default();
        // Bmi323 max SPI frequency is 10MHz
        spi_config.frequency = hz(1_000_000);

        let imu_spi = spi::Spi::new(
            peri, sck_pin, mosi_pin, miso_pin, tx_dma, rx_dma, spi_config,
        );

        let spi_cs = Output::new(spi_cs_pin, Level::High, Speed::VeryHigh);

        Bmi323 {
            spi: imu_spi,
            spi_cs,
            spi_buf,
            accel_mode: AccelMode::Disabled,
            accel_range: AccelRange::Range8g,
            accel_bw_mode: Bandwidth3DbCutoffFreq::AccOdrOver2,
            accel_odr: OutputDataRate::Odr100p0,
            accel_avg_window: DataAveragingWindow::NoFiltering,
            gyro_mode: GyroMode::Disabled,
            gyro_range: GyroRange::PlusMinus2000DegPerSec,
            gyro_bw_mode: Bandwidth3DbCutoffFreq::AccOdrOver2,
            gyro_odr: OutputDataRate::Odr100p0,
            gyro_avg_window: DataAveragingWindow::NoFiltering,
        }
    }

    fn select(&mut self) {
        self.spi_cs.set_low();
    }

    fn deselect(&mut self) {
        self.spi_cs.set_high();
    }

    async fn read(&mut self, reg: ImuRegisters) -> u16 {
        self.select();
        // addresses are 7 bits with MSB flagging read/!write
        // data is 16 bits

        self.spi_buf[0] = (reg as u8) | READ_BIT;
        self.spi_buf[1] = 0x00;
        let _ = self.spi.transfer_in_place(&mut self.spi_buf[..4]).await;

        self.deselect();

        ((self.spi_buf[3] as u16) << 8) | self.spi_buf[2] as u16
    }

    async fn burst_read(&mut self, reg: ImuRegisters, dest: &mut [u16]) {
        self.select();
        // the transaction length is either the dest buf size * 2 + 2
        // (the start addr + N data bytes) + 1 spurious byte for data to settle (or something)
        // OR upper bounded by internal length of the buffer.
        let trx_len = min((dest.len() * 2) + 2, self.spi_buf.len());

        self.spi_buf[0] = reg as u8 | READ_BIT;
        self.spi_buf[1] = 0x00;
        let _ = self
            .spi
            .transfer_in_place(&mut self.spi_buf[..trx_len])
            .await;

        for (i, dword) in dest.iter_mut().enumerate() {
            *dword = (self.spi_buf[(i * 2) + 2 + 1] as u16) << 8 | self.spi_buf[(i * 2) + 2] as u16;
        }

        self.deselect();
    }

    async fn write(&mut self, reg: ImuRegisters, val: u16) {
        self.select();

        self.spi_buf[0] = reg as u8;
        self.spi_buf[1] = (val & 0x00FF) as u8;
        self.spi_buf[2] = ((val & 0xFF00) >> 8) as u8;
        let _ = self.spi.transfer_in_place(&mut self.spi_buf[..3]).await;

        self.deselect();
    }

    #[allow(dead_code)]
    async fn burst_write(&mut self, reg: ImuRegisters, data: &[u16]) {
        self.select();

        // the transaction length is either the dest buf size * 2 + 2
        // (the start addr + N data bytes) + 1 spurious byte for data to settle (or something)
        // OR upper bounded by internal length of the buffer.
        let trx_len = min((data.len() * 2) + 1, self.spi_buf.len());

        self.spi_buf[0] = reg as u8 | READ_BIT;

        for (i, word) in data.iter().enumerate() {
            self.spi_buf[(i * 2) + 1] = (*word & 0x00FF_u16) as u8;
            self.spi_buf[(i * 2) + 1 + 1] = ((*word & 0xFF00_u16) >> 8) as u8;
        }

        let _ = self
            .spi
            .transfer_in_place(&mut self.spi_buf[..trx_len])
            .await;

        self.deselect();
    }

    pub async fn init(&mut self) {
        // device needs at least one dummy read to set the data mode to SPI from POn I2C
        // second read should be valid here for sanity purposes

        let _ = self.read(ImuRegisters::CHIP_ID).await;
        let _ = self.read(ImuRegisters::CHIP_ID).await;
    }

    async fn accel_self_test(&mut self) -> Result<(), ()> {
        defmt::warn!("BMI323 accel self test unimplemented");

        Ok(())
    }

    async fn gyro_self_test(&mut self) -> Result<(), ()> {
        defmt::warn!("BMI323 gyro self test unimplemented");

        Ok(())
    }

    pub async fn self_test(&mut self) -> Result<(), ()> {
        let mut has_self_test_error = Ok(());

        // self.deselect();
        // Timer::after_millis(10).await;
        // self.select();

        let _ = self.read(ImuRegisters::CHIP_ID).await;
        let chip_id = self.read(ImuRegisters::CHIP_ID).await & 0x00FF;
        if chip_id != BMI323_CHIP_ID {
            defmt::warn!(
                "read IMU ID (0x{:x}) does not match expected BMI323 ID (0x{:x})",
                chip_id,
                BMI323_CHIP_ID
            );
            has_self_test_error = Err(());
        } else {
            defmt::debug!("BMI323 id verified: 0x{:x}", chip_id);
        }

        if (self.accel_self_test().await).is_err() {
            has_self_test_error = Err(());
        }

        if (self.gyro_self_test().await).is_err() {
            has_self_test_error = Err(());
        }

        has_self_test_error
    }

    pub async fn accel_get_raw_data(&mut self) -> [i16; 3] {
        let mut buf: [u16; 3] = [0; 3];
        self.burst_read(ImuRegisters::ACC_DATA_X, &mut buf).await;

        [buf[0] as i16, buf[1] as i16, buf[2] as i16]
    }

    pub fn accel_range_scale(&self) -> f32 {
        match self.accel_range {
            AccelRange::Range2g => 2.0,
            AccelRange::Range4g => 4.0,
            AccelRange::Range8g => 8.0,
            AccelRange::Range16g => 16.0,
        }
    }

    pub fn convert_accel_raw_sample_g(&self, raw_sample: i16) -> f32 {
        (raw_sample as f32 / i16::MAX as f32) * self.accel_range_scale()
    }

    pub fn convert_accel_raw_sample_mps(&self, raw_sample: i16) -> f32 {
        self.convert_accel_raw_sample_g(raw_sample) * 9.80665
    }

    pub async fn accel_get_data_g(&mut self) -> [f32; 3] {
        let raw_data = self.accel_get_raw_data().await;

        [
            self.convert_accel_raw_sample_g(raw_data[0]),
            self.convert_accel_raw_sample_g(raw_data[1]),
            self.convert_accel_raw_sample_g(raw_data[2]),
        ]
    }

    pub async fn accel_get_data_mps(&mut self) -> [f32; 3] {
        let raw_data = self.accel_get_raw_data().await;

        [
            self.convert_accel_raw_sample_mps(raw_data[0]),
            self.convert_accel_raw_sample_mps(raw_data[1]),
            self.convert_accel_raw_sample_mps(raw_data[2]),
        ]
    }

    pub async fn apply_accel_config(&mut self) -> Result<(), ()> {
        let mut new_acc_conf_reg_val: u16 = 0x0000;
        new_acc_conf_reg_val |= (self.accel_mode as u16 & 0x07) << 12;
        new_acc_conf_reg_val |= (self.accel_avg_window as u16 & 0x07) << 8;
        new_acc_conf_reg_val |= (self.accel_bw_mode as u16 & 0x01) << 7;
        new_acc_conf_reg_val |= (self.accel_range as u16 & 0x07) << 4;
        new_acc_conf_reg_val |= self.accel_odr as u16 & 0x0F;

        self.write(ImuRegisters::ACC_CONF, new_acc_conf_reg_val)
            .await;

        let err_reg_val = self.read(ImuRegisters::ERR_REG).await;
        if err_reg_val & 0x0020 != 0 {
            defmt::error!("BMI323 accel config is invalid. Accel may be inop.");
            return Err(());
        }

        Ok(())
    }

    pub async fn set_accel_config(
        &mut self,
        accel_mode: AccelMode,
        accel_range: AccelRange,
        accel_bw_mode: Bandwidth3DbCutoffFreq,
        accel_odr: OutputDataRate,
        accel_avg_window: DataAveragingWindow,
    ) -> Result<(), ()> {
        self.accel_mode = accel_mode;
        self.accel_range = accel_range;
        self.accel_bw_mode = accel_bw_mode;
        self.accel_odr = accel_odr;
        self.accel_avg_window = accel_avg_window;

        self.apply_accel_config().await
    }

    pub async fn gyro_get_raw_data(&mut self) -> [i16; 3] {
        let mut buf: [u16; 3] = [0; 3];
        self.burst_read(ImuRegisters::GYR_DATA_X, &mut buf).await;

        [buf[0] as i16, buf[1] as i16, buf[2] as i16]
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

        [
            self.convert_raw_gyro_sample_dps(raw_data[0]),
            self.convert_raw_gyro_sample_dps(raw_data[1]),
            self.convert_raw_gyro_sample_dps(raw_data[2]),
        ]
    }

    pub async fn gyro_get_data_rads(&mut self) -> [f32; 3] {
        let raw_data = self.gyro_get_raw_data().await;

        [
            self.convert_raw_gyro_sample_rads(raw_data[0]),
            self.convert_raw_gyro_sample_rads(raw_data[1]),
            self.convert_raw_gyro_sample_rads(raw_data[2]),
        ]
    }

    pub async fn apply_gyro_config(&mut self) -> Result<(), ()> {
        let mut new_gyr_conf_reg_val: u16 = 0x0000;
        new_gyr_conf_reg_val |= (self.gyro_mode as u16 & 0x07) << 12;
        new_gyr_conf_reg_val |= (self.gyro_avg_window as u16 & 0x07) << 8;
        new_gyr_conf_reg_val |= (self.gyro_bw_mode as u16 & 0x01) << 7;
        new_gyr_conf_reg_val |= (self.gyro_range as u16 & 0x07) << 4;
        new_gyr_conf_reg_val |= self.gyro_odr as u16 & 0x0F;

        self.write(ImuRegisters::GYR_CONF, new_gyr_conf_reg_val)
            .await;

        let err_reg_val = self.read(ImuRegisters::ERR_REG).await;
        if err_reg_val & 0x0040 != 0 {
            defmt::error!("BMI323 gyro config is invalid. Gyro may be inop.");
            return Err(());
        }

        Ok(())
    }

    pub async fn set_gyro_config(
        &mut self,
        gyro_mode: GyroMode,
        gyro_range: GyroRange,
        gyro_bw_mode: Bandwidth3DbCutoffFreq,
        gyro_odr: OutputDataRate,
        gyro_avg_window: DataAveragingWindow,
    ) -> Result<(), ()> {
        self.gyro_mode = gyro_mode;
        self.gyro_range = gyro_range;
        self.gyro_bw_mode = gyro_bw_mode;
        self.gyro_odr = gyro_odr;
        self.gyro_avg_window = gyro_avg_window;

        self.apply_gyro_config().await
    }

    pub async fn set_gyro_interrupt_mode(&mut self, int_mode: InterruptMode) {
        // read the current int map 2 register
        let mut int_map2_reg_val = self.read(ImuRegisters::INT_MAP2).await;

        // clear the mapping for gyro int mode
        int_map2_reg_val &= !(0x3 << 8);
        // set the new mapping for gyro
        int_map2_reg_val |= (int_mode as u16 & 0x03) << 8;

        // write the modified register value
        self.write(ImuRegisters::INT_MAP2, int_map2_reg_val).await;
    }

    pub async fn set_accel_interrupt_mode(&mut self, int_mode: InterruptMode) {
        // read the current int map 2 register
        let mut int_map2_reg_val = self.read(ImuRegisters::INT_MAP2).await;

        // clear the mapping for accel int mode
        int_map2_reg_val &= !(0x3 << 10);
        // set the new mapping for accel
        int_map2_reg_val |= (int_mode as u16 & 0x03) << 10;

        // write the modified register value
        self.write(ImuRegisters::INT_MAP2, int_map2_reg_val).await;
    }

    pub async fn set_int1_pin_config(
        &mut self,
        pin_level: IntPinLevel,
        pin_drive_mode: IntPinDriveMode,
    ) {
        let mut io_int_ctrl_reg_val: u16 = self.read(ImuRegisters::IO_INT_CTRL).await;

        // clear the config for int1 pin
        io_int_ctrl_reg_val &= !(0x3);
        // set the new mapping for accel
        io_int_ctrl_reg_val |= pin_level as u16 & 0x1;
        io_int_ctrl_reg_val |= (pin_drive_mode as u16 & 0x1) << 1;

        self.write(ImuRegisters::IO_INT_CTRL, io_int_ctrl_reg_val)
            .await;
    }

    pub async fn set_int2_pin_config(
        &mut self,
        pin_level: IntPinLevel,
        pin_drive_mode: IntPinDriveMode,
    ) {
        let mut io_int_ctrl_reg_val: u16 = self.read(ImuRegisters::IO_INT_CTRL).await;

        // clear the config for int1 pin
        io_int_ctrl_reg_val &= !(0x3 << 8);
        // set the new mapping for accel
        io_int_ctrl_reg_val |= (pin_level as u16 & 0x1) << 8;
        io_int_ctrl_reg_val |= (pin_drive_mode as u16 & 0x1) << 9;

        self.write(ImuRegisters::IO_INT_CTRL, io_int_ctrl_reg_val)
            .await;
    }

    pub async fn set_int1_enabled(&mut self, enabled: bool) {
        let mut io_int_ctrl_reg_val: u16 = self.read(ImuRegisters::IO_INT_CTRL).await;

        // clear the config for int1 pin
        io_int_ctrl_reg_val &= !(0x1 << 2);
        // set the new mapping for accel
        io_int_ctrl_reg_val |= (enabled as u16 & 0x1) << 2;

        self.write(ImuRegisters::IO_INT_CTRL, io_int_ctrl_reg_val)
            .await;
    }

    pub async fn set_int2_enabled(&mut self, enabled: bool) {
        let mut io_int_ctrl_reg_val: u16 = self.read(ImuRegisters::IO_INT_CTRL).await;

        // clear the config for int1 pin
        io_int_ctrl_reg_val &= !(0x1 << 10);
        // set the new mapping for accel
        io_int_ctrl_reg_val |= (enabled as u16 & 0x1) << 10;

        self.write(ImuRegisters::IO_INT_CTRL, io_int_ctrl_reg_val)
            .await;
    }
}
