use embassy_stm32::{
    gpio::{Pin, Level},
    spi::{self, SckPin, MosiPin, MisoPin},
    Peripheral,
    time::hz, gpio::{Output, Speed}
};

use defmt::*;

use core::cmp::min;

pub struct Bmi085<
        'a,
        'buf,
        T: spi::Instance,
        TxDmaCh: embassy_stm32::spi::TxDma<T>,
        RxDmaCh: embassy_stm32::spi::RxDma<T>,
        AccelCsPin: Pin,
        GyroCsPin: Pin> {
    spi: spi::Spi<'a, T, TxDmaCh, RxDmaCh>,
    accel_cs: Output<'a, AccelCsPin>,
    gyro_cs: Output<'a, GyroCsPin>,
    spi_buf: &'buf mut [u8; 7],
}

#[repr(u8)]
#[allow(non_camel_case_types, dead_code)]
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

const ACCEL_CHIP_ID: u8 = 0x1F;

#[repr(u8)]
#[allow(non_camel_case_types, dead_code)]
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

const GYRO_CHIP_ID: u8 = 0x0F;

const READ_BIT: u8 = 0x80;

impl<'a,
        'buf,
        T: spi::Instance,
        TxDmaCh: embassy_stm32::spi::TxDma<T>,
        RxDmaCh: embassy_stm32::spi::RxDma<T>,
        AccelCsPin: Pin,
        GyroCsPin: Pin> 
    Bmi085<'a, 'buf, T, TxDmaCh, RxDmaCh, AccelCsPin, GyroCsPin> {
    pub fn new_from_spi(
        spi: spi::Spi<'a, T, TxDmaCh, RxDmaCh>, 
        accel_cs: Output<'a, AccelCsPin>, 
        gyro_cs: Output<'a, GyroCsPin>,
        spi_buf: &'buf mut [u8; 7],
    ) -> Self {
        Bmi085 {
            spi: spi,
            accel_cs: accel_cs,
            gyro_cs: gyro_cs,
            spi_buf: spi_buf,
        }
    }

    pub fn new_from_pins(
        peri: impl Peripheral<P = T> + 'a,
        sck: impl Peripheral<P = impl SckPin<T>> + 'a,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'a,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'a,
        txdma: impl Peripheral<P = TxDmaCh> + 'a,
        rxdma: impl Peripheral<P = RxDmaCh> + 'a,
        accel_cs: impl Peripheral<P = AccelCsPin> + 'a,
        gyro_cs: impl Peripheral<P = GyroCsPin> + 'a,
        spi_buf: &'buf mut [u8; 7],
    ) -> Self {
        let imu_spi = spi::Spi::new(
            peri,
            sck,
            mosi,
            miso,
            txdma,
            rxdma,
            hz(1_000_000),
            spi::Config::default(),
        );

        let accel_cs = Output::new(accel_cs, Level::High, Speed::VeryHigh);
        let imu_cs = Output::new(gyro_cs, Level::High, Speed::VeryHigh);

        Bmi085 { 
            spi: imu_spi,
            accel_cs: accel_cs,
            gyro_cs: imu_cs,
            spi_buf: spi_buf,
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

    async fn burst_read(&mut self, reg: u8, dest: &mut [u8]) {
        // the transaction length is either the dest buf size + 1 
        // (the start addr + N data bytes)
        // OR upper bounded by internal length of the buffer.
        let trx_len = min(dest.len() + 1, self.spi_buf.len());

        self.spi_buf[0] = reg | READ_BIT;
        let _ = self.spi.transfer_in_place(&mut self.spi_buf[..trx_len]).await;
        dest[1..trx_len].copy_from_slice(&self.spi_buf[1..trx_len]);
    }

    async fn read_accel(&mut self, reg: AccelRegisters) -> u8 {
        self.select_accel();
        self.read(reg as u8).await
    }

    async fn read_gyro(&mut self, reg: GyroRegisters) -> u8 {
        self.select_gyro();
        self.read(reg as u8).await
    }

    async fn write(&mut self, reg: u8, val: u8) {
        self.spi_buf[0] = reg & !READ_BIT;
        self.spi_buf[1] = val;
        let _ = self.spi.transfer_in_place(&mut self.spi_buf[..2]).await;
    }

    fn write_accel(&mut self, reg: AccelRegisters, val: u8) {
        self.select_accel();
        self.write(reg as u8, val);
    }

    fn write_gyro(&mut self, reg: GyroRegisters, val: u8) {
        self.select_gyro();
        self.write(reg as u8, val);
    }

    pub async fn self_test(&mut self) -> bool {
        let mut has_self_test_error = false;

        self.deselect();

        let acc_chip_id = self.read_accel(AccelRegisters::ACC_CHIP_ID).await;
        if acc_chip_id != ACCEL_CHIP_ID {
            warn!("read accel ID (0x{:x}) does not match expected BMI085 accel ID (0x{:x})", acc_chip_id, ACCEL_CHIP_ID);
            has_self_test_error = true;
        } else {
            debug!("accel id verified: 0x{:x}", acc_chip_id);
        }

        let gyro_chip_id = self.read_gyro(GyroRegisters::GYRO_CHIP_ID).await;
        if gyro_chip_id != GYRO_CHIP_ID {
            warn!("read gyro ID (0x{:x}) does not match expected BMI085 gyro ID (0x{:x})", gyro_chip_id, GYRO_CHIP_ID);
            has_self_test_error = true;
        } else {
            debug!("gyro id verified: 0x{:x}", gyro_chip_id);
        }
        
        // TODO: accel BIST
        // TODO: gyro BIST

        has_self_test_error
    }

}