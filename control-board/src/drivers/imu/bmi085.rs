use embassy_stm32::{
    gpio::{Pin, Level},
    spi::{self, SckPin, MosiPin, MisoPin},
    Peripheral,
    time::hz, gpio::{Output, Speed}
};

use defmt::*;

pub struct Bmi085<'a, 'buf, T: spi::Instance, TxDmaCh: embassy_stm32::spi::TxDma<T>, RxDmaCh: embassy_stm32::spi::RxDma<T>, AccelCsPin: Pin, GyroCsPin: Pin> {
    spi: spi::Spi<'a, T, TxDmaCh, RxDmaCh>,
    accel_cs: Output<'a, AccelCsPin>,
    gyro_cs: Output<'a, GyroCsPin>,
    spi_buf: &'buf mut [u8; 4],
}

#[repr(u8)]
#[allow(non_camel_case_types)]
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

impl Into<u8> for AccelRegisters {
    fn into(self) -> u8 {
        return self as u8;
    }
}

const ACCEL_CHIP_ID: u8 = 0x1F;

enum GyroRegisters {

}

const GYRO_CHIP_ID: u8 = 0x0F;

impl<'a, 'buf, T: spi::Instance, TxDmaCh: embassy_stm32::spi::TxDma<T>, RxDmaCh: embassy_stm32::spi::RxDma<T>, AccelCsPin: Pin, GyroCsPin: Pin> Bmi085<'a, 'buf, T, TxDmaCh, RxDmaCh, AccelCsPin, GyroCsPin> {
    pub fn new_from_spi(
        spi: spi::Spi<'a, T, TxDmaCh, RxDmaCh>, 
        accel_cs: Output<'a, AccelCsPin>, 
        gyro_cs: Output<'a, GyroCsPin>,
        spi_buf: &'buf mut [u8; 4],
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
        spi_buf: &'buf mut [u8; 4],
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

    fn select_accel(&self) {
        self.gyro_cs.set_high();
        self.accel_cs.set_low();
    }

    fn select_gyro(&self) {
        self.accel_cs.set_high();
        self.gyro_cs.set_low();
    }

    fn deselect(&self) {
        self.accel_cs.set_high();
        self.gyro_cs.set_low();
    }

    fn read(&mut self, reg: u8) -> u8 {
        0
    }

    fn read_accel(&mut self, reg: AccelRegisters) -> u8 {
        self.select_accel();
        return self.read(reg as u8);
    }

    fn read_gyro(&mut self, reg: GyroRegisters) -> u8 {
        self.select_gyro();
        return self.read(reg as u8);
    }

    fn write(&mut self, reg: u8, val: u8) {

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

        self.spi_buf[0] = AccelRegisters::ACC_CHIP_ID as u8;
        self.select_accel();
        let _ = self.spi.transfer_in_place(self.spi_buf).await;
        let accel_id = self.spi_buf[1];
        debug!("accelerometer id: 0x{:x}", accel_id);

        SPI6_BUF[0] = 0x80;
        imu_cs2.set_low();
        let _ = imu_spi.transfer_in_place(&mut SPI6_BUF[0..2]).await;
        imu_cs2.set_high();
        let gyro_id = SPI6_BUF[1];
        debug!("gyro id: 0x{:x}", gyro_id);

        true
    }

}