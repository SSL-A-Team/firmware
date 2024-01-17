/*
 * Driver for the Bosch BMI085 IMU.
 * 
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi085-ds001.pdf
 * 
 */


use embassy_stm32::{
    gpio::{Pin, Level},
    spi::{self, SckPin, MosiPin, MisoPin},
    Peripheral,
    time::hz, gpio::{Output, Speed}
};

use defmt::*;
use embassy_time::{Timer, Duration};

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

#[repr(u8)]
#[allow(dead_code)]
enum GyroSelfTestReg {
    RateOk = 0b10000,
    BistFail = 0b00100,
    BistRdy = 0b00010,
    TrigBist = 0b00001,
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

    async fn accel_self_test(&mut self) -> bool {
        warn!("accel BIST is unimplemented");

        true
    }

    async fn gyro_self_test(&mut self) -> bool {
        self.gyro_write(GyroRegisters::GYRO_SELF_TEST, GyroSelfTestReg::TrigBist as u8).await;
        let mut try_ct = 0;
        loop {
            let strreg_val = self.gyro_read(GyroRegisters::GYRO_SELF_TEST).await;
            if (strreg_val & GyroSelfTestReg::BistRdy as u8) != 0 {
                if (strreg_val & GyroSelfTestReg::BistFail as u8) != 0 {
                    warn!("gyro failed BIST. (the bist fail bit is high)");
                    return false;
                } else {
                    debug!("gyro passed self test.");
                    return true;
                }
            }

            Timer::after(Duration::from_millis(10)).await;
            try_ct += 1;

            if try_ct > 10 {
                warn!("gyro BIST did not converge.");
                return false;
            }
        }
    }

    pub async fn self_test(&mut self) -> bool {
        let mut has_self_test_error = false;

        self.deselect();

        let _ = self.accel_read(AccelRegisters::ACC_CHIP_ID).await;
        let acc_chip_id = self.accel_read(AccelRegisters::ACC_CHIP_ID).await;
        if acc_chip_id != ACCEL_CHIP_ID {
            warn!("read accel ID (0x{:x}) does not match expected BMI085 accel ID (0x{:x})", acc_chip_id, ACCEL_CHIP_ID);
            has_self_test_error = true;
        } else {
            debug!("accel id verified: 0x{:x}", acc_chip_id);
        }

        let _ = self.gyro_read(GyroRegisters::GYRO_CHIP_ID).await;
        let gyro_chip_id = self.gyro_read(GyroRegisters::GYRO_CHIP_ID).await;
        if gyro_chip_id != GYRO_CHIP_ID {
            warn!("read gyro ID (0x{:x}) does not match expected BMI085 gyro ID (0x{:x})", gyro_chip_id, GYRO_CHIP_ID);
            has_self_test_error = true;
        } else {
            debug!("gyro id verified: 0x{:x}", gyro_chip_id);
        }
        
        if !self.accel_self_test().await {
            has_self_test_error = true;
        }

        if !self.gyro_self_test().await {
            has_self_test_error = true;
        }

        has_self_test_error
    }

    const fn read_pair_to_i16(&self, lsb: u8, msb: u8) -> i16 {
        (msb as u16 * 256 + lsb as u16) as i16
    }
    
    pub async fn get_accel_data(&mut self) -> [i16; 3] {
        let mut buf: [u8; 6] = [0; 6];
        self.accel_burst_read(AccelRegisters::ACC_X_LSB, &mut buf).await;

        [self.read_pair_to_i16(buf[0], buf[1]),
            self.read_pair_to_i16(buf[2], buf[3]),
            self.read_pair_to_i16(buf[4], buf[5])]
    }

    pub async fn get_gyro_data(&mut self) -> [i16; 3] {
        let mut buf: [u8; 6] = [0; 6];
        self.gyro_burst_read(GyroRegisters::RATE_X_LSB, &mut buf).await;

        [self.read_pair_to_i16(buf[0], buf[1]),
            self.read_pair_to_i16(buf[2], buf[3]),
            self.read_pair_to_i16(buf[4], buf[5])]
    }


}