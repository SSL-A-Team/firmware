use embassy_stm32::{
    gpio::{Pin, Level},
    spi::{self, SckPin, MosiPin, MisoPin},
    Peripheral,
    time::hz, gpio::{Output, Speed}
};

pub struct Bmi085<'a, T: spi::Instance, TxDmaCh, RxDmaCh, AccelCsPin: Pin, GyroCsPin: Pin> {
    spi: spi::Spi<'a, T, TxDmaCh, RxDmaCh>,
    accel_cs: Output<'a, AccelCsPin>,
    gyro_cs: Output<'a, GyroCsPin>,
}

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

enum GyroRegisters {

}

impl<'a, T: spi::Instance, TxDmaCh, RxDmaCh, AccelCsPin: Pin, GyroCsPin: Pin> Bmi085<'a, T, TxDmaCh, RxDmaCh, AccelCsPin, GyroCsPin> {
    pub fn new_from_spi(
        spi: spi::Spi<'a, T, TxDmaCh, RxDmaCh>, 
        accel_cs: Output<'a, AccelCsPin>, 
        gyro_cs: Output<'a, GyroCsPin>
    ) -> Self {
        Bmi085 {
            spi: spi,
            accel_cs: accel_cs,
            gyro_cs: gyro_cs 
        }
    }

    pub fn new_from_pins(
        peri: impl Peripheral<P = T> + 'a,
        sck: impl Peripheral<P = impl SckPin<T>> + 'a,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'a,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'a,
        txdma: impl Peripheral<P = TxDmaCh> + 'a,
        rxdma: impl Peripheral<P = RxDmaCh> + 'a,
        cs1_pin: impl Peripheral<P = AccelCsPin> + 'a,
        cs2_pin: impl Peripheral<P = GyroCsPin> + 'a,
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

        let accel_cs = Output::new(cs1_pin, Level::High, Speed::VeryHigh);
        let imu_cs = Output::new(cs2_pin, Level::High, Speed::VeryHigh);

        Bmi085 { 
            spi: imu_spi,
            accel_cs: accel_cs,
            gyro_cs: imu_cs,
        }
    }

    fn self_test(&self) {
        
    }

}