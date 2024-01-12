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

    

}