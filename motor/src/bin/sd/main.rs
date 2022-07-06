#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_sdmmc::{Controller, Mode, VolumeIdx};
use panic_halt as _;
use stm32h7xx_hal::{gpio::Speed, pac, prelude::*};

// This is just a placeholder TimeSource. In a real world application
// one would probably use the RTC to provide time.
pub struct TimeSource;

impl embedded_sdmmc::TimeSource for TimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz())
        .pll1_q_ck(200.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

    let mut led_g = gpiob.pb0.into_push_pull_output();
    led_g.set_low();

    let clk = gpioc
        .pc12
        .into_alternate()
        .internal_pull_up(false)
        .speed(Speed::VeryHigh);
    let cmd = gpiod
        .pd2
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d0 = gpioc
        .pc8
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d1 = gpioc
        .pc9
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d2 = gpioc
        .pc10
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d3 = gpioc
        .pc11
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);

    // let mut sdmmc: Sdmmc<_, SdCard> = dp.SDMMC1.sdmmc(
    //     (clk, cmd, d0, d1, d2, d3),
    //     ccdr.peripheral.SDMMC1,
    //     &ccdr.clocks,
    // );

    let mut sd_fatfs = Controller::new(
        dp.SDMMC1
            .sdmmc(
                (clk, cmd, d0, d1, d2, d3),
                ccdr.peripheral.SDMMC1,
                &ccdr.clocks,
            )
            .sdmmc_block_device(),
        TimeSource,
    );

    let mut sd_fatfs_volume = sd_fatfs.get_volume(VolumeIdx(0)).unwrap();
    let sd_fatfs_root_dir = sd_fatfs.open_root_dir(&sd_fatfs_volume).unwrap();
    let mut f = sd_fatfs
        .open_file_in_dir(
            &mut sd_fatfs_volume,
            &sd_fatfs_root_dir,
            "file.txt",
            Mode::ReadWriteCreate,
        )
        .unwrap();

    let buffer1 = b"\nFile Test\n";
    sd_fatfs
        .write(&mut sd_fatfs_volume, &mut f, buffer1)
        .unwrap();

    led_g.set_high();

    loop {}
}
