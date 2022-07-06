#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32h7xx_hal::{pac, prelude::*, rcc::rec::UsbClkSel};

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc
        .sys_ck(200.MHz())
        .pll1_q_ck(200.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::HSI48);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let mut led_g = gpiob.pb0.into_push_pull_output();
    let (pin_dm, pin_dp) = { (gpioa.pa11.into_alternate(), gpioa.pa12.into_alternate()) };

    let usb = stm32h7xx_hal::usb_hs::USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        pin_dm,
        pin_dp,
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );

    let usb_bus = stm32h7xx_hal::usb_hs::UsbBus::new(usb, unsafe { &mut EP_MEMORY });
    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = usb_device::device::UsbDeviceBuilder::new(
        &usb_bus,
        usb_device::device::UsbVidPid(0x16c0, 0x27dd),
    )
    .manufacturer("Fake company")
    .product("Serial port")
    .serial_number("TEST")
    .device_class(usbd_serial::USB_CLASS_CDC)
    .build();

    let mut delay = cp.SYST.delay(ccdr.clocks);
    // let buffer1 = b"USB test\n";
    // loop {
    //     if !usb_dev.poll(&mut [&mut serial]) {
    //         continue;
    //     }

    //     let mut buf = [0u8; 64];

    //     match serial.read(&mut buf) {
    //         _ => {}
    //     }
        
    //     // serial.write(buffer1).unwrap();
    //     led_g.toggle();
    //     delay.delay_ms(1000u32);
    // }

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    // if 0x61 <= *c && *c <= 0x7a {
                    //     *c &= !0x20;
                    // }
                    *c = b'G';
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    //     delay.delay_ms(1000u32);
    }
}
