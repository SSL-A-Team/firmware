use embassy_stm32::{
    gpio::{Level, Output, Pin, Speed}, 
    mode::Async,
    spi::{self, Error}
};

pub struct AT25DF041B<'buf, T: spi::Instance, const CS_POL_N: bool> {
    spi: spi::Spi<'static, T, Async>,
    chip_select: Output<'static>,
    tx_buf: &'buf mut [u8; 256],
    rx_buf: &'buf mut [u8; 256],
}

impl<'buf, T: spi::Instance, const CS_POL_N: bool> AT25DF041B<'buf, T, CS_POL_N> {
    pub fn new(spi: spi::Spi<'static, T, Async>,
        chip_select: impl Pin,
        tx_buf: &'buf mut [u8; 256],
        rx_buf: &'buf mut [u8; 256]) -> AT25DF041B<'buf, T, CS_POL_N> {
        AT25DF041B {
            spi,
            chip_select: Output::new(chip_select, Level::High, Speed::High),
            tx_buf,
            rx_buf,
        }
    }

    fn select(&mut self) {
        if CS_POL_N {
            self.chip_select.set_low();
        } else {
            self.chip_select.set_high();
        }
    }

    fn deselect(&mut self) {
        if CS_POL_N {
            self.chip_select.set_high();
        } else {
            self.chip_select.set_low();
        }
    }

    async fn transfer(&mut self, len: usize) -> Result<(), Error> {
        let len = if len > 256 { 256 } else { len };

        let res = self.spi.transfer(&mut self.rx_buf[0..len], self.tx_buf).await;
        return res;
    }

    pub async fn verify_chip_id(&mut self) -> Result<(), ()> {
        self.select();

        self.rx_buf[0] = 0x9F;
        self.select();
        let res = self.transfer(5).await; // send cmd byte, get 4 back
        self.deselect();
        if res.is_err() {
            return Err(());
        }

        // let mfg_id = self.rx_buf[1];
        // let dev_id_upper = self.rx_buf[2];
        // let dev_id_lower = self.rx_buf[3];
        // let dev_edis_len = self.rx_buf[4];

        let expected_mfg_info: [u8; 4] = [0x1F, 0x44, 0x02, 0x00];
        let mut received_mfg_info: [u8; 4] = [0; 4];
        received_mfg_info.copy_from_slice(&self.rx_buf[1..5]);

        if received_mfg_info != expected_mfg_info {
            defmt::error!("AT25DF041B manufacturer information did not match datasheet.");
            defmt::debug!("expected {}, got {}", expected_mfg_info, received_mfg_info);
            return Err(())
        }

        return Ok(())
    }
}