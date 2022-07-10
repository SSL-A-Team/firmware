use crate::block_us;
use stm32h7xx_hal::block;

pub struct StmBootloader<'a, SERIAL> {
    serial: &'a mut SERIAL,
}

impl<'a, SERIAL> StmBootloader<'a, SERIAL>
where
    SERIAL: embedded_hal::serial::Read<u8> + embedded_hal::serial::Write<u8>,
{
    fn check_ack(serial: &mut SERIAL) -> Result<(), ()> {
        if block_us!(serial.read(), 1000).or(Err(()))? != 0x79 {
            return Err(());
        }
        Ok(())
    }

    fn write_32b(serial: &mut SERIAL, data: u32) -> Result<(), ()> {
        let data: [u8; 4] = data.to_be_bytes();
        for byte in data {
            block!(serial.write(byte)).or(Err(()))?;
        }
        let cks = data[0] ^ data[1] ^ data[2] ^ data[3];
        block!(serial.write(cks)).or(Err(()))?;

        Ok(())
    }

    pub fn new(serial: &'a mut SERIAL) -> Result<StmBootloader<'a, SERIAL>, ()> {
        block!(serial.write(0x7F)).or(Err(()))?;
        if block_us!(serial.read(), 1000).or(Err(()))? != 0x0 {
            return Err(());
        }
        Self::check_ack(serial)?;
        Ok(StmBootloader { serial })
    }

    pub fn get_version(&mut self) -> Result<u8, ()> {
        block!(self.serial.write(0x01)).or(Err(()))?;
        block!(self.serial.write(!0x01)).or(Err(()))?;
        Self::check_ack(self.serial)?;
        let version = block_us!(self.serial.read(), 1000).or(Err(()))?;
        block_us!(self.serial.read(), 1000).or(Err(()))?;
        block_us!(self.serial.read(), 1000).or(Err(()))?;
        Self::check_ack(self.serial)?;

        Ok(version)
    }

    pub fn get_id(&mut self) -> Result<u16, ()> {
        block!(self.serial.write(0x02)).or(Err(()))?;
        block!(self.serial.write(!0x02)).or(Err(()))?;
        Self::check_ack(self.serial)?;
        if block_us!(self.serial.read(), 1000).or(Err(()))? != 0x1 {
            return Err(());
        }
        let b0 = block_us!(self.serial.read(), 1000).or(Err(()))? as u16;
        let b1 = block_us!(self.serial.read(), 1000).or(Err(()))? as u16;
        Self::check_ack(self.serial)?;

        return Ok(b1 + (b0 << 8));
    }

    pub fn write(&mut self, address: u32, data: &[u8]) -> Result<(), ()> {
        if data.len() > 256 {
            return Err(());
        }
        block!(self.serial.write(0x31)).or(Err(()))?;
        block!(self.serial.write(!0x31)).or(Err(()))?;
        Self::check_ack(self.serial)?;
        Self::write_32b(self.serial, address)?;
        Self::check_ack(self.serial)?;
        let n = (data.len() - 1) as u8;
        block!(self.serial.write(n)).or(Err(()))?;
        let mut cks = n;
        for &byte in data {
            cks ^= byte;
            block!(self.serial.write(byte)).or(Err(()))?;
        }
        block!(self.serial.write(cks)).or(Err(()))?;
        // TODO: this has no timeout
        if block!(self.serial.read()).or(Err(()))? != 0x79 {
            return Err(());
        }

        Ok(())
    }

    pub fn write_flash(&mut self, data: &[u8]) -> Result<(), ()> {
        let mut addr = 0x0800_0000;
        for start in (0..data.len()).step_by(256) {
            let end = core::cmp::min(start + 256, data.len());
            self.write(addr, &data[start..end])?;
            addr += 256;
        }
        Ok(())
    }

    pub fn erase_all(&mut self) -> Result<(), ()> {
        block!(self.serial.write(0x44)).or(Err(())).unwrap();
        block!(self.serial.write(!0x44)).or(Err(())).unwrap();
        Self::check_ack(self.serial)?;
        block!(self.serial.write(0xFF)).or(Err(())).unwrap();
        block!(self.serial.write(0xFF)).or(Err(())).unwrap();
        block!(self.serial.write(0x00)).or(Err(())).unwrap();
        // TODO: this has no timeout
        if block!(self.serial.read()).or(Err(()))? != 0x79 {
            return Err(());
        }

        Ok(())
    }
}
