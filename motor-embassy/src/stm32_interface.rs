use core::cmp::min;
use core::future::Future;

use defmt_rtt as _;
use defmt::*;

use embassy_stm32::gpio::{Output, OutputOpenDrain, Pin};
use embassy_stm32::usart;
use embassy_time::{Duration, Timer};

use crate::uart_queue::{Reader, Writer, UartReadQueue, UartWriteQueue};

const STM32_BOOTLOADER_ACK: u8 = 0x79;
const STM32_BOOTLOADER_NACK: u8 = 0x1F;
const STM32_BOOTLOADER_CODE_SEQUENCE_BYTE: u8 = 0x7F;

const STM32_BOOTLOADER_CMD_GET: u8 = 0x00;
const STM32_BOOTLOADER_CMD_GET_VERSION: u8 = 0x01;
const STM32_BOOTLOADER_CMD_GET_ID: u8 = 0x02;
const STM32_BOOTLOADER_CMD_READ_MEM: u8 = 0x11;
const STM32_BOOTLOADER_CMD_GO: u8 = 0x21;
const STM32_BOOTLOADER_CMD_WRITE_MEM: u8 = 0x31;
const STM32_BOOTLOADER_CMD_ERASE: u8 = 0x43;
const STM32_BOOTLOADER_CMD_EXTEMDED_ERASE: u8 = 0x44;
const STM32_BOOTLOADER_CMD_WRITE_PROT: u8 = 0x63;
const STM32_BOOTLOADER_CMD_WRITE_UNPROT: u8 = 0x73;
const STM32_BOOTLOADER_CMD_READ_PROT: u8 = 0x82;
const STM32_BOOTLOADER_CMD_READ_UNPROT: u8 = 0x92;
const STM32_BOOTLOADER_CMD_GET_CHECKSUM: u8 = 0xA1;

pub struct Stm32Interface<
        'a,
        // R: Reader<'a>,
        // W: Writer<'a>, 
        UART: usart::BasicInstance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize, 
        const DEPTH_TX: usize,
        Boot0Pin: Pin, 
        ResetPin: Pin
> {
    reader: &'a UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_TX>,
    writer: &'a UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_RX>,
    // reader: R,
    // writer: W,
    boot0_pin: Option<Output<'a, Boot0Pin>>,
    reset_pin: Option<OutputOpenDrain<'a, ResetPin>>,

    in_bootloader: bool,
}

impl<
        'a,
        // R: Reader<'a>,
        // W: Writer<'a>, 
        UART: usart::BasicInstance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize, 
        const DEPTH_TX: usize,
        Boot0Pin: Pin, 
        ResetPin: Pin
    > Stm32Interface<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, Boot0Pin, ResetPin>
{
    pub fn new(        
        read_queue: &'a UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_TX>,
        write_queue: &'a UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_RX>,
        mut boot0_pin: Option<Output<'a, Boot0Pin>>,
        mut reset_pin: Option<OutputOpenDrain<'a, ResetPin>>
    ) -> Stm32Interface<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, Boot0Pin, ResetPin> {
        if boot0_pin.is_some() {
            boot0_pin.as_mut().unwrap().set_low();
        }

        if reset_pin.is_some() {
            reset_pin.as_mut().unwrap().set_high();
        }
        
        Stm32Interface {
            reader: read_queue,
            writer: write_queue,
            boot0_pin,
            reset_pin,
            in_bootloader: false,
        }
    }

    pub async fn soft_reset(&self) {
        defmt::panic!("implement soft reset if needed.");
    }

    pub async fn hard_reset(&mut self) -> Result<(), ()> {
        if self.reset_pin.is_none() {
            return Err(());
        }

        self.reset_pin.as_mut().unwrap().set_low();
        Timer::after(Duration::from_micros(100)).await;
        self.reset_pin.as_mut().unwrap().set_high();
        Timer::after(Duration::from_micros(100)).await;

        Ok(())
    }

    pub async fn reset_into_bootloader(&mut self) -> Result<(), ()> {
        if self.boot0_pin.is_none() || self.reset_pin.is_none() {
            return Err(());
        }

        // set the boot0 line high to enter the UART bootloader upon reset
        self.boot0_pin.as_mut().unwrap().set_high();

        // reset the device
        self.hard_reset().await?;

        Timer::after(Duration::from_millis(10)).await;

        defmt::debug!("sending the bootloader baud calibration command...");
        self.writer
        .write(|buf| {
            buf[0] = STM32_BOOTLOADER_CODE_SEQUENCE_BYTE;
            1
        })
        .await?;

        let mut res = Err(());
        self.reader.read(|buf| {
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    defmt::info!("bootloader replied with ACK after calibration.");
                    self.in_bootloader = true;
                    res = Ok(());
                } else {
                    defmt::error!("bootloader replied with NACK after calibration.");
                }
            }
        }).await?;

        res
    }

    pub async fn reset_into_program(&mut self) -> Result<(), ()> {
        if self.boot0_pin.is_none() || self.reset_pin.is_none() {
            return Err(());
        }

        // set the boot0 line low to disable startup bootloader
        self.boot0_pin.as_mut().unwrap().set_low();
        Timer::after(Duration::from_millis(5)).await;

        // reset the device
        self.hard_reset().await?;

        Ok(())
    }

    ////////////////////////////////////////////////
    //  BOOTLOADER FUNCTIONS                      //
    //                                            //
    //  async traits aren't support *yet*         //
    //  TODO: move this to a trait when they are  //
    ////////////////////////////////////////////////

    fn bootloader_checksum_u32(word: u32) -> u8 {
        let word_b: [u8; 4] = word.to_be_bytes();
        let cks = word_b[0] ^ word_b[1] ^ word_b[2] ^ word_b[3];
        cks
    }

    fn bootloader_checksum_buf(buf: &[u8]) -> u8 {
        let mut cks = buf.len() as u8;
        for &byte in buf {
            cks ^= byte;
        }

        cks
    }

    pub async fn verify_bootloader(&self) -> Result<(), ()> {
        if !self.in_bootloader {
            defmt::error!("called bootloader operation when not in bootloader context.");
            return Err(());
        }

        self.writer
        .write(|buf| {
            buf[0] = STM32_BOOTLOADER_CMD_GET;
            buf[1] = !STM32_BOOTLOADER_CMD_GET;
            2
        })
        .await?;

        let mut res = Err(());
        self.reader.read(|buf| {
            // TODO discovery and return checksum capability and erase/ext erase capability
            if buf.len() == 1 && buf[0] == STM32_BOOTLOADER_NACK {
                defmt::error!("failed to read bootloader command list. (NACK).");
                res = Err(());
            } else if buf.len() == 15 && buf[0] == STM32_BOOTLOADER_ACK && buf[14] == STM32_BOOTLOADER_ACK {
                defmt::debug!("read bootloader command list. (NO AUTO CS).");
                res = Ok(());
            } else if buf.len() == 16 && buf[0] == STM32_BOOTLOADER_ACK && buf[15] == STM32_BOOTLOADER_ACK {
                defmt::debug!("read bootloader command list. (WITH CS).");
                res = Ok(());
            } else {
                defmt::error!("unknown command enumeration error.");
                res = Err(());
            }
        }).await?;

        res
    }

    pub async fn get_version(&self) -> Result<u8, ()> {
        defmt::panic!("implement get_version if needed.");
    }

    pub async fn get_device_id(&self) -> Result<u16, ()> {
        if !self.in_bootloader {
            defmt::error!("called bootloader operation when not in bootloader context.");
            return Err(());
        }

        self.writer
        .write(|buf| {
            buf[0] = STM32_BOOTLOADER_CMD_GET_ID;
            buf[1] = !STM32_BOOTLOADER_CMD_GET_ID;
            2
        })
        .await?;

        let mut res = Err(());
        self.reader.read(|buf| {
            if buf.len() == 1 && buf[0] == STM32_BOOTLOADER_NACK {
                defmt::error!("could not read device ID.");
                res = Err(());
            } else if buf.len() == 5 && buf[1] == 1 && buf[4] == STM32_BOOTLOADER_ACK {
                let pid: u16 = buf[3] as u16;
                defmt::info!("found 1 byte pid {:?}", pid);
                res = Ok(pid);
            } else if buf.len() == 5 && buf[1] == 2 && buf[4] == STM32_BOOTLOADER_ACK {
                let pid: u16 = ((buf[2] as u16) << 8) | (buf[3] as u16);
                defmt::info!("found 2 byte pid {:?}", pid);
                res = Ok(pid);
            } else {
                defmt::error!("malformed response in device ID read.");
                res = Err(());
            }
        }).await?;

        res
    }

    pub async fn read_device_memory(&self) -> Result<(), ()> {
        defmt::panic!("implement if needed.");
    }

    async fn write_device_memory_chunk(&self, data: &[u8], write_base_addr: u32) -> Result<(), ()> {
        if !self.in_bootloader {
            defmt::error!("called bootloader operation when not in bootloader context.");
            return Err(());
        }

        // stm bl can only support 256 byte data payloads, right now we don't automatically handle 
        // larger chunks than the tx buffer
        if data.len() > 256 || data.len() + 1 > LEN_TX {
            return Err(());
        }

        defmt::debug!("sending the write command...");
        self.writer
        .write(|buf| {
            buf[0] = STM32_BOOTLOADER_CMD_WRITE_MEM;
            buf[1] = !STM32_BOOTLOADER_CMD_WRITE_MEM;
            2
        })
        .await?;

        let mut res = Err(());
        self.reader.read(|buf| {
            defmt::info!("go cmd reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                } else {
                    defmt::error!("write mem replied with NACK");
                }
            }
        }).await?;

        if res.is_err() {
            return res;
        }

        defmt::debug!("sending the load address {:?}...", write_base_addr);
        self.writer
        .write(|buf| {
            let sa_bytes: [u8; 4] = write_base_addr.to_be_bytes();
            let cs = Self::bootloader_checksum_u32(write_base_addr);
            buf[0] = sa_bytes[0];
            buf[1] = sa_bytes[1];
            buf[2] = sa_bytes[2];
            buf[3] = sa_bytes[3];
            buf[4] = cs;
            // defmt::debug!("send buffer {:?}", buf);
            5
        })
        .await?;

        defmt::debug!("wait for load address reply");
        self.reader.read(|buf| {
            defmt::info!("load address reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                    defmt::info!("load address accepted.");
                } else {
                    defmt::error!("load address rejected (NACK)");
                }
            }
        }).await?;

        defmt::debug!("sending the data...");
        self.writer
        .write(|buf| {
            let cs = Self::bootloader_checksum_buf(data);
            let data_len = data.len();
            buf[0] = data_len as u8;
            buf[1..(data_len + 1)].copy_from_slice(data);
            buf[data_len] = cs;
            defmt::debug!("send data buffer {:?}", buf);
            5
        })
        .await?;

        Ok(())
    }

    pub async fn write_device_memory(&self, buf: &[u8], write_base_addr: Option<u32>) -> Result<(), ()> {
        if !self.in_bootloader {
            defmt::error!("called bootloader operation when not in bootloader context.");
            return Err(());
        }

        // ensure step size is below bootlaoder supported, below transmit buffer size (incl len and cs bytes),
        // and is 4-byte aligned
        let step_size = min(256, LEN_TX - 2) & 0xFFFF_FFFC;
        defmt::debug!("will use data chunk sizes of {:?}", step_size);

        // if user doesn't supply a start address, assume base of mapped flash
        let mut addr = write_base_addr.unwrap_or(0x0800_0000);
        defmt::debug!("will use start address {:?}", step_size);

        for start in (0..buf.len()).step_by(step_size) {
            let end = core::cmp::min(start + step_size, buf.len());
            self.write_device_memory_chunk( &buf[start..end], addr).await?;
            addr += step_size as u32;
        }
        Ok(())
    }

    pub async fn execute_code(&mut self, start_address: Option<u32>) -> Result<(), ()> {
        defmt::info!("firmware jump/go command implementation not working. will reset part.");
        self.reset_into_program().await?;
        return Ok(());

        if !self.in_bootloader {
            defmt::error!("called bootloader operation when not in bootloader context.");
            return Err(());
        }

        let start_address = start_address.unwrap_or(0x0800_0000);
        // let start_address = start_address.unwrap_or(0x20002000);


        // set the boot0 line low to disable startup bootloader
        // not needed for command, but makes sense on principle
        self.boot0_pin.as_mut().unwrap().set_low();

        defmt::debug!("sending the go command...");
        self.writer
        .write(|buf| {
            buf[0] = STM32_BOOTLOADER_CMD_GO;
            buf[1] = !STM32_BOOTLOADER_CMD_GO;
            2
        })
        .await?;

        let mut res = Err(());
        self.reader.read(|buf| {
            defmt::info!("go cmd reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                } else {
                    defmt::error!("bootloader replied with NACK");
                }
            }
        }).await?;

        if res.is_err() {
            return res;
        }

        defmt::debug!("sending the start address {:?}...", start_address);
        self.writer
        .write(|buf| {
            let sa_bytes: [u8; 4] = start_address.to_be_bytes();
            let cs = Self::bootloader_checksum_u32(start_address);
            buf[0] = sa_bytes[0];
            buf[1] = sa_bytes[1];
            buf[2] = sa_bytes[2];
            buf[3] = sa_bytes[3];
            buf[4] = cs;
            // defmt::debug!("send buffer {:?}", buf);
            5
        })
        .await?;

        defmt::debug!("wait for sa reply");
        self.reader.read(|buf| {
            defmt::info!("sa reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                    self.in_bootloader = false;
                    defmt::info!("program started.");
                } else {
                    defmt::error!("bootloader replied with NACK");
                }
            }
        }).await?;

        res
    }


}