use core::cmp::min;

use defmt_rtt as _;
use defmt::*;

use embassy_stm32::gpio::{Level, Output, OutputOpenDrain, Pin, Speed, Pull};
use embassy_stm32::usart::{self, Config, DataBits, Parity, StopBits};
use embassy_time::{Duration, Timer};
use embassy_time::with_timeout;

use ateam_lib_stm32::queue::{DequeueRef, Error};
use ateam_lib_stm32::uart::queue::{IdleBufferedUart, Reader, UartReadQueue, UartWriteQueue, Writer};

pub const STM32_BOOTLOADER_MAX_BAUD_RATE: u32 = 115_200;
pub const STM32_BOOTLOADER_ACK: u8 = 0x79;
pub const STM32_BOOTLOADER_NACK: u8 = 0x1F;
pub const STM32_BOOTLOADER_CODE_SEQUENCE_BYTE: u8 = 0x7F;

pub const STM32_BOOTLOADER_CMD_GET: u8 = 0x00;
pub const STM32_BOOTLOADER_CMD_GET_VERSION: u8 = 0x01;
pub const STM32_BOOTLOADER_CMD_GET_ID: u8 = 0x02;
pub const STM32_BOOTLOADER_CMD_READ_MEM: u8 = 0x11;
pub const STM32_BOOTLOADER_CMD_GO: u8 = 0x21;
pub const STM32_BOOTLOADER_CMD_WRITE_MEM: u8 = 0x31;
pub const STM32_BOOTLOADER_CMD_ERASE: u8 = 0x43;
pub const STM32_BOOTLOADER_CMD_EXTENDED_ERASE: u8 = 0x44;
pub const STM32_BOOTLOADER_CMD_WRITE_PROT: u8 = 0x63;
pub const STM32_BOOTLOADER_CMD_WRITE_UNPROT: u8 = 0x73;
pub const STM32_BOOTLOADER_CMD_READ_PROT: u8 = 0x82;
pub const STM32_BOOTLOADER_CMD_READ_UNPROT: u8 = 0x92;
pub const STM32_BOOTLOADER_CMD_GET_CHECKSUM: u8 = 0xA1;

pub fn get_bootloader_uart_config() -> Config {
    let mut config = usart::Config::default();
    config.baudrate = 115_200; // max officially support baudrate
    config.parity = Parity::ParityEven;
    config.stop_bits = StopBits::STOP1;
    config
}

pub struct Stm32Interface<
        'a,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize,
        const DEPTH_TX: usize,
> {
    uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX>,
    reader: &'a UartReadQueue<LEN_RX, DEPTH_RX>,
    writer: &'a UartWriteQueue<LEN_TX, DEPTH_TX>,
    boot0_pin: Output<'a>,
    reset_pin: Output<'a>,

    reset_pin_noninverted: bool,

    in_bootloader: bool,
}

impl<
        'a,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize,
        const DEPTH_TX: usize,
    > Stm32Interface<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
{
    pub fn new(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX>,
        boot0_pin: Output<'a>,
        reset_pin: Output<'a>,
        reset_polarity_high: bool,
    ) -> Stm32Interface<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX> {
        Stm32Interface {
            uart,
            reader: read_queue,
            writer: write_queue,
            boot0_pin,
            reset_pin,
            reset_pin_noninverted: reset_polarity_high,
            in_bootloader: false,
        }
    }

    pub fn new_from_pins(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX>,
        boot0_pin: impl Pin,
        reset_pin: impl Pin,
        reset_pin_pull: Pull,
        reset_polarity_high: bool,
    ) -> Stm32Interface<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX> {
        let boot0_output = Output::new(boot0_pin, Level::Low, Speed::Medium);

        let initial_reset_level = if reset_polarity_high {
            Level::Low
        } else {
            Level::High
        };
        let reset_output = Output::new(reset_pin, initial_reset_level, Speed::Medium);

        Stm32Interface {
            uart,
            reader: read_queue,
            writer: write_queue,
            boot0_pin: boot0_output,
            reset_pin: reset_output,
            reset_pin_noninverted: reset_polarity_high,
            in_bootloader: false,
        }
    }

    pub async fn soft_reset(&self) {
        defmt::panic!("implement soft reset if needed.");
    }

    pub async fn enter_reset(&mut self) {
        if self.reset_pin_noninverted {
            self.reset_pin.set_high();
        } else {
            self.reset_pin.set_low();
        }
        Timer::after(Duration::from_millis(50)).await;
    }

    pub async fn leave_reset(&mut self) {
        if self.reset_pin_noninverted {
            self.reset_pin.set_low();
        } else {
            self.reset_pin.set_high();
        }
        Timer::after(Duration::from_millis(10)).await;
    }

    pub async fn hard_reset(&mut self) {
        self.enter_reset().await;
        // Timer::after_millis(1).await;
        self.leave_reset().await;
        // Timer::after_millis(1).await;
    }

    pub async fn reset_into_bootloader(&mut self) -> Result<(), ()> {
        // ensure UART is in the expected config for the bootloader
        // this operation is unsafe because it takes the uart module offline
        // when the executor may be relying on rx interrupts to unblock a thread
        self.update_uart_config(STM32_BOOTLOADER_MAX_BAUD_RATE, Parity::ParityEven).await;
        Timer::after_millis(100).await;


        // set the boot0 line high to enter the UART bootloader upon reset
        self.boot0_pin.set_high();
        Timer::after_millis(1).await;

        // reset the device
        self.hard_reset().await;

        // ensure it has time to setup it's bootloader
        // this time isn't documented and can possibly be lowered.
        Timer::after(Duration::from_millis(10)).await;



        defmt::debug!("sending the bootloader baud calibration command...");
        Timer::after_millis(1000).await;
        if self.writer
        .write(|buf| {
            buf[0] = STM32_BOOTLOADER_CODE_SEQUENCE_BYTE;
            1
        })
        .await.is_err() {
            defmt::debug!("failed to send bootloader start seq");
            return Err(());
        }
        Timer::after_millis(10).await;

        let mut res = Err(());
        let sync_res = with_timeout(Duration::from_millis(5000), self.reader.read(|buf| {
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    defmt::debug!("bootloader replied with ACK after calibration.");
                    self.in_bootloader = true;
                    res = Ok(());
                } else {
                    defmt::debug!("bootloader replied with NACK after calibration.");
                }
            }
        })).await;

        if sync_res.is_err() {
            defmt::warn!("*** HARDWARE CHECK *** - bootloader baud calibration timed out.");
            return Err(())
        }

        res
    }

    pub async fn reset_into_program(&mut self, stay_in_reset: bool) {
        // set the boot0 line low to disable startup bootloader
        self.boot0_pin.set_low();
        Timer::after(Duration::from_millis(5)).await;

        if stay_in_reset {
            // nrst, so reset and hold
            self.enter_reset().await;
        } else {
            // reset the device
            self.hard_reset().await;
        }
    }

    pub async fn update_uart_config(&self, baudrate: u32, parity: Parity) {
        let mut config = usart::Config::default();
        config.baudrate = baudrate;
        config.data_bits = DataBits::DataBits8;
        config.parity = parity;

        if self.uart.update_uart_config(config).await.is_err() {
            defmt::panic!("failed to update uart config");
        }
    }

    pub fn try_read_data(&self) -> Result<DequeueRef<LEN_RX, DEPTH_RX>, Error> {
        return self.reader.try_dequeue();
    }

    pub fn try_send_data(&self, data: &[u8]) -> Result<(), Error> {
        return self.writer.enqueue_copy(data);
    }

    pub fn send_or_discard_data(&self, data: &[u8]) {
        if self.try_send_data(data).is_err() {
            defmt::error!("Failed to send motor data");
        };
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
        let mut cks = buf.len() as u8 - 1;
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
                defmt::error!("unknown command enumeration error: {}", buf);
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
                defmt::trace!("found 1 byte pid {:?}", pid);
                res = Ok(pid);
            } else if buf.len() == 5 && buf[1] == 2 && buf[4] == STM32_BOOTLOADER_ACK {
                let pid: u16 = ((buf[2] as u16) << 8) | (buf[3] as u16);
                defmt::trace!("found 2 byte pid {:?}", pid);
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

        // defmt::debug!("sending the write command...");
        self.writer
        .write(|buf| {
            buf[0] = STM32_BOOTLOADER_CMD_WRITE_MEM;
            buf[1] = !STM32_BOOTLOADER_CMD_WRITE_MEM;
            2
        })
        .await?;

        let mut res = Err(());
        self.reader.read(|buf| {
            // defmt::info!("go cmd reply {:?}", buf);
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

        // defmt::debug!("sending the load address {:?}...", write_base_addr);
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

        // defmt::debug!("wait for load address reply");
        self.reader.read(|buf| {
            // defmt::info!("load address reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                    // defmt::info!("load address accepted.");
                } else {
                    defmt::error!("load address rejected (NACK)");
                }
            }
        }).await?;

        // defmt::debug!("sending the data...");
        self.writer
        .write(|buf| {
            let cs = Self::bootloader_checksum_buf(data);
            let data_len = data.len();
            buf[0] = data_len as u8 - 1;
            buf[1..(data_len + 1)].copy_from_slice(data);
            buf[data_len + 1] = cs;
            // defmt::debug!("send data buffer {:?}", buf);
            data.len() + 2
        })
        .await?;

        // defmt::debug!("wait send data reply");
        self.reader.read(|buf| {
            // defmt::info!("send data reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                    // defmt::info!("data accepted.");
                } else {
                    defmt::error!("data rejected (NACK)");
                }
            }
        }).await?;

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
        // defmt::debug!("will use data chunk sizes of {:?}", step_size);

        // if user doesn't supply a start address, assume base of mapped flash
        let mut addr = write_base_addr.unwrap_or(0x0800_0000);
        // defmt::debug!("will use start address {:?}", write_base_addr);

        for start in (0..buf.len()).step_by(step_size) {
            let end = core::cmp::min(start + step_size, buf.len());
            self.write_device_memory_chunk( &buf[start..end], addr).await?;
            addr += step_size as u32;
        }

        defmt::debug!("wrote device memory");

        Ok(())
    }

    pub async fn erase_flash_memory(&self) -> Result<(), ()> {
        if !self.in_bootloader {
            defmt::error!("called bootloader operation when not in bootloader context.");
            return Err(());
        }

        defmt::debug!("sending the erase command...");
        self.writer
        .write(|buf| {
            buf[0] = STM32_BOOTLOADER_CMD_EXTENDED_ERASE;
            buf[1] = !STM32_BOOTLOADER_CMD_EXTENDED_ERASE;
            2
        })
        .await?;

        let mut res = Err(());
        self.reader.read(|buf| {
            defmt::trace!("extended erase reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                } else {
                    defmt::error!("bootloader replied with NACK");
                }
            }
        }).await?;

        defmt::debug!("sending erase type...");
        self.writer
        .write(|buf| {
            buf[0] = 0xFF;
            buf[1] = 0xFF;
            buf[2] = 0x00;
            3
        })
        .await?;

        let mut res = Err(());
        self.reader.read(|buf| {
            defmt::debug!("erase reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    defmt::debug!("flash erased");
                    res = Ok(());
                } else {
                    defmt::error!("bootloader replied with NACK");
                }
            }
        }).await?;

        Ok(())
    }

    pub async fn load_firmware_image(&mut self, fw_image_bytes: &[u8]) -> Result<(), ()> {
        if !self.in_bootloader {
            if let Err(err) = self.reset_into_bootloader().await {
                return Err(err);
            }
        }

        if let Err(err) = self.verify_bootloader().await {
            return Err(err);
        }

        match self.get_device_id().await {
            Err(err) => return Err(err),
            Ok(device_id) => match device_id {
                68 => {
                    defmt::trace!("found stm32f1 device");
                }
                19 => {
                    defmt::trace!("found stm32f40xxx device");
                }
                _ => {
                    defmt::trace!("found unknown device id {}", device_id);
                    return Err(());
                }
            }
        };

        // erase part
        if let Err(err) = self.erase_flash_memory().await {
            return Err(err);
        }

        // program image
        if let Err(err) = self.write_device_memory(fw_image_bytes, None).await {
            return Err(err);
        }

        self.reset_into_program(false).await;

        Ok(())
    }

    pub async fn execute_code(&mut self, start_address: Option<u32>) -> Result<(), ()> {
        defmt::debug!("firmware jump/go command implementation not working. will reset part.");
        self.reset_into_program(false).await;
        return Ok(());

        if !self.in_bootloader {
            defmt::error!("called bootloader operation when not in bootloader context.");
            return Err(());
        }

        let start_address = start_address.unwrap_or(0x0800_0000);
        // let start_address = start_address.unwrap_or(0x20002000);


        // set the boot0 line low to disable startup bootloader
        // not needed for command, but makes sense on principle
        self.boot0_pin.set_low();

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