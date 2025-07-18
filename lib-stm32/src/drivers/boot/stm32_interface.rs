use core::cmp::min;

use defmt_rtt as _;

use embassy_stm32::gpio::{Level, Output, Pin, Speed, Pull};
use embassy_stm32::usart::{self, Config, DataBits, Parity, StopBits};
use embassy_time::{Duration, Timer};
use embassy_time::with_timeout;

use crate::queue::{DequeueRef, Error};
use crate::uart::queue::{IdleBufferedUart, Reader, UartReadQueue, UartWriteQueue, Writer};

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

// TODO Make this shared in software-communication
pub const MOTOR_CURRENT_START_ADDRESS: u32 = 0x0800_7C00; // 512k flash, so this is the start of the last page
pub const MOTOR_CURRENT_PAGE: u8 = 31;
pub const MOTOR_CURRENT_MAGIC: [u8; 4] = [0xAA, 0xBB, 0xCC, 0xDD]; // Magic number to identify the motor current calibration data

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
        const DEBUG_UART_QUEUES: bool,
> {
    uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
    reader: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_UART_QUEUES>,
    writer: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
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
        const DEBUG_UART_QUEUES: bool,
    > Stm32Interface<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, DEBUG_UART_QUEUES>
{
    pub fn new(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_UART_QUEUES>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        boot0_pin: Output<'a>,
        reset_pin: Output<'a>,
        reset_polarity_high: bool,
    ) -> Stm32Interface<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, DEBUG_UART_QUEUES> {
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
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_UART_QUEUES>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_UART_QUEUES>,
        boot0_pin: impl Pin,
        reset_pin: impl Pin,
        _reset_pin_pull: Pull,
        reset_polarity_high: bool,
    ) -> Stm32Interface<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, DEBUG_UART_QUEUES> {
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
            } else {
                defmt::debug!("bootloader reply too short after calibration.");
            }
            res
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
            // defmt::info!("resetting device");
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

    // Based on 3.4 of AN3155
    pub async fn read_device_memory(&self, data: &mut [u8], read_base_addr: u32) -> Result<(), ()> {
        if !self.in_bootloader {
            defmt::error!("called bootloader operation when not in bootloader context.");
            return Err(());
        }

        let data_len = data.len();
        if data_len > 255 || data_len + 1 > LEN_TX {
            defmt::error!("Data length too large for bootloader read mem command.");
            return Err(());
        }

        // defmt::debug!("sending the read command...");
        self.writer
        .write(|buf| {
            buf[0] = STM32_BOOTLOADER_CMD_READ_MEM;
            buf[1] = !STM32_BOOTLOADER_CMD_READ_MEM;
            //defmt::info!("send buffer {:?}", buf);
            2
        })
        .await?;

        // Wait for the bootloader to acknowledge the command
        let mut res = Err(());
        self.reader.read(|buf| {
            // defmt::info!("Read cmd reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                } else {
                    defmt::error!("Read mem cmd replied with NACK");
                }
            } else {
                defmt::error!("Read mem cmd reply too short.");
            }
        }).await?;

        if res.is_err() {
            return res;
        }

        // defmt::debug!("sending the load address {:?}...", read_base_addr);
        self.writer
        .write(|buf| {
            let start_address_bytes: [u8; 4] = read_base_addr.to_be_bytes();
            let cs = Self::bootloader_checksum_u32(read_base_addr);
            buf[0] = start_address_bytes[0];
            buf[1] = start_address_bytes[1];
            buf[2] = start_address_bytes[2];
            buf[3] = start_address_bytes[3];
            buf[4] = cs;
            // defmt::debug!("send buffer {:?}", buf);
            5
        })
        .await?;

        res = Err(());
        // Wait for the bootloader to acknowledge the address
        self.reader.read(|buf| {
            // defmt::info!("go cmd reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                } else {
                    defmt::error!("Address read mem replied with NACK");
                }
            } else {
                defmt::error!("Address read mem reply too short.");
            }
        }).await?;

        if res.is_err() {
            return res;
        }

        // defmt::debug!("sending the data length...");
        self.writer
        .write(|buf| {
            let data_len_minus_one = data_len as u8 - 1;
            buf[0] = data_len_minus_one;
            buf[1] = !data_len_minus_one;
            // defmt::debug!("send buffer {:?}", buf);
            2
        })
        .await?;

        res = Err(());
        // defmt::debug!("reading the data...");
        self.reader.read(|buf| {
            // defmt::info!("data reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    data.copy_from_slice(&buf[1..]);
                    res = Ok(());
                } else {
                    defmt::error!("Data read mem replied with NACK");
                }
            } else {
                defmt::error!("Data read mem reply too short!");
            }
        }).await?;

        res
    }

    // Based on 3.8 of AN3155
    // Have to use extended erase command since the normal erase command
    // doesn't seem to work?
    // Designed to erase a up to N + 1 page of device memory.
    pub async fn erase_device_memory_to_page(&self, start_page: u8, end_page: u8) -> Result<(), ()> {
        if !self.in_bootloader {
            defmt::error!("Called bootloader operation when not in bootloader context.");
            return Err(());
        }

        if end_page < start_page {
            defmt::error!("End page must be greater than or equal to start page.");
            return Err(());
        }

        // defmt::debug!("sending the erase command...");
        self.writer
        .write(|buf| {
            buf[0] = STM32_BOOTLOADER_CMD_EXTENDED_ERASE;
            buf[1] = !STM32_BOOTLOADER_CMD_EXTENDED_ERASE;
            2
        })
        .await?;

        let mut res = Err(());
        self.reader.read(|buf| {
            // defmt::info!("erase cmd reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                } else {
                    defmt::error!("Bootloader replied to erase command with NACK");
                }
            } else {
                defmt::error!("Erase command reply too short.");
            }
        }).await?;

        if res.is_err() {
            return res;
        }

        // defmt::debug!("sending the page number...");
        self.writer
        .write(|buf| {
            // Quantity is N + 1 lead with MSB. Limited to 32 pages on STM32F1.
            let erase_page_quantity = end_page - start_page;
            buf[0] = 0x00;
            buf[1] = erase_page_quantity;
            // Need to send the page number for each page to erase in two bytes, MSB first
            let mut checksum = erase_page_quantity;
            // Track the index for use in returning the buffer length
            let mut buf_indx: usize = 2;
            for i in start_page ..= end_page {
                // Won't erase more than 256 pages, so always lead with 0x00
                buf[buf_indx] = 0x00;
                buf[buf_indx + 1] = i;
                buf_indx += 2;
                // Checksum is XOR of all previous bytes. Ignore 0x00 so just LSB
                checksum ^= i;
            }
            // Checksum for all previous bytes is just the erase page number
            buf[buf_indx] = checksum;
            // defmt::debug!("send buffer {:?}", buf);
            // Final size is buf_indx + 1 from checksum byte
            buf_indx + 1
        })
        .await?;

        // defmt::debug!("wait for erase reply");
        self.reader.read(|buf| {
            // defmt::info!("erase reply {:?}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                    // defmt::info!("erase accepted.");
                } else {
                    defmt::error!("bootloader replied to erase payload with NACK");
                }
            }
        }).await?;

        res
    }

    // Single page of device memory is just a page to the same page.
    pub async fn erase_device_memory_single(&self, erase_page: u8) -> Result<(), ()> {
        self.erase_device_memory_to_page(erase_page, erase_page).await
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

        // defmt::debug!("sending the load address {:X}...", write_base_addr);
        self.writer
        .write(|buf| {
            let sa_bytes: [u8; 4] = write_base_addr.to_be_bytes();
            let cs = Self::bootloader_checksum_u32(write_base_addr);
            buf[0] = sa_bytes[0];
            buf[1] = sa_bytes[1];
            buf[2] = sa_bytes[2];
            buf[3] = sa_bytes[3];
            buf[4] = cs;
            // defmt::trace!("send load address buffer {:X}", buf);
            5
        })
        .await?;

        // defmt::debug!("wait for load address reply");
        self.reader.read(|buf| {
            defmt::trace!("load address reply {:X}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                    // defmt::trace!("load address accepted.");
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
            // defmt::trace!("firmware data buffer len {:?}", data_len);
            buf[0] = data_len as u8 - 1;
            buf[1..(data_len + 1)].copy_from_slice(data);
            buf[data_len + 1] = cs;
            // defmt::trace!("send data buffer {:X}", buf);
            data.len() + 2
        })
        .await?;

        // defmt::debug!("wait send data reply");
        self.reader.read(|buf| {
            // defmt::trace!("send data reply {:X}", buf);
            if buf.len() >= 1 {
                if buf[0] == STM32_BOOTLOADER_ACK {
                    res = Ok(());
                    // defmt::trace!("data accepted.");
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

        // ensure step size is below bootloader supported, below transmit buffer size (incl len and cs bytes),
        // and is 4-byte aligned
        let step_size = min(256, LEN_TX - 2) & 0xFFFF_FFF8;
        defmt::debug!("bootloader will use data chunk sizes of {:?}", step_size);
        if step_size < 8 {
            defmt::error!("bootloader buffer too small.");
            return Err(());
        }

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

    pub async fn load_firmware_image(&mut self, fw_image_bytes: &[u8], leave_in_reset: bool) -> Result<(), ()> {
        if self.in_bootloader {
            defmt::trace!("device is already in bootloader");
        } else {
            defmt::trace!("resetting device into bootloader");
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
                105 => {
                    defmt::trace!("found stm32g474xx device");
                }
                _ => {
                    defmt::error!("found unknown device id {}", device_id);
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

        self.reset_into_program(leave_in_reset).await;

        Ok(())
    }

    pub async fn load_motor_firmware_image(&mut self, fw_image_bytes: &[u8]) -> Result<(), ()> {
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
                105 => {
                    defmt::trace!("found stm32g474xx device");
                }
                _ => {
                    defmt::error!("found unknown device id {}", device_id);
                    return Err(());
                }
            }
        };

        // Erase up to Page 31 since the last page is used for current calibration constants.
        if let Err(err) = self.erase_device_memory_to_page(0, 15).await {
            return Err(err);
        }

        // Split up to 2 commands to reduce UART packet size.
        if let Err(err) = self.erase_device_memory_to_page(16, MOTOR_CURRENT_PAGE).await {
            return Err(err);
        }

        // program image
        if let Err(err) = self.write_device_memory(fw_image_bytes, None).await {
            return Err(err);
        }

        self.reset_into_program(true).await;

        Ok(())
    }

    pub async fn write_current_calibration_constants(&mut self, current_constant: f32) -> Result<(), ()> {
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
                    defmt::trace!("found STSPINF0 device");
                }
                _ => {
                    defmt::error!("Invalid device id for current calibration constants {}", device_id);
                    return Err(());
                }
            }
        };

        // Erase the last page
        if let Err(err) = self.erase_device_memory_single(MOTOR_CURRENT_PAGE).await {
            return Err(err);
        }

        // Write the constants to the last page but with the magic number prepended
        let mut data_to_write = [0u8; 8];
        data_to_write[..4].copy_from_slice(&MOTOR_CURRENT_MAGIC);
        // Convert the f32 to bytes and write it after the magic number
        let current_bytes = current_constant.to_le_bytes();
        data_to_write[4..8].copy_from_slice(&current_bytes);

        if let Err(err) = self.write_device_memory(&data_to_write, Some(MOTOR_CURRENT_START_ADDRESS)).await {
            return Err(err);
        }

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