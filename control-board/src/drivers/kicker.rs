use ateam_lib_stm32::{
    drivers::boot::stm32_interface::Stm32Interface,
    uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue},
};
use embassy_stm32::{
    gpio::{AnyPin, Pull},
    usart::Parity,
    Peri,
};
use embassy_time::{with_timeout, Duration, Timer};

use core::mem::MaybeUninit;

use ateam_common_packets::bindings::{
    DribblerCommand, ErrorTelemetry, KickRequest::KR_DISABLE, KickerControl, KickerTelemetry,
};

use crate::{image_hash, DEBUG_KICKER_UART_QUEUES};

// Packet dispatch uses length to distinguish types — assert sizes differ so a future
// struct change doesn't silently break the dispatch.
const _: () = assert!(
    core::mem::size_of::<KickerTelemetry>() != core::mem::size_of::<ErrorTelemetry>(),
    "KickerTelemetry and ErrorTelemetry must have different sizes for length-based dispatch"
);

#[derive(defmt::Format)]
pub enum KickerFlashStatus {
    OkHashMatch,
    OkFlashed,
    ErrFlashFail,
    ErrHashTimeoutFlashFail,
}

impl KickerFlashStatus {
    pub fn is_ok(&self) -> bool {
        matches!(self, Self::OkHashMatch | Self::OkFlashed)
    }

    pub fn is_err(&self) -> bool {
        !self.is_ok()
    }
}

pub struct Kicker<
    'a,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_RX: usize,
    const DEPTH_TX: usize,
> {
    stm32_uart_interface:
        Stm32Interface<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, DEBUG_KICKER_UART_QUEUES>,
    firmware_image: &'a [u8],

    command_state: KickerControl,
    telemetry_state: KickerTelemetry,

    telemetry_enabled: bool,
}

impl<
        'a,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize,
        const DEPTH_TX: usize,
    > Kicker<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
{
    pub fn new(
        stm32_interface: Stm32Interface<
            'a,
            LEN_RX,
            LEN_TX,
            DEPTH_RX,
            DEPTH_TX,
            DEBUG_KICKER_UART_QUEUES,
        >,
        firmware_image: &'a [u8],
    ) -> Kicker<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX> {
        let mut command_state: KickerControl = Default::default();
        command_state.kick_request = KR_DISABLE;

        Self {
            stm32_uart_interface: stm32_interface,
            firmware_image,

            command_state: command_state,
            telemetry_state: Default::default(),

            telemetry_enabled: false,
        }
    }

    pub fn new_from_pins(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_KICKER_UART_QUEUES>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_KICKER_UART_QUEUES>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_KICKER_UART_QUEUES>,
        boot0_pin: Peri<'static, AnyPin>,
        reset_pin: Peri<'static, AnyPin>,
        firmware_image: &'a [u8],
    ) -> Self {
        let stm32_interface = Stm32Interface::new_from_pins(
            uart,
            read_queue,
            write_queue,
            boot0_pin,
            reset_pin,
            Pull::None,
            true,
        );

        Self::new(stm32_interface, firmware_image)
    }

    pub fn process_telemetry(&mut self) -> bool {
        let mut received_packet = false;
        while let Ok(res) = self.stm32_uart_interface.try_read_data() {
            let buf = res.data();

            if buf.len() == core::mem::size_of::<KickerTelemetry>() {
                received_packet = true;
                unsafe {
                    core::ptr::copy_nonoverlapping(
                        buf.as_ptr(),
                        &mut self.telemetry_state as *mut _ as *mut u8,
                        core::mem::size_of::<KickerTelemetry>(),
                    );
                }
            } else if buf.len() == core::mem::size_of::<ErrorTelemetry>() {
                unsafe {
                    let mut err_telem: ErrorTelemetry = MaybeUninit::zeroed().assume_init();
                    core::ptr::copy_nonoverlapping(
                        buf.as_ptr(),
                        &mut err_telem as *mut _ as *mut u8,
                        core::mem::size_of::<ErrorTelemetry>(),
                    );
                    let msg_len = err_telem
                        .error_message
                        .iter()
                        .position(|&b| b == 0)
                        .unwrap_or(60);
                    defmt::error!(
                        "kicker ErrorTelemetry [ts={}]: {=[u8]:a}",
                        err_telem.timestamp,
                        &err_telem.error_message[..msg_len]
                    );
                }
            } else {
                defmt::warn!(
                    "kicker interface - invalid packet of len {:?} data: {:?}",
                    buf.len(),
                    buf
                );
            }
        }

        received_packet
    }

    pub fn send_command(&mut self) {
        unsafe {
            let struct_bytes = core::slice::from_raw_parts(
                (&self.command_state as *const KickerControl) as *const u8,
                core::mem::size_of::<KickerControl>(),
            );

            self.stm32_uart_interface.send_or_discard_data(struct_bytes);
        }
    }

    pub fn get_lastest_state(&self) -> KickerTelemetry {
        self.telemetry_state
    }

    pub fn set_telemetry_enabled(&mut self, telemetry_enabled: bool) {
        self.telemetry_enabled = telemetry_enabled;
        self.command_state
            .set_telemetry_enabled(telemetry_enabled as u16);
    }

    pub fn request_kick(&mut self, request: u32) {
        self.command_state.kick_request = request as u8;
    }

    pub fn set_kick_strength(&mut self, kick_str: f32) {
        self.command_state.kick_speed = kick_str;
    }

    pub fn set_drib_command(&mut self, mode: DribblerCommand::Type, setpoint: f32) {
        self.command_state.dribbler_mode = mode;
        self.command_state.drib_setpoint = setpoint;
    }

    pub fn ball_detected(&self) -> bool {
        self.telemetry_state.ball_detected() != 0
    }

    ///////////////////////////
    //  shutdown processing  //
    ///////////////////////////

    pub fn request_shutdown(&mut self) {
        self.command_state.set_request_power_down(1);
    }

    pub fn shutdown_acknowledge(&self) -> bool {
        self.telemetry_state.power_down_requested() != 0
    }

    pub fn shutdown_completed(&self) -> bool {
        self.telemetry_state.power_down_complete() != 0
    }

    ///////////////////////
    //  error reporting  //
    ///////////////////////

    pub fn error_reported(&self) -> bool {
        self.telemetry_state.error_detected() != 0
    }

    pub fn hv_rail_voltage(&self) -> f32 {
        self.telemetry_state.rail_voltage
    }

    pub fn battery_voltage(&self) -> f32 {
        self.telemetry_state.battery_voltage
    }

    //////////////////////////////////
    //  firmware loading functions  //
    //////////////////////////////////

    pub async fn reset(&mut self) {
        self.stm32_uart_interface.hard_reset().await;
    }

    pub async fn enter_reset(&mut self) {
        self.stm32_uart_interface.enter_reset().await;
    }

    pub async fn leave_reset(&mut self) {
        self.stm32_uart_interface.leave_reset().await;
    }

    pub fn get_latest_default_img_hash(&mut self) -> [u8; 16] {
        image_hash::get_kicker_img_hash()
    }

    /// Get the first 4 bytes of the currently loaded image hash on the device, Run with timeout!
    pub async fn get_current_device_img_hash(&mut self) -> [u8; 4] {
        loop {
            defmt::trace!("Kicker Interface - Enabling kicker telemetry");
            self.set_telemetry_enabled(true);
            self.send_command();

            Timer::after(Duration::from_millis(5)).await;

            // Parse incoming packets
            let received_telemetry = self.process_telemetry();
            if received_telemetry {
                let current_img_hash = self.telemetry_state.kicker_image_hash;
                defmt::debug!("Kicker Interface - Received telemetry");
                defmt::trace!(
                    "Kicker Interface - Current device image hash {:x}",
                    current_img_hash
                );
                return current_img_hash;
            };

            Timer::after(Duration::from_millis(5)).await;
        }
    }

    async fn check_device_has_latest_default_image(&mut self) -> Result<bool, ()> {
        let latest_img_hash = self.get_latest_default_img_hash();
        defmt::debug!(
            "Kicker Interface - Latest default image hash - {:x}",
            latest_img_hash
        );

        defmt::trace!("Kicker Interface - Update UART config 2 MHz");
        self.stm32_uart_interface
            .update_uart_config(2_000_000, Parity::ParityEven)
            .await;
        Timer::after(Duration::from_millis(1)).await;

        let res;
        let timeout = Duration::from_millis(100);
        defmt::trace!("Kicker Interface - Waiting for device response");
        match with_timeout(timeout, self.get_current_device_img_hash()).await {
            Ok(current_img_hash) => {
                if current_img_hash == latest_img_hash[..4] {
                    defmt::trace!("Kicker Interface - Device has the latest default image");
                    res = Ok(true);
                } else {
                    defmt::trace!(
                        "Kicker Interface - Device does not have the latest default image"
                    );
                    res = Ok(false);
                }
            }
            Err(_) => {
                defmt::debug!("Kicker Interface - No device response, image hash unknown");
                res = Err(());
            }
        }
        // drain any parameter response packets before flashing
        self.process_telemetry();
        res
    }

    pub async fn init_firmware_image(
        &mut self,
        flash: bool,
        fw_image_bytes: &[u8],
    ) -> Result<(), ()> {
        if flash {
            defmt::info!("Kicker Interface - Flashing firmware image");
            self.stm32_uart_interface
                .load_firmware_image(fw_image_bytes, true)
                .await?;
        } else {
            defmt::info!("Kicker Interface - Skipping firmware flash");
        }

        self.stm32_uart_interface
            .update_uart_config(2_000_000, Parity::ParityEven)
            .await;

        Timer::after(Duration::from_millis(1)).await;

        self.stm32_uart_interface.leave_reset().await;

        Ok(())
    }

    pub async fn init_default_firmware_image(&mut self, force_flash: bool) -> KickerFlashStatus {
        let (flash, hash_timed_out) = if force_flash {
            defmt::info!("Kicker Interface - Force flash enabled");
            (true, false)
        } else {
            defmt::debug!("Kicker Interface - Resetting kicker");
            self.reset().await;
            match self.check_device_has_latest_default_image().await {
                Ok(has_latest) => (!has_latest, false),
                Err(_) => (true, true),
            }
        };

        if !flash {
            let _ = self.init_firmware_image(false, self.firmware_image).await;
            return KickerFlashStatus::OkHashMatch;
        }

        match self.init_firmware_image(true, self.firmware_image).await {
            Ok(()) => KickerFlashStatus::OkFlashed,
            Err(()) => {
                if hash_timed_out {
                    KickerFlashStatus::ErrHashTimeoutFlashFail
                } else {
                    KickerFlashStatus::ErrFlashFail
                }
            }
        }
    }
}
