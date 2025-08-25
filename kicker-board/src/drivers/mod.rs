use core::mem::MaybeUninit;

use ateam_common_packets::bindings::{
    MotionCommandType::{self, OPEN_LOOP},
    MotorCommandPacket,
    MotorCommandType::{MCP_MOTION, MCP_PARAMS},
    MotorResponse,
    MotorResponseType::{MRP_MOTION, MRP_PARAMS},
    MotorTelemetry, ParameterMotorResponse,
};
use ateam_lib_stm32::{
    drivers::boot::stm32_interface::Stm32Interface,
    uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue},
};
use embassy_stm32::{
    gpio::{AnyPin, Pull},
    usart::Parity, Peri,
};
use embassy_time::{with_timeout, Duration, Timer};

use crate::image_hash;

use crate::DEBUG_DRIB_UART_QUEUES;

pub mod breakbeam;

pub struct DribblerMotor<
    'a,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_RX: usize,
    const DEPTH_TX: usize,
> {
    stm32_uart_interface:
        Stm32Interface<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, DEBUG_DRIB_UART_QUEUES>,
    firmware_image: &'a [u8],

    current_state: MotorTelemetry,
    current_params_state: ParameterMotorResponse,

    setpoint: f32,
    motion_type: MotionCommandType::Type,
    reset_flagged: bool,
    telemetry_enabled: bool,

    ball_detected_thresh: f32,
}

impl<
        'a,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize,
        const DEPTH_TX: usize,
    > DribblerMotor<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
{
    pub fn new(
        stm32_interface: Stm32Interface<
            'a,
            LEN_RX,
            LEN_TX,
            DEPTH_RX,
            DEPTH_TX,
            DEBUG_DRIB_UART_QUEUES,
        >,
        firmware_image: &'a [u8],
        ball_detected_thresh: f32,
    ) -> DribblerMotor<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX> {
        let start_state: MotorTelemetry = { unsafe { MaybeUninit::zeroed().assume_init() } };

        let start_params_state: ParameterMotorResponse =
            { unsafe { MaybeUninit::zeroed().assume_init() } };

        DribblerMotor {
            stm32_uart_interface: stm32_interface,
            firmware_image,

            current_state: start_state,
            current_params_state: start_params_state,

            setpoint: 0.0,
            motion_type: OPEN_LOOP,
            reset_flagged: false,
            telemetry_enabled: false,

            ball_detected_thresh,
        }
    }

    pub fn new_from_pins(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_DRIB_UART_QUEUES>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_DRIB_UART_QUEUES>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_DRIB_UART_QUEUES>,
        boot0_pin: Peri<'a, AnyPin>,
        reset_pin: Peri<'a, AnyPin>,
        firmware_image: &'a [u8],
        ball_detected_thresh: f32,
    ) -> DribblerMotor<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX> {
        // Need a Pull None to allow for STSPIN watchdog usage.
        let stm32_interface = Stm32Interface::new_from_pins(
            uart,
            read_queue,
            write_queue,
            boot0_pin,
            reset_pin,
            Pull::None,
            false,
        );

        let start_state: MotorTelemetry = { unsafe { MaybeUninit::zeroed().assume_init() } };

        let start_params_state: ParameterMotorResponse =
            { unsafe { MaybeUninit::zeroed().assume_init() } };

        DribblerMotor {
            stm32_uart_interface: stm32_interface,
            firmware_image,

            current_state: start_state,
            current_params_state: start_params_state,

            setpoint: 0.0,
            motion_type: OPEN_LOOP,
            reset_flagged: false,
            telemetry_enabled: false,

            ball_detected_thresh,
        }
    }

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
        image_hash::get_dribbler_img_hash()
    }

    /// Get the first 4 bytes of the currently loaded image hash on the device, Run with timeout!
    pub async fn get_current_device_img_hash(&mut self) -> [u8; 4] {
        loop {
            // defmt::trace!("Dribbler Interface - Sending parameter command packet");
            self.send_params_command();

            Timer::after(Duration::from_millis(5)).await;

            // defmt::debug!("Dribbler Interface - Checking for parameter response");
            // Parse incoming packets
            self.process_packets();
            // Check if curret_params_state has updated, assuming that the
            // params state firmware_img_hash field is initialized as 0's
            if self.current_params_state.firmware_img_hash != [0; 4] {
                let current_img_hash = self.current_params_state.firmware_img_hash;
                defmt::debug!("Dribbler Interface - Received parameter response");
                defmt::trace!(
                    "Dribbler Interface - Current device image hash {:x}",
                    current_img_hash
                );
                return current_img_hash;
            };

            Timer::after(Duration::from_millis(5)).await;
        }
    }

    pub async fn check_device_has_latest_default_image(&mut self) -> Result<bool, ()> {
        let latest_img_hash = self.get_latest_default_img_hash();
        defmt::debug!(
            "Dribbler Interface - Latest default image hash - {:x}",
            latest_img_hash
        );

        defmt::trace!("Dribbler Interface - Update UART config 2 MHz");
        self.stm32_uart_interface
            .update_uart_config(2_000_000, Parity::ParityEven)
            .await;
        Timer::after(Duration::from_millis(1)).await;

        let res;
        let timeout = Duration::from_millis(100);
        defmt::trace!("Dribbler Interface - Waiting for device response");
        match with_timeout(timeout, self.get_current_device_img_hash()).await {
            Ok(current_img_hash) => {
                if current_img_hash == latest_img_hash[..4] {
                    defmt::trace!("Dribbler Interface - Device has the latest default image");
                    res = Ok(true);
                } else {
                    defmt::trace!(
                        "Dribbler Interface - Device does not have the latest default image"
                    );
                    res = Ok(false);
                }
            }
            Err(_) => {
                defmt::debug!("Dribbler Interface - No device response, image hash unknown");
                res = Err(());
            }
        }
        // Make sure that the uart queue is empty of any possible parameter
        // response packets, which may cause side effects for the flashing
        // process
        self.process_packets();
        return res;
    }

    pub async fn init_firmware_image(
        &mut self,
        flash: bool,
        fw_image_bytes: &[u8],
    ) -> Result<(), ()> {
        if flash {
            defmt::info!("Dribbler Interface - Flashing firmware image");
            self.stm32_uart_interface
                .load_firmware_image(fw_image_bytes, true)
                .await?;
        } else {
            defmt::info!("Dribbler Interface - Skipping firmware flash");
        }

        self.stm32_uart_interface
            .update_uart_config(2_000_000, Parity::ParityEven)
            .await;

        Timer::after(Duration::from_millis(1)).await;

        // load firmware image call leaves the part in reset, now that our uart is ready, bring the part out of reset
        self.stm32_uart_interface.leave_reset().await;

        return Ok(());
    }

    pub async fn init_default_firmware_image(&mut self, force_flash: bool) -> Result<(), ()> {
        let flash;
        if force_flash {
            defmt::info!("Dribbler Interface - Force flash enabled");
            flash = true
        } else {
            defmt::debug!("Dribbler Interface - Resetting dribbler");
            self.reset().await;
            let res = self.check_device_has_latest_default_image().await;
            match res {
                Ok(has_latest) => {
                    if has_latest {
                        flash = false;
                    } else {
                        flash = true;
                    }
                }
                Err(_) => {
                    flash = true;
                }
            }
        }
        return self.init_firmware_image(flash, self.firmware_image).await;
    }

    pub fn process_packets(&mut self) {
        while let Ok(res) = self.stm32_uart_interface.try_read_data() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<MotorResponse>() {
                defmt::warn!(
                    "Dribbler - Got invalid packet of len {:?} (expected {:?}) data: {:?}",
                    buf.len(),
                    core::mem::size_of::<MotorResponse>(),
                    buf
                );
                continue;
            }

            // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            unsafe {
                // zero initialize a local response packet
                let mut mrp: MotorResponse = { MaybeUninit::zeroed().assume_init() };

                // copy receieved uart bytes into packet
                let state = &mut mrp as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<MotorTelemetry>() {
                    *state.add(i) = buf[i];
                }

                // TODO probably do some checksum stuff eventually

                // decode union type, and reinterpret subtype
                if mrp.type_ == MRP_MOTION {
                    self.current_state = mrp.data.motion;

                    // if mrp.data.motion.master_error() != 0 {
                    //     error!("drib error: {:?}", &mrp.data.motion._bitfield_1.get(0, 32));
                    // }

                    // // // info!("{:?}", defmt::Debug2Format(&mrp.data.motion));
                    // // info!("\n");
                    // // // info!("vel set {:?}", mrp.data.motion.vel_setpoint + 0.);
                    // info!("vel enc {:?}", mrp.data.motion.vel_enc_estimate + 0.);
                    // // // info!("vel hall {:?}", mrp.data.motion.vel_hall_estimate + 0.);
                    // info!("master_error {:?}", mrp.data.motion.master_error());
                    // info!("{:?}", &mrp.data.motion._bitfield_1.get(0, 32));
                    // info!("hall_power_error {:?}", mrp.data.motion.hall_power_error());
                    // info!("hall_disconnected_error {:?}", mrp.data.motion.hall_disconnected_error());
                    // info!("bldc_transition_error {:?}", mrp.data.motion.bldc_transition_error());
                    // info!("bldc_commutation_watchdog_error {:?}", mrp.data.motion.bldc_commutation_watchdog_error());
                    // info!("enc_disconnected_error {:?}", mrp.data.motion.enc_disconnected_error());
                    // info!("enc_decoding_error {:?}", mrp.data.motion.enc_decoding_error());
                    // info!("hall_enc_vel_disagreement_error {:?}", mrp.data.motion.hall_enc_vel_disagreement_error());
                    // info!("overcurrent_error {:?}", mrp.data.motion.overcurrent_error());
                    // info!("undervoltage_error {:?}", mrp.data.motion.undervoltage_error());
                    // info!("overvoltage_error {:?}", mrp.data.motion.overvoltage_error());
                    // info!("torque_limited {:?}", mrp.data.motion.torque_limited());
                    // info!("control_loop_time_error {:?}", mrp.data.motion.control_loop_time_error());
                    // info!("reset_watchdog_independent {:?}", mrp.data.motion.reset_watchdog_independent());
                    // info!("reset_watchdog_window {:?}", mrp.data.motion.reset_watchdog_window());
                    // info!("reset_low_power {:?}", mrp.data.motion.reset_low_power());
                    // info!("reset_software {:?}", mrp.data.motion.reset_software());
                } else if mrp.type_ == MRP_PARAMS {
                    self.current_params_state = mrp.data.params;
                }
            }
        }
    }

    pub fn get_latest_state(&self) -> MotorTelemetry {
        self.current_state
    }

    pub fn log_reset(&self, motor_id: &str) {
        if self.current_state.reset_watchdog_independent() != 0 {
            defmt::warn!("Dribbler {} Reset: Watchdog Independent", motor_id);
        }
        if self.current_state.reset_watchdog_window() != 0 {
            defmt::warn!("Dribbler {} Reset: Watchdog Window", motor_id);
        }
        if self.current_state.reset_low_power() != 0 {
            defmt::warn!("Dribbler {} Reset: Low Power", motor_id);
        }
        if self.current_state.reset_software() != 0 {
            defmt::warn!("Dribbler {} Reset: Software", motor_id);
        }
        if self.current_state.reset_pin() != 0 {
            defmt::warn!("Dribbler {} Reset: Pin", motor_id);
        }
    }

    pub fn send_params_command(&mut self) {
        unsafe {
            let mut cmd: MotorCommandPacket = { MaybeUninit::zeroed().assume_init() };

            cmd.type_ = MCP_PARAMS;
            cmd.crc32 = 0;

            // TODO figure out what to set here
            // Update a param like this
            // cmd.data.params.set_update_timestamp(1);
            // cmd.data.params.timestamp = 0x0;

            let struct_bytes = core::slice::from_raw_parts(
                (&cmd as *const MotorCommandPacket) as *const u8,
                core::mem::size_of::<MotorCommandPacket>(),
            );

            self.stm32_uart_interface.send_or_discard_data(struct_bytes);
        }
    }

    pub fn send_motion_command(&mut self) {
        unsafe {
            let mut cmd: MotorCommandPacket = { MaybeUninit::zeroed().assume_init() };

            cmd.type_ = MCP_MOTION;
            cmd.crc32 = 0;
            cmd.data.motion.set_reset(self.reset_flagged as u32);
            cmd.data
                .motion
                .set_enable_telemetry(self.telemetry_enabled as u32);
            cmd.data.motion.motion_control_type = self.motion_type;
            cmd.data.motion.setpoint = self.setpoint;

            let struct_bytes = core::slice::from_raw_parts(
                (&cmd as *const MotorCommandPacket) as *const u8,
                core::mem::size_of::<MotorCommandPacket>(),
            );

            self.stm32_uart_interface.send_or_discard_data(struct_bytes);
        }

        self.reset_flagged = false;
    }

    pub fn set_motion_type(&mut self, motion_type: MotionCommandType::Type) {
        self.motion_type = motion_type;
    }

    pub fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
    }

    pub fn flag_reset(&mut self) {
        self.reset_flagged = true;
    }

    pub fn set_telemetry_enabled(&mut self, telemetry_enabled: bool) {
        self.telemetry_enabled = telemetry_enabled;
    }

    pub fn read_is_error(&self) -> bool {
        self.current_state.master_error() != 0
    }

    pub fn check_hall_error(&self) -> bool {
        self.current_state.hall_power_error() != 0
            || self.current_state.hall_disconnected_error() != 0
            || self.current_state.hall_enc_vel_disagreement_error() != 0
    }

    pub fn read_current(&self) -> f32 {
        self.current_state.current_estimate
    }

    pub fn ball_detected(&self) -> bool {
        self.current_state.current_estimate > self.ball_detected_thresh
    }
}
