use core::{mem::MaybeUninit, f32::consts::PI};

use ateam_lib_stm32::{drivers::boot::stm32_interface::Stm32Interface, uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue}};
use defmt::*;
use embassy_stm32::{
    gpio::{Pin, Pull},
    usart::Parity,
};
use embassy_time::{with_timeout, Duration, Timer};
use nalgebra::Vector3;

use ateam_common_packets::bindings::{
    MotionCommandType::{self, OPEN_LOOP}, MotorCommandPacket, MotorCommandType::{MCP_MOTION, MCP_PARAMS}, MotorResponse, MotorResponseType::{MRP_MOTION, MRP_PARAMS}, MotorTelemetry, ParameterMotorResponse
};
use crate::{image_hash, DEBUG_MOTOR_UART_QUEUES};

pub struct WheelMotor<
    'a,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_RX: usize,
    const DEPTH_TX: usize,
> {
    stm32_uart_interface: Stm32Interface<
        'a,
        LEN_RX,
        LEN_TX,
        DEPTH_RX,
        DEPTH_TX,
        DEBUG_MOTOR_UART_QUEUES,
    >,
    firmware_image: &'a [u8],
    current_timestamp_ms: u32,
    current_state: MotorTelemetry,
    current_params_state: ParameterMotorResponse,
    version_major: u8,
    version_minor: u8,
    version_patch: u16,
    vel_pid_constants: Vector3<f32>,
    vel_pid_i_max: f32,
    torque_pid_constants: Vector3<f32>,
    torque_pid_i_max: f32,
    torque_limit: f32,

    setpoint: f32,
    motion_type: MotionCommandType::Type,
    reset_flagged: bool,
    telemetry_enabled: bool,
    motion_enabled: bool,
    calibrate_current: bool,
}

impl<
        'a,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize,
        const DEPTH_TX: usize,
    > WheelMotor<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
{
    pub fn new(
        stm32_interface: Stm32Interface<
            'a,
            LEN_RX,
            LEN_TX,
            DEPTH_RX,
            DEPTH_TX,
            DEBUG_MOTOR_UART_QUEUES
        >,
        firmware_image: &'a [u8],
    ) -> WheelMotor<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
    {
        let start_state: MotorTelemetry = Default::default();
        let start_params_state: ParameterMotorResponse = Default::default();

        WheelMotor {
            stm32_uart_interface: stm32_interface,
            firmware_image,

            version_major: 0,
            version_minor: 0,
            version_patch: 0,
            current_timestamp_ms: 0,
            current_state: start_state,
            current_params_state: start_params_state,
            vel_pid_constants: Vector3::new(0.0, 0.0, 0.0),
            vel_pid_i_max: 0.0,
            torque_pid_constants: Vector3::new(0.0, 0.0, 0.0),
            torque_pid_i_max: 0.0,
            torque_limit: 0.0,

            setpoint: 0.0,
            motion_type: OPEN_LOOP,
            reset_flagged: false,
            telemetry_enabled: false,
            motion_enabled: false,
            calibrate_current: false,
        }
    }

    pub fn new_from_pins(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_MOTOR_UART_QUEUES>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_MOTOR_UART_QUEUES>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_MOTOR_UART_QUEUES>,
        boot0_pin: impl Pin,
        reset_pin: impl Pin,
        firmware_image: &'a [u8],
    ) -> WheelMotor<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
    {
        // Need a Pull None to allow for STSPIN watchdog usage.
        let stm32_interface = Stm32Interface::new_from_pins(uart, read_queue, write_queue, boot0_pin, reset_pin, Pull::None, true);

        let start_state: MotorTelemetry = Default::default();
        let start_params_state: ParameterMotorResponse = Default::default();

        WheelMotor {
            stm32_uart_interface: stm32_interface,
            firmware_image,

            version_major: 0,
            version_minor: 0,
            version_patch: 0,
            current_timestamp_ms: 0,
            current_state: start_state,
            current_params_state: start_params_state,
            vel_pid_constants: Vector3::new(0.0, 0.0, 0.0),
            vel_pid_i_max: 0.0,
            torque_pid_constants: Vector3::new(0.0, 0.0, 0.0),
            torque_pid_i_max: 0.0,
            torque_limit: 0.0,

            setpoint: 0.0,
            motion_type: OPEN_LOOP,
            reset_flagged: false,
            telemetry_enabled: false,
            motion_enabled: false,
            calibrate_current: false,
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
        image_hash::get_wheel_img_hash()
    }

    /// Get the first 4 bytes of the currently loaded image hash on the device, Run with timeout!
    pub async fn get_current_device_img_hash(&mut self) -> [u8; 4] {
        loop {
            // defmt::trace!("Wheel Interface - Sending parameter command packet");
            self.send_params_command();

            Timer::after(Duration::from_millis(5)).await;

            // defmt::debug!("Wheel Interface - Checking for parameter response");
            // Parse incoming packets
            self.process_packets();
            // Check if curret_params_state has updated, assuming that the
            // params state firmware_img_hash field is initialized as 0's
            if self.current_params_state.firmware_img_hash != [0; 4] {
                let current_img_hash = self.current_params_state.firmware_img_hash;
                defmt::debug!("Wheel Interface - Received parameter response");
                defmt::trace!("Wheel Interface - Current device image hash {:x}", current_img_hash);
                return current_img_hash
            };

            Timer::after(Duration::from_millis(5)).await;
        }
    }

    pub async fn check_device_has_latest_default_image(&mut self) -> Result<bool, ()> {
        let latest_img_hash = self.get_latest_default_img_hash();
        defmt::debug!("Wheel Interface - Latest default image hash - {:x}", latest_img_hash);

        defmt::trace!("Wheel Interface - Update UART config 2 MHz");
        self.stm32_uart_interface.update_uart_config(2_000_000, Parity::ParityEven).await;
        Timer::after(Duration::from_millis(1)).await;

        let res;
        let timeout = Duration::from_millis(100);
        defmt::trace!("Wheel Interface - Waiting for device response");
        match with_timeout(timeout, self.get_current_device_img_hash()).await {
            Ok(current_img_hash) => {
                if current_img_hash == latest_img_hash[..4] {
                    defmt::trace!("Wheel Interface - Device has the latest default image");
                    res = Ok(true);
                } else {
                    defmt::trace!("Wheel Interface - Device does not have the latest default image");
                    res = Ok(false);
                }
            },
            Err(_) => {
                defmt::debug!("Wheel Interface - No device response, image hash unknown");
                res = Err(());
            },
        }

        // Make sure that the uart queue is empty of any possible parameter
        // response packets, which may cause side effects for the flashing
        // process
        self.process_packets();
        return res;
    }

    pub async fn init_firmware_image(&mut self, flash: bool, fw_image_bytes: &[u8]) -> Result<(), ()> {
        if flash {
            defmt::info!("Wheel Interface - Flashing firmware image");
            self.stm32_uart_interface.load_firmware_image(fw_image_bytes, true).await?;
        } else {
            defmt::info!("Wheel Interface - Skipping firmware flash");
        }

        self.stm32_uart_interface.update_uart_config(2_000_000, Parity::ParityEven).await;

        Timer::after(Duration::from_millis(1)).await;

        // load firmware image call leaves the part in reset, now that our uart is ready, bring the part out of reset
        self.stm32_uart_interface.leave_reset().await;

        return Ok(());
    }

    pub async fn init_default_firmware_image(&mut self, force_flash: bool) -> Result<(), ()> {
        let flash;
        if force_flash {
            defmt::info!("Wheel Interface - Force flash enabled");
            flash = true
        } else {
            defmt::debug!("Wheel Interface - Resetting motor controller");
            self.reset().await;
            let res = self.check_device_has_latest_default_image().await;
            match res {
                Ok(has_latest) => {
                    if has_latest {
                        flash = false;
                    } else {
                        flash = true;
                    }
                },
                Err(_) => {
                    flash = true;
                }
            }
        }
        return self.init_firmware_image(flash, self.firmware_image).await;
    }
    
    pub async fn save_motor_current_constants(&mut self, current_constant: f32) -> Result<(), ()> {
        defmt::debug!("Drive Motor - Saving motor current constant: {:?}", current_constant);
        self.stm32_uart_interface.write_current_calibration_constants(current_constant).await

    }


    pub fn process_packets(&mut self) {
        while let Ok(res) = self.stm32_uart_interface.try_read_data() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<MotorResponse>() {
                defmt::warn!("Drive Motor - Got invalid packet of len {:?} (expected {:?}) data: {:?}", buf.len(), core::mem::size_of::<MotorResponse>(), buf);
                continue;
            }

            // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            unsafe {
                // zero initialize a local response packet
                let mut mrp: MotorResponse = Default::default();

                // copy receieved uart bytes into packet
                let state = &mut mrp as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<MotorResponse>() {
                    *state.offset(i as isize) = buf[i];
                }

                self.current_timestamp_ms = mrp.timestamp;
                // TODO probably do some checksum stuff eventually

                // decode union type, and reinterpret subtype
                if mrp.type_ == MRP_MOTION {
                    self.current_state = mrp.data.motion;
                    // // // info!("{:?}", defmt::Debug2Format(&mrp.data.motion));
                    // // info!("\n");
                    // // // info!("vel set {:?}", mrp.data.motion.vel_setpoint + 0.);
                    // info!("vel enc {:?}", mrp.data.motion.vel_enc_estimate + 0.);
                    // // // info!("vel hall {:?}", mrp.data.motion.vel_hall_estimate + 0.);
                    if mrp.data.motion.master_error() != 0 {
                        error!("Drive Motor - Error: {:?}", &mrp.data.motion._bitfield_1.get(0, 32));
                    }
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
                    trace!("Received parameter response packet");
                    debug!("Parameter response data: {:?}", buf);
                    self.current_params_state = mrp.data.params;
                }
            }
        }
    }

    pub fn log_reset(&self, motor_id: &str) {
        if self.current_state.reset_watchdog_independent() != 0 {
            defmt::warn!("Drive Motor {} Reset: Watchdog Independent", motor_id);
        }
        if self.current_state.reset_watchdog_window() != 0 {
            defmt::warn!("Drive Motor {} Reset: Watchdog Window", motor_id);
        }
        if self.current_state.reset_low_power() != 0 {
            defmt::warn!("Drive Motor {} Reset: Low Power", motor_id);
        }
        if self.current_state.reset_software() != 0 {
            defmt::warn!("Drive Motor {} Reset: Software", motor_id);
        }
        if self.current_state.reset_pin() != 0 {
            defmt::warn!("Drive Motor {} Reset: Pin", motor_id);
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
            cmd.data.motion.set_enable_telemetry(self.telemetry_enabled as u32);
            cmd.data.motion.set_enable_motion(self.motion_enabled as u32);
            cmd.data.motion.set_calibrate_current(self.calibrate_current as u32);
            cmd.data.motion.motion_control_type = self.motion_type;
            cmd.data.motion.setpoint = self.setpoint;
            //info!("setpoint: {:?}", cmd.data.motion.setpoint);

            let struct_bytes = core::slice::from_raw_parts(
                (&cmd as *const MotorCommandPacket) as *const u8,
                core::mem::size_of::<MotorCommandPacket>(),
            );

            self.stm32_uart_interface.send_or_discard_data(struct_bytes);
        }

        self.reset_flagged = false;
    }

    pub fn get_latest_state(&self) -> MotorTelemetry {
        self.current_state
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

    pub fn set_motion_enabled(&mut self, enabled: bool) {
        self.motion_enabled = enabled;
    }

    pub fn set_calibrate_current(&mut self, calibrate_current: bool) {
        self.calibrate_current = calibrate_current;
    }

    pub fn read_current_timestamp_ms(&self) -> u32 {
        return self.current_timestamp_ms;
    }

    pub fn read_is_error(&self) -> bool {
        return self.current_state.master_error() != 0;
    }

    pub fn check_hall_error(&self) -> bool {
        return self.current_state.hall_power_error() != 0 || self.current_state.hall_disconnected_error() != 0 || self.current_state.hall_enc_vel_disagreement_error() != 0;
    }

    pub fn read_encoder_delta(&self) -> i32 {
        return self.current_state.encoder_delta;
    }

    pub fn read_rads(&self) -> f32 {
        return self.current_state.vel_enc_estimate;
    }

    pub fn read_rpm(&self) -> f32 {
        return self.current_state.vel_enc_estimate * 60.0 / (2.0 * PI);
    }

    pub fn read_vel_setpoint(&self) -> f32 {
        return self.current_state.vel_setpoint;
    }

    pub fn read_vel_computed_duty(&self) -> f32 {
        return self.current_state.vel_computed_duty;
    }

    pub fn read_current(&self) -> f32 {
        return self.current_state.current_estimate;
    }

    pub fn read_torque_setpoint(&self) -> f32 {
        return self.current_state.torque_setpoint;
    }

    pub fn read_torque_estimate(&self) -> f32 {
        return self.current_state.torque_estimate;
    }

    pub fn read_torque_computed_error(&self) -> f32 {
        return self.current_state.torque_computed_error;
    }

    pub fn read_torque_computed_nm(&self) -> f32 {
        return self.current_state.torque_computed_nm;
    }

    pub fn read_torque_computed_duty(&self) -> f32 {
        return self.current_state.torque_computed_duty;
    }

    pub fn read_vbus_voltage(&self) -> f32 {
        return self.current_state.vbus_voltage;
    }
}
