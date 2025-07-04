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
use crate::image_hash;

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
    >,
    firmware_image: &'a [u8],

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
        }
    }

    pub fn new_from_pins(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX>,
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

    pub async fn check_wheel_needs_flash(&mut self) -> bool {
        let mut wheel_needs_flash = true;
        let wheel_img_hash_ctrl: [u8; 16] = image_hash::get_wheel_img_hash();
        defmt::debug!("Wheel Image Hash from Control Board - {:x}", wheel_img_hash_ctrl);

        defmt::trace!("Drive Motor - Update UART config 2 MHz");
        self.stm32_uart_interface.update_uart_config(2_000_000, Parity::ParityEven).await;
        Timer::after(Duration::from_millis(1)).await;

        let wheel_img_hash_future = async {
            loop {
                defmt::trace!("Drive Motor - sending parameter command packet");
                self.send_params_command();

                Timer::after(Duration::from_millis(10)).await;

                defmt::debug!("Drive Motor - Checking for parameter response");
                // Parse incoming packets
                self.process_packets();
                // Check if curret_params_state has updated
                if self.current_params_state.wheel_img_hash != [0; 4] {
                    let wheel_img_hash_motr = self.current_params_state.wheel_img_hash;
                    defmt::debug!("Drive Motor - Received parameter response");
                    defmt::trace!("Wheel Image Hash from Motor - {:x}", wheel_img_hash_motr);
                    return wheel_img_hash_motr
                };
            }
        };
        let wheel_response_timeout = Duration::from_millis(100);

        defmt::trace!("Drive Motor - waiting for parameter response packet");
        match with_timeout(wheel_response_timeout, wheel_img_hash_future).await {
            Ok(wheel_img_hash_motr) => {
                if wheel_img_hash_motr == wheel_img_hash_ctrl[..4] {
                    wheel_needs_flash = false;
                }
            },
            Err(_) => {
                defmt::debug!("Drive Motor - No parameter response, motor controller needs flashing");
                wheel_needs_flash = true;
            },
        }
        return wheel_needs_flash;
    }

    pub async fn load_firmware_image(&mut self, fw_image_bytes: &[u8]) -> Result<(), ()> {
        let controller_needs_flash: bool = self.check_wheel_needs_flash().await;
        defmt::debug!("Motor Controller Needs Flash - {:?}", controller_needs_flash);

        let res;
        if controller_needs_flash {
            defmt::trace!("UART config updated");
            res = self.stm32_uart_interface.load_firmware_image(fw_image_bytes).await;
        } else {
            defmt::info!("Wheel image is up to date, skipping flash");
            res = Ok(());
        }

        // let res = self.stm32_uart_interface.load_firmware_image(fw_image_bytes).await;

        // this is safe because load firmware image call will reset the target device
        // it will begin issueing telemetry updates
        // these are the only packets it sends so any blocked process should get the data it now needs
        defmt::debug!("Drive Motor - Update config");
        self.stm32_uart_interface.update_uart_config(2_000_000, Parity::ParityEven).await;
        Timer::after(Duration::from_millis(1)).await;

        return res;
    }

    pub async fn load_default_firmware_image(&mut self) -> Result<(), ()> {
        return self.load_firmware_image(self.firmware_image).await;
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

    pub fn read_is_error(&self) -> bool {
        return self.current_state.master_error() != 0;
    }

    pub fn check_hall_error(&self) -> bool {
        return self.current_state.hall_power_error() != 0 || self.current_state.hall_disconnected_error() != 0 || self.current_state.hall_enc_vel_disagreement_error() != 0;
    }

    pub fn read_current(&self) -> f32 {
        return self.current_state.current_estimate;
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

    pub fn read_vel_computed_setpoint(&self) -> f32 {
        return self.current_state.vel_computed_setpoint;
    }
}
