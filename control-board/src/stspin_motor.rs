use core::{mem::MaybeUninit, f32::consts::PI};

use ateam_lib_stm32::uart::queue::{UartReadQueue, UartWriteQueue};
use defmt::*;
use embassy_stm32::{
    gpio::{Pin, Pull},
    usart::{self, Parity},
};
use embassy_time::{Duration, Timer};
use nalgebra::Vector3;

use ateam_common_packets::bindings::{
    MotorCommandPacket,
    MotorCommandPacketType::MCP_MOTION,
    MotorCommand_MotionType,
    MotorCommand_MotionType::OPEN_LOOP,
    MotorResponsePacket,
    MotorResponsePacketType::{MRP_MOTION, MRP_PARAMS},
    MotorResponse_Motion_Packet, MotorResponse_Params_Packet,
};

use crate::stm32_interface::Stm32Interface;

pub struct WheelMotor<
    'a,
    UART: usart::Instance,
    DmaRx: usart::RxDma<UART>,
    DmaTx: usart::TxDma<UART>,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_RX: usize,
    const DEPTH_TX: usize,
> {
    stm32_uart_interface: Stm32Interface<
        'a,
        UART,
        DmaRx,
        DmaTx,
        LEN_RX,
        LEN_TX,
        DEPTH_RX,
        DEPTH_TX,
    >,
    firmware_image: &'a [u8],

    current_state: MotorResponse_Motion_Packet,
    current_params_state: MotorResponse_Params_Packet,

    version_major: u8,
    version_minor: u8,
    version_patch: u16,
    vel_pid_constants: Vector3<f32>,
    vel_pid_i_max: f32,
    torque_pid_constants: Vector3<f32>,
    torque_pid_i_max: f32,
    torque_limit: f32,

    setpoint: f32,
    motion_type: MotorCommand_MotionType::Type,
    reset_flagged: bool,
    telemetry_enabled: bool,
}

impl<
        'a,
        UART: usart::Instance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize,
        const DEPTH_TX: usize,
    > WheelMotor<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
{
    pub fn new(
        stm32_interface: Stm32Interface<
            'a,
            UART,
            DmaRx,
            DmaTx,
            LEN_RX,
            LEN_TX,
            DEPTH_RX,
            DEPTH_TX,
        >,
        firmware_image: &'a [u8],
    ) -> WheelMotor<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
    {
        let start_state: MotorResponse_Motion_Packet =
            { unsafe { MaybeUninit::zeroed().assume_init() } };

        let start_params_state: MotorResponse_Params_Packet =
            { unsafe { MaybeUninit::zeroed().assume_init() } };

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
        read_queue: &'a UartReadQueue<UART, DmaRx, LEN_RX, DEPTH_RX>,
        write_queue: &'a UartWriteQueue<UART, DmaTx, LEN_TX, DEPTH_TX>,
        boot0_pin: impl Pin,
        reset_pin: impl Pin,
        firmware_image: &'a [u8],
    ) -> WheelMotor<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
    {
        // Need a Pull None to allow for STSPIN watchdog usage.
        let stm32_interface = Stm32Interface::new_from_pins(read_queue, write_queue, boot0_pin, reset_pin, Pull::None, false);

        let start_state: MotorResponse_Motion_Packet =
            { unsafe { MaybeUninit::zeroed().assume_init() } };

        let start_params_state: MotorResponse_Params_Packet =
            { unsafe { MaybeUninit::zeroed().assume_init() } };

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

    pub async fn load_firmware_image(&mut self, fw_image_bytes: &[u8]) -> Result<(), ()> {
        let res = self
            .stm32_uart_interface
            .load_firmware_image(fw_image_bytes)
            .await;

        // this is safe because load firmware image call will reset the target device
        // it will begin issueing telemetry updates
        // these are the only packets it sends so any blocked process should get the data it now needs
        defmt::debug!("update config");
        self.stm32_uart_interface.update_uart_config(2_000_000, Parity::ParityEven).await;
        Timer::after(Duration::from_millis(1)).await;

        // load firmware image call leaves the part in reset, now that our uart is ready, bring the part out of reset
        self.stm32_uart_interface.leave_reset().await;

        return res;
    }

    pub async fn load_default_firmware_image(&mut self) -> Result<(), ()> {
        return self.load_firmware_image(self.firmware_image).await;
    }

    pub fn process_packets(&mut self) {
        while let Ok(res) = self.stm32_uart_interface.try_read_data() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<MotorResponsePacket>() {
                defmt::warn!("got invalid packet of len {:?} data: {:?}", buf.len(), buf);
                continue;
            }

            // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            unsafe {
                // zero initialize a local response packet
                let mut mrp: MotorResponsePacket = { MaybeUninit::zeroed().assume_init() };

                // copy receieved uart bytes into packet
                let state = &mut mrp as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<MotorResponse_Motion_Packet>() {
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
                        error!("motor error: {:?}", &mrp.data.motion._bitfield_1.get(0, 32));
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
                    self.current_params_state = mrp.data.params;
                }
            }
        }
    }

    pub fn log_reset(&self, motor_id: &str) {
        if self.current_state.reset_watchdog_independent() != 0 {
            defmt::warn!("Motor {} Reset: Watchdog Independent", motor_id);
        }
        if self.current_state.reset_watchdog_window() != 0 {
            defmt::warn!("Motor {} Reset: Watchdog Window", motor_id);
        }
        if self.current_state.reset_low_power() != 0 {
            defmt::warn!("Motor {} Reset: Low Power", motor_id);
        }
        if self.current_state.reset_software() != 0 {
            defmt::warn!("Motor {} Reset: Software", motor_id);
        }
        if self.current_state.reset_pin() != 0 {
            defmt::warn!("Motor {} Reset: Pin", motor_id);
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
            //info!("setpoint: {:?}", cmd.data.motion.setpoint);

            let struct_bytes = core::slice::from_raw_parts(
                (&cmd as *const MotorCommandPacket) as *const u8,
                core::mem::size_of::<MotorCommandPacket>(),
            );

            self.stm32_uart_interface.send_or_discard_data(struct_bytes);
        }

        self.reset_flagged = false;
    }

    pub fn get_latest_state(&self) -> MotorResponse_Motion_Packet {
        self.current_state
    }

    pub fn set_motion_type(&mut self, motion_type: MotorCommand_MotionType::Type) {
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

    pub fn read_current(&self) -> f32 {
        return self.current_state.current_estimate;
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

    pub fn read_vel_computed_setpoint(&self) -> f32 {
        return self.current_state.vel_computed_setpoint;
    }
}

pub struct DribblerMotor<
    'a,
    UART: usart::Instance,
    DmaRx: usart::RxDma<UART>,
    DmaTx: usart::TxDma<UART>,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_RX: usize,
    const DEPTH_TX: usize,
> {
    stm32_uart_interface: Stm32Interface<
        'a,
        UART,
        DmaRx,
        DmaTx,
        LEN_RX,
        LEN_TX,
        DEPTH_RX,
        DEPTH_TX,
    >,
    firmware_image: &'a [u8],

    current_state: MotorResponse_Motion_Packet,
    current_params_state: MotorResponse_Params_Packet,

    version_major: u8,
    version_minor: u8,
    version_patch: u16,
    vel_pid_constants: Vector3<f32>,
    vel_pid_i_max: f32,
    torque_pid_constants: Vector3<f32>,
    torque_pid_i_max: f32,
    torque_limit: f32,

    setpoint: f32,
    motion_type: MotorCommand_MotionType::Type,
    reset_flagged: bool,
    telemetry_enabled: bool,

    ball_detected_thresh: f32,
}

impl<
        'a,
        UART: usart::Instance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize,
        const DEPTH_TX: usize,
    > DribblerMotor<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
{
    pub fn new(
        stm32_interface: Stm32Interface<
            'a,
            UART,
            DmaRx,
            DmaTx,
            LEN_RX,
            LEN_TX,
            DEPTH_RX,
            DEPTH_TX,
        >,
        firmware_image: &'a [u8],
        ball_detected_thresh: f32,
    ) -> DribblerMotor<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
    {
        let start_state: MotorResponse_Motion_Packet =
            { unsafe { MaybeUninit::zeroed().assume_init() } };

        let start_params_state: MotorResponse_Params_Packet =
            { unsafe { MaybeUninit::zeroed().assume_init() } };

        DribblerMotor {
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

            ball_detected_thresh: ball_detected_thresh,
        }
    }

    pub fn new_from_pins(
        read_queue: &'a UartReadQueue<UART, DmaRx, LEN_RX, DEPTH_RX>,
        write_queue: &'a UartWriteQueue<UART, DmaTx, LEN_TX, DEPTH_TX>,
        boot0_pin: impl Pin,
        reset_pin: impl Pin,
        firmware_image: &'a [u8],
        ball_detected_thresh: f32,
    ) -> DribblerMotor<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
    {
        // Need a Pull None to allow for STSPIN watchdog usage.
        let stm32_interface = Stm32Interface::new_from_pins(read_queue, write_queue, boot0_pin, reset_pin, Pull::None, false);

        let start_state: MotorResponse_Motion_Packet =
            { unsafe { MaybeUninit::zeroed().assume_init() } };

        let start_params_state: MotorResponse_Params_Packet =
            { unsafe { MaybeUninit::zeroed().assume_init() } };

        DribblerMotor {
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

            ball_detected_thresh: ball_detected_thresh,
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

    pub async fn load_firmware_image(&mut self, fw_image_bytes: &[u8]) -> Result<(), ()> {
        let res = self
            .stm32_uart_interface
            .load_firmware_image(fw_image_bytes)
            .await;

        // this is safe because load firmware image call will reset the target device
        // it will begin issueing telemetry updates
        // these are the only packets it sends so any blocked process should get the data it now needs
        defmt::debug!("update config");
        self.stm32_uart_interface.update_uart_config(2_000_000, Parity::ParityEven).await;
        Timer::after(Duration::from_millis(1)).await;

        // load firmware image call leaves the part in reset, now that our uart is ready, bring the part out of reset
        self.stm32_uart_interface.leave_reset().await;

        return res;
    }

    pub async fn load_default_firmware_image(&mut self) -> Result<(), ()> {
        return self.load_firmware_image(self.firmware_image).await;
    }

    pub fn process_packets(&mut self) {
        while let Ok(res) = self.stm32_uart_interface.try_read_data() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<MotorResponsePacket>() {
                defmt::warn!("got invalid packet of len {:?} data: {:?}", buf.len(), buf);
                continue;
            }

            // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            unsafe {
                // zero initialize a local response packet
                let mut mrp: MotorResponsePacket = { MaybeUninit::zeroed().assume_init() };

                // copy receieved uart bytes into packet
                let state = &mut mrp as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<MotorResponse_Motion_Packet>() {
                    *state.offset(i as isize) = buf[i];
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

    pub fn get_latest_state(&self) -> MotorResponse_Motion_Packet {
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

    pub fn set_motion_type(&mut self, motion_type: MotorCommand_MotionType::Type) {
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

    pub fn ball_detected(&self) -> bool {
        return self.current_state.current_estimate > self.ball_detected_thresh;
    }
}
