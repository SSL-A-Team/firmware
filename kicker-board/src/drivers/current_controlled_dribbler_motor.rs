use core::mem::MaybeUninit;

use ateam_common_packets::bindings::{
    CcmCommand,
    CcmCommandType::{CCM_CMD_MOTION, CCM_CMD_PARAMS},
    CcmMotionControlType::{CCM_MCT_CURRENT, CCM_MCT_MOTOR_OFF, CCM_MCT_VELOCITY_CURRENT},
    CcmParameter::CCM_PARAM_FIRMWARE_IMAGE_HASH,
    CcmParameterDirection::CCM_PARAMDIR_COMMAND,
    CcmParameterOperation::CCM_PARAMOP_READ,
    CcmResponse,
    CcmResponseType::{CCM_RESP_PARAMS, CCM_RESP_TELEM},
    CcmTelemetry,
    DribblerCommand::{
        self, DC_CURRENT, DC_DISABLE, DC_DRIBBLE, DC_HARD_RECEIVE, DC_SOFT_RECEIVE, DC_VELOCITY,
    },
};
use ateam_lib_stm32::{
    drivers::boot::stm32_interface::{get_bootloader_uart_config, Stm32Interface},
    uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue},
};
use embassy_stm32::{
    gpio::{AnyPin, Pull},
    usart::Parity,
    Peri,
};
use embassy_time::{with_timeout, Duration, Timer};

use crate::image_hash;
use crate::DEBUG_DRIB_UART_QUEUES;

/// Number of current samples to average for ball detection.
const BALL_DETECT_SAMPLE_COUNT: usize = 20;

/// 0.0-1.0 setpoint scaling limits.
/// Matches MAX_CURR_DRIBBLER_TURNING (90% peak) in dribbler-torque/main.h.
/// Motor controller still clamps to 1503 mA when not turning.
const MAX_VELOCITY_SETPOINT_RADS: f32 = 500.0;
const MAX_CURRENT_SETPOINT_MA: f32 = 2646.0;

#[derive(defmt::Format)]
pub enum DribFlashOutcome {
    OkHashMatch,
    OkFlashed,
    ErrFlashFail,
    ErrHashTimeoutFlashFail,
}

pub struct CurrentControlledDribblerMotor<
    'a,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_RX: usize,
    const DEPTH_TX: usize,
> {
    stm32_uart_interface:
        Stm32Interface<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, DEBUG_DRIB_UART_QUEUES>,
    firmware_image: &'a [u8],

    current_state: CcmTelemetry,
    current_params_hash: [u8; 4],

    // outbound command state
    motion_control_type: u8,
    setpoint: f32,
    current_setpoint_ma: i16,
    reset_flagged: bool,
    telemetry_enabled: bool,
    params_requested: bool,

    ball_detected_thresh_ma: u16,
}

impl<
        'a,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize,
        const DEPTH_TX: usize,
    > CurrentControlledDribblerMotor<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX>
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
        ball_detected_thresh_ma: u16,
    ) -> Self {
        let current_state: CcmTelemetry = unsafe { MaybeUninit::zeroed().assume_init() };

        CurrentControlledDribblerMotor {
            stm32_uart_interface: stm32_interface,
            firmware_image,

            current_state,
            current_params_hash: [0u8; 4],

            motion_control_type: CCM_MCT_MOTOR_OFF,
            setpoint: 0.0,
            current_setpoint_ma: 0,
            reset_flagged: false,
            telemetry_enabled: false,
            params_requested: false,

            ball_detected_thresh_ma,
        }
    }

    pub fn new_from_pins(
        uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, DEBUG_DRIB_UART_QUEUES>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX, DEBUG_DRIB_UART_QUEUES>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX, DEBUG_DRIB_UART_QUEUES>,
        boot0_pin: Peri<'a, AnyPin>,
        reset_pin: Peri<'a, AnyPin>,
        firmware_image: &'a [u8],
        ball_detected_thresh_ma: u16,
    ) -> Self {
        let stm32_interface = Stm32Interface::new_from_pins(
            uart,
            read_queue,
            write_queue,
            boot0_pin,
            reset_pin,
            Pull::None,
            false,
        );

        Self::new(stm32_interface, firmware_image, ball_detected_thresh_ma)
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

    pub fn set_telemetry_enabled(&mut self, enabled: bool) {
        self.telemetry_enabled = enabled;
    }

    pub fn flag_reset(&mut self) {
        self.reset_flagged = true;
    }

    /// Translate a DribblerCommand + setpoint into CCM motion control fields.
    /// Smart modes (HARD_RECEIVE, SOFT_RECEIVE, DRIBBLE) are stubbed as 0 mA current
    /// until full implementation on the kicker board side.
    pub fn set_drib_command(&mut self, mode: DribblerCommand::Type, setpoint: f32) {
        let setpoint = setpoint.clamp(0.0, 1.0);
        match mode {
            DC_DISABLE => {
                self.motion_control_type = CCM_MCT_MOTOR_OFF;
                self.setpoint = 0.0;
                self.current_setpoint_ma = 0;
            }
            DC_VELOCITY => {
                self.motion_control_type = CCM_MCT_VELOCITY_CURRENT;
                self.setpoint = setpoint * MAX_VELOCITY_SETPOINT_RADS;
                self.current_setpoint_ma = 0;
            }
            DC_CURRENT => {
                self.motion_control_type = CCM_MCT_CURRENT;
                self.setpoint = 0.0;
                self.current_setpoint_ma = (setpoint * MAX_CURRENT_SETPOINT_MA) as i16;
            }
            // smart modes: stubbed as 0 mA current
            DC_HARD_RECEIVE | DC_SOFT_RECEIVE | DC_DRIBBLE => {
                self.motion_control_type = CCM_MCT_CURRENT;
                self.setpoint = 0.0;
                self.current_setpoint_ma = 0;
            }
            _ => {
                self.motion_control_type = CCM_MCT_MOTOR_OFF;
                self.setpoint = 0.0;
                self.current_setpoint_ma = 0;
            }
        }
    }

    pub fn process_packets(&mut self) {
        while let Ok(res) = self.stm32_uart_interface.try_read_data() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<CcmResponse>() {
                defmt::warn!(
                    "CurrentControlledDribblerMotor - invalid packet len {:?} (expected {:?})",
                    buf.len(),
                    core::mem::size_of::<CcmResponse>(),
                );
                continue;
            }

            unsafe {
                let mut resp: CcmResponse = MaybeUninit::zeroed().assume_init();

                let dst = &mut resp as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<CcmResponse>() {
                    *dst.add(i) = buf[i];
                }

                if resp.type_ == CCM_RESP_TELEM {
                    self.current_state = resp.data.motion;
                } else if resp.type_ == CCM_RESP_PARAMS {
                    // extract first 4 bytes of firmware image hash
                    self.current_params_hash
                        .copy_from_slice(&resp.data.params.value.val_u8x4[..4]);
                }
            }
        }
    }

    pub fn send_motion_command(&mut self) {
        unsafe {
            let mut cmd: CcmCommand = MaybeUninit::zeroed().assume_init();

            cmd.type_ = CCM_CMD_MOTION;
            cmd.data.motion.set_reset(self.reset_flagged as u32);
            cmd.data
                .motion
                .set_enable_telemetry(self.telemetry_enabled as u32);
            cmd.data.motion.motion_control_type = self.motion_control_type;
            cmd.data.motion.setpoint = self.setpoint;
            cmd.data.motion.current_setpoint_ma = self.current_setpoint_ma;

            let struct_bytes = core::slice::from_raw_parts(
                (&cmd as *const CcmCommand) as *const u8,
                core::mem::size_of::<CcmCommand>(),
            );

            self.stm32_uart_interface.send_or_discard_data(struct_bytes);
        }

        self.reset_flagged = false;
    }

    pub fn send_params_command(&mut self) {
        unsafe {
            let mut cmd: CcmCommand = MaybeUninit::zeroed().assume_init();

            cmd.type_ = CCM_CMD_PARAMS;
            cmd.data.param.parameter = CCM_PARAM_FIRMWARE_IMAGE_HASH;
            cmd.data.param.parameter_operation = CCM_PARAMOP_READ;
            cmd.data.param.parameter_direction = CCM_PARAMDIR_COMMAND;

            let struct_bytes = core::slice::from_raw_parts(
                (&cmd as *const CcmCommand) as *const u8,
                core::mem::size_of::<CcmCommand>(),
            );

            self.stm32_uart_interface.send_or_discard_data(struct_bytes);
        }
    }

    pub fn get_latest_state(&self) -> CcmTelemetry {
        self.current_state
    }

    pub fn read_is_error(&self) -> bool {
        self.current_state.master_error() != 0
    }

    pub fn check_hall_error(&self) -> bool {
        self.current_state.hall_power_error() != 0
            || self.current_state.hall_disconnected_error() != 0
    }

    /// Average the 20 current samples and compare against threshold.
    pub fn ball_detected(&self) -> bool {
        let samples = &self.current_state.current_telemetry.current_samples_ma;
        let sum: u32 = samples[..BALL_DETECT_SAMPLE_COUNT]
            .iter()
            .map(|&s| s as u32)
            .sum();
        let avg = (sum / BALL_DETECT_SAMPLE_COUNT as u32) as u16;
        avg > self.ball_detected_thresh_ma
    }

    pub fn get_latest_default_img_hash(&self) -> [u8; 16] {
        image_hash::get_dribbler_img_hash()
    }

    async fn get_current_device_img_hash(&mut self) -> [u8; 4] {
        loop {
            self.current_params_hash = [0u8; 4];
            self.send_params_command();

            Timer::after(Duration::from_millis(5)).await;

            self.process_packets();
            if self.current_params_hash != [0u8; 4] {
                let hash = self.current_params_hash;
                defmt::debug!(
                    "CurrentControlledDribblerMotor - device image hash {:x}",
                    hash
                );
                return hash;
            }

            Timer::after(Duration::from_millis(5)).await;
        }
    }

    async fn check_device_has_latest_default_image(&mut self) -> Result<bool, ()> {
        let latest = self.get_latest_default_img_hash();
        defmt::debug!(
            "CurrentControlledDribblerMotor - default image hash {:x}",
            latest
        );

        self.stm32_uart_interface
            .update_uart_config(2_000_000, Parity::ParityEven)
            .await;
        Timer::after(Duration::from_millis(1)).await;

        match with_timeout(
            Duration::from_millis(100),
            self.get_current_device_img_hash(),
        )
        .await
        {
            Ok(current) => {
                if current == latest[..4] {
                    Ok(true)
                } else {
                    Ok(false)
                }
            }
            Err(_) => {
                defmt::debug!("CurrentControlledDribblerMotor - no device response for hash");
                Err(())
            }
        }
    }

    async fn init_firmware_image(&mut self, flash: bool, fw_image: &[u8]) -> Result<(), ()> {
        if flash {
            defmt::debug!("CurrentControlledDribblerMotor - calling load_firmware_image");
            self.stm32_uart_interface
                .load_firmware_image(fw_image, true)
                .await?;
            defmt::debug!("CurrentControlledDribblerMotor - load_firmware_image returned ok");
        } else {
            defmt::debug!("CurrentControlledDribblerMotor - skipping flash (image up to date)");
        }

        defmt::debug!("CurrentControlledDribblerMotor - updating uart config 2MHz");
        self.stm32_uart_interface
            .update_uart_config(2_000_000, Parity::ParityEven)
            .await;

        Timer::after(Duration::from_millis(1)).await;

        defmt::debug!("CurrentControlledDribblerMotor - leaving reset");
        self.stm32_uart_interface.leave_reset().await;

        Ok(())
    }

    pub async fn init_default_firmware_image(&mut self, force_flash: bool) -> DribFlashOutcome {
        let (flash, hash_timed_out) = if force_flash {
            defmt::debug!("CurrentControlledDribblerMotor - force flash enabled");
            (true, false)
        } else {
            defmt::debug!("CurrentControlledDribblerMotor - resetting device");
            self.reset().await;
            defmt::debug!("CurrentControlledDribblerMotor - reset done, checking image hash");
            match self.check_device_has_latest_default_image().await {
                Ok(has_latest) => {
                    defmt::debug!("CurrentControlledDribblerMotor - has_latest={}", has_latest);
                    (!has_latest, false)
                }
                Err(_) => {
                    defmt::info!(
                        "CurrentControlledDribblerMotor - hash check timed out, will flash"
                    );
                    (true, true)
                }
            }
        };

        defmt::debug!(
            "CurrentControlledDribblerMotor - flash={}, hash_timed_out={}",
            flash,
            hash_timed_out
        );

        if !flash {
            let _ = self.init_firmware_image(false, self.firmware_image).await;
            return DribFlashOutcome::OkHashMatch;
        }

        // always drain stale RX bytes and reset UART to bootloader baud before flashing;
        // hash check path leaves 2MHz baud and queued CCM bytes that corrupt bootloader comms
        defmt::debug!(
            "CurrentControlledDribblerMotor - draining rx queue and resetting uart before flash"
        );
        while self.stm32_uart_interface.try_read_data().is_ok() {}
        let bl_cfg = get_bootloader_uart_config();
        self.stm32_uart_interface
            .update_uart_config(bl_cfg.baudrate, bl_cfg.parity)
            .await;

        match self.init_firmware_image(true, self.firmware_image).await {
            Ok(()) => DribFlashOutcome::OkFlashed,
            Err(()) => {
                if hash_timed_out {
                    DribFlashOutcome::ErrHashTimeoutFlashFail
                } else {
                    DribFlashOutcome::ErrFlashFail
                }
            }
        }
    }
}
