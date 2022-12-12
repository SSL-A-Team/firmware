use core::mem::MaybeUninit;

use embassy_stm32::{
    gpio::Pin,
    usart::{self, Parity},
};
use embassy_time::{Duration, Timer};
use nalgebra::Vector3;

use ateam_common_packets::bindings_stspin::{
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
    UART: usart::BasicInstance,
    DmaRx: usart::RxDma<UART>,
    DmaTx: usart::TxDma<UART>,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_RX: usize,
    const DEPTH_TX: usize,
    Boot0Pin: Pin,
    ResetPin: Pin,
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
        Boot0Pin,
        ResetPin,
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
        UART: usart::BasicInstance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize,
        const DEPTH_TX: usize,
        Boot0Pin: Pin,
        ResetPin: Pin,
    > WheelMotor<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, Boot0Pin, ResetPin>
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
            Boot0Pin,
            ResetPin,
        >,
        firmware_image: &'a [u8],
    ) -> WheelMotor<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, Boot0Pin, ResetPin>
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
        unsafe {
            self.stm32_uart_interface
                .update_uart_config(2_000_000, Parity::ParityEven)
        };
        Timer::after(Duration::from_millis(1)).await;

        // load firmware image call leaves the part in reset, now that our uart is ready, bring the part out of reset
        self.stm32_uart_interface.leave_reset().await?;

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

                    let err = self.current_state.current_estimate;
                    defmt::info!("current read: {:?}", err);
                } else if mrp.type_ == MRP_PARAMS {
                    self.current_params_state = mrp.data.params;
                }
            }
        }
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
}
