use core::mem::MaybeUninit;

use embassy_stm32::{usart::{self, Parity}, gpio::Pin};
use embassy_time::{Timer, Duration};
use nalgebra::Vector3;

use ateam_common_packets::stspin_packets::{
    MotorResponsePacket,
    MotorResponse_Motion_Packet,
    MotorResponse_Params_Packet,
    MotorResponsePacketType_MRP_PARAMS,
    MotorResponsePacketType_MRP_MOTION, MotorCommand_MotionType};

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
    ResetPin: Pin  
> {
    current_state: MotorResponse_Motion_Packet,
    current_params_state: MotorResponse_Params_Packet,

    stm32_uart_interface: Stm32Interface<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, Boot0Pin, ResetPin>,

    vel_pid_constants: Vector3<f32>,
    vel_pid_i_max: f32,
    torque_pid_constants: Vector3<f32>,
    torque_pid_i_max: f32,
    torque_limit: f32,

    firmware_image: &'a [u8],


    version_major: u8,
    version_minor: u8,
    version_patch: u16,
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
    ResetPin: Pin  
    > WheelMotor<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, Boot0Pin, ResetPin>
{
    pub fn new(stm32_interface: Stm32Interface<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, Boot0Pin, ResetPin>,
            firmware_image: &'a [u8]) -> WheelMotor<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, Boot0Pin, ResetPin> {
        let start_state: MotorResponse_Motion_Packet = {
            unsafe { MaybeUninit::zeroed().assume_init() }
        };
        
        let start_params_state: MotorResponse_Params_Packet = {
            unsafe { MaybeUninit::zeroed().assume_init() }
        };

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
        let res = self.stm32_uart_interface.load_firmware_image(fw_image_bytes).await;

        // this is safe because load firmware image call will reset the target device
        // it will begin issueing telemetry updates
        // these are the only packets it sends so any blocked process should get the data it now needs
        unsafe { self.stm32_uart_interface.update_uart_config(2_000_000, Parity::ParityEven) };
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
            }

            // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            unsafe {
                // zero initialize a local response packet
                let mut mrp: MotorResponsePacket = {
                    MaybeUninit::zeroed().assume_init()
                };
            
                // copy receieved uart bytes into packet
                let state = &mut mrp as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<MotorResponse_Motion_Packet>() {
                    *state.offset(i as isize) = buf[i];
                }

                // TODO probably do some checksum stuff eventually

                // decode union type, and reinterpret subtype
                if mrp.type_ == MotorResponsePacketType_MRP_MOTION {
                    self.current_state = mrp.__bindgen_anon_1.motion;
                } else if mrp.type_ == MotorResponsePacketType_MRP_PARAMS {
                    self.current_params_state = mrp.__bindgen_anon_1.params;
                }
            }
        }
    }

    pub fn set_motion_type(&self, motion_type: MotorCommand_MotionType) {

    }

    pub fn set_setpoint(&self, setpoint: f32) {

    }

    pub fn flag_reset(&self) {

    }

    pub fn send_motion_command(&self) {

    }
}