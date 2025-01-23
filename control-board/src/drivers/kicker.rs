use ateam_lib_stm32::uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue};
use embassy_stm32::{gpio::Pin, usart::Parity};
use embassy_time::{Duration, Timer};

use crate::stm32_interface::Stm32Interface;

use ateam_common_packets::bindings::{KickerControl, KickerTelemetry, KickRequest};

pub struct Kicker<
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
        >,
        firmware_image: &'a [u8],
    ) -> Kicker<'a, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX> {
        Self {
            stm32_uart_interface: stm32_interface,
            firmware_image,

            command_state: get_empty_control_packet(),
            telemetry_state: get_empty_telem_packet(),

            telemetry_enabled: false,
        }
    }

    pub fn new_from_pins(uart: &'a IdleBufferedUart<LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX>,
        read_queue: &'a UartReadQueue<LEN_RX, DEPTH_RX>,
        write_queue: &'a UartWriteQueue<LEN_TX, DEPTH_TX>,
        boot0_pin: impl Pin,
        reset_pin: impl Pin,
        firmware_image: &'a [u8]) -> Self {

        let stm32_interface = Stm32Interface::new_from_pins(uart, read_queue, write_queue, boot0_pin, reset_pin, true);

        Self::new(stm32_interface, firmware_image)
    }

    pub fn process_telemetry(&mut self) -> bool {
        let mut received_packet = false;
        while let Ok(res) = self.stm32_uart_interface.try_read_data() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<KickerTelemetry>() {
                defmt::warn!("kicker interface - invalid packet of len {:?} data: {:?}", buf.len(), buf);
                continue;
            }

            received_packet = true;

            // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            unsafe {
                // copy receieved uart bytes into packet
                let state = &mut self.telemetry_state as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<KickerTelemetry>() {
                    *state.offset(i as isize) = buf[i];
                }
            }
        }

        return received_packet; 
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

    pub fn set_telemetry_enabled(&mut self, telemetry_enabled: bool) {
        self.telemetry_enabled = telemetry_enabled;
        self.command_state.set_telemetry_enabled(telemetry_enabled as u32);
    }

    pub fn request_kick(&mut self, request: u32) {
        self.command_state.kick_request = request;
    }

    pub fn set_kick_strength(&mut self, kick_str: f32) {
        self.command_state.kick_speed = kick_str;
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

    pub async fn init_firmware_image(&mut self, flash: bool, fw_image_bytes: &[u8]) -> Result<(), ()> {
        if flash {
            self.stm32_uart_interface.load_firmware_image(fw_image_bytes).await?;
        } else {
            defmt::warn!("currently skipping kicker firmware flash");
        }

        self.stm32_uart_interface.update_uart_config(2_000_000, Parity::ParityEven).await;

        Timer::after(Duration::from_millis(1)).await;

        // load firmware image call leaves the part in reset, now that our uart is ready, bring the part out of reset
        self.stm32_uart_interface.leave_reset().await;

        return Ok(());
    }

    pub async fn init_default_firmware_image(&mut self, flash: bool) -> Result<(), ()> {
        return self.init_firmware_image(flash, self.firmware_image).await;
    }
}

fn get_empty_control_packet() -> KickerControl {
    KickerControl {
        _bitfield_align_1: [],
        _bitfield_1: KickerControl::new_bitfield_1(0, 0, 0),
        kick_request: KickRequest::KR_DISABLE,
        kick_speed: 0.0,
    }
}

fn get_empty_telem_packet() -> KickerTelemetry {
    KickerTelemetry {
        _bitfield_align_1: [],
        _bitfield_1: KickerTelemetry::new_bitfield_1(0, 0, 0, 0),
        rail_voltage: 0.0,
        battery_voltage: 0.0,
    }
}