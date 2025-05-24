use ateam_lib_stm32::anim::{self, AnimInterface, AnimRepeatMode, Blink, CompositeAnimation, Lerp};
use ateam_lib_stm32::drivers::led::apa102::{apa102_buf_len, Apa102, Apa102Anim};
use embassy_executor::Spawner;
use embassy_stm32::time::{hz, Hertz};
use embassy_time::{Duration, Ticker, Timer};

use smart_leds::colors::{BLACK, GREEN, ORANGE, RED, WHITE, YELLOW};

use crate::robot_state::SharedRobotState;

use crate::{pins::*, MotorIndex};

const DOTSTAR_ANIMATION_RATE: Duration = Duration::from_millis(50);

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum MotorStatusLedCommand {
    Off,
    Configuring,
    Ok,
    Warn,
    Error,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ImuStatusLedCommand {
    Ok
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum RadioStatusLedCommand {
    Ok
} 

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum KickerStatusLedCommand {
    Ok,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ControlGeneralLedCommand {
    Ok,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ControlBoardLedCommand {
    Motor((MotorIndex, MotorStatusLedCommand)),
    Imu(ImuStatusLedCommand),
    Radio(RadioStatusLedCommand),
    Kicker(KickerStatusLedCommand),
    General(ControlGeneralLedCommand),
}

#[macro_export]
macro_rules! create_dotstar_task {
    ($spawner:ident, $robot_state:ident, $led_command_subscriber:ident, $p:ident) => {
        ateam_control_board::tasks::dotstar::start_dotstar_task(&$spawner,
            $robot_state,
            $led_command_subscriber,
            $p.SPI6, $p.PB3, $p.PB5, $p.BDMA_CH0).await;
    };
}

#[embassy_executor::task]
async fn dotstar_task_entry(
    robot_state: &'static SharedRobotState,
    mut led_command_subscriber: LedCommandSubscriber,
    mut dotstars: Apa102<'static, 'static, NUM_LEDS>,
) {
    defmt::info!("user io task initialized");

    // dotstar colors survive soft resets so turn all of them off
    dotstars.set_drv_str_all(32);
    dotstars.set_color_all(BLACK);
    dotstars.update().await;

    let dotstar_animation_update_rate_ticker = Ticker::every(DOTSTAR_ANIMATION_RATE);

    loop {
        if let Some(led_command) = led_command_subscriber.try_next_message_pure() {
            match led_command {
                ControlBoardLedCommand::Motor((motor, status)) => {

                },
                ControlBoardLedCommand::Imu(imu_status_led_command) => {

                },
                ControlBoardLedCommand::Radio(radio_status_led_command) => {

                },
                ControlBoardLedCommand::Kicker(kicker_status_led_command) => {

                },
                ControlBoardLedCommand::General(control_general_led_command) => {

                },
            }
        }
    }
}

const NUM_LEDS: usize = 11;
pub type Apa102Buf = [u8; apa102_buf_len(NUM_LEDS)];

#[link_section = ".sram4"]
static mut DOTSTAR_SPI_BUFFER_CELL: Apa102Buf = [0; apa102_buf_len(NUM_LEDS)];

pub async fn start_dotstar_task(spawner: &Spawner,
    robot_state: &'static SharedRobotState,
    led_command_subscriber: LedCommandSubscriber,
    dotstar_peri: DotstarSpi,
    dotstar_sck_pin: DotstarSpiSck,
    dotstar_mosi_pin: DotstarSpiMosi,
    dotstar_tx_dma: DotstarTxDma,
    ) {

    // defmt::info!("taking buf");
    // let dotstar_spi_buf: &'static mut [u8; 16] = DOTSTAR_SPI_BUFFER_CELL.take();
    // defmt::info!("took buf");

    let dotstar_spi_buf: &'static mut Apa102Buf = unsafe { &mut *(&raw mut DOTSTAR_SPI_BUFFER_CELL) };
    let dotstars = Apa102::<NUM_LEDS>::new_from_pins(dotstar_peri, dotstar_sck_pin, dotstar_mosi_pin, dotstar_tx_dma, dotstar_spi_buf.into());

    spawner.spawn(dotstar_task_entry(robot_state, led_command_subscriber, dotstars)).unwrap();
}