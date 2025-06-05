use ateam_lib_stm32::drivers::led::apa102::{apa102_buf_len, Apa102};
use embassy_executor::Spawner;

use smart_leds::colors::{BLACK, BLUE, CYAN, GREEN, ORANGE_RED, PURPLE, RED, VIOLET, YELLOW};

use crate::{pins::*, MotorIndex};

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
    Configuring,
    Ok,
    Error,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum RadioStatusLedCommand {
    Off,
    ConnectedPhys,
    ConnectedUart,
    ConnectedNetwork,
    ConnectedSoftware,
    Ok,
    Error,
} 

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum KickerStatusLedCommand {
    Ok,
    Error,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ControlGeneralLedCommand {
    ShutdownRequested,
    BallDetected,
    BallNotDetected,
    Ok,
    Warn,
    Error,
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
    ($spawner:ident, $led_command_subscriber:ident, $p:ident) => {
        ateam_control_board::tasks::dotstar_task::start_dotstar_task(&$spawner,
            $led_command_subscriber,
            $p.SPI6, $p.PB3, $p.PB5, $p.BDMA_CH0).await;
    };
}

#[embassy_executor::task]
async fn dotstar_task_entry(
    mut led_command_subscriber: LedCommandSubscriber,
    mut dotstars: Apa102<'static, 'static, NUM_LEDS>,
) {
    defmt::debug!("user io task initialized");

    // dotstar colors survive soft resets so turn all of them off
    dotstars.set_drv_str_all(32);
    dotstars.set_color_all(BLACK);
    dotstars.update().await;
    
    loop {
        let led_command = led_command_subscriber.next_message_pure().await;
        match led_command {
            ControlBoardLedCommand::Motor((motor, status)) => {
                let led_index = match motor {
                    MotorIndex::FrontLeft => ControlDotstarIndex::MotorFrontLeft,
                    MotorIndex::BackLeft => ControlDotstarIndex::MotorBackLeft,
                    MotorIndex::BackRight => ControlDotstarIndex::MotorBackRight,
                    MotorIndex::FrontRight => ControlDotstarIndex::MotorFrontRight,
                };

                let led_color = match status {
                    MotorStatusLedCommand::Off => BLACK,
                    MotorStatusLedCommand::Configuring => PURPLE,
                    MotorStatusLedCommand::Ok => GREEN,
                    MotorStatusLedCommand::Warn => YELLOW,
                    MotorStatusLedCommand::Error => RED,
                };

                dotstars.set_color(led_color, led_index as usize);
            },
            ControlBoardLedCommand::Imu(imu_status_led_command) => {
                match imu_status_led_command {
                    ImuStatusLedCommand::Configuring => dotstars.set_color(PURPLE, ControlDotstarIndex::Imu.into()),
                    ImuStatusLedCommand::Ok=> dotstars.set_color(GREEN, ControlDotstarIndex::Imu.into()),
                    ImuStatusLedCommand::Error => dotstars.set_color(RED, ControlDotstarIndex::Imu.into()), 
                }
            },
            ControlBoardLedCommand::Radio(radio_status_led_command) => {
                match radio_status_led_command {
                    RadioStatusLedCommand::Off => dotstars.set_color(BLACK, ControlDotstarIndex::Radio.into()),
                    RadioStatusLedCommand::ConnectedPhys => dotstars.set_color(CYAN, ControlDotstarIndex::Radio.into()),
                    RadioStatusLedCommand::ConnectedUart => dotstars.set_color(BLUE, ControlDotstarIndex::Radio.into()),
                    RadioStatusLedCommand::ConnectedNetwork => dotstars.set_color(VIOLET, ControlDotstarIndex::Radio.into()),
                    RadioStatusLedCommand::ConnectedSoftware => dotstars.set_color(GREEN, ControlDotstarIndex::Radio.into()),
                    RadioStatusLedCommand::Ok=> dotstars.set_color(GREEN, ControlDotstarIndex::Radio.into()),
                    RadioStatusLedCommand::Error => dotstars.set_color(RED, ControlDotstarIndex::Radio.into())
                }
            },
            ControlBoardLedCommand::Kicker(kicker_status_led_command) => {
                match kicker_status_led_command {
                    KickerStatusLedCommand::Ok => dotstars.set_color(GREEN, ControlDotstarIndex::Kicker.into()),
                    KickerStatusLedCommand::Error => dotstars.set_color(RED, ControlDotstarIndex::Kicker.into()), }
            },
            ControlBoardLedCommand::General(control_general_led_command) => {
                match control_general_led_command {
                    ControlGeneralLedCommand::ShutdownRequested => dotstars.set_color(ORANGE_RED, ControlDotstarIndex::User2.into()),
                    ControlGeneralLedCommand::BallDetected => dotstars.set_color(BLUE, ControlDotstarIndex::User2.into()),
                    ControlGeneralLedCommand::BallNotDetected => dotstars.set_color(BLACK, ControlDotstarIndex::User2.into()),
                    ControlGeneralLedCommand::Ok => dotstars.set_color(GREEN, ControlDotstarIndex::User1.into()),
                    ControlGeneralLedCommand::Warn => dotstars.set_color(YELLOW, ControlDotstarIndex::User1.into()),
                    ControlGeneralLedCommand::Error => dotstars.set_color(RED, ControlDotstarIndex::User1.into()),
                }
            },
        }

        dotstars.update().await;
    }
}

const NUM_LEDS: usize = core::mem::variant_count::<ControlDotstarIndex>();
const LED_BUF_LEN: usize = apa102_buf_len(NUM_LEDS);

pub enum ControlDotstarIndex {
    MotorFrontLeft = 1,
    MotorBackLeft = 0,
    MotorBackRight = 3,
    MotorFrontRight = 2,
    User2 = 4,
    User1 = 5,
    Power = 6,
    Kicker = 7,
    Imu = 8,
    Optical = 9,
    Radio = 10,
}

impl From<ControlDotstarIndex> for usize {
    fn from(value: ControlDotstarIndex) -> Self {
        value as usize
    }
}

pub type Apa102Buf = [u8; LED_BUF_LEN];
#[link_section = ".sram4"]
static mut DOTSTAR_SPI_BUFFER_CELL: Apa102Buf = [0; LED_BUF_LEN];

pub async fn start_dotstar_task(spawner: &Spawner,
    led_command_subscriber: LedCommandSubscriber,
    dotstar_peri: DotstarSpi,
    dotstar_sck_pin: DotstarSpiSck,
    dotstar_mosi_pin: DotstarSpiMosi,
    dotstar_tx_dma: DotstarTxDma,
    ) {

    let dotstar_spi_buf: &'static mut Apa102Buf = unsafe { &mut *(&raw mut DOTSTAR_SPI_BUFFER_CELL) };
    let dotstars = Apa102::<NUM_LEDS>::new_from_pins(dotstar_peri, dotstar_sck_pin, dotstar_mosi_pin, dotstar_tx_dma, dotstar_spi_buf.into());

    spawner.spawn(dotstar_task_entry(led_command_subscriber, dotstars)).unwrap();
}