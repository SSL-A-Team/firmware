use ateam_common::task::Task;
use embassy_sync::{blocking_mutex::{raw::CriticalSectionRawMutex}, signal::Signal, mutex::Mutex};

#[derive(Clone, Copy, Debug)]
pub enum RadioState {
    Reset,
    Initializing,
    NetworkUp,
    Broadcasting,
    Connected,
}

#[derive(Clone, Copy, Debug)]
pub enum MotorState<E = ()> {
    Reset,
    Initializing,
    Error(E),
    Connected,
}

#[derive(Clone, Copy, Debug)]
pub enum KickerState {
    Reset,
    Initializing,
    Error,
    Connected,
}

#[derive(Clone, Copy, Debug)]
pub enum ImuState {
    Reset,
    Error,
    Connected,
}

#[derive(Clone, Copy, Debug)]
pub enum TeamColor {
    Yellow,
    Blue,
}

#[derive(Clone, Copy, Debug)]
pub struct RobotState {
    pub robot_id: u8,
    pub team: TeamColor,
    pub wifi_ssid: &'static str,
    pub wifi_pass: &'static str,
    pub radio: RadioState,
    pub kicker: KickerState,
    pub imu: ImuState,
    pub motor_fr: MotorState,
    pub motor_fl: MotorState,
    pub motor_br: MotorState,
    pub motor_bl: MotorState,
    pub motor_drib: MotorState,
}

struct RobotStateTask {
    state: Mutex<CriticalSectionRawMutex, RobotState>,
    state_update: Signal<CriticalSectionRawMutex, RobotState>,
}

// async fn test() {
// //    let string: &str =  "".into();
//     core::future::pending().await
// }

impl RobotStateTask {
    pub async fn get_state(&self) -> RobotState {
        *self.state.lock().await
    }
}

impl Task for RobotStateTask {
    type Data = &'static RobotStateTask;

    async fn task(data: Self::Data) {

    }
}