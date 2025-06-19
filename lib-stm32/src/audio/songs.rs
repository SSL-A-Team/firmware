#[derive(Clone, Copy, Debug)]
pub enum SongId {
    Test,
    PowerOn,
    PowerOff,
    ShutdownRequested,
    BatteryLow,
    BatteryCritical,
    BatteryUhOh,
    BalanceConnected,
    BalanceDisconnected
}