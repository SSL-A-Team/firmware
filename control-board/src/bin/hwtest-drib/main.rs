#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]
#![feature(generic_const_exprs)]

use ateam_common_packets::bindings::{
    DribblerCommand::{
        self, DC_CURRENT, DC_DISABLE, DC_DRIBBLE, DC_HARD_RECEIVE, DC_SOFT_RECEIVE, DC_VELOCITY,
    },
    KickerTelemetry,
};
use ateam_control_board::{
    drivers::kicker::Kicker, get_system_config, include_kicker_bin, SystemIrqs,
    DEBUG_KICKER_UART_QUEUES,
};
use ateam_lib_stm32::{
    drivers::boot::stm32_interface::{self, Stm32Interface},
    idle_buffered_uart_spawn_tasks, static_idle_buffered_uart,
};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    gpio::{Input, Pull},
    interrupt,
    pac::Interrupt,
    usart::Uart,
};
use embassy_time::{Duration, Ticker, Timer};
use {defmt_rtt as _, panic_probe as _};

include_kicker_bin! {KICKER_FW_IMG, "kicker.bin"}

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = core::mem::size_of::<KickerTelemetry>();
const RX_BUF_DEPTH: usize = 20;
static_idle_buffered_uart!(KICKER, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, DEBUG_KICKER_UART_QUEUES, #[link_section = ".axisram.buffers"]);

static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[allow(non_snake_case)]
#[interrupt]
unsafe fn CEC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

// Mode table. DC_DISABLE is not cycled — reached via the back action (DOWN at setpoint=0).
const MODE_COUNT: usize = 5;
const MODES: [DribblerCommand::Type; MODE_COUNT] = [
    DC_HARD_RECEIVE,
    DC_SOFT_RECEIVE,
    DC_DRIBBLE,
    DC_VELOCITY,
    DC_CURRENT,
];
const MODE_NAMES: [&str; MODE_COUNT] = [
    "HARD_RECEIVE",
    "SOFT_RECEIVE",
    "DRIBBLE",
    "VELOCITY",
    "CURRENT",
];
// HARD_RECEIVE / SOFT_RECEIVE / DRIBBLE: dimensionless 0.0-1.0, step 0.1
// VELOCITY: rad/s — max and step TBD experimentally, initial values used
// CURRENT: mA     — max and step TBD experimentally, initial values used
const MODE_MAX: [f32; MODE_COUNT] = [1.0, 1.0, 1.0, 500.0, 1500.0];
const MODE_STEP: [f32; MODE_COUNT] = [0.1, 0.1, 0.1, 5.0, 50.0];

// Print telemetry every 200 ms (400 ticks at 500 µs/tick)
const TELEM_PRINT_TICKS: u32 = 400;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let stm32_config = get_system_config();
    let p = embassy_stm32::init(stm32_config);

    defmt::info!("hwtest-drib startup");

    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CEC,
        embassy_stm32::interrupt::Priority::P5,
    );
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    // Buttons: active-low, internal pull-up
    // PE10=back(toggle telemetry print)
    // PE11=center(toggle), PE12=left(prev mode), PE13=right(next mode)
    // PE14=up(+setpoint),  PE15=down(-setpoint; at 0 while disabled = back/reset)
    let btn_back = Input::new(p.PE10, Pull::Up);
    let btn_center = Input::new(p.PE11, Pull::Up);
    let btn_left = Input::new(p.PE12, Pull::Up);
    let btn_right = Input::new(p.PE13, Pull::Up);
    let btn_up = Input::new(p.PE14, Pull::Up);
    let btn_down = Input::new(p.PE15, Pull::Up);

    let kicker_usart = Uart::new(
        p.UART8,
        p.PE0,
        p.PE1,
        SystemIrqs,
        p.DMA2_CH2,
        p.DMA2_CH3,
        stm32_interface::get_bootloader_uart_config(),
    )
    .unwrap();

    KICKER_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, KICKER, kicker_usart);

    let kicker_stm32_interface = Stm32Interface::new_from_pins(
        &KICKER_IDLE_BUFFERED_UART,
        KICKER_IDLE_BUFFERED_UART.get_uart_read_queue(),
        KICKER_IDLE_BUFFERED_UART.get_uart_write_queue(),
        p.PG2.into(),
        p.PG3.into(),
        Pull::Up,
        true,
    );

    defmt::info!("flashing kicker...");
    let mut kicker = Kicker::new(kicker_stm32_interface, KICKER_FW_IMG);
    let res = kicker.init_default_firmware_image(false).await;
    if res.is_err() {
        defmt::warn!("kicker flash failed");
    } else {
        defmt::info!("kicker flash complete");
    }

    kicker.set_telemetry_enabled(true);

    // Block until kicker reports dribbler firmware load attempted
    defmt::info!("waiting for dribbler firmware load...");
    let mut wait_ticks: u32 = 0;
    loop {
        kicker.process_telemetry();
        kicker.send_command();
        if kicker.get_lastest_state().dribbler_fw_loaded() != 0 {
            break;
        }
        wait_ticks += 1;
        if wait_ticks % 20 == 0 {
            defmt::info!("  still waiting for dribbler fw ({} s)...", wait_ticks / 20);
        }
        Timer::after(Duration::from_millis(50)).await;
    }
    defmt::info!("dribbler firmware load complete (success or error), starting test");

    // Test state
    let mut mode_idx: usize = 4; // start in CURRENT mode
    let mut setpoints: [f32; MODE_COUNT] = [0.0; MODE_COUNT]; // per-mode setpoint retained across mode switches
    let mut enabled: bool = false;
    let mut telem_print: bool = false;
    let mut telem_tick: u32 = 0;

    // Button edge detection + debounce (500 µs tick, 200 ticks = 100 ms cooldown)
    const BTN_COOLDOWN: u32 = 200;
    let mut prev_back = false;
    let mut prev_center = false;
    let mut prev_left = false;
    let mut prev_right = false;
    let mut prev_up = false;
    let mut prev_down = false;
    let mut cd_back: u32 = 0;
    let mut cd_center: u32 = 0;
    let mut cd_left: u32 = 0;
    let mut cd_right: u32 = 0;
    let mut cd_up: u32 = 0;
    let mut cd_down: u32 = 0;

    defmt::info!(
        "Ready. BACK=toggle telem, CENTER=toggle, LEFT/RIGHT=mode, UP/DOWN=setpoint, DOWN@0=back/reset"
    );
    defmt::info!(
        "Mode: {}, Setpoint: {}, Enabled: {}",
        MODE_NAMES[mode_idx],
        setpoints[mode_idx],
        enabled
    );

    let mut ticker = Ticker::every(Duration::from_micros(500));
    loop {
        kicker.process_telemetry();

        // Debounce cooldown
        if cd_back > 0 {
            cd_back -= 1;
        }
        if cd_center > 0 {
            cd_center -= 1;
        }
        if cd_left > 0 {
            cd_left -= 1;
        }
        if cd_right > 0 {
            cd_right -= 1;
        }
        if cd_up > 0 {
            cd_up -= 1;
        }
        if cd_down > 0 {
            cd_down -= 1;
        }

        let now_back = btn_back.is_low();
        let now_center = btn_center.is_low();
        let now_left = btn_left.is_low();
        let now_right = btn_right.is_low();
        let now_up = btn_up.is_low();
        let now_down = btn_down.is_low();

        // BACK: toggle telemetry printing
        if now_back && !prev_back && cd_back == 0 {
            cd_back = BTN_COOLDOWN;
            telem_print = !telem_print;
            telem_tick = 0;
            defmt::info!("Telemetry print {}", if telem_print { "ON" } else { "OFF" });
        }

        // CENTER: toggle dribbler enabled/disabled
        if now_center && !prev_center && cd_center == 0 {
            cd_center = BTN_COOLDOWN;
            enabled = !enabled;
            defmt::info!(
                "Dribbler {}: mode={}, setpoint={}",
                if enabled { "ENABLED" } else { "DISABLED" },
                MODE_NAMES[mode_idx],
                setpoints[mode_idx]
            );
        }

        // LEFT: previous mode (wraps)
        if now_left && !prev_left && cd_left == 0 {
            cd_left = BTN_COOLDOWN;
            mode_idx = if mode_idx == 0 {
                MODE_COUNT - 1
            } else {
                mode_idx - 1
            };
            defmt::info!(
                "Mode -> {} | setpoint={}",
                MODE_NAMES[mode_idx],
                setpoints[mode_idx]
            );
        }

        // RIGHT: next mode (wraps)
        if now_right && !prev_right && cd_right == 0 {
            cd_right = BTN_COOLDOWN;
            mode_idx = (mode_idx + 1) % MODE_COUNT;
            defmt::info!(
                "Mode -> {} | setpoint={}",
                MODE_NAMES[mode_idx],
                setpoints[mode_idx]
            );
        }

        // UP: increase setpoint, clamped to mode max
        if now_up && !prev_up && cd_up == 0 {
            cd_up = BTN_COOLDOWN;
            let new_sp = setpoints[mode_idx] + MODE_STEP[mode_idx];
            setpoints[mode_idx] = if new_sp > MODE_MAX[mode_idx] {
                MODE_MAX[mode_idx]
            } else {
                new_sp
            };
            defmt::info!(
                "Setpoint UP -> {} (mode={})",
                setpoints[mode_idx],
                MODE_NAMES[mode_idx]
            );
        }

        // DOWN: decrease setpoint; at 0 while disabled acts as back/force-reset
        if now_down && !prev_down && cd_down == 0 {
            cd_down = BTN_COOLDOWN;
            if setpoints[mode_idx] >= MODE_STEP[mode_idx] {
                setpoints[mode_idx] -= MODE_STEP[mode_idx];
                defmt::info!(
                    "Setpoint DOWN -> {} (mode={})",
                    setpoints[mode_idx],
                    MODE_NAMES[mode_idx]
                );
            } else if setpoints[mode_idx] > 0.0 {
                setpoints[mode_idx] = 0.0;
                defmt::info!("Setpoint DOWN -> 0 (mode={})", MODE_NAMES[mode_idx]);
            } else {
                // Back: setpoint already 0, force full disable and reset all state
                enabled = false;
                mode_idx = 4; // return to CURRENT mode
                setpoints = [0.0; MODE_COUNT];
                defmt::info!("BACK: dribbler force-disabled, all state reset");
            }
        }

        prev_back = now_back;
        prev_center = now_center;
        prev_left = now_left;
        prev_right = now_right;
        prev_up = now_up;
        prev_down = now_down;

        // Periodic telemetry print
        if telem_print {
            telem_tick += 1;
            if telem_tick >= TELEM_PRINT_TICKS {
                telem_tick = 0;
                let telem = kicker.get_lastest_state();
                let ct = &telem.dribbler_motor.current_telemetry;
                let vel_rads = (ct.hall_vel_est_drads as f32) / 10.0;
                let vel_rpm = vel_rads * 60.0 / (2.0 * core::f32::consts::PI);
                let samples = &ct.current_samples_ma;
                let sum: u32 = samples[..20].iter().map(|&s| s as u32).sum();
                let avg_ma = (sum / 20) as u16;
                let ball = telem.ball_detected() != 0;
                defmt::info!(
                    "TELEM | vel={} rad/s ({} RPM) | curr_avg={} mA | ball={}",
                    vel_rads,
                    vel_rpm,
                    avg_ma,
                    ball
                );
            }
        } else {
            telem_tick = 0;
        }

        // Send command every tick
        if enabled {
            kicker.set_drib_command(MODES[mode_idx], setpoints[mode_idx]);
        } else {
            kicker.set_drib_command(DC_DISABLE, 0.0);
        }
        kicker.send_command();

        ticker.next().await;
    }
}
