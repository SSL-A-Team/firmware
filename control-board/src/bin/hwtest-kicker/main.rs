#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]
#![feature(generic_const_exprs)]

use core::mem::MaybeUninit;

use ateam_common_packets::bindings::{
    DribblerCommand::DC_CURRENT,
    KickRequest::{KR_ARM, KR_DISABLE, KR_KICK_NOW},
    KickerTelemetry, PowerCommand, PowerTelemetry,
};
use ateam_control_board::{
    drivers::kicker::Kicker,
    get_system_config,
    include_kicker_bin,
    SystemIrqs,
    DEBUG_KICKER_UART_QUEUES,
    DEBUG_POWER_UART_QUEUES,
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

// Kicker UART buffers
const KICKER_MAX_TX_PACKET_SIZE: usize = 64;
const KICKER_TX_BUF_DEPTH: usize = 3;
const KICKER_MAX_RX_PACKET_SIZE: usize = core::mem::size_of::<KickerTelemetry>();
const KICKER_RX_BUF_DEPTH: usize = 20;
static_idle_buffered_uart!(KICKER, KICKER_MAX_RX_PACKET_SIZE, KICKER_RX_BUF_DEPTH, KICKER_MAX_TX_PACKET_SIZE, KICKER_TX_BUF_DEPTH, DEBUG_KICKER_UART_QUEUES, #[link_section = ".axisram.buffers"]);

// Power board UART buffers
const POWER_MAX_TX_PACKET_SIZE: usize = core::mem::size_of::<PowerCommand>();
const POWER_MAX_RX_PACKET_SIZE: usize = core::mem::size_of::<PowerTelemetry>();
const POWER_TX_BUF_DEPTH: usize = 4;
const POWER_RX_BUF_DEPTH: usize = 4;
static_idle_buffered_uart!(HWTEST_POWER, POWER_MAX_RX_PACKET_SIZE, POWER_RX_BUF_DEPTH, POWER_MAX_TX_PACKET_SIZE, POWER_TX_BUF_DEPTH, DEBUG_POWER_UART_QUEUES, #[link_section = ".axisram.buffers"]);

static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[allow(non_snake_case)]
#[interrupt]
unsafe fn CEC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

const KICK_SPEED_STEP: f32 = 0.5;
const KICK_SPEED_MAX: f32 = 6.5;
const DRIB_CURRENT_STEP: f32 = 50.0;
const DRIB_CURRENT_MAX: f32 = 1500.0;

const TELEM_PRINT_TICKS: u32 = 400;
// Power board runs at 10ms; main loop is 500µs, so send every 20 ticks.
const POWER_TICK_PERIOD: u32 = 20;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let stm32_config = get_system_config();
    let p = embassy_stm32::init(stm32_config);

    defmt::info!("hwtest-kicker startup");

    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CEC,
        embassy_stm32::interrupt::Priority::P5,
    );
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    // Buttons: active-low, internal pull-up
    // PE10=back(toggle telem), PE11=center(kick), PE12=left(drib-), PE13=right(drib+)
    // PE14=up(kick speed+),    PE15=down(kick speed-)
    let btn_back   = Input::new(p.PE10, Pull::Up);
    let btn_center = Input::new(p.PE11, Pull::Up);
    let btn_left   = Input::new(p.PE12, Pull::Up);
    let btn_right  = Input::new(p.PE13, Pull::Up);
    let btn_up     = Input::new(p.PE14, Pull::Up);
    let btn_down   = Input::new(p.PE15, Pull::Up);

    // Kicker UART (UART8, PE0=rx, PE1=tx)
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

    // Power board UART (UART9, PG0=rx, PG1=tx)
    let power_usart = Uart::new(
        p.UART9,
        p.PG0,
        p.PG1,
        SystemIrqs,
        p.DMA2_CH5,
        p.DMA2_CH4,
        ateam_control_board::tasks::power_task::power_uart_config(),
    )
    .unwrap();
    HWTEST_POWER_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, HWTEST_POWER, power_usart);

    let power_rx = HWTEST_POWER_IDLE_BUFFERED_UART.get_uart_read_queue();
    let power_tx = HWTEST_POWER_IDLE_BUFFERED_UART.get_uart_write_queue();

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

    defmt::info!("waiting for dribbler firmware...");
    let mut wait_ticks: u32 = 0;
    loop {
        kicker.process_telemetry();
        kicker.send_command();
        if kicker.get_lastest_state().dribbler_fw_loaded() != 0 {
            break;
        }
        wait_ticks += 1;
        if wait_ticks % 20 == 0 {
            defmt::info!("  still waiting ({} s)...", wait_ticks / 20);
        }
        Timer::after(Duration::from_millis(50)).await;
    }
    defmt::info!("dribbler fw loaded, starting test");

    let mut kick_speed: f32 = 0.5;
    let mut drib_current_ma: f32 = 0.0;
    // Two-phase kick: send KR_KICK_NOW briefly, then KR_DISABLE to flush the kicker board's
    // RX queue before returning to KR_ARM. Prevents re-fires from queued KR_KICK_NOW packets.
    let mut kick_now_remaining: u32 = 0;
    let mut kick_clear_remaining: u32 = 0;
    let mut telem_print: bool = false;
    let mut telem_tick: u32 = 0;

    let mut power_shutdown_requested: bool = false;
    let mut kicker_discharge_complete: bool = false;
    let mut power_tick: u32 = 0;

    // Button edge detection + debounce (500 µs tick)
    // Center gets extra margin over kicker KICK_COOLDOWN (100ms) to prevent double-fire
    const BTN_COOLDOWN: u32 = 200;       // 100ms — general buttons
    const BTN_KICK_COOLDOWN: u32 = 600;  // 300ms — center/kick button
    let mut prev_back   = false;
    let mut prev_center = false;
    let mut prev_left   = false;
    let mut prev_right  = false;
    let mut prev_up     = false;
    let mut prev_down   = false;
    let mut cd_back:   u32 = 0;
    let mut cd_center: u32 = 0;
    let mut cd_left:   u32 = 0;
    let mut cd_right:  u32 = 0;
    let mut cd_up:     u32 = 0;
    let mut cd_down:   u32 = 0;

    defmt::info!(
        "Ready. BACK=toggle telem, CENTER=kick, UP/DOWN=kick speed, LEFT/RIGHT=drib current"
    );
    defmt::info!(
        "kick_speed={} m/s, drib_current={} mA",
        kick_speed,
        drib_current_ma
    );

    let mut ticker = Ticker::every(Duration::from_micros(500));
    loop {
        kicker.process_telemetry();

        // Poll power board telemetry
        while let Ok(res) = power_rx.try_dequeue() {
            let buf = res.data();
            if buf.len() == core::mem::size_of::<PowerTelemetry>() {
                let mut pkt: PowerTelemetry = unsafe { MaybeUninit::zeroed().assume_init() };
                unsafe {
                    let dst = &mut pkt as *mut _ as *mut u8;
                    for i in 0..core::mem::size_of::<PowerTelemetry>() {
                        *dst.offset(i as isize) = buf[i];
                    }
                }
                if pkt.shutdown_requested() != 0 && !power_shutdown_requested {
                    power_shutdown_requested = true;
                    defmt::warn!("power board requested shutdown — beginning kicker discharge");
                }
            }
        }

        // Track kicker discharge completion
        if power_shutdown_requested && !kicker_discharge_complete && kicker.shutdown_completed() {
            kicker_discharge_complete = true;
            defmt::info!("kicker discharge complete — signaling power board");
        }

        // Send power board command every POWER_TICK_PERIOD ticks
        power_tick += 1;
        if power_tick >= POWER_TICK_PERIOD {
            power_tick = 0;
            let mut cmd: PowerCommand = unsafe { MaybeUninit::zeroed().assume_init() };
            if kicker_discharge_complete {
                cmd.set_ready_shutdown(1);
            } else if power_shutdown_requested {
                cmd.set_request_shutdown(1);
            }
            let cmd_bytes = unsafe {
                core::slice::from_raw_parts(
                    (&cmd as *const PowerCommand) as *const u8,
                    core::mem::size_of::<PowerCommand>(),
                )
            };
            let _ = power_tx.enqueue_copy(cmd_bytes);
        }

        // Button handling suppressed during shutdown
        if !power_shutdown_requested {
            if cd_back   > 0 { cd_back   -= 1; }
            if cd_center > 0 { cd_center -= 1; }
            if cd_left   > 0 { cd_left   -= 1; }
            if cd_right  > 0 { cd_right  -= 1; }
            if cd_up     > 0 { cd_up     -= 1; }
            if cd_down   > 0 { cd_down   -= 1; }

            let now_back   = btn_back.is_low();
            let now_center = btn_center.is_low();
            let now_left   = btn_left.is_low();
            let now_right  = btn_right.is_low();
            let now_up     = btn_up.is_low();
            let now_down   = btn_down.is_low();

            // BACK: toggle telemetry printing
            if now_back && !prev_back && cd_back == 0 {
                cd_back = BTN_COOLDOWN;
                telem_print = !telem_print;
                telem_tick = 0;
                defmt::info!("Telemetry print {}", if telem_print { "ON" } else { "OFF" });
            }

            // CENTER: fire kick — 4 ticks (2ms) of KR_KICK_NOW, then 20 ticks (10ms) of
            // KR_DISABLE to flush any queued KR_KICK_NOW from kicker board's RX buffer.
            if now_center && !prev_center && cd_center == 0 {
                cd_center = BTN_KICK_COOLDOWN;
                kick_now_remaining = 4;
                kick_clear_remaining = 0;
                defmt::info!("KICK @ {} m/s", kick_speed);
            }

            // LEFT: decrease dribbler current
            if now_left && !prev_left && cd_left == 0 {
                cd_left = BTN_COOLDOWN;
                drib_current_ma -= DRIB_CURRENT_STEP;
                if drib_current_ma < 0.0 {
                    drib_current_ma = 0.0;
                }
                defmt::info!("drib_current={} mA", drib_current_ma);
            }

            // RIGHT: increase dribbler current
            if now_right && !prev_right && cd_right == 0 {
                cd_right = BTN_COOLDOWN;
                drib_current_ma += DRIB_CURRENT_STEP;
                if drib_current_ma > DRIB_CURRENT_MAX {
                    drib_current_ma = DRIB_CURRENT_MAX;
                }
                defmt::info!("drib_current={} mA", drib_current_ma);
            }

            // UP: increase kick speed
            if now_up && !prev_up && cd_up == 0 {
                cd_up = BTN_COOLDOWN;
                kick_speed += KICK_SPEED_STEP;
                if kick_speed > KICK_SPEED_MAX {
                    kick_speed = KICK_SPEED_MAX;
                }
                defmt::info!("kick_speed={} m/s", kick_speed);
            }

            // DOWN: decrease kick speed
            if now_down && !prev_down && cd_down == 0 {
                cd_down = BTN_COOLDOWN;
                kick_speed -= KICK_SPEED_STEP;
                if kick_speed < 0.0 {
                    kick_speed = 0.0;
                }
                defmt::info!("kick_speed={} m/s", kick_speed);
            }

            prev_back   = now_back;
            prev_center = now_center;
            prev_left   = now_left;
            prev_right  = now_right;
            prev_up     = now_up;
            prev_down   = now_down;
        }

        if telem_print {
            telem_tick += 1;
            if telem_tick >= TELEM_PRINT_TICKS {
                telem_tick = 0;
                let telem = kicker.get_lastest_state();
                defmt::info!(
                    "TELEM | hv={} V | batt={} V | ball={} | error={} | shutdown={}",
                    telem.rail_voltage,
                    telem.battery_voltage,
                    telem.ball_detected() != 0,
                    kicker.error_reported(),
                    power_shutdown_requested,
                );
            }
        } else {
            telem_tick = 0;
        }

        // Kicker command: during shutdown, request_shutdown drives kicker discharge.
        // Buttons suppressed during shutdown; kick_now_remaining won't be set.
        kicker.set_kick_strength(kick_speed);
        kicker.set_drib_command(DC_CURRENT, drib_current_ma);
        if power_shutdown_requested {
            kicker.request_shutdown();
            kicker.request_kick(KR_ARM as u32);
        } else if kick_now_remaining > 0 {
            kicker.request_kick(KR_KICK_NOW as u32);
            kick_now_remaining -= 1;
            if kick_now_remaining == 0 {
                kick_clear_remaining = 20;
            }
        } else if kick_clear_remaining > 0 {
            kicker.request_kick(KR_DISABLE as u32);
            kick_clear_remaining -= 1;
        } else {
            kicker.request_kick(KR_ARM as u32);
        }
        kicker.send_command();

        ticker.next().await;
    }
}
