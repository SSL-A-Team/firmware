#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]
#![feature(generic_const_exprs)]

use ateam_control_board::{get_system_config, SystemIrqs};
use ateam_lib_stm32::{
    drivers::radio::nora_w36x::NoraW36x,
    idle_buffered_uart_read_task, idle_buffered_uart_write_task, static_idle_buffered_uart,
};
use embassy_executor::InterruptExecutor;
use embassy_futures::select::{select, Either};
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    interrupt,
    pac::Interrupt,
    usart::{DataBits, Parity, StopBits, Uart},
};
use embassy_time::Timer;

use defmt_rtt as _;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    cortex_m::asm::delay(100_000_000);
    cortex_m::peripheral::SCB::sys_reset();
}

const MAX_RX_PKT: usize = 256;
const RX_DEPTH: usize = 4;
const MAX_TX_PKT: usize = 128;
const TX_DEPTH: usize = 4;
const DBG_UART: bool = true;

static_idle_buffered_uart!(
    BOOTSTRAP_NORA,
    MAX_RX_PKT,
    RX_DEPTH,
    MAX_TX_PKT,
    TX_DEPTH,
    DBG_UART,
    #[link_section = ".axisram.buffers"]
);

static UART_Q_EXEC: InterruptExecutor = InterruptExecutor::new();

#[allow(non_snake_case)]
#[interrupt]
unsafe fn CEC() {
    UART_Q_EXEC.on_interrupt();
}

const TARGET_BAUD: u32 = 3_000_000;
const TARGET_FLOW: bool = true;
const TARGET_REGION: u8 = 2; // FCC

const FACTORY_BAUD: u32 = 115_200;

const EXPECTED_MANUFACTURER: &str = "u-blox";
const EXPECTED_MODEL_SUBSTR: &str = "NORA-W36";
const MIN_FW_MAJ: u32 = 3;
const MIN_FW_MIN: u32 = 4;
const MIN_FW_PAT: u32 = 0;

fn factory_uart_cfg() -> embassy_stm32::usart::Config {
    let mut c = embassy_stm32::usart::Config::default();
    c.baudrate = FACTORY_BAUD;
    c.data_bits = DataBits::DataBits8;
    c.stop_bits = StopBits::STOP1;
    c.parity = Parity::ParityNone;
    c
}

fn target_uart_cfg() -> embassy_stm32::usart::Config {
    let mut c = embassy_stm32::usart::Config::default();
    c.baudrate = TARGET_BAUD;
    c.data_bits = DataBits::DataBits8;
    c.stop_bits = StopBits::STOP1;
    c.parity = Parity::ParityNone;
    c
}

fn version_ge(s: &str, req_maj: u32, req_min: u32, req_pat: u32) -> bool {
    let mut it = s.split('.');
    let maj: u32 = it.next().and_then(|x| x.trim().parse().ok()).unwrap_or(0);
    let min: u32 = it.next().and_then(|x| x.trim().parse().ok()).unwrap_or(0);
    let pat: u32 = it.next().and_then(|x| x.trim().parse().ok()).unwrap_or(0);
    (maj, min, pat) >= (req_maj, req_min, req_pat)
}

/// Active-low button (Pull::Up). Waits for first press (blocking),
/// then counts additional presses within gap_ms of each release.
async fn count_presses(btn: &mut ExtiInput<'_>, gap_ms: u64) -> usize {
    btn.wait_for_falling_edge().await;
    let mut n = 1usize;
    loop {
        btn.wait_for_rising_edge().await;
        Timer::after_millis(50).await; // debounce
        match select(btn.wait_for_falling_edge(), Timer::after_millis(gap_ms)).await {
            Either::First(_) => n += 1,
            Either::Second(_) => break,
        }
    }
    n
}

/// PD7 drives inverting transistor: High = nRESET asserted, Low = running.
async fn pulse_reset(pin: &mut Output<'_>) {
    pin.set_low();
    Timer::after_millis(10).await;
    pin.set_high();
    Timer::after_millis(15).await;
    pin.set_low();
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = embassy_stm32::init(get_system_config());
    defmt::info!("bootstrap-radio-nora start");

    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CEC,
        embassy_stm32::interrupt::Priority::P5,
    );
    let uart_q_spawner = UART_Q_EXEC.start(Interrupt::CEC);

    let uart = Uart::new_with_rtscts(
        p.USART2,
        p.PD6,  // RX
        p.PD5,  // TX
        SystemIrqs,
        p.PD4,  // RTS (host → radio CTS)
        p.PD3,  // CTS (radio → host)
        p.DMA2_CH0,
        p.DMA2_CH1,
        factory_uart_cfg(),
    )
    .unwrap();
    let (uart_tx, uart_rx) = Uart::split(uart);

    BOOTSTRAP_NORA_IDLE_BUFFERED_UART.init();
    uart_q_spawner
        .spawn(idle_buffered_uart_read_task!(BOOTSTRAP_NORA, uart_rx))
        .unwrap();
    uart_q_spawner
        .spawn(idle_buffered_uart_write_task!(BOOTSTRAP_NORA, uart_tx))
        .unwrap();

    let radio = NoraW36x::<
        MAX_TX_PKT,
        MAX_RX_PKT,
        TX_DEPTH,
        RX_DEPTH,
        DBG_UART,
    >::new(
        BOOTSTRAP_NORA_IDLE_BUFFERED_UART.get_uart_read_queue(),
        BOOTSTRAP_NORA_IDLE_BUFFERED_UART.get_uart_write_queue(),
        &BOOTSTRAP_NORA_IDLE_BUFFERED_UART,
    );

    let mut rst = Output::new(p.PD7, Level::Low, Speed::Medium);
    let mut enter_btn = ExtiInput::new(p.PE11, p.EXTI11, Pull::Up);

    loop {
        defmt::info!("--- reading radio state ---");

        // ── Phase 1: connect ──────────────────────────────────────────────
        // Probe without resetting first: radio is likely already running.
        // wait_startup() only dequeues one packet, so a stale garbage packet
        // from a wrong-baud reset attempt causes immediate failure and we miss
        // the real +STARTUP. Probe avoids this: read_ok() drains stale URCs.
        let mut connected = false;
        for &baud in &[TARGET_BAUD, FACTORY_BAUD] {
            let cfg = if baud == TARGET_BAUD { target_uart_cfg() } else { factory_uart_cfg() };
            let cfg_ok = radio.update_host_uart_config(cfg).await.is_ok();
            defmt::info!("uart cfg update to {} baud: {}", baud, cfg_ok);
            Timer::after_millis(50).await;
            match select(radio.probe(), Timer::after_millis(500)).await {
                Either::First(Ok(_)) => {
                    defmt::info!("probed at {} baud", baud);
                    connected = true;
                    break;
                }
                _ => defmt::warn!("no probe at {} baud", baud),
            }
        }

        // Fallback: reset once per baud and wait for full boot before probing.
        // Fixed delay avoids wait_startup() stale-packet race entirely.
        if !connected {
            defmt::warn!("no probe response — trying reset");
            'reset: for &baud in &[TARGET_BAUD, FACTORY_BAUD] {
                let cfg = if baud == TARGET_BAUD { target_uart_cfg() } else { factory_uart_cfg() };
                radio.update_host_uart_config(cfg).await.ok();
                pulse_reset(&mut rst).await;
                Timer::after_millis(3500).await;
                match select(radio.probe(), Timer::after_millis(500)).await {
                    Either::First(Ok(_)) => {
                        defmt::info!("connected at {} baud after reset", baud);
                        connected = true;
                        break 'reset;
                    }
                    _ => defmt::warn!("no response at {} baud after reset", baud),
                }
            }
        }

        if !connected {
            defmt::error!("cannot connect — retrying in 5s");
            Timer::after_millis(5000).await;
            continue;
        }

        // ── Phase 2: read state ───────────────────────────────────────────
        let manufacturer = match radio.read_manufacturer().await {
            Ok(v) => v,
            Err(e) => { defmt::error!("read_manufacturer {:?}", e); Timer::after_millis(2000).await; continue; }
        };
        let model = match radio.read_model().await {
            Ok(v) => v,
            Err(e) => { defmt::error!("read_model {:?}", e); Timer::after_millis(2000).await; continue; }
        };
        let fw_ver = match radio.read_firmware_version().await {
            Ok(v) => v,
            Err(e) => { defmt::error!("read_firmware_version {:?}", e); Timer::after_millis(2000).await; continue; }
        };
        let (cur_baud, cur_flow) = match radio.read_uart_settings().await {
            Ok(v) => v,
            Err(e) => { defmt::error!("read_uart_settings {:?}", e); Timer::after_millis(2000).await; continue; }
        };
        let cur_region = match radio.read_regulatory_domain().await {
            Ok(v) => v,
            Err(e) => { defmt::error!("read_regulatory_domain {:?}", e); Timer::after_millis(2000).await; continue; }
        };

        // ── Phase 3: identity validation ─────────────────────────────────
        if manufacturer.as_str().trim() != EXPECTED_MANUFACTURER {
            defmt::error!("unexpected manufacturer '{}' — wrong radio?", manufacturer.as_str());
            Timer::after_millis(5000).await;
            continue;
        }
        if !model.as_str().contains(EXPECTED_MODEL_SUBSTR) {
            defmt::error!("unexpected model '{}' — wrong radio?", model.as_str());
            Timer::after_millis(5000).await;
            continue;
        }
        if !version_ge(fw_ver.as_str(), MIN_FW_MAJ, MIN_FW_MIN, MIN_FW_PAT) {
            defmt::error!(
                "fw {} < {}.{}.{} — update radio firmware before bootstrap",
                fw_ver.as_str(), MIN_FW_MAJ, MIN_FW_MIN, MIN_FW_PAT
            );
            Timer::after_millis(5000).await;
            continue;
        }

        // ── Phase 4: diff + report ────────────────────────────────────────
        let uart_wrong   = cur_baud != TARGET_BAUD || cur_flow != TARGET_FLOW;
        let region_wrong = cur_region != TARGET_REGION;

        defmt::info!("=== radio state ===");
        defmt::info!("  manufacturer : {}", manufacturer.as_str());
        defmt::info!("  model        : {}", model.as_str());
        defmt::info!("  fw version   : {}", fw_ver.as_str());
        defmt::info!("  baud         : {} (want {}) {}", cur_baud,   TARGET_BAUD,   if uart_wrong   { "MISMATCH" } else { "ok" });
        defmt::info!("  flow ctrl    : {} (want {}) {}", cur_flow,   TARGET_FLOW,   if uart_wrong   { "MISMATCH" } else { "ok" });
        defmt::info!("  region       : {} (want {} FCC) {}", cur_region, TARGET_REGION, if region_wrong { "MISMATCH" } else { "ok" });
        if uart_wrong || region_wrong {
            defmt::warn!("  -> settings need update");
        } else {
            defmt::info!("  -> all settings correct");
        }
        defmt::info!("===================");
        defmt::info!("  ENTER 1x : apply config");
        defmt::info!("  ENTER 5x : factory reset + apply config");
        defmt::info!("waiting for button...");

        // ── Phase 5: wait for button ──────────────────────────────────────
        let presses = count_presses(&mut enter_btn, 600).await;
        defmt::info!("{} press(es) detected", presses);

        // ── Phase 6: act ─────────────────────────────────────────────────
        let do_uart   = presses == 5 || (presses == 1 && uart_wrong);
        let do_region = presses == 5 || (presses == 1 && region_wrong);

        if presses == 5 {
            defmt::warn!("factory reset requested");

            if radio.factory_restore().await.is_err() {
                defmt::error!("AT+USYFR failed"); Timer::after_millis(2000).await; continue;
            }
            if radio.store_config().await.is_err() {
                defmt::error!("AT&W after factory_restore failed"); Timer::after_millis(2000).await; continue;
            }

            // switch host to factory baud BEFORE reset — radio boots at 115200
            radio.update_host_uart_config(factory_uart_cfg()).await.ok();
            pulse_reset(&mut rst).await;

            match select(radio.wait_startup(), Timer::after_millis(3500)).await {
                Either::First(Ok(_)) => defmt::info!("factory-reset reboot complete"),
                _ => {
                    defmt::error!("no +STARTUP after factory reset"); Timer::after_millis(2000).await; continue;
                }
            }
            if radio.probe().await.is_err() {
                defmt::error!("probe after factory reset failed"); Timer::after_millis(2000).await; continue;
            }
        } else if presses == 1 {
            if !uart_wrong && !region_wrong {
                defmt::info!("nothing to update");
                Timer::after_millis(2000).await;
                continue;
            }
        } else {
            defmt::warn!("press 1x to apply config, 5x for factory reset + config");
            Timer::after_millis(2000).await;
            continue;
        }

        if do_uart {
            defmt::info!("setting UART {} baud flow={}", TARGET_BAUD, TARGET_FLOW);
            // config_uart uses change_after_confirm=1: radio switches immediately after OK
            if radio.config_uart(TARGET_BAUD, TARGET_FLOW).await.is_err() {
                defmt::error!("config_uart failed"); Timer::after_millis(2000).await; continue;
            }
            // follow the radio to its new baud
            radio.update_host_uart_config(target_uart_cfg()).await.ok();
            if radio.store_config().await.is_err() {
                defmt::error!("store_config (uart) failed"); Timer::after_millis(2000).await; continue;
            }
            defmt::info!("UART stored");
        }

        if do_region {
            defmt::info!("setting region FCC ({})", TARGET_REGION);
            if radio.set_regulatory_domain(TARGET_REGION).await.is_err() {
                defmt::error!("set_regulatory_domain failed"); Timer::after_millis(2000).await; continue;
            }
            if radio.store_config().await.is_err() {
                defmt::error!("store_config (region) failed"); Timer::after_millis(2000).await; continue;
            }
            defmt::info!("region stored");
        }

        // ── Phase 7: verify ───────────────────────────────────────────────
        // Ensure host is at target baud for verification query
        radio.update_host_uart_config(target_uart_cfg()).await.ok();
        Timer::after_millis(50).await;

        if radio.probe().await.is_err() {
            defmt::error!("verify probe failed"); Timer::after_millis(2000).await; continue;
        }

        let (v_baud, v_flow) = match radio.read_uart_settings().await {
            Ok(v) => v,
            Err(_) => { defmt::error!("verify: read_uart_settings failed"); Timer::after_millis(2000).await; continue; }
        };
        let v_region = match radio.read_regulatory_domain().await {
            Ok(v) => v,
            Err(_) => { defmt::error!("verify: read_regulatory_domain failed"); Timer::after_millis(2000).await; continue; }
        };

        let pass = v_baud == TARGET_BAUD && v_flow == TARGET_FLOW && v_region == TARGET_REGION;

        defmt::info!("=== verification ===");
        defmt::info!("  baud   {} {}", v_baud,   if v_baud   == TARGET_BAUD   { "PASS" } else { "FAIL" });
        defmt::info!("  flow   {} {}", v_flow,   if v_flow   == TARGET_FLOW   { "PASS" } else { "FAIL" });
        defmt::info!("  region {} {}", v_region, if v_region == TARGET_REGION { "PASS" } else { "FAIL" });
        if pass {
            defmt::info!("=== BOOTSTRAP PASS ===");
        } else {
            defmt::error!("=== BOOTSTRAP FAIL ===");
        }

        Timer::after_millis(2000).await;
    }
}
