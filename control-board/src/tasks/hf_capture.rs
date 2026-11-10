//! High-rate (control loop, 1 kHz) signal capture for frequency analysis.
//!
//! Enabled with the `hf-capture` cargo feature. Each time the motors are enabled
//! the capture arms; after `START_DELAY_TICKS` it records `CAPTURE_LEN` samples
//! into AXI SRAM, then streams them out over defmt (one row every
//! `DUMP_EVERY_TICKS` ticks so RTT keeps up). Rows are logged as
//! `HFCAP <idx> <signals...>`; decode with `analysis/hf_capture_decode.py`.

use core::cell::SyncUnsafeCell;

pub const NUM_SIGNALS: usize = 24;
pub const CAPTURE_LEN: usize = 2000;
const START_DELAY_TICKS: u32 = 1300;
const DUMP_EVERY_TICKS: u32 = 10;

/// Signals per sample, stored as i16 with fixed scales:
/// 0 tick (wraps), 1 gyro z [mrad/s], 2-4 KF pose x,y [0.1 mm], theta [0.1 mrad],
/// 5-8 wheel speed meas [0.01 rad/s], 9-12 wheel speed setpoint sent [0.01 rad/s],
/// 13-16 FF current sent [mA], 17-20 motor total current setpoint [mA],
/// 21-23 body velocity output u (global) [mm/s, mm/s, mrad/s]
pub type Sample = [i16; NUM_SIGNALS];

#[link_section = ".axisram.buffers"]
static BUFFER: SyncUnsafeCell<[Sample; CAPTURE_LEN]> =
    SyncUnsafeCell::new([[0; NUM_SIGNALS]; CAPTURE_LEN]);

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Waiting(u32),
    Recording(usize),
    Dumping(usize, u32),
    Done,
}

pub struct HfCapture {
    state: State,
}

pub fn q(v: f32, scale: f32) -> i16 {
    (v * scale).clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

impl HfCapture {
    pub const fn new() -> Self {
        Self { state: State::Idle }
    }

    /// Arm on motor enable (not while a capture/dump is in progress).
    pub fn on_motors_enabled(&mut self) {
        if matches!(self.state, State::Idle | State::Done) {
            self.state = State::Waiting(0);
            defmt::info!("HFCAP armed: start in {} ticks, {} samples", START_DELAY_TICKS, CAPTURE_LEN);
        }
    }

    pub fn recording(&self) -> bool {
        matches!(self.state, State::Recording(_))
    }

    /// Call once per control tick. `sample` is only invoked while recording.
    pub fn tick(&mut self, sample: impl FnOnce() -> Sample) {
        // SAFETY: only the control task touches the buffer.
        let buf = unsafe { &mut *BUFFER.get() };
        self.state = match self.state {
            State::Waiting(n) if n + 1 >= START_DELAY_TICKS => State::Recording(0),
            State::Waiting(n) => State::Waiting(n + 1),
            State::Recording(i) => {
                buf[i] = sample();
                if i + 1 >= CAPTURE_LEN {
                    defmt::info!("HFCAP recorded, dumping");
                    State::Dumping(0, 0)
                } else {
                    State::Recording(i + 1)
                }
            }
            State::Dumping(i, n) if n + 1 >= DUMP_EVERY_TICKS => {
                let r = &buf[i];
                defmt::info!("HFCAP {} {}", i, *r);
                if i + 1 >= CAPTURE_LEN {
                    defmt::info!("HFCAP done");
                    State::Done
                } else {
                    State::Dumping(i + 1, 0)
                }
            }
            State::Dumping(i, n) => State::Dumping(i, n + 1),
            s => s,
        };
    }
}
