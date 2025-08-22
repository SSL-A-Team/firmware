use defmt::Format;
use embassy_futures::select;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Pin, Pull},
    Peripheral,
};
use embassy_time::{Instant, Timer};

#[derive(Clone, Copy, Eq, PartialEq, Debug, Format)]
pub enum AdvButtonEvent {
    None,
    ShortPress,
    LongPress,
    Held,
}

pub const ADV_BTN_EVENT_DOUBLE_TAP: [AdvButtonEvent; 3] = [
    AdvButtonEvent::ShortPress,
    AdvButtonEvent::ShortPress,
    AdvButtonEvent::None,
];

#[derive(Clone, Copy, Debug)]
enum BtnState {
    Pressed(Instant),
    Released(Instant),
}

pub struct AdvExtiButton<
    const SHORT_PRESS_TIME_MS: u64 = 300,
    const LONG_PRESS_TIME_MS: u64 = 600,
    const HOLD_PRESS_TIME_MS: u64 = 1200,
    const PRESS_TO_MS: u64 = 400,
> {
    input: ExtiInput<'static>,
    input_inverted: bool,

    prev_btn_state: BtnState,
    btn_event_ind: usize,
    btn_event_buf: [AdvButtonEvent; 3],
}

impl<
        const SHORT_PRESS_TIME_MS: u64,
        const LONG_PRESS_TIME_MS: u64,
        const HOLD_PRESS_TIME_MS: u64,
        const PRESS_TO_MS: u64,
    > AdvExtiButton<SHORT_PRESS_TIME_MS, LONG_PRESS_TIME_MS, HOLD_PRESS_TIME_MS, PRESS_TO_MS>
{
    pub fn new(input: ExtiInput<'static>, input_inverted: bool) -> Self {
        Self {
            input,
            input_inverted,
            prev_btn_state: BtnState::Released(Instant::now()),
            btn_event_ind: 0,
            btn_event_buf: [AdvButtonEvent::None; 3],
        }
    }

    pub fn new_from_pins<PIN: Pin>(
        input_pin: PIN,
        input_pin_exti: impl Peripheral<P = <PIN as Pin>::ExtiChannel> + 'static,
        input_inverted: bool,
    ) -> Self {
        let input = ExtiInput::new(input_pin, input_pin_exti, Pull::None);
        Self::new(input, input_inverted)
    }

    fn btn_pressed(&self) -> bool {
        if self.input_inverted {
            self.input.is_low()
        } else {
            self.input.is_high()
        }
    }

    pub async fn wait_for_press(&mut self) {
        if self.input_inverted {
            self.input.wait_for_falling_edge().await;
        } else {
            self.input.wait_for_rising_edge().await;
        }
    }

    pub async fn wait_for_release(&mut self) {
        if self.input_inverted {
            self.input.wait_for_rising_edge().await;
        } else {
            self.input.wait_for_falling_edge().await;
        }
    }

    pub fn poll_btn_event(&mut self) -> Option<[AdvButtonEvent; 3]> {
        let now = Instant::now();

        let is_btn_pressed = self.btn_pressed();
        match (self.prev_btn_state, is_btn_pressed) {
            (BtnState::Pressed(_), true) => {
                // btn state didn't change
            }
            (BtnState::Pressed(pressed_time), false) => {
                let btn_held_time = (now - pressed_time).as_millis();
                // 50 ms debounce
                if 50 <= btn_held_time {
                    if btn_held_time <= SHORT_PRESS_TIME_MS {
                        self.btn_event_buf[self.btn_event_ind] = AdvButtonEvent::ShortPress;
                        self.btn_event_ind += 1;
                    } else if SHORT_PRESS_TIME_MS <= btn_held_time
                        && btn_held_time <= LONG_PRESS_TIME_MS
                    {
                        self.btn_event_buf[self.btn_event_ind] = AdvButtonEvent::LongPress;
                        self.btn_event_ind += 1;
                    } else if HOLD_PRESS_TIME_MS <= btn_held_time {
                        self.btn_event_buf[self.btn_event_ind] = AdvButtonEvent::Held;
                        self.btn_event_ind += 1;
                    }

                    self.prev_btn_state = BtnState::Released(now);

                    // we've filled the max number of events, return the chain
                    if self.btn_event_ind >= self.btn_event_buf.len() {
                        let ret = Some(self.btn_event_buf);

                        // reset the buffer
                        self.btn_event_ind = 0;
                        self.btn_event_buf = [AdvButtonEvent::None; 3];
                        return ret;
                    }
                }
            }
            (BtnState::Released(released_time), true) => {
                let btn_rel_time = (now - released_time).as_millis();

                // 50 ms debounce
                if 50 <= btn_rel_time {
                    self.prev_btn_state = BtnState::Pressed(now);
                }
            }
            (BtnState::Released(released_time), false) => {
                // btn state didn't change
                let btn_rel_time = (now - released_time).as_millis();

                // button has been released for a while, return the events so far
                if PRESS_TO_MS <= btn_rel_time && self.btn_event_ind > 0 {
                    let ret = Some(self.btn_event_buf);

                    self.btn_event_ind = 0;
                    self.btn_event_buf = [AdvButtonEvent::None; 3];

                    return ret;
                }
            }
        }

        None
    }

    pub async fn wait_for_btn_event(&mut self, btn_event: [AdvButtonEvent; 3]) {
        'event_loop: loop {
            match self.prev_btn_state {
                BtnState::Pressed(prev_press_time) => {
                    self.wait_for_release().await;

                    let now = Instant::now();
                    let btn_held_time = (now - prev_press_time).as_millis();
                    // 50 ms debounce
                    if 50 <= btn_held_time {
                        if btn_held_time <= SHORT_PRESS_TIME_MS {
                            self.btn_event_buf[self.btn_event_ind] = AdvButtonEvent::ShortPress;
                            self.btn_event_ind += 1;
                        } else if SHORT_PRESS_TIME_MS <= btn_held_time
                            && btn_held_time <= LONG_PRESS_TIME_MS
                        {
                            self.btn_event_buf[self.btn_event_ind] = AdvButtonEvent::LongPress;
                            self.btn_event_ind += 1;
                        } else if HOLD_PRESS_TIME_MS <= btn_held_time {
                            self.btn_event_buf[self.btn_event_ind] = AdvButtonEvent::Held;
                            self.btn_event_ind += 1;
                        }

                        self.prev_btn_state = BtnState::Released(now);

                        // check if our events match the requested event
                        if self.btn_event_buf == btn_event {
                            // reset the buffer
                            self.btn_event_ind = 0;
                            self.btn_event_buf = [AdvButtonEvent::None; 3];

                            break 'event_loop;
                        }

                        // we've filled the max number of events, but didn't match in the
                        // previous block, clear the buffer
                        if self.btn_event_ind >= self.btn_event_buf.len() {
                            // reset the buffer
                            self.btn_event_ind = 0;
                            self.btn_event_buf = [AdvButtonEvent::None; 3];
                        }
                    }
                }
                BtnState::Released(prev_rel_time) => {
                    match select::select(self.wait_for_press(), Timer::after_millis(PRESS_TO_MS))
                        .await
                    {
                        select::Either::First(_) => {
                            let now = Instant::now();
                            let btn_rel_time = (now - prev_rel_time).as_millis();

                            // 50 ms debounce
                            if 50 <= btn_rel_time {
                                self.prev_btn_state = BtnState::Pressed(now);
                            }
                        }
                        select::Either::Second(_) => {
                            // timed out waiting for a press, reset the buffer
                            self.btn_event_buf = [AdvButtonEvent::None; 3];
                            self.btn_event_ind = 0;
                        }
                    }
                }
            }
        }
    }
}
