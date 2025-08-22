use core::marker::PhantomData;

use embassy_time::{Duration, Instant};

use crate::math::lerp::{Lerp, LerpNumeric};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum AnimState {
    Disabled,
    Waiting,
    Running,
    Completed,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AnimRepeatMode {
    None,
    Fixed(usize),
    Forever,
}

pub trait AnimInterface<T>: Sized {
    fn get_id(&self) -> usize;
    fn enable(&mut self);
    fn disable(&mut self);
    fn enabled(&self) -> bool;

    fn start_animation(&mut self);
    fn reset_animation(&mut self);
    fn animation_running(&self) -> bool;
    fn animation_completed(&self) -> bool;
    fn update(&mut self);

    fn get_value(&self) -> T;
}

/////////////////////////////////
//  composite animation types  //
/////////////////////////////////

pub enum Animation<N, L>
where
    N: LerpNumeric,
    f32: core::convert::From<N>,
    L: crate::math::lerp::Lerp<N>,
{
    Blink(BlinkAnimation<L>),
    Lerp(LerpAnimation<N, L>),
}

impl<N, L> AnimInterface<L> for Animation<N, L>
where
    N: LerpNumeric,
    f32: core::convert::From<N>,
    L: crate::math::lerp::Lerp<N>,
{
    fn get_id(&self) -> usize {
        match self {
            Animation::Blink(a) => a.get_id(),
            Animation::Lerp(a) => a.get_id(),
        }
    }

    fn enable(&mut self) {
        match self {
            Animation::Blink(a) => a.enable(),
            Animation::Lerp(a) => a.enable(),
        }
    }

    fn disable(&mut self) {
        match self {
            Animation::Blink(a) => a.disable(),
            Animation::Lerp(a) => a.disable(),
        }
    }

    fn enabled(&self) -> bool {
        match self {
            Animation::Blink(a) => a.enabled(),
            Animation::Lerp(a) => a.enabled(),
        }
    }

    fn start_animation(&mut self) {
        match self {
            Animation::Blink(a) => {
                a.start_animation();
            }
            Animation::Lerp(a) => {
                a.start_animation();
            }
        }
    }

    fn reset_animation(&mut self) {
        match self {
            Animation::Blink(a) => {
                a.reset_animation();
            }
            Animation::Lerp(a) => {
                a.reset_animation();
            }
        }
    }

    fn animation_running(&self) -> bool {
        match self {
            Animation::Blink(a) => a.animation_running(),
            Animation::Lerp(a) => a.animation_running(),
        }
    }

    fn animation_completed(&self) -> bool {
        match self {
            Animation::Blink(a) => a.animation_completed(),
            Animation::Lerp(a) => a.animation_completed(),
        }
    }

    fn update(&mut self) {
        match self {
            Animation::Blink(a) => {
                a.update();
            }
            Animation::Lerp(a) => {
                a.update();
            }
        }
    }

    fn get_value(&self) -> L {
        match self {
            Animation::Blink(a) => a.get_value(),
            Animation::Lerp(a) => a.get_value(),
        }
    }
}

pub struct CompositeAnimation<'a, N, L>
where
    N: LerpNumeric,
    f32: core::convert::From<N>,
    L: crate::math::lerp::Lerp<N>,
{
    id: usize,

    animations: &'a mut [Animation<N, L>],
    active_animation: usize,

    repeat_mode: AnimRepeatMode,
    anim_state: AnimState,
    repeat_counter: usize,

    last_value: L,
}

impl<'a, N, L> CompositeAnimation<'a, N, L>
where
    N: LerpNumeric,
    f32: core::convert::From<N>,
    L: crate::math::lerp::Lerp<N>,
{
    pub fn new(
        id: usize,
        animations: &'a mut [Animation<N, L>],
        repeat_mode: AnimRepeatMode,
    ) -> Self {
        let first_val = animations[0].get_value();
        CompositeAnimation {
            id,
            animations,
            active_animation: 0,
            repeat_mode,
            anim_state: AnimState::Disabled,
            repeat_counter: 0,
            last_value: first_val,
        }
    }
}

impl<N, L> AnimInterface<L> for CompositeAnimation<'_, N, L>
where
    N: LerpNumeric,
    f32: core::convert::From<N>,
    L: crate::math::lerp::Lerp<N>,
{
    fn get_id(&self) -> usize {
        self.id
    }

    fn enable(&mut self) {
        self.anim_state = AnimState::Waiting;
    }

    fn disable(&mut self) {
        self.anim_state = AnimState::Disabled;
    }

    fn enabled(&self) -> bool {
        self.anim_state != AnimState::Disabled
    }

    fn start_animation(&mut self) {
        if !self.enabled() {
            return;
        }

        self.active_animation = 0;
        self.anim_state = AnimState::Running;
        self.animations[self.active_animation].start_animation();

        if let AnimRepeatMode::Fixed(num) = self.repeat_mode {
            self.repeat_counter = num;
        }
    }

    fn reset_animation(&mut self) {
        if !self.enabled() {
            return;
        }

        self.anim_state = AnimState::Waiting;
    }

    fn animation_running(&self) -> bool {
        self.anim_state == AnimState::Running
    }

    fn animation_completed(&self) -> bool {
        self.anim_state == AnimState::Completed
    }

    fn update(&mut self) {
        if self.anim_state != AnimState::Running {
            return;
        }

        // update the current animation
        self.animations[self.active_animation].update();
        self.last_value = self.animations[self.active_animation].get_value();

        // check if the current animation is done
        if self.animations[self.active_animation].animation_completed() {
            // advance to the next animation
            self.active_animation += 1;

            if self.active_animation >= self.animations.len() {
                self.active_animation = 0;

                // we've completed the last animation in the buffer
                match self.repeat_mode {
                    AnimRepeatMode::None => {
                        self.anim_state = AnimState::Completed;
                    }
                    AnimRepeatMode::Fixed(_) => {
                        if self.repeat_counter == 0 {
                            self.anim_state = AnimState::Completed;
                        } else {
                            self.repeat_counter -= 1;
                        }
                    }
                    AnimRepeatMode::Forever => {
                        // we've already reset the animation index so it'll loop around
                    }
                }
            }

            if self.anim_state != AnimState::Completed {
                self.animations[self.active_animation].start_animation();
            }
        }
    }

    fn get_value(&self) -> L {
        self.last_value
    }
}

//////////////////
//  animations  //
//////////////////

#[derive(Clone, Copy, Debug)]
pub struct BlinkAnimation<T> {
    id: usize,

    val_one: T,
    val_two: T,
    v1_time: u64,
    v2_time: u64,
    repeat_style: AnimRepeatMode,

    anim_state: AnimState,
    repeat_counter: usize,
    start_time: Instant,
    last_value: T,
}

impl<T: Clone + Copy + Sized> BlinkAnimation<T> {
    pub fn new(
        id: usize,
        val_one: T,
        val_two: T,
        v1_time: Duration,
        v2_time: Duration,
        repeat_style: AnimRepeatMode,
    ) -> Self {
        BlinkAnimation {
            id,

            val_one,
            val_two,
            v1_time: v1_time.as_millis(),
            v2_time: v2_time.as_millis(),
            repeat_style,

            anim_state: AnimState::Waiting,
            repeat_counter: 0,
            start_time: Instant::now(),
            last_value: val_one,
        }
    }
}

impl<T: Clone + Copy + Sized> AnimInterface<T> for BlinkAnimation<T> {
    fn get_id(&self) -> usize {
        self.id
    }

    fn enable(&mut self) {
        self.anim_state = AnimState::Disabled;
    }

    fn disable(&mut self) {
        self.anim_state = AnimState::Waiting;
    }

    fn enabled(&self) -> bool {
        self.anim_state != AnimState::Disabled
    }

    fn start_animation(&mut self) {
        if !self.enabled() {
            return;
        }

        self.start_time = Instant::now();
        self.anim_state = AnimState::Running;

        if let AnimRepeatMode::Fixed(num) = self.repeat_style {
            self.repeat_counter = num;
        }
    }

    fn reset_animation(&mut self) {
        if !self.enabled() {
            return;
        }

        self.anim_state = AnimState::Waiting;
    }

    fn animation_running(&self) -> bool {
        self.anim_state == AnimState::Running
    }

    fn animation_completed(&self) -> bool {
        self.anim_state == AnimState::Completed
    }

    fn update(&mut self) {
        if self.anim_state != AnimState::Running {
            return;
        }

        let now = Instant::now();
        let elapsed_time = (now - self.start_time).as_millis();

        if elapsed_time <= self.v1_time {
            self.last_value = self.val_one;
        } else if self.v1_time < elapsed_time && elapsed_time <= self.v1_time + self.v2_time {
            self.last_value = self.val_two;
        } else if elapsed_time > self.v1_time + self.v2_time {
            match self.repeat_style {
                AnimRepeatMode::None => {
                    self.anim_state = AnimState::Completed;
                }
                AnimRepeatMode::Forever => {
                    self.start_time = now;
                    self.last_value = self.val_one;
                }
                AnimRepeatMode::Fixed(_) => {
                    if self.repeat_counter == 0 {
                        self.anim_state = AnimState::Completed;
                    } else {
                        self.repeat_counter -= 1;
                        self.start_time = now;
                        self.last_value = self.val_one;
                    }
                }
            }
        }
    }

    fn get_value(&self) -> T {
        self.last_value
    }
}

#[derive(Clone, Copy, Debug)]
pub struct LerpAnimation<N, L>
where
    N: LerpNumeric,
    f32: core::convert::From<N>,
    L: crate::math::lerp::Lerp<N>,
{
    id: usize,

    val_one: L,
    val_two: L,
    lerp_duration_ms: u64,
    repeat_style: AnimRepeatMode,

    anim_state: AnimState,
    repeat_counter: usize,
    start_time: Instant,
    last_value: L,

    pd: PhantomData<N>,
}

impl<N, L> LerpAnimation<N, L>
where
    N: LerpNumeric,
    f32: core::convert::From<N>,
    L: crate::math::lerp::Lerp<N>,
{
    pub fn new(
        id: usize,
        val_one: L,
        val_two: L,
        lerp_duration_ms: Duration,
        repeat_style: AnimRepeatMode,
    ) -> Self {
        LerpAnimation {
            id,

            val_one,
            val_two,
            lerp_duration_ms: lerp_duration_ms.as_millis(),
            repeat_style,

            anim_state: AnimState::Waiting,
            repeat_counter: 0,
            start_time: Instant::now(),
            last_value: val_one,

            pd: PhantomData,
        }
    }
}

impl<N, L> AnimInterface<L> for LerpAnimation<N, L>
where
    N: LerpNumeric,
    f32: core::convert::From<N>,
    L: crate::math::lerp::Lerp<N>,
{
    fn get_id(&self) -> usize {
        self.id
    }

    fn enable(&mut self) {
        self.anim_state = AnimState::Waiting
    }

    fn disable(&mut self) {
        self.anim_state = AnimState::Disabled
    }

    fn enabled(&self) -> bool {
        self.anim_state != AnimState::Disabled
    }

    fn start_animation(&mut self) {
        if !self.enabled() {
            return;
        }

        self.start_time = Instant::now();
        self.anim_state = AnimState::Running;

        if let AnimRepeatMode::Fixed(num) = self.repeat_style {
            self.repeat_counter = num;
        }
    }

    fn reset_animation(&mut self) {
        if !self.enabled() {
            return;
        }

        self.anim_state = AnimState::Waiting;
    }

    fn animation_running(&self) -> bool {
        self.anim_state == AnimState::Running
    }

    fn animation_completed(&self) -> bool {
        self.anim_state == AnimState::Completed
    }

    fn update(&mut self) {
        if self.anim_state != AnimState::Running {
            return;
        }

        let now = Instant::now();
        let elapsed_time = (now - self.start_time).as_millis();
        let elapsed_time_frac: f32 = elapsed_time as f32 / self.lerp_duration_ms as f32;

        self.last_value = Lerp::<N>::lerp_f(self.val_one, self.val_two, elapsed_time_frac);

        if elapsed_time_frac > 1.0 {
            match self.repeat_style {
                AnimRepeatMode::None => {
                    self.anim_state = AnimState::Completed;
                }
                AnimRepeatMode::Forever => {
                    self.start_time = now;
                    self.last_value = self.val_one;
                }
                AnimRepeatMode::Fixed(_) => {
                    if self.repeat_counter == 0 {
                        self.anim_state = AnimState::Completed;
                    } else {
                        self.repeat_counter -= 1;
                        self.start_time = now;
                        self.last_value = self.val_one;
                    }
                }
            }
        }
    }

    fn get_value(&self) -> L {
        self.last_value
    }
}
