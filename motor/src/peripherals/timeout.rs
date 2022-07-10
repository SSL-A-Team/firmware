use core::fmt;
use cortex_m::peripheral::DWT;
use stm32h7xx_hal::rcc;

static mut SYSCLK: Option<u32> = None;

// TODO: probably something better
pub fn setup_timeout(mut dwt: DWT, clocks: &rcc::CoreClocks) {
    dwt.enable_cycle_counter();
    unsafe { SYSCLK = Some(clocks.sys_ck().raw()) };
}
pub fn get_sysclk() -> u32 {
    unsafe { SYSCLK }.unwrap()
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Error<E> {
    Other(E),
    Timeout,
}

impl<E> fmt::Debug for Error<E>
where
    E: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Error::Other(ref e) => fmt::Debug::fmt(e, f),
            Error::Timeout => f.write_str("Timeout"),
        }
    }
}

/// Same functionality as `nb::block!($e)` but with timeout added
///
/// Loops until `$e` no longer returns `Error::WouldBlock` or `us` has elapsed
///
/// Uses `DWT` to count cycles so only should be used for short timeouts
///
/// # Input
///
/// - An expression `$e` that evaluates to `nb::Result<T, E>`
/// - A `u32` value `$us` that will cause timeout after elapsed
///     -  Max `$us` value is 2^32/(sysclock speed)
///
/// # Output
///
/// - `Ok(t)` if `$e` evaluates to `Ok(t)`
/// - `Err(timeout::Error::Timeout)` if timeout is reached
/// - `Err(timeout::Error::Other(e))` if `$e` evaluates to `Err(nb::Error::Other(e))`
#[macro_export]
macro_rules! block_us {
    ($e:expr, $us:literal) => {{
        use cortex_m::peripheral::DWT;
        use $crate::peripherals::timeout;
        let start = DWT::cycle_count();
        let cycles = timeout::get_sysclk() / 1000 * $us / 1000;
        loop {
            if DWT::cycle_count().wrapping_sub(start) > cycles {
                break Err(timeout::Error::Timeout);
            } else {
                #[allow(unreachable_patterns)]
                match $e {
                    Err(stm32h7xx_hal::nb::Error::Other(e)) =>
                    {
                        #[allow(unreachable_code)]
                        break Err(timeout::Error::Other(e))
                    }
                    Err(stm32h7xx_hal::nb::Error::WouldBlock) => {}
                    Ok(x) => break Ok(x),
                }
            }
        }
    }};
}

pub fn delay_us(us: u32) {
    let start = DWT::cycle_count();
    let cycles = get_sysclk() / 1000 * us / 1000;
    while DWT::cycle_count().wrapping_sub(start) <= cycles {}
}
