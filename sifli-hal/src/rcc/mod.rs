use critical_section::CriticalSection;

mod clock;
pub use clock::*;

mod clock_configure;
pub use clock_configure::*;

use crate::time::Hertz;

// TODO: should we split this into `RccEnable` and `RccReset` ?
pub(crate) trait SealedRccEnableReset {
    fn rcc_enable() {}

    fn rcc_disable() {}

    fn rcc_reset() {}
}
#[allow(private_bounds)]
pub trait RccEnableReset: SealedRccEnableReset + 'static {}

pub(crate) trait SealedRccGetFreq {
    /// Get peripheral frequency
    /// Returns `None` if clock is disabled
    fn get_freq() -> Option<Hertz>;
}

#[allow(private_bounds)]
pub trait RccGetFreq: SealedRccGetFreq + 'static {
    /// Get peripheral frequency
    /// Returns `None` if clock is disabled
    fn frequency() -> Option<Hertz> {
        Self::get_freq()
    }
}

/// Enables and resets peripheral `T`.
///
/// # Safety
///
/// Peripheral must not be in use.
// TODO: should this be `unsafe`?
pub fn enable_and_reset_with_cs<T: RccEnableReset>(_cs: CriticalSection) {
    T::rcc_enable();
    T::rcc_reset();
}


/// Enables and resets peripheral `T`.
///
/// # Safety
///
/// Peripheral must not be in use.
// TODO: should this be `unsafe`?
pub fn enable_and_reset<T: RccEnableReset>() {
    critical_section::with(|cs| enable_and_reset_with_cs::<T>(cs));
}

/// Disables peripheral `T`.
///
/// # Safety
///
/// Peripheral must not be in use.
// TODO: should this be `unsafe`?
pub fn disable_with_cs<T: RccEnableReset>(_cs: CriticalSection) {
    T::rcc_disable();
}

/// Disables peripheral `T`.
///
/// # Safety
///
/// Peripheral must not be in use.
// TODO: should this be `unsafe`?
pub fn disable<T: RccEnableReset>() {
    critical_section::with(|cs| disable_with_cs::<T>(cs));
}
