use crate::peripherals;
use crate::Peripheral;
use crate::interrupt;

#[allow(private_interfaces)]
pub(crate) trait SealedInstance: crate::rcc::RccEnableReset + crate::rcc::RccGetFreq {}

pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {
    /// Interrupt for this peripheral.
    type Interrupt: interrupt::typelevel::Interrupt;
}

pin_trait!(SpiRstbPin, Instance);

impl SealedInstance for peripherals::LCDC1 {}
impl Instance for peripherals::LCDC1 {
    type Interrupt = crate::interrupt::typelevel::LCDC1;
}