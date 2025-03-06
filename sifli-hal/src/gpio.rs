//! GPIO driver.
#![macro_use]
use core::convert::Infallible;
use core::future::Future;
use core::pin::Pin as FuturePin;
use core::task::{Context, Poll};

use embassy_hal_internal::{impl_peripheral, into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::InterruptExt;
use crate::pac::hpsys_gpio::regs;
use crate::pac::hpsys_pinmux::vals;
use crate::{interrupt, pac, peripherals, Peripheral};

// TODO: move this const to _generated.rs
#[cfg(any(feature = "sf32lb52x"))]
pub(crate) const PA_PIN_COUNT: usize = 30;

static PA_WAKERS: [AtomicWaker; PA_PIN_COUNT] = [const { AtomicWaker::new() }; PA_PIN_COUNT];

/// Represents a digital input or output level.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Level {
    /// Logical low.
    Low,
    /// Logical high.
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

impl core::ops::Not for Level {
    type Output = Self;

    fn not(self) -> Self::Output {
        (!bool::from(self)).into()
    }
}

/// Represents a pull setting for an input.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum Pull {
    /// No pull.
    None,
    /// Internal pull-up resistor.
    Up,
    /// Internal pull-down resistor.
    Down,
}

/// Drive strength of an output
#[derive(Debug, Eq, PartialEq)]
pub enum Drive {
    /// min drive, {ds1, ds0} = 0b00
    Drive0,
    /// {ds1, ds0} = 0b01
    Drive1,
    /// {ds1, ds0} = 0b10
    Drive2,
    /// {ds1, ds0} = 0b11
    Drive3,
}
/// Slew rate of an output
#[derive(Debug, Eq, PartialEq)]
pub enum SlewRate {
    /// Fast slew rate.
    Fast,
    /// Slow slew rate.
    Slow,
}

/// GPIO input driver.
pub struct Input<'d> {
    pin: Flex<'d>,
}

impl<'d> Input<'d> {
    /// Create GPIO input driver for a [Pin] with the provided [Pull] configuration.
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_input();
        pin.set_pull(pull);
        Self { pin }
    }

    /// Set the pin's Schmitt trigger.
    #[inline]
    pub fn set_schmitt(&mut self, enable: bool) {
        self.pin.set_schmitt(enable)
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Returns current pin level
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await;
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await;
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await;
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await;
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        self.pin.wait_for_any_edge().await;
    }
}

/// Interrupt trigger levels.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptTrigger {
    /// Trigger on pin low.
    LevelLow,
    /// Trigger on pin high.
    LevelHigh,
    /// Trigger on high to low transition.
    EdgeLow,
    /// Trigger on low to high transition.
    EdgeHigh,
    /// Trigger on any transition.
    AnyEdge,
}

pub(crate) unsafe fn init(gpio1_it_priority: interrupt::Priority) {
    unsafe {
        interrupt::GPIO1.disable();
        interrupt::GPIO1.set_priority(gpio1_it_priority);
        interrupt::GPIO1.enable();
    }
    crate::rcc::enable_and_reset::<peripherals::HPSYS_GPIO>();

    // We should not reset PINMUX here, otherwise the pins used for PSRAM 
    // or FLASH will be invalid. 
    // PINMUX is already turned on in the bootloader.
    // crate::rcc::enable_and_reset_with_cs::<peripherals::HPSYS_PINMUX>(cs);
}

#[cfg(feature = "rt")]
fn irq_handler<const N: usize>(wakers: &[AtomicWaker; N]) {
    // let cpu = SIO.cpuid().read() as usize;
    // // There are two sets of interrupt registers, one for cpu0 and one for cpu1
    // // and here we are selecting the set that belongs to the currently executing
    // // cpu.
    // let proc_intx: pac::io::Int = bank.int_proc(cpu);
    // for pin in 0..N {
    //     // There are 4 raw interrupt status registers, PROCx_INTS0, PROCx_INTS1,
    //     // PROCx_INTS2, and PROCx_INTS3, and we are selecting the one that the
    //     // current pin belongs to.
    //     let intsx = proc_intx.ints(pin / 8);
    //     // The status register is divided into groups of four, one group for
    //     // each pin. Each group consists of four trigger levels LEVEL_LOW,
    //     // LEVEL_HIGH, EDGE_LOW, and EDGE_HIGH for each pin.
    //     let pin_group = pin % 8;
    //     let event = (intsx.read().0 >> (pin_group * 4)) & 0xf;

    //     // no more than one event can be awaited per pin at any given time, so
    //     // we can just clear all interrupt enables for that pin without having
    //     // to check which event was signalled.
    //     if event != 0 {
    //         proc_intx.inte(pin / 8).write_clear(|w| {
    //             w.set_edge_high(pin_group, true);
    //             w.set_edge_low(pin_group, true);
    //             w.set_level_high(pin_group, true);
    //             w.set_level_low(pin_group, true);
    //         });
    //         wakers[pin].wake();
    //     }
    // }
}

#[cfg(feature = "rt")]
#[interrupt]
fn GPIO1() {
    irq_handler(&PA_WAKERS);
}


#[must_use = "futures do nothing unless you `.await` or poll them"]
struct InputFuture<'d> {
    pin: PeripheralRef<'d, AnyPin>,
}

impl<'d> InputFuture<'d> {
    fn new(pin: PeripheralRef<'d, AnyPin>, level: InterruptTrigger) -> Self {
        todo!();
        Self { pin }
    }
}

impl<'d> Future for InputFuture<'d> {
    type Output = ();

    fn poll(self: FuturePin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // // We need to register/re-register the waker for each poll because any
        // // calls to wake will deregister the waker.
        // let waker = match self.pin.bank() {
        //     Bank::Bank0 => &BANK0_WAKERS[self.pin.pin() as usize],
        //     #[cfg(feature = "qspi-as-gpio")]
        //     Bank::Qspi => &QSPI_WAKERS[self.pin.pin() as usize],
        // };
        // waker.register(cx.waker());

        // // self.int_proc() will get the register offset for the current cpu,
        // // then we want to access the interrupt enable register for our
        // // pin (there are 4 of these PROC0_INTE0, PROC0_INTE1, PROC0_INTE2, and
        // // PROC0_INTE3 per cpu).
        // let inte: pac::io::regs::Int = self.pin.int_proc().inte((self.pin.pin() / 8) as usize).read();
        // // The register is divided into groups of four, one group for
        // // each pin. Each group consists of four trigger levels LEVEL_LOW,
        // // LEVEL_HIGH, EDGE_LOW, and EDGE_HIGH for each pin.
        // let pin_group = (self.pin.pin() % 8) as usize;

        // // since the interrupt handler clears all INTE flags we'll check that
        // // all have been cleared and unconditionally return Ready(()) if so.
        // // we don't need further handshaking since only a single event wait
        // // is possible for any given pin at any given time.
        // if !inte.edge_high(pin_group)
        //     && !inte.edge_low(pin_group)
        //     && !inte.level_high(pin_group)
        //     && !inte.level_low(pin_group)
        // {
        //     return Poll::Ready(());
        // }
        Poll::Pending
    }
}

/// GPIO output driver.
pub struct Output<'d> {
    pin: Flex<'d>,
}

impl<'d> Output<'d> {
    /// Create GPIO output driver for a [Pin] with the provided [Level].
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        match initial_output {
            Level::High => pin.set_high(),
            Level::Low => pin.set_low(),
        }

        pin.set_as_output();
        Self { pin }
    }

    /// Set the pin's drive strength.
    #[inline]
    pub fn set_drive_strength(&mut self, strength: Drive) {
        self.pin.set_drive_strength(strength)
    }

    /// Set the pin's slew rate.
    #[inline]
    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        self.pin.set_slew_rate(slew_rate)
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high()
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low()
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }
}

/// GPIO output open-drain.
pub struct OutputOpenDrain<'d> {
    pin: Flex<'d>,
}

impl<'d> OutputOpenDrain<'d> {
    /// Create GPIO output driver for a [Pin] in open drain mode with the provided [Level].
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_low();
        match initial_output {
            Level::High => pin.set_as_input(),
            Level::Low => pin.set_as_output(),
        }
        Self { pin }
    }

    /// Set the pin's pull-up.
    #[inline]
    pub fn set_pullup(&mut self, enable: bool) {
        if enable {
            self.pin.set_pull(Pull::Up);
        } else {
            self.pin.set_pull(Pull::None);
        }
    }

    /// Set the pin's drive strength.
    #[inline]
    pub fn set_drive_strength(&mut self, strength: Drive) {
        self.pin.set_drive_strength(strength)
    }

    /// Set the pin's slew rate.
    #[inline]
    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        self.pin.set_slew_rate(slew_rate)
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        // For Open Drain High, disable the output pin.
        self.pin.set_as_input()
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        // For Open Drain Low, enable the output pin.
        self.pin.set_as_output()
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    /// Is the output level high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    /// Is the output level low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_as_output()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Returns current pin level
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await;
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await;
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await;
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await;
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        self.pin.wait_for_any_edge().await;
    }
}

/// GPIO flexible pin.
///
/// This pin can be either an input or output pin. The output level register bit will remain
/// set while not in output mode, so the pin's level will be 'remembered' when it is not in output
/// mode.
pub struct Flex<'d> {
    pub(crate) pin: PeripheralRef<'d, AnyPin>,
}

impl<'d> Flex<'d> {
    /// Wrap the pin in a `Flex`.
    ///
    /// The pin remains disconnected. The initial output level is unspecified, but can be changed
    /// before the pin is put into output mode.
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd) -> Self {
        into_ref!(pin);

        let mut pin = Self { pin: pin.map_into() };
        
        // DISABLE_ISR
        if pin.pin.pin() % 32 == 0 {
            pin.pin.gpio().iecr0().write_value(regs::Iecr0(pin.bit()));
        } else {
            pin.pin.gpio().iecr1().write_value(regs::Iecr1(pin.bit()));
        }

        // WAIT_ISR_DISABLED
        crate::cortex_m_blocking_delay_us(1);

        if pin.pin.pin() % 32 == 0 {
            pin.pin.gpio().isr0().write_value(sifli_pac::hpsys_gpio::regs::Isr0(pin.bit()));

            // CLEAR_OPEN_DRAIN_FLAG
            pin.pin.gpio().iphcr0().write_value(sifli_pac::hpsys_gpio::regs::Iphcr0(pin.bit()));
        } else {
            pin.pin.gpio().isr1().write_value(sifli_pac::hpsys_gpio::regs::Isr1(pin.bit()));
            pin.pin.gpio().iphcr1().write_value(sifli_pac::hpsys_gpio::regs::Iphcr1(pin.bit()));
        }
        
        unsafe {
            pin.set_fsel_unchecked(0);
        }
        pin     
    }

    #[inline]
    fn bit(&self) -> u32 {
        1 << (self.pin.pin() % 32)
    }

    /// Set the pin's pull.
    #[inline]
    pub fn set_pull(&mut self, pull: Pull) {
        let (pe, ps) = match pull {
            Pull::None => (false, vals::Ps::Down),
            Pull::Up => (true, vals::Ps::Up),
            Pull::Down => (true, vals::Ps::Down),
        };

        let pin_id = self.pin.pin();
        match pin_id {
            0..=38 => {
                self.pin.pinmux().pad_pa0_38(pin_id as _).modify(|w| {
                    w.set_pe(pe);
                    w.set_ps(ps);
                });
            },
            39..=42 => {
                self.pin.pinmux().pad_pa39_42((pin_id - 39) as _).modify(|w| {
                    w.set_pe(pe);
                    w.set_ps(ps);
                });
            },
            43..=44 => {
                self.pin.pinmux().pad_pa43_44((pin_id - 43) as _).modify(|w| {
                    w.set_pe(pe);
                    w.set_ps(ps);
                });
            },
            _ => unreachable!(),
        }
    }

    /// Set the pin's drive strength.
    #[inline]
    pub fn set_drive_strength(&mut self, strength: Drive) {
        // It would be best to merge ds1 and ds0, but currently chiptool does not 
        // seem to have such a transform?
        let (ds1, ds0) = match strength {
            Drive::Drive0 => (false, false),
            Drive::Drive1 => (false, true),
            Drive::Drive2 => (true, false),
            Drive::Drive3 => (true, true),
        };
        let pin_id = self.pin.pin();
        match pin_id {
            0..=38 => {
                self.pin.pinmux().pad_pa0_38(pin_id as _).modify(|w| {
                    w.set_ds0(ds0);
                    w.set_ds1(ds1);
                });
            },
            39..=42 => {
                let ds = match strength {
                    Drive::Drive0 => false,
                    Drive::Drive1 => true,
                    _ => {
                        warn!("PA39-42 can only be set to Drive0 or Drive1");
                        true
                    },
                };
                self.pin.pinmux().pad_pa39_42((pin_id - 39) as _).modify(|w| {
                    w.set_ds(ds);
                });
            },
            43..=44 => {
                self.pin.pinmux().pad_pa43_44((pin_id - 43) as _).modify(|w| {
                    w.set_ds0(ds0);
                    w.set_ds1(ds1);
                });
            },
            _ => unreachable!(),
        } 
    }

    /// Set the pin's slew rate.
    #[inline]
    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        let sr = match slew_rate {
            SlewRate::Fast => vals::Sr::Fast,
            SlewRate::Slow => vals::Sr::Slow,
        };
        let pin_id = self.pin.pin();
        match pin_id {
            0..=38 => {
                self.pin.pinmux().pad_pa0_38(pin_id as _).modify(|w| {
                    w.set_sr(sr);
                });
            },
            39..=42 => {
                // TODO: should this be a panic?
                warn!("Cannot set slew rate on pad 39-42");
            },
            43..=44 => {
                self.pin.pinmux().pad_pa43_44((pin_id - 43) as _).modify(|w| {
                    w.set_sr(sr);
                });
            },
            _ => unreachable!(),
        }
    }

    /// Set the pin's Schmitt trigger input.
    /// if disabled, it will be a CMOS input.
    #[inline]
    pub fn set_schmitt(&mut self, enable: bool) {
        let is = if enable {
            vals::Is::Schmitt
        } else {
            vals::Is::Cmos
        };
        let pin_id = self.pin.pin();
        match pin_id {
            0..=38 => {
                self.pin.pinmux().pad_pa0_38(pin_id as _).modify(|w| {
                    w.set_is(is);
                });
            },
            39..=42 => {
                self.pin.pinmux().pad_pa39_42((pin_id - 39) as _).modify(|w| {
                    w.set_is(is);
                });
            },
            43..=44 => {
                self.pin.pinmux().pad_pa43_44((pin_id - 43) as _).modify(|w| {
                    w.set_is(is);
                });
            },
            _ => unreachable!(),
        }
    }

    /// Put the pin into input mode.
    ///
    /// The pull setting is left unchanged.
    #[inline]
    pub fn set_as_input(&mut self) {
        if self.pin.pin() / 32 == 0 {
            self.pin.gpio().doecr0().write_value(sifli_pac::hpsys_gpio::regs::Doecr0(self.bit()));
        } else {
            self.pin.gpio().doecr1().write_value(sifli_pac::hpsys_gpio::regs::Doecr1(self.bit()));
        }
    }

    /// Put the pin into output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn set_as_output(&mut self) {
        if self.pin.pin() / 32 == 0 {
            self.pin.gpio().doesr0().write_value(sifli_pac::hpsys_gpio::regs::Doesr0(self.bit()));
        } else {
            self.pin.gpio().doesr1().write_value(sifli_pac::hpsys_gpio::regs::Doesr1(self.bit()));
        }
    }

    /// Put the pin into open-drain output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn set_as_output_od(&mut self) {
        if self.pin.pin() / 32 == 0 {
            self.pin.gpio().iphsr0().write_value(sifli_pac::hpsys_gpio::regs::Iphsr0(self.bit()));
            self.pin.gpio().iplsr0().write_value(sifli_pac::hpsys_gpio::regs::Iplsr0(self.bit()));
        } else {
            self.pin.gpio().iphsr1().write_value(sifli_pac::hpsys_gpio::regs::Iphsr1(self.bit()));
            self.pin.gpio().iplsr1().write_value(sifli_pac::hpsys_gpio::regs::Iplsr1(self.bit()));
        }
    }

    /// Set as output pin.
    #[inline]
    pub fn is_set_as_output(&self) -> bool {
        let bit = if self.pin.pin() / 32 == 0 {
            self.pin.gpio().doesr0().read().0
        } else {
            self.pin.gpio().doesr1().read().0
        };
        (bit & self.bit()) != 0
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        let bit = if self.pin.pin() / 32 == 0 {
            self.pin.gpio().dir0().read().0
        } else {
            self.pin.gpio().dir1().read().0
        };
        (bit & self.bit()) == 0
    }

    /// Returns current pin level
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        if self.pin.pin() / 32 == 0 {
            self.pin.gpio().dosr0().write_value(sifli_pac::hpsys_gpio::regs::Dosr0(self.bit()));
        } else {
            self.pin.gpio().dosr1().write_value(sifli_pac::hpsys_gpio::regs::Dosr1(self.bit()));
        }
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        if self.pin.pin() / 32 == 0 {
            self.pin.gpio().docr0().write_value(sifli_pac::hpsys_gpio::regs::Docr0(self.bit()));
        } else {
            self.pin.gpio().docr1().write_value(sifli_pac::hpsys_gpio::regs::Docr1(self.bit()));
        }
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    /// Is the output level high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    /// Is the output level low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        let bit = if self.pin.pin() / 32 == 0 {
            self.pin.gpio().dor0().read().0
        } else {
            self.pin.gpio().dor1().read().0
        };
        (bit & self.bit()) == 0
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.set_level(!self.get_output_level());
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::LevelHigh).await;
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::LevelLow).await;
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::EdgeHigh).await;
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::EdgeLow).await;
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::AnyEdge).await;
    }

    /// set pinmux fsel
    pub unsafe fn set_fsel_unchecked(&mut self, fsel: u8) {
        let pin_id = self.pin.pin();
        match pin_id {
            0..=38 => {
                self.pin.pinmux().pad_pa0_38(pin_id as _).modify(|w| w.set_fsel(fsel));
            },
            39..=42 => {
                self.pin.pinmux().pad_pa39_42((pin_id - 39) as _).modify(|w| w.set_fsel(fsel));
            },
            43..=44 => {
                self.pin.pinmux().pad_pa43_44((pin_id - 43) as _).modify(|w| w.set_fsel(fsel));
            },
            _ => unreachable!(),
        }
    }
}

impl<'d> Drop for Flex<'d> {
    #[inline]
    fn drop(&mut self) {
        // let idx = self.pin._pin() as usize;
        // self.pin.pad_ctrl().write(|_| {});
        // self.pin.gpio().ctrl().write(|w| {
        //     w.set_funcsel(pac::io::vals::Gpio0ctrlFuncsel::NULL as _);
        // });
    }
}

pub(crate) trait SealedPin: Sized {
    fn pin_bank(&self) -> u8;

    #[inline]
    fn _pin(&self) -> u8 {
        self.pin_bank() & 0x7f
    }

    #[inline]
    fn _bank(&self) -> u8 {
        self.pin_bank() >> 7
    }

    fn gpio(&self) -> pac::hpsys_gpio::HpsysGpio {
        pac::HPSYS_GPIO
    }

    fn pinmux(&self) -> pac::hpsys_pinmux::HpsysPinmux {
        pac::HPSYS_PINMUX
    }

    /// Set the pin as "disconnected", ie doing nothing and consuming the lowest
    /// amount of power possible.
    #[inline]
    fn set_as_disconnected(&self) {
        let pin_id = self._pin();
        let pin_bit = 1 << (pin_id % 32);

        // DISABLE_ISR
        if pin_id % 32 == 0 {
            self.gpio().iecr0().write_value(regs::Iecr0(pin_bit));
        } else {
            self.gpio().iecr1().write_value(regs::Iecr1(pin_bit));
        }

        // WAIT_ISR_DISABLED
        crate::cortex_m_blocking_delay_us(1);

        if pin_id % 32 == 0 {
            self.gpio().isr0().write_value(sifli_pac::hpsys_gpio::regs::Isr0(pin_bit));

            // CLEAR_OPEN_DRAIN_FLAG
            self.gpio().iphcr0().write_value(sifli_pac::hpsys_gpio::regs::Iphcr0(pin_bit));
        } else {
            self.gpio().isr1().write_value(sifli_pac::hpsys_gpio::regs::Isr1(pin_bit));
            self.gpio().iphcr1().write_value(sifli_pac::hpsys_gpio::regs::Iphcr1(pin_bit));
        }

        match pin_id {
            0..=38 => {
                self.pinmux().pad_pa0_38(pin_id as _).modify(|w| w.set_fsel(0));
            },
            39..=42 => {
                self.pinmux().pad_pa39_42((pin_id - 39) as _).modify(|w| w.set_fsel(0));
            },
            43..=44 => {
                self.pinmux().pad_pa43_44((pin_id - 43) as _).modify(|w| w.set_fsel(0));
            },
            _ => unreachable!(),
        }
    }
}

/// Interface for a Pin that can be configured by an [Input] or [Output] driver, or converted to an [AnyPin].
#[allow(private_bounds)]
pub trait Pin: Peripheral<P = Self> + Into<AnyPin> + SealedPin + Sized + 'static {
    /// Degrade to a generic pin struct
    fn degrade(self) -> AnyPin {
        AnyPin {
            pin_bank: self.pin_bank(),
        }
    }

    /// Returns the pin number within a bank
    #[inline]
    fn pin(&self) -> u8 {
        self._pin()
    }

    /// Returns the bank of this pin (PA=0, PB=1)
    #[inline]
    fn bank(&self) -> u8 {
        self._bank()
    }
}

/// Type-erased GPIO pin
pub struct AnyPin {
    pin_bank: u8,
}

impl AnyPin {
    /// Unsafely create a new type-erased pin.
    ///
    /// # Safety
    ///
    /// You must ensure that you’re only using one instance of this type at a time.
    pub unsafe fn steal(pin_bank: u8) -> Self {
        Self { pin_bank }
    }
}

impl_peripheral!(AnyPin);

impl Pin for AnyPin {}
impl SealedPin for AnyPin {
    fn pin_bank(&self) -> u8 {
        self.pin_bank
    }
}

// ==========================

macro_rules! impl_pin {
    ($name:ident, $bank:expr, $pin_num:expr) => {
        impl Pin for peripherals::$name {}
        impl SealedPin for peripherals::$name {
            #[inline]
            fn pin_bank(&self) -> u8 {
                ($bank as u8) * 128 + $pin_num
            }
        }

        impl From<peripherals::$name> for crate::gpio::AnyPin {
            fn from(val: peripherals::$name) -> Self {
                crate::gpio::Pin::degrade(val)
            }
        }
    };
}

impl_pin!(PA0, 0, 0);
impl_pin!(PA1, 0, 1);
impl_pin!(PA2, 0, 2);
impl_pin!(PA3, 0, 3);
impl_pin!(PA4, 0, 4);
impl_pin!(PA5, 0, 5);
impl_pin!(PA6, 0, 6);
impl_pin!(PA7, 0, 7);
impl_pin!(PA8, 0, 8);
impl_pin!(PA9, 0, 9);
impl_pin!(PA10, 0, 10);
impl_pin!(PA11, 0, 11);
impl_pin!(PA12, 0, 12);
impl_pin!(PA13, 0, 13);
impl_pin!(PA14, 0, 14);
impl_pin!(PA15, 0, 15);
impl_pin!(PA16, 0, 16);
impl_pin!(PA17, 0, 17);
impl_pin!(PA18, 0, 18);
impl_pin!(PA19, 0, 19);
impl_pin!(PA20, 0, 20);
impl_pin!(PA21, 0, 21);
impl_pin!(PA22, 0, 22);
impl_pin!(PA23, 0, 23);
impl_pin!(PA24, 0, 24);
impl_pin!(PA25, 0, 25);
impl_pin!(PA26, 0, 26);
impl_pin!(PA27, 0, 27);
impl_pin!(PA28, 0, 28);
impl_pin!(PA29, 0, 29);

// ====================

mod eh02 {
    use super::*;

    impl<'d> embedded_hal_02::digital::v2::InputPin for Input<'d> {
        type Error = Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_high())
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::OutputPin for Output<'d> {
        type Error = Infallible;

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Ok(self.set_high())
        }

        fn set_low(&mut self) -> Result<(), Self::Error> {
            Ok(self.set_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::StatefulOutputPin for Output<'d> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }

        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::ToggleableOutputPin for Output<'d> {
        type Error = Infallible;
        #[inline]
        fn toggle(&mut self) -> Result<(), Self::Error> {
            Ok(self.toggle())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::InputPin for OutputOpenDrain<'d> {
        type Error = Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_high())
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::OutputPin for OutputOpenDrain<'d> {
        type Error = Infallible;

        #[inline]
        fn set_high(&mut self) -> Result<(), Self::Error> {
            Ok(self.set_high())
        }

        #[inline]
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Ok(self.set_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::StatefulOutputPin for OutputOpenDrain<'d> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }

        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::ToggleableOutputPin for OutputOpenDrain<'d> {
        type Error = Infallible;
        #[inline]
        fn toggle(&mut self) -> Result<(), Self::Error> {
            Ok(self.toggle())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::InputPin for Flex<'d> {
        type Error = Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_high())
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::OutputPin for Flex<'d> {
        type Error = Infallible;

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Ok(self.set_high())
        }

        fn set_low(&mut self) -> Result<(), Self::Error> {
            Ok(self.set_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::StatefulOutputPin for Flex<'d> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }

        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::ToggleableOutputPin for Flex<'d> {
        type Error = Infallible;
        #[inline]
        fn toggle(&mut self) -> Result<(), Self::Error> {
            Ok(self.toggle())
        }
    }
}

impl<'d> embedded_hal_1::digital::ErrorType for Input<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal_1::digital::InputPin for Input<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal_1::digital::ErrorType for Output<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal_1::digital::OutputPin for Output<'d> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }
}

impl<'d> embedded_hal_1::digital::StatefulOutputPin for Output<'d> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<'d> embedded_hal_1::digital::ErrorType for OutputOpenDrain<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal_1::digital::OutputPin for OutputOpenDrain<'d> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }
}

impl<'d> embedded_hal_1::digital::StatefulOutputPin for OutputOpenDrain<'d> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<'d> embedded_hal_1::digital::InputPin for OutputOpenDrain<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal_1::digital::ErrorType for Flex<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal_1::digital::InputPin for Flex<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal_1::digital::OutputPin for Flex<'d> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }
}

impl<'d> embedded_hal_1::digital::StatefulOutputPin for Flex<'d> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<'d> embedded_hal_async::digital::Wait for Flex<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

impl<'d> embedded_hal_async::digital::Wait for Input<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

impl<'d> embedded_hal_async::digital::Wait for OutputOpenDrain<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

