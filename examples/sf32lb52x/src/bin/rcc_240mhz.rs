#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use embassy_time::Timer;
use embassy_executor::Spawner;

use sifli_hal;
use sifli_hal::gpio;
use sifli_hal::rcc::{self, ConfigOption, DllConfig};

// **WARN**:
// The RCC clock configuration module is still under construction, 
// and there is no guarantee that other clock configurations will 
// run correctly.
// https://github.com/OpenSiFli/sifli-rs/issues/7

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");
    let mut config = sifli_hal::Config::default();
    // 240MHz Dll1 Freq = (stg + 1) * 24MHz
    config.rcc.dll1 = ConfigOption::Update(DllConfig { enable: true, stg: 9, div2: false });
    let p = sifli_hal::init(config);

    rcc::test_print_clocks();

    // SF32LB52-DevKit-LCD LED pin
    let mut led = gpio::Output::new(p.PA26, gpio::Level::Low);
    
    loop {
        info!("led on!");
        led.set_high();
        Timer::after_secs(1).await;

        info!("led off!");
        led.set_low();
        Timer::after_secs(1).await;
    }
}
