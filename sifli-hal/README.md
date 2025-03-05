# SiFli HAL

[![Crates.io][badge-license]][crates]
[![Crates.io][badge-version]][crates]
[![docs.rs][badge-docsrs]][docsrs]
[![Support status][badge-support-status]][githubrepo]

[badge-license]: https://img.shields.io/crates/l/sifli-hal?style=for-the-badge
[badge-version]: https://img.shields.io/crates/v/sifli-hal?style=for-the-badge
[badge-docsrs]: https://img.shields.io/docsrs/sifli-hal?style=for-the-badge
[badge-support-status]: https://img.shields.io/badge/Support_status-Community-yellow?style=for-the-badge
[crates]: https://crates.io/crates/sifli-hal
[docsrs]: https://docs.rs/sifli-hal
[githubrepo]: https://github.com/OpenSiFli/sifli-hal

Rust Hardware Abstraction Layer (HAL) and [Embassy](https://github.com/embassy-rs/embassy) driver for SiFli MCUs.

> [!WARNING]
> 
> This crate is a working-in-progress and not ready for production use.

## Get Started

[Get Started](../README.md#get-started)



## Status

| Family    | SF32LB52x        |
| --------- | ---------------- |
| Embassy   | ✅+               |
| RCC       | ✅                |
| GPIO      | ✅                |
| INTERRUPT | ✅                |
| PMU       | DVFS switch only |
| DMA       |                  |
| USART     |                  |
| I2C       |                  |
| SPI       |                  |
| Bluetooth |                  |
| USB       |                  |
| ePicasso  |                  |

- ✅ : Implemented
- Blank : Not implemented
- ❓ : Requires demo verification
- `+` : Async support
- N/A : Not available

## Features

- `defmt`, `log`: Debug log output.

- `sf32lb52x`: Target chip selection. Currently, only `sf32lb52x` is supported.

- `set-msplim`: Set the MSPLIM register in `__pre_init`. This register must be set before the main function’s stack setup (since the bootloader may have already configured it to a different value), otherwise, it will cause a HardFault [SiFli-SDK #32](https://github.com/OpenSiFli/SiFli-SDK/issues/32).

  This feature will be removed after [cortex-m-rt #580](https://github.com/rust-embedded/cortex-m/pull/580)  is released.

- `time-driver-xxx`: Timer configuration for `time-driver`. It requires at least two capture/compare channels. For the `sf32lb52x hcpu`, only `atim1` (TODO: [#5](https://github.com/OpenSiFli/sifli-rs/issues/5)), `gptim1`, and `gptim2` are available.

- `unchecked-overclocking`: Enable this feature to disable the overclocking check. DO NOT ENABLE THIS FEATURE UNLESS YOU KNOW WHAT YOU'RE DOING.

## License

This project is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)

- MIT license ([LICENSE-MIT](../LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.