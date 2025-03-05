# SiFli Rust HAL

[![Crates.io][badge-license]][crates]
[![Support status][badge-support-status]][githubrepo]

[badge-license]: https://img.shields.io/crates/l/sifli-hal?style=for-the-badge
[badge-support-status]: https://img.shields.io/badge/Support_status-Community-mediumpurple?style=for-the-badge
[crates]: https://crates.io/crates/sifli-hal
[githubrepo]: https://github.com/OpenSiFli/sifli-rs

Rust Hardware Abstraction Layer (HAL) for SiFli MCUs.

> [!WARNING]
> 
> This crate is a working-in-progress and not ready for production use.

## Crates
| Github                                                       | crates.io                                       | docs.rs                                    | Support Status       |
| ------------------------------------------------------------ | ----------------------------------------------- | ------------------------------------------ | -------------------- |
| [sifli-hal](https://github.com/OpenSiFli/sifli-rs/tree/main/sifli-hal) | [![Crates.io][hal-badge-version]][hal-cratesio] | [![docs.rs][hal-badge-docsrs]][hal-docsrs] | ![][badge-community] |
| [sifli-pac](https://github.com/OpenSiFli/sifli-pac)          | [![Crates.io][pac-badge-version]][pac-cratesio] | [![docs.rs][pac-badge-docsrs]][pac-docsrs] | ![][badge-community] |
| [sifli-flash-table ](https://github.com/OpenSiFli/sifli-rs/tree/main/sifli-flash-table) |                                                 |                                            | ![][badge-community] |

[badge-community]: https://img.shields.io/badge/Community-mediumpurple?style=for-the-badge

[hal-cratesio]: https://crates.io/crates/sifli-hal
[hal-docsrs]: https://docs.rs/sifli-hal
[hal-badge-license]: https://img.shields.io/crates/l/sifli-hal?style=for-the-badge
[hal-badge-version]: https://img.shields.io/crates/v/sifli-hal?style=for-the-badge
[hal-badge-docsrs]: https://img.shields.io/docsrs/sifli-hal?style=for-the-badge

[pac-cratesio]: https://crates.io/crates/sifli-pac
[pac-docsrs]: https://docs.rs/sifli-pac
[pac-badge-license]: https://img.shields.io/crates/l/sifli-pac?style=for-the-badge
[pac-badge-version]: https://img.shields.io/crates/v/sifli-pac?style=for-the-badge
[pac-badge-docsrs]: https://img.shields.io/docsrs/sifli-pac?style=for-the-badge

## Get Started

### Build & Flash

First, install [cargo-binutils](https://github.com/rust-embedded/cargo-binutils):

```bash
cargo install cargo-binutils
rustup component add llvm-tools
```

Next, use `objcopy` to generate a `.bin` file:

```bash
cargo objcopy --bin blinky -- -O binary main.bin
```

Then, compile the [blink/no-os](https://github.com/OpenSiFli/SiFli-SDK/tree/main/example/get-started/blink/no-os) project in the SDK and copy the `main.bin` file into the build directory (e.g., `build_em-lb525_hcpu`), replacing the existing `main.bin` file.

Make sure the new firmware size is smaller than the old one; otherwise, you may need to manually modify the `ftab` or use [sifli-flash-table](sifli-flash-table/README.md) to generate a new `ftab.bin`.

Afterward, use the same programming method as with the SDK (for example, running `build_em-lb525_hcpu\uart_download.bat` or programming via JLink).

### Debug

By utilizing [SifliUsartServer](https://github.com/OpenSiFli/SiFli-SDK/tree/main/tools/SifliUsartServer) , you can generate a J-Link server, which then allows you to connect to it using Cortex-Debug within VS Code.

```json
"configurations": [
        {
            "cwd": "${workspaceFolder}",
            "name": "Cortex Debug",
            "request": "attach",
            "type": "cortex-debug",
            "device": "Cortex-M33",
            "runToEntryPoint": "entry",
            "showDevDebugOutput": "none",
            "servertype": "jlink",
            "serverpath": "xxx/Dev/Jlink/JLink_V812e/JLinkGDBServerCL.exe",
            "ipAddress": "127.0.0.1:19025",
            "interface": "swd",
            "svdFile": "xxx/sifli-pac/svd/SF32LB52x.svd",
            "executable": "examples/sf32lb52x/target/thumbv8m.main-none-eabi/debug/blinky"
        },
    ]
```

In certain HardFault scenarios, the Cortex-Debug connection may be interrupted. If this occurs, you might need to resort to J-Link Commander or alternative tools for debugging.

## License

This project is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.