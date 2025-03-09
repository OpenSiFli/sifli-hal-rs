use std::env;
use std::fs;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::path::PathBuf;
use std::collections::BTreeMap;
use std::process::Command;

use proc_macro2::TokenStream;
use quote::quote;
use quote::format_ident;
use serde_yaml;

mod build_serde;
// Structures imported from build_serde.rs
use build_serde::{IR, FieldSet, Field, Interrupts, Peripherals};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Retrieve all enabled features
    let chip_name = match env::vars()
        .map(|(a, _)| a)
        .filter(|x| x.starts_with("CARGO_FEATURE_SF32"))
        .get_one()
    {
        Ok(x) => x,
        Err(GetOneError::None) => panic!("No sf32xx Cargo feature enabled"),
        Err(GetOneError::Multiple) => panic!("Multiple sf32xx Cargo features enabled"),
    }
        .strip_prefix("CARGO_FEATURE_")
        .unwrap()
        .to_ascii_lowercase();

    let _time_driver_peripheral = match env::vars()
        .map(|(key, _)| key)
        .filter(|x| x.starts_with("CARGO_FEATURE_TIME_DRIVER_"))
        .get_one()
    {
        Ok(x) => Some(
            x.strip_prefix("CARGO_FEATURE_TIME_DRIVER_")
                .unwrap()
                .to_ascii_uppercase()
        ),
        Err(GetOneError::None) => None,
        Err(GetOneError::Multiple) => panic!("Multiple time-driver-xx Cargo features enabled"),
    };

    println!("cargo:rerun-if-changed=data/{}", chip_name);
    let data_dir = Path::new("data").join(chip_name);

    // Read and parse HPSYS_RCC.yaml
    let rcc_path = data_dir.join("HPSYS_RCC.yaml");
    let rcc_content = fs::read_to_string(&rcc_path).unwrap();
    
    let ir: IR = serde_yaml::from_str(&rcc_content)
        .map_err(|e| format!("Failed to parse HPSYS_RCC.yaml: {}", e))?;

    let _blocks = ir.blocks;
    let fieldsets = ir.fieldsets;

    // Read and parse interrupts.yaml
    let interrupts_path = data_dir.join("interrupts.yaml");
    let interrupts_content = fs::read_to_string(&interrupts_path)
        .map_err(|e| format!("Failed to read interrupts.yaml: {}", e))?;
    
    let interrupts: Interrupts = serde_yaml::from_str(&interrupts_content)
        .map_err(|e| format!("Failed to parse interrupts.yaml: {}", e))?;

    // Read and parse peripherals.yaml
    let peripherals_path = data_dir.join("peripherals.yaml");
    let peripherals_content = fs::read_to_string(&peripherals_path)
        .map_err(|e| format!("Failed to read peripherals.yaml: {}", e))?;
    
    let peripherals: Peripherals = serde_yaml::from_str(&peripherals_content)
        .map_err(|e| format!("Failed to parse peripherals.yaml: {}", e))?;

    // Read and parse HPSYS_CFG.yaml
    let cfg_path = data_dir.join("HPSYS_CFG.yaml");
    let cfg_content = fs::read_to_string(&cfg_path)?;
    let cfg_ir: IR = serde_yaml::from_str(&cfg_content)?;

    // Read and parse pinmux_signals.yaml
    let pinmux_signals_path = data_dir.join("pinmux_signals.yaml");
    let pinmux_signals_content = fs::read_to_string(&pinmux_signals_path)?;
    let pinmux_signals: build_serde::PinmuxSignals = serde_yaml::from_str(&pinmux_signals_content)?;

    // Read and parse pinmux.yaml
    let pinmux_path = data_dir.join("pinmux.yaml");
    let pinmux_content = fs::read_to_string(&pinmux_path)?;
    let pinmux: build_serde::Pinmux = serde_yaml::from_str(&pinmux_content)?;
    
    // Get output path from env
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let dest_path = out_dir.join("_generated.rs");

    let mut token_stream = TokenStream::new();

    token_stream.extend(quote! {
        use crate::peripherals::*;
        use crate::gpio::Pin;
        use crate::gpio::SealedPin;
    });

    // Generate interrupt mod
    let interrupt_mod = generate_interrupt_mod(&interrupts);
    token_stream.extend(interrupt_mod);

    // Generate peripherals singleton
    let peripherals_singleton = generate_peripherals_singleton(&peripherals);
    token_stream.extend(peripherals_singleton);

    // Generate implementations
    let implementations = generate_rcc_impl(&peripherals, &fieldsets);
    token_stream.extend(implementations);

    // Generate pin implementations
    let pin_impls = generate_pin_impls(&pinmux, &pinmux_signals, &cfg_ir.fieldsets);
    token_stream.extend(pin_impls);

    // Write to file
    let mut file = File::create(&dest_path).unwrap();
    write!(file, "{}", token_stream).unwrap();
    rustfmt(&dest_path);
    Ok(())
}

fn generate_rcc_impl(peripherals: &Peripherals, fieldsets: &BTreeMap<String, FieldSet>) -> TokenStream {
    let mut implementations = TokenStream::new();
    
    // Get RCC register fieldsets
    let rstr1 = fieldsets.get("RSTR1").expect("RSTR1 fieldset not found");
    let rstr2 = fieldsets.get("RSTR2").expect("RSTR2 fieldset not found");
    let enr1 = fieldsets.get("ENR1").expect("ENR1 fieldset not found");
    let enr2 = fieldsets.get("ENR2").expect("ENR2 fieldset not found");

    for peripheral in &peripherals.hcpu {
        if !peripheral.enable_reset {
            continue;
        }

        // TODO: Is there a better way to handle this?
        if peripheral.ignore_missing_enable_reset {
            let peripheral_name_ident = format_ident!("{}", peripheral.name);
            let impl_tokens = quote! {
                impl crate::rcc::SealedRccEnableReset for #peripheral_name_ident {}
                impl crate::rcc::RccEnableReset for #peripheral_name_ident {}
            };
            implementations.extend(impl_tokens);
            continue;
        }
        // Get field name (prefer rcc_field if available)
        let field_name = &peripheral.rcc_field.clone()
            .unwrap_or(peripheral.name.clone()).to_lowercase();

        // Find matching fields in RCC registers
        let (enr_reg, _enr_field) = find_field_in_registers(&[
            ("ENR1", enr1),
            ("ENR2", enr2),
        ], &field_name).expect(&format!("No ENR field found for peripheral {}", peripheral.name));

        let (rstr_reg, _rstr_field) = find_field_in_registers(&[
            ("RSTR1", rstr1),
            ("RSTR2", rstr2),
        ], &field_name).expect(&format!("No RSTR field found for peripheral {}", peripheral.name));
        let field_set_ident = format_ident!("set_{}", field_name);
        let field_name_ident = format_ident!("{}", field_name);
        let enr_reg_ident = format_ident!("{}", enr_reg.to_lowercase());
        let rstr_reg_ident = format_ident!("{}", rstr_reg.to_lowercase());

        let peripheral_name_ident = format_ident!("{}", peripheral.name);
        let impl_tokens = quote! {
            impl crate::rcc::SealedRccEnableReset for #peripheral_name_ident {
                #[inline(always)]
                fn rcc_enable() {
                    crate::pac::HPSYS_RCC.#enr_reg_ident().modify(|w| w.#field_set_ident(true));
                }

                #[inline(always)]
                fn rcc_disable() {
                    crate::pac::HPSYS_RCC.#enr_reg_ident().modify(|w| w.#field_set_ident(false));
                }

                #[inline(always)]
                fn rcc_reset() {
                    crate::pac::HPSYS_RCC.#rstr_reg_ident().modify(|w| w.#field_set_ident(true));
                    while !crate::pac::HPSYS_RCC.#rstr_reg_ident().read().#field_name_ident() {}; 
                    crate::pac::HPSYS_RCC.#rstr_reg_ident().modify(|w| w.#field_set_ident(false));
                }
            }
            impl crate::rcc::RccEnableReset for #peripheral_name_ident {}
        };
        implementations.extend(impl_tokens);
    }

    implementations.extend(quote! {use crate::time::Hertz;});
    for peripheral in &peripherals.hcpu {
        if let Some(clock) = peripheral.clock.clone() {
            let clock_fn_ident = format_ident!("get_{}_freq", clock);
            let peripheral_name_ident = format_ident!("{}", peripheral.name);
            let impl_tokens = quote! {
                impl crate::rcc::SealedRccGetFreq for #peripheral_name_ident {
                    fn get_freq() -> Option<Hertz> {
                        crate::rcc::#clock_fn_ident()
                    }
                }
                impl crate::rcc::RccGetFreq for #peripheral_name_ident {}
            };

            implementations.extend(impl_tokens);
        }
    }
    implementations
}

fn find_field_in_registers<'a>(
    registers: &[(&str, &'a FieldSet)],
    field_name: &str,
) -> Option<(String, &'a Field)> {
    for (reg_name, fieldset) in registers {
        if let Some(field) = fieldset.fields.iter().find(|f| f.name.to_lowercase() == field_name) {
            return Some((reg_name.to_string(), field));
        }
    }
    None
}

fn generate_interrupt_mod(interrupts: &Interrupts) -> TokenStream {
    let interrupt_names: Vec<_> = interrupts.hcpu
        .iter()
        .map(|int| {
            let name = &int.name;
            quote::format_ident!("{}", name)
        })
        .collect();
    quote! {
        embassy_hal_internal::interrupt_mod!(
            #(#interrupt_names),*
        );
    }
}

fn generate_peripherals_singleton(peripherals: &Peripherals) -> TokenStream {
    let peripheral_names: Vec<_> = peripherals.hcpu
        .iter()
        .map(|p| {
            let name = &p.name;
            quote::format_ident!("{}", name)
        })
        .collect();
    
    // TODO: move pin num to chip info
    let gpio_pins: Vec<_> = (0..=44)
        .map(|i| {
            let pin_name = format!("PA{}", i);
            quote::format_ident!("{}", pin_name)
        })
        .collect();
    
    let dmac_channels: Vec<_> = (1..=8)
        .map(|i| {
            let channel_name = format!("DMAC_CH{}", i);
            quote::format_ident!("{}", channel_name)
        })
        .collect();
    
    quote! {
        embassy_hal_internal::peripherals! {
            #(#peripheral_names,)*
            #(#gpio_pins,)*
            #(#dmac_channels,)*
        }
    }
}

fn generate_pin_impls(
    pinmux: &build_serde::Pinmux,
    pinmux_signals: &build_serde::PinmuxSignals,
    fieldsets: &BTreeMap<String, FieldSet>,
) -> TokenStream {
    let mut implementations = TokenStream::new();

    for pin in &pinmux.hcpu {
        let pin_name = pin.pin.replace("GPIO_", "P");
        
        for func in &pin.functions {
            // Try to match function against pinmux_signals
            if let Some(signal_def) = find_matching_signal(&func.function, &pinmux_signals.hcpu) {
                generate_signal_impl(
                    &mut implementations,
                    signal_def,
                    &pin_name,
                    func,
                    pinmux_signals,
                    fieldsets,
                );
            }
        }
    }
    
    implementations
}

fn generate_signal_impl(
    implementations: &mut TokenStream,
    signal_def: &build_serde::SignalDefinition,
    pin_name: &str,
    func: &build_serde::PinFunction,
    pinmux_signals: &build_serde::PinmuxSignals,
    fieldsets: &BTreeMap<String, FieldSet>,
) {
    let pin_ident = format_ident!("{}", pin_name);
    
    match &signal_def.r#type {
        build_serde::SignalType::Gpio => {
            generate_signal_gpio_impl(implementations, pin_name, &pin_ident);
        },
        build_serde::SignalType::Superimposed => {
            // For superimposed type, process each sub-signal
            for signal_name in &signal_def.signals {
                // Find the corresponding signal definition
                if let Some(sub_signal) = find_matching_signal(&signal_name, &pinmux_signals.hcpu){
                    // Recursively process the sub-signal
                    generate_signal_impl(
                        implementations,
                        sub_signal,
                        pin_name,
                        func,
                        pinmux_signals,
                        fieldsets,
                    );
                }
            }
        },
        build_serde::SignalType::PeripheralMux => {
            generate_signal_peripheral_mux_impl(
                implementations,
                signal_def,
                &pin_ident,
                func,
                fieldsets,
            );
        },
        build_serde::SignalType::PeripheralNomux => {
            generate_signal_peripheral_nomux_impl(
                implementations,
                signal_def,
                &pin_ident,
                func,
            );
        },
    }
}

fn generate_signal_gpio_impl(
    implementations: &mut TokenStream,
    pin_name: &str,
    pin_ident: &proc_macro2::Ident,
) { 
    let pin_num = pin_name[2..].parse::<u8>().unwrap();
    let bank = 0u8; // For GPIO_Ax, bank is always 0
    implementations.extend(quote! {
        impl_pin!(#pin_ident, #bank, #pin_num);
    });
}

fn generate_signal_peripheral_mux_impl(
    implementations: &mut TokenStream,
    signal_def: &build_serde::SignalDefinition,
    pin_ident: &proc_macro2::Ident,
    func: &build_serde::PinFunction,
    fieldsets: &BTreeMap<String, FieldSet>,
) {
    let signal = signal_def.name.clone();
    // Handle peripheral mux implementations
    let pattern = regex::Regex::new(&format!(
        r"^{}(\d+)?_PINR(\d+)?$", signal
    )).unwrap();

    for (name, fieldset) in fieldsets.iter() {
        if let Some(captures) = pattern.captures(name) {
            let peripheral = if let Some(num) = captures.get(1) {
                format!("{}{}", signal, num.as_str())
            } else {
                signal.to_string()
            };

            for field in &fieldset.fields {
                if field.name.ends_with("_PIN") {
                    // Trait name. First letter upper case
                    let name = field.name.replace("_PIN", "").to_lowercase();
                    let cfg_pin = format!("{}Pin", 
                        name.chars().next().unwrap_or_default().to_uppercase().to_string() + 
                        &name[1..]);
                    let trait_path_str = signal_def.pin_trait.clone().unwrap()
                        .replace("$peripheral", &peripheral)
                        .replace("$cfg_pin", &cfg_pin);
                    let trait_path = syn::parse_str::<syn::Path>(&trait_path_str)
                        .unwrap_or_else(|_| panic!("Invalid trait path: {}", trait_path_str));

                    let reg_name = format_ident!("{}_pinr", peripheral.to_lowercase());
                    let set_field = format_ident!("set_{}", field.name.to_lowercase());
                    let func_value = func.value;

                    implementations.extend(quote! {
                        impl #trait_path for #pin_ident {
                            fn fsel(&self) -> u8 {
                                #func_value
                            }

                            fn set_cfg_pin(&self) {
                                crate::pac::HPSYS_CFG.#reg_name().modify(|w| 
                                    w.#set_field(self.pin_bank() as _)
                                );
                            }
                        }
                    });
                }
            }
        }
        
    }
}

fn generate_signal_peripheral_nomux_impl(
    implementations: &mut TokenStream,
    signal_def: &build_serde::SignalDefinition,
    pin_ident: &proc_macro2::Ident,
    func: &build_serde::PinFunction,
) {
    if let Some(pin_trait) = &signal_def.pin_trait {
        // Extract peripheral from signal name
        let peripheral = signal_def.name.split("_").next().unwrap();
        let trait_path_str = pin_trait.replace("$peripheral", peripheral);

        let trait_path = syn::parse_str::<syn::Path>(&trait_path_str)
            .unwrap_or_else(|_| panic!("Invalid trait path: {}", trait_path_str));
        let func_value = func.value;

        implementations.extend(quote! {
            impl #trait_path for #pin_ident {
                fn fsel(&self) -> u8 {
                    #func_value
                }
            }
        });
    } else {
        panic!("No pin trait specified for peripheral nomux signal {}", signal_def.name);
    }
}

fn find_matching_signal<'a>(
    function: &str,
    signals: &'a [build_serde::SignalDefinition],
) -> Option<&'a build_serde::SignalDefinition> {
    for signal in signals {
        if regex::Regex::new(&signal.name)
            .unwrap()
            .is_match(function) {
            return Some(signal);
        }
    }
    None
}
enum GetOneError {
    None,
    Multiple,
}

trait IteratorExt: Iterator {
    fn get_one(self) -> Result<Self::Item, GetOneError>;
}

impl<T: Iterator> IteratorExt for T {
    fn get_one(mut self) -> Result<Self::Item, GetOneError> {
        match self.next() {
            None => Err(GetOneError::None),
            Some(res) => match self.next() {
                Some(_) => Err(GetOneError::Multiple),
                None => Ok(res),
            },
        }
    }
}

/// rustfmt a given path.
/// Failures are logged to stderr and ignored.
fn rustfmt(path: impl AsRef<Path>) {
    let path = path.as_ref();
    match Command::new("rustfmt").args([path]).output() {
        Err(e) => {
            eprintln!("failed to exec rustfmt {:?}: {:?}", path, e);
        }
        Ok(out) => {
            if !out.status.success() {
                eprintln!("rustfmt {:?} failed:", path);
                eprintln!("=== STDOUT:");
                std::io::stderr().write_all(&out.stdout).unwrap();
                eprintln!("=== STDERR:");
                std::io::stderr().write_all(&out.stderr).unwrap();
            }
        }
    }
}