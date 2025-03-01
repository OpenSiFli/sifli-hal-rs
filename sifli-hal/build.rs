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

    // Get output path from env
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let dest_path = out_dir.join("_generated.rs");

    let mut token_stream = TokenStream::new();

    // Generate interrupt mod
    let interrupt_mod = generate_interrupt_mod(&interrupts);
    token_stream.extend(interrupt_mod);

    // Generate peripherals singleton
    let peripherals_singleton = generate_peripherals_singleton(&peripherals);
    token_stream.extend(peripherals_singleton);

    // Generate implementations
    let implementations = generate_rcc_impl(&peripherals, &fieldsets);
    token_stream.extend(implementations);

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
                impl crate::rcc::SealedRccEnableReset for crate::peripherals::#peripheral_name_ident {}
                impl crate::rcc::RccEnableReset for crate::peripherals::#peripheral_name_ident {}
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
            impl crate::rcc::SealedRccEnableReset for crate::peripherals::#peripheral_name_ident {
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
            impl crate::rcc::RccEnableReset for crate::peripherals::#peripheral_name_ident {}
        };
        implementations.extend(impl_tokens);
    }

    implementations.extend(quote! {use crate::time::Hertz;});
    for peripheral in &peripherals.hcpu {
        if let Some(clock) = peripheral.clock.clone() {
            let clock_fn_ident = format_ident!("get_{}_freq", clock);
            let peripheral_name_ident = format_ident!("{}", peripheral.name);
            let impl_tokens = quote! {
                impl crate::rcc::SealedRccGetFreq for crate::peripherals::#peripheral_name_ident {
                    fn get_freq() -> Option<Hertz> {
                        crate::rcc::#clock_fn_ident()
                    }
                }
                impl crate::rcc::RccGetFreq for crate::peripherals::#peripheral_name_ident {}
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