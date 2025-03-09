#![macro_use]

macro_rules! new_pin {
    ($name:ident, $af_type:expr) => {{
        let pin = $name.into_ref();
        pin.set_function(pin.fsel(), $af_type);
        pin.set_cfg_pin();
        Some(pin.map_into())
    }};
}

macro_rules! pin_trait {
    ($signal:ident, $instance:path $(, $mode:path)?) => {
        #[doc = concat!(stringify!($signal), " pin trait")]
        pub trait $signal<T: $instance $(, M: $mode)?>: crate::gpio::Pin {
            #[doc = concat!("Get the fsel number needed to use this pin as ", stringify!($signal))]
            fn fsel(&self) -> u8;

            #[doc = concat!("Configure HPSYS_CFG if needed", stringify!($signal))]
            fn set_cfg_pin(&self) {}
        }
    };
}
