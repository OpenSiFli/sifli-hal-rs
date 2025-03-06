#![macro_use]

macro_rules! new_pin {
    ($name:ident, $af_type:expr) => {{
        let pin = $name.into_ref();
        // pin.set_as_af(pin.af_num(), $af_type);
        Some(pin.map_into())
    }};
}

macro_rules! pin_trait {
    ($signal:ident, $instance:path $(, $mode:path)?) => {
        #[doc = concat!(stringify!($signal), " pin trait")]
        pub trait $signal<T: $instance $(, M: $mode)?>: crate::gpio::Pin {
        }
    };
}